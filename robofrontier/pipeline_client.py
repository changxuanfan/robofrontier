"""Role-bound tool client with durable requests and receipt acknowledgements."""
import argparse
import fcntl
import json
import re
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request
import uuid
from .run_setup import write_json


def client_call(connection_path, op, data, call_id=None, timeout=45, log_call_id=True):
    connection_path=Path(connection_path).resolve()
    cfg=json.loads(connection_path.read_text())
    if cfg.get('schema_version')!=3:
        raise ValueError('Legacy run: use its frozen client; v3 will not mutate old runs')
    state_dir=connection_path.parent.parent/'client-state'/cfg['actor']
    state_dir.mkdir(parents=True,exist_ok=True)
    lock=(state_dir/'client.lock').open('a+')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if op=='pending':
            return {'pending':[json.loads(p.read_text())['body'] for p in state_dir.glob('*.json')
                               if p.name!='receipt.json' and json.loads(p.read_text()).get('result') is None]}
        if not cfg.get('url'):
            raise ValueError('Execution service not started; client never starts one automatically')
        call_id=call_id or uuid.uuid4().hex
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',call_id):
            raise ValueError('Use a simple call ID, not a path')
        path=state_dir/(call_id+'.json')
        if path.exists():
            saved=json.loads(path.read_text());body=saved['body']
            if body['op']!=op or body['data']!=data:
                raise ValueError('Call ID reused with different operation/parameters')
            if saved.get('result') is not None:return saved['result']
        else:
            receipt_path=state_dir/'receipt.json'
            receipt=json.loads(receipt_path.read_text()).get('delivery_id') if receipt_path.exists() else None
            body={'schema_version':3,'run_id':cfg['run_id'],'actor':cfg['actor'],
                  'call_id':call_id,'op':op,'data':data,'ack_delivery':receipt}
            write_json(path,{'body':body,'result':None});path.chmod(0o600)
        # Protect durable state, not network/robot duration. Independent calls
        # from one Controller must be able to overlap on different arms.
        fcntl.flock(lock,fcntl.LOCK_UN)
        if log_call_id:print('call_id='+call_id,file=sys.stderr,flush=True)
        request=urllib.request.Request(cfg['url']+'/op',json.dumps(body,allow_nan=False,ensure_ascii=False).encode(),
                    headers={'Content-Type':'application/json','X-Token':cfg['token']})
        # The service's wait deadline belongs to this original call, not each
        # HTTP slice. Reusing it does not register a new wait or reset its clock.
        wait_deadline=time.monotonic()+float(data.get('timeout_s',30))+timeout if op=='wait_events' else None
        while True:
            with urllib.request.urlopen(request,timeout=timeout) as response:result=json.load(response)
            if result.get('execution_status')!='pending':
                break
            if op=='delegate':continue  # preserve the parent's fork boundary; original request only
            # The server slices HTTP waits at 40 s; a slow valid action may
            # span several slices. Reuse the durable ID, never resubmit motion.
            if op in ('move','gripper','action_batch') and data.get('wait',True):continue
            if op!='wait_events' or time.monotonic()>=wait_deadline:break
        fcntl.flock(lock,fcntl.LOCK_EX)
        if result.get('execution_status') not in ('pending','running'):
            write_json(path,{'body':body,'result':result});path.chmod(0o600)
        if result.get('delivery_id'):
            receipt=state_dir/'receipt.json';write_json(receipt,{'delivery_id':result['delivery_id']});receipt.chmod(0o600)
        return result
    finally:
        fcntl.flock(lock,fcntl.LOCK_UN);lock.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connection',type=Path,required=True)
    p.add_argument('--call-id')
    p.add_argument('--quiet',action='store_true',help='JSON stdout only; errors still include the original call ID')
    p.add_argument('op')
    p.add_argument('--data',default='{}')
    args=p.parse_args()
    call_id=args.call_id or uuid.uuid4().hex
    try:
        result=client_call(args.connection,args.op,json.loads(args.data),call_id,log_call_id=not args.quiet)
        print(json.dumps(result,ensure_ascii=False))
    except (ValueError,OSError,urllib.error.URLError) as error:
        print(json.dumps({'call_id':call_id,'error':str(error),'note':'Read pending/retry original call ID after an uncertain transport outcome; do not infer non-execution'},ensure_ascii=False))
        raise SystemExit(1)

if __name__=='__main__':main()
