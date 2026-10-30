"""No inference or physics: durable conflicts, HTTP errors and safe CLI feedback."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace

from robofrontier.pipeline_client import client_call
from robofrontier.runtime_store import RuntimeStore
from robofrontier.task_config import load_harness
from robofrontier.tool_errors import CallIdConflict, conflict_error


class ToolErrorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.store = RuntimeStore(self.root/'runtime.sqlite')
        self.store.initialize('run', load_harness(), ['A','B'])
        self.mode = 'normal'
        self.received = []
        test = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_POST(self):
                body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                test.received.append(body)
                if test.mode == 'lost-response':
                    test.store.begin_call(body['call_id'],body['actor'],body['op'],body['data'])
                    test.store.finish_call(body['call_id'],{'call_id':body['call_id'],'messages':[]})
                    self.close_connection=True
                    return
                if test.mode == 'normal':
                    try:
                        old=test.store.begin_call(body['call_id'],body['actor'],body['op'],body['data'])
                    except CallIdConflict as error:
                        status, value=409,conflict_error(error.call_id)
                    else:
                        status=200
                        value=json.loads(old['result']) if old['result'] else {'call_id':body['call_id'],'messages':[]}
                        if old['status']=='new': test.store.finish_call(body['call_id'],value)
                elif test.mode == 'wrong-id': status,value=409,conflict_error('another-id')
                elif test.mode == 'arbitrary-409': status,value=409,{'error':'not our conflict protocol'}
                elif test.mode == 'bad-json': status,value=409,None
                else: status,value=500,{'error':'sk-fake-secret'}
                raw=b'not JSON' if value is None else json.dumps(value).encode()
                self.send_response(status);self.send_header('Content-Type','application/json')
                self.end_headers();self.wfile.write(raw)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.connections={}
        for actor in ('A','B'):
            path=self.root/'private/state/connections'/(actor+'.json')
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps({'schema_version':3,'run_id':'run','actor':actor,
                'url':f'http://127.0.0.1:{self.server.server_port}','token':'fake-test-token'}))
            self.connections[actor]=path

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.store.close();self.tmp.cleanup()

    def cli(self, actor, op, data, call_id):
        return subprocess.run([sys.executable,'-m','robofrontier.pipeline_client','--quiet',
            '--connection',str(self.connections[actor]),'--call-id',call_id,op,'--data',json.dumps(data)],
            capture_output=True,text=True,timeout=10)

    def test_local_conflict_preserves_original_and_never_sends_changed_request(self):
        original=client_call(self.connections['A'],'observe',{},'same',log_call_id=False)
        saved=(self.root/'private/state/client-state/A/same.json').read_bytes()
        count=len(self.received)
        result=self.cli('A','move',{'target_m':[1,2,3]},'same')
        self.assertEqual(result.returncode,1)
        error=json.loads(result.stdout)
        self.assertEqual(error['error_code'],'CALL_ID_CONFLICT')
        self.assertEqual(error['request_status'],'rejected')
        self.assertEqual(error['original_call_status'],'unknown')
        self.assertEqual(len(self.received),count)
        self.assertEqual((self.root/'private/state/client-state/A/same.json').read_bytes(),saved)
        self.assertEqual(client_call(self.connections['A'],'observe',{},'same',log_call_id=False),original)
        self.assertEqual(len(self.received),count)

    def test_query_uses_new_request_id_and_original_action_target(self):
        client_call(self.connections['A'],'move',{'target_m':[1,2,3]},'action-A',log_call_id=False)
        count=len(self.received)
        result=self.cli('A','job',{'call_id':'action-A'},'query-A')
        self.assertEqual(result.returncode,0)
        self.assertEqual(len(self.received),count+1)
        self.assertEqual(self.received[-1]['call_id'],'query-A')
        self.assertEqual(self.received[-1]['data']['call_id'],'action-A')
        rejected=self.cli('A','job',{'call_id':'action-A'},'action-A')
        self.assertEqual(json.loads(rejected.stdout)['error_code'],'CALL_ID_CONFLICT')
        self.assertEqual(len(self.received),count+1)
        self.assertIn('data.call_id',json.loads(rejected.stdout)['note'])

    def test_uncertain_physical_call_reuses_id_without_automatic_replay(self):
        self.mode='lost-response'
        result=self.cli('A','move',{'target_m':[1,2,3]},'action-uncertain')
        value=json.loads(result.stdout)
        self.assertEqual(value['error_code'],'TRANSPORT_UNCERTAIN')
        self.assertEqual(value['call_id'],'action-uncertain')
        self.assertEqual(len(self.received),1)
        self.assertEqual(self.store.call('action-uncertain')['status'],'completed')
        self.assertEqual(len(client_call(self.connections['A'],'pending',{},log_call_id=False)['pending']),1)
        self.assertIn('data.call_id',value['note'])

    def test_remote_cross_role_conflict_preserves_owner_and_has_no_replay(self):
        client_call(self.connections['A'],'observe',{},'shared-id',log_call_id=False)
        original=self.store.call('shared-id')
        result=self.cli('B','move',{'target_m':[1,2,3]},'shared-id')
        self.assertEqual(json.loads(result.stdout)['error_code'],'CALL_ID_CONFLICT')
        self.assertEqual(result.returncode,1)
        self.assertEqual(self.store.call('shared-id'),original)
        self.assertEqual(self.store.actions(),{})
        self.assertNotIn('target_m',result.stdout)
        self.assertEqual(client_call(self.connections['B'],'pending',{},log_call_id=False)['pending'],[])
        count=len(self.received)
        with self.assertRaises(CallIdConflict):
            client_call(self.connections['B'],'move',{'target_m':[1,2,3]},'shared-id',log_call_id=False)
        self.assertEqual(len(self.received),count)

    def test_unrecognized_server_errors_remain_unknown_and_do_not_retry(self):
        for index,mode in enumerate(('wrong-id','arbitrary-409','bad-json','server-error')):
            self.mode=mode;before=len(self.received)
            result=self.cli('A','observe',{},'unknown-'+str(index))
            self.assertEqual(result.returncode,1)
            value=json.loads(result.stdout)
            self.assertEqual(value['error_code'],'TRANSPORT_UNCERTAIN')
            self.assertEqual(value['request_status'],'unknown')
            self.assertEqual(len(self.received),before+1)
            self.assertNotIn('sk-fake-secret',result.stdout+result.stderr)

    def test_connection_failure_remains_pending_under_original_id(self):
        p=self.connections['A'];cfg=json.loads(p.read_text());cfg['url']='http://127.0.0.1:0';p.write_text(json.dumps(cfg))
        result=self.cli('A','observe',{},'unconfirmed')
        self.assertEqual(json.loads(result.stdout)['error_code'],'TRANSPORT_UNCERTAIN')
        pending=client_call(p,'pending',{},log_call_id=False)['pending']
        self.assertEqual([v['call_id'] for v in pending],['unconfirmed'])

    def test_lost_reply_after_completion_stays_unknown_and_original_retry_is_safe(self):
        self.mode='lost-response'
        result=self.cli('A','observe',{},'completed-but-unconfirmed')
        self.assertEqual(json.loads(result.stdout)['error_code'],'TRANSPORT_UNCERTAIN')
        self.assertEqual(len(self.received),1)
        original=self.store.call('completed-but-unconfirmed')
        self.assertEqual(original['status'],'completed')
        self.assertEqual(len(client_call(self.connections['A'],'pending',{},log_call_id=False)['pending']),1)
        self.mode='normal'
        client_call(self.connections['A'],'observe',{},'completed-but-unconfirmed',log_call_id=False)
        self.assertEqual(self.store.call('completed-but-unconfirmed'),original)
        self.assertEqual(client_call(self.connections['A'],'pending',{},log_call_id=False)['pending'],[])

    def test_production_http_handler_returns_typed_conflict_without_world(self):
        from robofrontier.pipeline import Service
        def accept(actor,op,data,call_id,ack):
            self.store.begin_call(call_id,actor,op,data)
            done=threading.Event();done.set()
            return {'id':call_id,'result':{'call_id':call_id},'done':done}
        service=SimpleNamespace(directory=Path('run'),tokens={'A':'fake-test-token','B':'fake-test-token'},accept=accept)
        handler=Service.http_handler(service,'fake-observer')
        server=ThreadingHTTPServer(('127.0.0.1',0),handler)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            for p in self.connections.values():
                cfg=json.loads(p.read_text());cfg['url']=f'http://127.0.0.1:{server.server_port}';p.write_text(json.dumps(cfg))
            client_call(self.connections['A'],'observe',{},'production-id',log_call_id=False)
            result=self.cli('B','observe',{},'production-id')
            self.assertEqual(result.returncode,1)
            self.assertEqual(json.loads(result.stdout)['error_code'],'CALL_ID_CONFLICT')
            self.assertEqual(self.store.call('production-id')['actor'],'A')
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__': unittest.main()
