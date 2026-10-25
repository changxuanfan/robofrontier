"""Historical external app-server probe, outside the active harness.

Only for a separately authorized idle test thread; never a setup/check command.
"""
import argparse,json,queue,subprocess,threading,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--thread-id',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.parent.mkdir(parents=True,exist_ok=True)
proc=subprocess.Popen(['codex','app-server','--listen','stdio://'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=a.output.with_suffix('.stderr').open('w'),text=True,bufsize=1)
q=queue.Queue();log=[]
def reader():
 for line in proc.stdout:
  try:q.put(json.loads(line))
  except ValueError:pass
threading.Thread(target=reader,daemon=True).start()
def save(r):
 log.append(r);a.output.write_text(json.dumps(log,ensure_ascii=False,indent=2)+'\n')
def rpc(i,m,params):
 proc.stdin.write(json.dumps({'id':i,'method':m,'params':params})+'\n');proc.stdin.flush()
 deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  r=q.get(timeout=30);save(r)
  if r.get('id')==i:return r
 raise TimeoutError(m)
try:
 r=rpc(1,'initialize',{'clientInfo':{'name':'robofrontier_scheduler','version':'0.1'},'capabilities':{'experimentalApi':True}})
 if r.get('error'):raise RuntimeError(r['error'])
 proc.stdin.write('{"method":"initialized"}\n');proc.stdin.flush()
 r=rpc(2,'thread/resume',{'threadId':a.thread_id,'excludeTurns':True,'model':'gpt-6-luna'})
 print('resume',r.get('error') or {'status':r.get('result',{}).get('thread',{}).get('status')},flush=True)
 if r.get('error'):raise RuntimeError(r['error'])
 r=rpc(3,'turn/start',{'threadId':a.thread_id,'model':'gpt-6-luna','effort':'xhigh','input':[{'type':'text','text':'独立调度工程验收：这一轮由独立程序通过 App Server 发起，research 会话没有发原生续轮通知。请仅回复 SCHEDULER_EXTERNAL_TURN_OK 并结束；不调用任何工具，不操作机器人，不修改文件。'}]})
 print('turn',r.get('error') or r.get('result',{}).get('turn',{}).get('id'),flush=True)
 if r.get('error'):raise RuntimeError(r['error'])
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  r=q.get(timeout=90);save(r)
  if r.get('method') in ('turn/started','turn/completed','thread/status/changed','error'):print(json.dumps(r,ensure_ascii=False)[:1800],flush=True)
  if r.get('method')=='turn/completed':break
 else:raise TimeoutError('probe turn')
finally:proc.terminate()
