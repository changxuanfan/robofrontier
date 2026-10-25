"""Read-only configured-camera panel for an already-running shared world.

Runs independently of the research conversation; never submits robot actions.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import urllib.request
import uuid

CAMERAS=['global','global_side']+[a+'_'+s for a in 'ABC' for s in ('front','side')]
p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--port',type=int,default=0);args=p.parse_args()
run=args.run_dir.resolve()
connection_path=next((run/p for p in ('private/state/connections/Meta.json','private/connections/Meta.json','public/connection-research.json') if (run/p).exists()),None)
if connection_path is None:raise SystemExit('No existing role connection; camera panel never starts a world')
connection=json.loads(connection_path.read_text())
task_path=next((run/p for p in ('public/inputs/task.json','public/task.json') if (run/p).exists()),run/'public/inputs/task.json')
if task_path.exists():CAMERAS=json.loads(task_path.read_text()).get('cameras',CAMERAS)
if not connection.get('url'):raise SystemExit('Execution owner not started; camera panel never starts one')
lock=threading.Lock();cache={'images':{},'captured_at':None,'sim_time_s':None,'error':None}

def capture():
    with lock:
        body={'actor':connection['actor'],'op':'observe','call_id':uuid.uuid4().hex,'data':{'cameras':CAMERAS}}
        if connection.get('schema_version') in (2,3):body.update(schema_version=connection['schema_version'],run_id=connection['run_id'])
        req=urllib.request.Request(connection['url']+'/op',json.dumps(body).encode(),headers={'Content-Type':'application/json','X-Token':connection['token']})
        with urllib.request.urlopen(req,timeout=45) as r:result=json.load(r)
        if result.get('error'):raise RuntimeError(result['error'])
        if 'observation' not in result:raise RuntimeError('Capture pending; try refresh later')
        obs=result['observation'];cache['images']={c:Path(f).read_bytes() for c,f in obs['images'].items()}
        cache.update(captured_at=time.time(),sim_time_s=obs['sim_time_s'],error=None)
        return {'cameras':CAMERAS,'sim_time_s':cache['sim_time_s'],'captured_at':cache['captured_at']}

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*a):pass
    def reply(self,body,kind='application/json',status=200):
        raw=body if isinstance(body,bytes) else json.dumps(body,ensure_ascii=False).encode()
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
    def do_GET(self):
        path=self.path.split('?')[0]
        if path=='/':return self.reply((Path(__file__).resolve().parents[1]/'assets/pipeline_cameras.html').read_bytes(),'text/html; charset=utf-8')
        if path.startswith('/frames/'):
            name=path.removeprefix('/frames/').removesuffix('.png')
            with lock:raw=cache['images'].get(name)
            return self.reply(raw,'image/png') if raw else self.reply({'error':'refresh first'},status=404)
        if path=='/api/status':
            with lock:return self.reply({'cameras':CAMERAS,**{k:v for k,v in cache.items() if k!='images'}})
        return self.reply({'error':'not found'},status=404)
    def do_POST(self):
        if self.path!='/api/refresh':return self.reply({'error':'not found'},status=404)
        try:return self.reply(capture())
        except Exception as e:return self.reply({'error':str(e)},status=503)

server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.daemon_threads=True
url=f'http://127.0.0.1:{server.server_port}/';panel_file=run/('private/state/camera-panel.json' if connection.get('schema_version')==3 else 'camera-panel.json');panel_file.write_text(json.dumps({'url':url,'pid':__import__('os').getpid(),'read_only':True},indent=2)+'\n')
print(json.dumps({'url':url,'read_only':True}),flush=True)
server.serve_forever()
