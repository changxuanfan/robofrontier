"""Loopback-only read-only Oracle status and simple-background live frames."""
import argparse
import json
import time
import os
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

FAMILIES=('transport','mosaic','relay','dial')
PAGE='''<!doctype html><meta charset="utf-8"><title>RoboFrontier Oracle</title>
<style>body{font:17px system-ui;background:#eef2f5;color:#203342;margin:28px}main{display:grid;grid-template-columns:1fr 1fr;gap:22px}article{padding:18px;background:white;border-radius:14px}img{width:100%;min-height:120px;background:#dce4e9}pre{white-space:pre-wrap;font:14px system-ui}h1{font-size:28px}</style>
<h1>Oracle · 四类任务</h1><p>真实简单背景物理执行；工厂仅离线回放。画面约每五秒更新，规划阶段可能更久。</p><main id="cards"></main><script>
const families=['transport','mosaic','relay','dial'];for(const f of families)cards.innerHTML+=`<article><h2>${f}</h2><pre id="s-${f}">等待</pre><img id="i-${f}"></article>`;
async function update(){let data=await(await fetch('/status')).json();for(const f of families){document.getElementById('s-'+f).textContent=JSON.stringify(data[f],null,2);if(data[f].image)document.getElementById('i-'+f).src='/image/'+f+'?t='+Date.now()}}update();setInterval(update,5000);
</script>'''


def main():
    p=argparse.ArgumentParser();p.add_argument('--batch',type=Path,action='append');p.add_argument('--port',type=int,default=0);a=p.parse_args()
    batches=a.batch or [Path('runs/oracle/suite-v2-r10-20261025')]
    def paths(family):
        return [path for batch in batches for path in (batch/family).glob('*/seed-*/attempt-*/status.json')]
    def latest(family):
        candidates=paths(family)
        return max(candidates,key=lambda f:f.stat().st_mtime).parent if candidates else None
    def frame(folder):
        if folder is None:return None
        status=json.loads((folder/'status.json').read_text())
        terminal=status.get('phase') in ('passed','failed','interrupted','commissioning_complete')
        names=('final-global.png','failure-global.png','live.png','initial-global.png') if terminal else ('live.png','initial-global.png')
        for name in names:
            if (folder/name).exists():return folder/name
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route=urlparse(self.path).path
            if route=='/':payload=PAGE.encode();kind='text/html; charset=utf-8'
            elif route=='/status':
                state={}
                for f in FAMILIES:
                    folder=latest(f)
                    status=json.loads((folder/'status.json').read_text()) if folder else {'phase':'waiting_for_gate'}
                    status['case']=folder.parents[1].name if folder else None
                    status['batch']=folder.parents[3].name if folder else None
                    picture=frame(folder);status['image']=bool(picture)
                    status['image_age_s']=round(time.time()-picture.stat().st_mtime,1) if picture else None
                    status['image_source']=picture.name if picture else None
                    active=[]
                    for path in paths(f):
                        item=json.loads(path.read_text())
                        process_path=path.parents[2]/'process.json'
                        alive=time.time()-path.stat().st_mtime<120
                        if process_path.exists():
                            process=json.loads(process_path.read_text());alive=False
                            if process.get('phase')=='running':
                                try:os.kill(process['pid'],0);alive=True
                                except ProcessLookupError:pass
                        if item.get('phase') not in ('passed','failed','interrupted','commissioning_complete') and alive:
                            active.append(dict(batch=path.parents[4].name,case=path.parents[2].name,attempt=path.parent.name,**item))
                    status['active_worlds']=active;state[f]=status
                payload=json.dumps(state).encode();kind='application/json'
            elif route.startswith('/image/') and route.rsplit('/',1)[1] in FAMILIES:
                image=frame(latest(route.rsplit('/',1)[1]))
                if image is None:self.send_error(404);return
                payload=image.read_bytes();kind='image/png'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(payload)
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler)
    print('http://127.0.0.1:'+str(server.server_port)+'/',flush=True);server.serve_forever()


if __name__=='__main__':main()
