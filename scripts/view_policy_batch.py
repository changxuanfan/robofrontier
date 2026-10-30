"""Read-only bounded GPT trial dashboard; never submits robot/model requests."""
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.parse import urlsplit

from robofrontier.runtime_store import RuntimeStore

HTML='''<!doctype html><meta charset="utf-8"><title>GPT ·12项限时试验</title>
<style>body{margin:32px;background:#f4f6f8;color:#20313d;font:16px system-ui}h1{font-size:29px;margin-bottom:8px}.sub{color:#60707b;margin:8px 0 25px}.group{margin:26px 0}h2{font-size:21px}.grid{display:grid;grid-template-columns:repeat(3,minmax(260px,1fr));gap:18px}.card{background:white;border:1px solid #dce4ea;border-radius:13px;padding:18px}.head{display:flex;justify-content:space-between;gap:10px}.badge{color:#00766c;background:#e2f4ef;border-radius:20px;padding:4px 10px;font-size:13px}img{width:100%;aspect-ratio:1.6;object-fit:contain;margin:12px 0;background:#edf1f4;border-radius:7px}.dim{color:#687883;font-size:13px}.actions{display:flex;gap:16px;margin-top:12px}a{color:#12659b;text-decoration:none}.warn{color:#9c5a12}footer{color:#60707b;margin-top:30px}</style>
<h1>GPT ·12项限时试验</h1><p class="sub">Adaptive · gpt-6.1-sol / high · 每项START后最多20分钟 · 四类独立运行</p><div id="content"></div>
<footer>图像是Agent最近一次全局观测或结束末帧，可能滞后于动作；原生MuJoCo窗口显示实时运动。录像为原速，完整动态状态另存用于离线回放。试验不等于Oracle认证。</footer>
<script>
const labels={transport:'物品传递',mosaic:'拼图',relay:'空中接力',dial:'旋钮密码锁'};
const states={prepared:'尚未开始',running:'运行中',ended:'已结束',interrupted:'中断',recovery_required:'待核实'};
const esc=x=>String(x??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function update(){try{const d=await(await fetch('/api')).json();let html='';for(const [key,label] of Object.entries(labels)){html+=`<section class="group"><h2>${label}</h2><div class="grid">`;for(const r of d.cases.filter(x=>x.family===key)){const left=r.start_at&&r.lifecycle==='running'?Math.max(0,1200-(Date.now()/1000-r.start_at)):null;html+=`<article class="card"><div class="head"><b>Level ${r.level} · ${r.arms}臂</b><span class="badge">${esc(states[r.lifecycle]||r.lifecycle)}</span></div><div class="dim">${esc(r.case_id)}</div><img src="/frame/${encodeURIComponent(r.case_id)}?v=${r.image_mtime||0}" onerror="this.style.opacity=.3"><div>${left!==null?`剩余 ${Math.floor(left/60)}分${Math.floor(left%60)}秒`:r.evaluation?(r.evaluation.task_success?'验收通过':'本项结束，未完整通过'):'等待本项启动'}</div><p class="dim">动作 ${r.actions} · 分身 ${r.delegates} · ${r.image_kind==='preview'?'静态环境预览':r.image_mtime?`最近图像 ${Math.max(0,Math.floor(Date.now()/1000-r.image_mtime))}秒前`:'尚无图像'}</p><div class="actions">${r.viewer&&r.lifecycle==='running'?`<a target="_blank" href="${esc(r.viewer)}">观察面板</a>`:''}${r.video?`<a target="_blank" href="/video/${encodeURIComponent(r.case_id)}">原速录像</a>`:''}</div></article>`;}html+='</div></section>';}document.getElementById('content').innerHTML=html;}catch(e){document.getElementById('content').textContent='正在读取运行状态…';}}update();setInterval(update,3000);
</script>'''


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);p.add_argument('--port',type=int,default=0);a=p.parse_args();base=a.batch.resolve()
    def rows():return json.loads((base/'batch.json').read_text())['cases']
    def row(case):return next(x for x in rows() if x['case_id']==case)
    def frame(r):
        run=Path(r['run_dir'])
        finals=[run/'private/outputs'/('final-global'+ext) for ext in ('.png','.jpg','.jpeg')]
        finals=[p for p in finals if p.exists()]
        if finals:return max(finals,key=lambda x:x.stat().st_mtime),'final'
        files=[p for ext in ('png','jpg','jpeg') for p in (run/'public/observations').glob('*-global.'+ext)]
        if files:return max(files,key=lambda x:x.stat().st_mtime),'observation'
        return run/'public/inputs/preview-global.png','preview'
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,data,kind):
            self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def do_GET(self):
            path=urlsplit(self.path).path
            try:
                if path=='/':return self.reply(HTML.encode(),'text/html;charset=utf-8')
                if path=='/api':
                    values=[]
                    for r in rows():
                        run=Path(r['run_dir']);s=RuntimeStore(run/'private/state/runtime.sqlite')
                        item={k:r[k] for k in ('case_id','family','level','arms')};item.update(lifecycle=s.get('lifecycle'),start_at=s.get('start_at'),actions=len(s.actions()),delegates=sum(v['kind']=='delegate' for v in s.agents().values()));s.close()
                        image,kind=frame(r);item.update(image_kind=kind,image_mtime=image.stat().st_mtime if image.exists() else None,video=(run/'private/outputs/global-realtime.mp4').is_file())
                        observer=run/'private/state/observer.json';item['viewer']=json.loads(observer.read_text())['url'] if observer.exists() else None
                        evaluation=run/'private/outputs/evaluation.json';item['evaluation']=json.loads(evaluation.read_text()) if evaluation.exists() else None;values.append(item)
                    return self.reply(json.dumps(dict(cases=values)).encode(),'application/json')
                if path.startswith('/frame/'):
                    f,_=frame(row(path.removeprefix('/frame/')));return self.reply(f.read_bytes(),'image/jpeg' if f.suffix.lower() in ('.jpg','.jpeg') else 'image/png')
                if path.startswith('/video/'):
                    f=Path(row(path.removeprefix('/video/'))['run_dir'])/'private/outputs/global-realtime.mp4';size=f.stat().st_size;start=0;end=size-1;part=self.headers.get('Range')
                    if part:
                        m=re.fullmatch(r'bytes=(\d+)-(\d*)',part)
                        if not m:raise ValueError('Unsupported range')
                        start=int(m[1]);end=min(end,int(m[2]) if m[2] else end)
                        if not 0<=start<=end:raise ValueError('Invalid range')
                    self.send_response(206 if part else 200);self.send_header('Content-Type','video/mp4');self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
                    if part:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
                    self.end_headers()
                    with f.open('rb') as stream:
                        stream.seek(start);left=end-start+1
                        while left:
                            data=stream.read(min(left,1024*1024))
                            if not data:break
                            self.wfile.write(data);left-=len(data)
                    return
                self.send_error(404)
            except (FileNotFoundError,StopIteration,ValueError):self.send_error(404)
            except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler);server.daemon_threads=True
    print(json.dumps(dict(url=f'http://127.0.0.1:{server.server_port}/',read_only=True)),flush=True);server.serve_forever()


if __name__=='__main__':main()
