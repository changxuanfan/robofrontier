"""Exercise Claude CLI against local simulated model and robot servers; no paid calls."""
import http.server, json, os, pathlib, subprocess, tempfile, threading

ROOT = pathlib.Path(__file__).resolve().parents[1]
REQUESTS = []
from http import HTTPStatus
class H(http.server.BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def do_POST(self):
        body=json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))))
        tools=body.get('tools',[])
        names=[x.get('name') for x in tools if isinstance(x,dict)]
        REQUESTS.append(body)
        found=next((x for x in names if 'robot' in x),None)
        previous=any(x.get('role')=='user' and any(c.get('type')=='tool_result' for c in x.get('content',[]) if isinstance(c,dict)) for x in body.get('messages',[]) if isinstance(x.get('content'),list))
        if found and not previous:
            content=[{'type':'tool_use','id':'toolu_test','name':found,'input':{'op':'observe','data':{},'views':['top']}}]; reason='tool_use'
        else:
            content=[{'type':'text','text':'done'}]; reason='end_turn'
        message={'id':'msg_stub','type':'message','role':'assistant','model':'claude-opus-5-5','content':content,'stop_reason':reason,'stop_sequence':None,'usage':{'input_tokens':10,'output_tokens':4}}
        if body.get('stream'):
            events=[('message_start',{'type':'message_start','message':{**message,'content':[], 'stop_reason':None}})]
            for idx,item in enumerate(content):
                initial={'type':'tool_use','id':item['id'],'name':item['name'],'input':{}} if item['type']=='tool_use' else {'type':'text','text':''}
                delta={'type':'input_json_delta','partial_json':json.dumps(item['input'])} if item['type']=='tool_use' else {'type':'text_delta','text':item['text']}
                events.extend([('content_block_start',{'type':'content_block_start','index':idx,'content_block':initial}),('content_block_delta',{'type':'content_block_delta','index':idx,'delta':delta}),('content_block_stop',{'type':'content_block_stop','index':idx})])
            events.extend([('message_delta',{'type':'message_delta','delta':{'stop_reason':reason,'stop_sequence':None},'usage':{'output_tokens':4}}),('message_stop',{'type':'message_stop'})])
            data=''.join('event: '+name+'\ndata: '+json.dumps(payload)+'\n\n' for name,payload in events).encode();typ='text/event-stream'
        else:data=json.dumps(message).encode();typ='application/json'
        self.send_response(HTTPStatus.OK);self.send_header('Content-Type',typ);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
class World(http.server.BaseHTTPRequestHandler):
    image = None
    def log_message(self, *args): pass
    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length', 0)))
        data = json.dumps({'observation': {'images': {'top': str(self.image)}}}).encode()
        self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

def main():
    with tempfile.TemporaryDirectory() as tmp:
        t=pathlib.Path(tmp);run=t/'run';conn=run/'private/state/connections/Controller.json';conn.parent.mkdir(parents=True)
        image = run/'public/observations/top.png'; image.parent.mkdir(parents=True); __import__('PIL.Image', fromlist=['new']).new('RGB', (4, 4), 'red').save(image)
        World.image = image
        world=http.server.ThreadingHTTPServer(('127.0.0.1',0),World);threading.Thread(target=world.serve_forever,daemon=True).start()
        conn.write_text(json.dumps({'run_id':'run','actor':'Controller','schema_version':3,'url':f'http://127.0.0.1:{world.server_port}','token':'fake'}))
        root=str(ROOT)
        config={'mcpServers':{'robot':{'command':root+'/.venv/bin/python','args':['-m','robofrontier.claude.robot_mcp','--run-dir',str(run),'--connection',str(conn)],'env':{'PYTHONPATH':root}}}}
        cfg=t/'mcp.json';cfg.write_text(json.dumps(config))
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),H);threading.Thread(target=server.serve_forever,daemon=True).start()
        env={'PATH':os.environ.get('PATH',''), 'HOME':str(t), 'ANTHROPIC_BASE_URL':f'http://127.0.0.1:{server.server_port}','ANTHROPIC_API_KEY':'local-fake-key','CLAUDE_CONFIG_DIR':str(t/'config'),'DISABLE_TELEMETRY':'1','DISABLE_ERROR_REPORTING':'1','CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC':'1'}
        cmd=['claude','--bare','-p','Call robot observe tool.','--model','claude-opus-5-5','--mcp-config',str(cfg),'--strict-mcp-config','--allowedTools','mcp__robot__robot','--permission-mode','dontAsk','--tools','','--output-format','stream-json','--verbose']
        try:
            p=subprocess.run(cmd,cwd=run,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=45)
            assert p.returncode == 0, 'Claude CLI failed: ' + p.stderr[-1000:]
            assert len(REQUESTS) >= 2, 'Expected tool and follow-up model requests'
            assert all([x.get('name') for x in req.get('tools', [])] == ['mcp__robot__robot'] for req in REQUESTS), 'Unexpected tools exposed'
            followup = json.dumps(REQUESTS[-1])
            assert '"type": "image"' in followup and 'iVBORw0KGgo' in followup, 'Tool image was not forwarded to the model endpoint'
            assert 'tool_result' in followup, 'Missing tool result'
            events = [json.loads(line) for line in p.stdout.splitlines() if line.strip()]
            parent = next(e['session_id'] for e in events if e.get('type') == 'result')
            old_count = len(REQUESTS)
            fork_cmd = cmd + ['--resume', parent, '--fork-session']
            fork = subprocess.run(fork_cmd, cwd=run, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=45)
            assert fork.returncode == 0, 'Fork failed: ' + fork.stderr[-1000:]
            fork_events = [json.loads(line) for line in fork.stdout.splitlines() if line.strip()]
            child = next(e['session_id'] for e in fork_events if e.get('type') == 'result')
            assert child != parent, 'Fork reused the parent session'
            assert len(REQUESTS) > old_count and 'toolu_test' in json.dumps(REQUESTS[-1]['messages']), 'Fork did not include prior tool history'
            assert '"type": "image"' in json.dumps(REQUESTS[-1]['messages']), 'Fork did not include prior tool image'
            print(json.dumps({'passed': True, 'claude_exit': p.returncode, 'mcp_tool': 'robot', 'tool_image_forwarded': True, 'fork_has_prior_tool_and_image_history': True, 'real_model_calls': 0, 'note': 'Local simulated model and world; no provider compatibility or trusted provenance claim'}))
        finally:
            server.shutdown(); server.server_close()
            world.shutdown(); world.server_close()


if __name__ == "__main__":
    main()
