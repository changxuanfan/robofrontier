"""Exercise snapshot forks and per-child prompts with a local Responses stub.

Uses the installed Codex CLI in an isolated temporary CODEX_HOME. No credentials,
external model calls, robot world, or production sessions are used.
"""
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

from PIL import Image
from scripts.benchmark import launch_config
from robofrontier.run_setup import write_json
from robofrontier.runtime_cli import fork_provenance


def check():
    requests = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            requests.append(body)
            value = json.dumps(body.get('input'))
            tool = ('probe' if 'input_image' not in value else
                    'delegate' if 'DELEGATE_TRIGGER' in value and 'delegate result' not in value
                    and 'CHILD_SCOPE_' not in value else None)
            item = (dict(id='fc_'+str(len(requests)), type='function_call',
                         call_id='call_'+str(len(requests)), name=tool, arguments='{}', status='completed')
                    if tool else dict(id='msg_'+str(len(requests)), type='message', role='assistant',
                                      status='completed', content=[dict(type='output_text',
                                      text='OFFLINE_OK', annotations=[])]))
            response = dict(id='resp_'+str(len(requests)), object='response', created_at=int(time.time()),
                            status='completed', model=body.get('model'), output=[item],
                            usage=dict(input_tokens=10,output_tokens=10,total_tokens=20,
                                       input_tokens_details={'cached_tokens':0},
                                       output_tokens_details={'reasoning_tokens':0}))
            self.send_response(200);self.send_header('Content-Type','text/event-stream');self.end_headers()
            events=[('response.created',{'response':{**response,'status':'in_progress','output':[]}}),
                    ('response.output_item.added',{'output_index':0,'item':{**item,'status':'in_progress'}}),
                    ('response.output_item.done',{'output_index':0,'item':item}),
                    ('response.completed',{'response':response})]
            for i,(kind,data) in enumerate(events):
                self.wfile.write(('event: '+kind+'\ndata: '+json.dumps({'type':kind,'sequence_number':i,**data})+'\n\n').encode())
            self.wfile.flush()
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    previous_home=os.environ.get('CODEX_HOME')
    try:
        with tempfile.TemporaryDirectory(prefix='robofrontier-snapshot-check-') as tmp:
            root=Path(tmp);home=root/'home';home.mkdir();os.environ['CODEX_HOME']=str(home)
            p=dict(model='offline-protocol',provider='offline_protocol',effort='high',vision=True,
                   context_window=272000,auto_compact_token_limit=245000,
                   base_url=f'http://127.0.0.1:{server.server_port}',env_key='ROBOFRONTIER_OFFLINE_STUB_TOKEN')
            write_json(root/'launch.json',launch_config(p,root))
            pixels=io.BytesIO();Image.new('RGB',(32,32),'red').save(pixels,format='PNG')
            write_json(root/'image.json',{'url':'data:image/png;base64,'+base64.b64encode(pixels.getvalue()).decode()})
            script=r'''
const fs=require('fs'),{CodexRpc}=require(process.argv[1]+'/scripts/codex_rpc');
const out=process.argv[2],launch=JSON.parse(fs.readFileSync(out+'/launch.json'));
process.env.ROBOFRONTIER_OFFLINE_STUB_TOKEN='local-stub-no-credential';
(async()=>{
const waiters=new Map();let release,entered,calls=0;
const hold=new Promise(r=>release=r), ready=new Promise(r=>entered=r);
const wait=id=>new Promise(r=>waiters.set(id,r));
const rpc=new CodexRpc(launch.args,{stderr:fs.openSync(out+'/server.log','w'),
 onEvent:m=>{if(m.method==='turn/completed'){const cb=waiters.get(m.params.threadId);if(cb){waiters.delete(m.params.threadId);cb(m.params);}}},
 onRequest:async(m,p)=>{if(m!=='item/tool/call')throw Error(m);calls++;
  if(p.tool==='delegate'){entered();await hold;return {success:true,contentItems:[{type:'inputText',text:'delegate result'}]};}
  if(p.tool!=='probe')throw Error('unexpected tool');
  return {success:true,contentItems:[{type:'inputText',text:'local tool result'},
   {type:'inputImage',imageUrl:JSON.parse(fs.readFileSync(out+'/image.json')).url}]};}});
try {
 await rpc.initialize();const a=await rpc.call('thread/start',{cwd:out,model:launch.model,modelProvider:launch.provider,
 approvalPolicy:'never',sandbox:'read-only',dynamicTools:['probe','delegate'].map(name=>({type:'function',name,description:name,inputSchema:{type:'object',properties:{}}}))});
 let done=wait(a.thread.id);await rpc.call('turn/start',{threadId:a.thread.id,input:[{type:'text',text:'PARENT_TEXT_MARKER'}]});
 let end=await done;if(end.turn.status!=='completed')throw Error(JSON.stringify(end.turn.error));
 done=wait(a.thread.id);await rpc.call('turn/start',{threadId:a.thread.id,input:[{type:'text',text:'DELEGATE_TRIGGER'}]});
 await Promise.race([ready,new Promise((_,reject)=>setTimeout(()=>reject(Error('delegate timeout')),20000))]);
 const anchor=await rpc.call('thread/fork',{threadId:a.thread.id,excludeTurns:true});const forks=[];
 for(let i=0;i<3;i++){const b=await rpc.call('thread/fork',{threadId:anchor.thread.id,excludeTurns:true});forks.push(b.thread);}
 release();end=await done;if(end.turn.status!=='completed')throw Error(JSON.stringify(end.turn.error));
 const b=await rpc.call('thread/fork',{threadId:anchor.thread.id,excludeTurns:true});forks.push(b.thread);
 for(let i=0;i<forks.length;i++){done=wait(forks[i].id);await rpc.call('turn/start',{threadId:forks[i].id,
  input:[{type:'text',text:'CHILD_SCOPE_'+i}]});end=await done;if(end.turn.status!=='completed')throw Error(JSON.stringify(end.turn.error));}
 fs.writeFileSync(out+'/native.json',JSON.stringify({parent:a.thread.id,anchor:anchor.thread.id,
  anchorParent:anchor.thread.forkedFromId,children:forks.map(f=>({id:f.id,parent:f.forkedFromId})),calls}));
} finally {await rpc.close();}
})().then(()=>process.exit(0)).catch(e=>{console.error(e.stack);process.exit(1)});
'''
            env=os.environ.copy();env['ROBOFRONTIER_OFFLINE_STUB_TOKEN']='local-stub-no-credential'
            result=subprocess.run(['node','-e',script,str(Path(__file__).resolve().parents[1]),str(root)],
                                  capture_output=True,text=True,timeout=45,env=env)
            if result.returncode:raise RuntimeError(result.stderr[-3000:])
            native=json.loads((root/'native.json').read_text())
            anchor=fork_provenance(native['parent'],native['anchor'])
            proofs=[fork_provenance(native['anchor'],child['id']) for child in native['children']]
            child_checks=[]
            for i in range(4):
                inputs=[json.dumps(r.get('input')) for r in requests if f'CHILD_SCOPE_{i}' in json.dumps(r.get('input'))]
                value=inputs[-1] if inputs else ''
                child_checks.append(bool(value) and 'PARENT_TEXT_MARKER' in value and 'local tool result' in value
                                    and 'DELEGATE_TRIGGER' in value and 'delegate result' not in value
                                    and 'input_image' in value and all((f'CHILD_SCOPE_{j}' in value)==(i==j) for j in range(4)))
            report={'passed':len(proofs)==4 and all(x==proofs[0] for x in proofs) and all(child_checks)
                    and native['anchorParent']==native['parent'] and all(c['parent']==native['anchor'] for c in native['children'])
                    and native['calls']==2,
                    'external_model_calls':0,'controller_boundary':anchor['source_ordinal'],
                    'child_boundaries':[p['source_ordinal'] for p in proofs],
                    'child_context_and_scope_checks':child_checks,'parent_delegate_pending_at_snapshot':True}
            return report
    finally:
        server.shutdown();server.server_close()
        if previous_home is None:os.environ.pop('CODEX_HOME',None)
        else:os.environ['CODEX_HOME']=previous_home


if __name__=='__main__':
    result=check();print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
