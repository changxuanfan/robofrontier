"""Exercise installed Codex RPC, images, dynamic tools and forks against a LOCAL stub.

No external inference, API keys, robot world or benchmark scoring is involved.
"""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import subprocess
import threading
import time

from PIL import Image
from scripts.benchmark import launch_config
from robofrontier.run_setup import write_json
from robofrontier.runtime_cli import fork_provenance


def check(output):
    output=Path(output).absolute();output.mkdir(parents=True,exist_ok=True)
    requests=[]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_POST(self):
            body=json.loads(self.rfile.read(int(self.headers['Content-Length'])));requests.append(body)
            image_seen='input_image' in json.dumps(body.get('input'))
            item=(dict(id='fc_probe',type='function_call',call_id='call_probe',name='probe',arguments='{}',status='completed')
                  if not image_seen else dict(id='msg_probe',type='message',role='assistant',status='completed',
                    content=[dict(type='output_text',text='OFFLINE_PROTOCOL_OK',annotations=[])]))
            response=dict(id='resp_'+str(len(requests)),object='response',created_at=int(time.time()),
                status='completed',model=body.get('model'),output=[item],
                usage=dict(input_tokens=10,output_tokens=10,total_tokens=20,input_tokens_details={'cached_tokens':0},output_tokens_details={'reasoning_tokens':0}))
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
    p=dict(model='offline-protocol',provider='offline_protocol',effort='high',vision=True,
        context_window=272000,auto_compact_token_limit=245000,
        base_url=f'http://127.0.0.1:{server.server_port}',env_key='ROBOFRONTIER_OFFLINE_STUB_TOKEN')
    write_json(output/'launch.json',launch_config(p,output))
    pixels=io.BytesIO();Image.new('RGB',(32,32),'red').save(pixels,format='PNG')
    write_json(output/'image.json',{'url':'data:image/png;base64,'+base64.b64encode(pixels.getvalue()).decode()})
    script=r'''
const fs=require('fs'),{CodexRpc}=require(process.argv[1]+'/scripts/codex_rpc');
const out=process.argv[2],launch=JSON.parse(fs.readFileSync(out+'/launch.json'));
process.env.ROBOFRONTIER_OFFLINE_STUB_TOKEN='local-stub-no-credential';
(async()=>{let done,completed=new Promise(r=>done=r),calls=0;
const rpc=new CodexRpc(launch.args,{stderr:fs.openSync(out+'/server.log','w'),
onEvent:m=>{if(m.method==='turn/completed')done(m.params);},
onRequest:async(m,p)=>{if(m!=='item/tool/call')throw new Error(m);calls++;
return {success:true,contentItems:[{type:'inputText',text:'local tool result'},
{type:'inputImage',imageUrl:JSON.parse(fs.readFileSync(out+'/image.json')).url}]};}});
try {await rpc.initialize();const a=await rpc.call('thread/start',{cwd:out,model:launch.model,modelProvider:launch.provider,
approvalPolicy:'never',sandbox:'read-only',dynamicTools:[{type:'function',name:'probe',description:'Offline protocol test',inputSchema:{type:'object',properties:{}}}]});
await rpc.call('thread/name/set',{threadId:a.thread.id,name:'RoboFrontier offline CLI protocol test'});
await rpc.call('turn/start',{threadId:a.thread.id,input:[{type:'text',text:'Call probe, inspect its image, reply OFFLINE_PROTOCOL_OK.'}]});
const end=await Promise.race([completed,new Promise((_,reject)=>setTimeout(()=>reject(new Error('Offline turn timeout')),20000))]);
if(end.turn.status!=='completed')throw new Error(JSON.stringify(end.turn.error));
const forks=[];for(let i=0;i<2;i++){const b=await rpc.call('thread/fork',{threadId:a.thread.id,excludeTurns:true});forks.push({id:b.thread.id,parent:b.thread.forkedFromId});}
const read=await rpc.call('thread/read',{threadId:a.thread.id});
fs.writeFileSync(out+'/native.json',JSON.stringify({parent:a.thread.id,status:read.thread.status,tool_calls:calls,forks},null,2));
for(const f of forks)await rpc.call('thread/archive',{threadId:f.id});await rpc.call('thread/archive',{threadId:a.thread.id});
}finally{await rpc.close();}})().then(()=>process.exit(0)).catch(e=>{console.error(e.stack);process.exit(1)});
'''
    try:
        result=subprocess.run(['node','-e',script,str(Path(__file__).resolve().parents[1]),str(output)],
            capture_output=True,text=True,timeout=40)
        (output/'client.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(result.stderr[-3000:])
        native=json.loads((output/'native.json').read_text())
        proofs=[fork_provenance(native['parent'],f['id']) for f in native['forks']]
        report=dict(passed=native['tool_calls']==1 and len(requests)==2 and proofs[0]==proofs[1],
            external_model_calls=0,local_stub_requests=len(requests),native=native,lineage=proofs,
            image_returned_to_model='input_image' in json.dumps(requests[-1]),
            scope='Actual installed CLI, local Responses stub; not provider access or Desktop UI acceptance')
        write_json(output/'report.json',report);return report
    finally:server.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    result=check(parser.parse_args().output);print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
