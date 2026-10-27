// Explicitly invoked paid connectivity test, separate from scored robot trials.
const fs=require('node:fs'),path=require('node:path');
const {CodexRpc}=require('./codex_rpc');
async function preflight(folder) {
  const launch=JSON.parse(fs.readFileSync(path.join(folder,'launch.json')));
  let resolveTurn,resolveCompact;
  const finished=new Promise(r=>resolveTurn=r),compacted=new Promise(r=>resolveCompact=r);
  let tool=false,compact=false,id;
  const rpc=new CodexRpc(launch.args,{stderr:fs.openSync(path.join(folder,'server.log'),'a'),
    onEvent:m=>{
      if(m.method==='turn/completed')resolveTurn(m.params.turn);
      if(m.method==='thread/compacted' || m.method==='item/completed' && m.params?.item?.type==='contextCompaction') {compact=true;resolveCompact(true);}
      if(m.method==='error' && !m.params?.willRetry)resolveCompact(false);
    },onRequest:async(m,p)=>{
      if(m!=='item/tool/call' || p.tool!=='protocol_check')throw new Error('Unsupported preflight request');
      const a=typeof p.arguments==='string'?JSON.parse(p.arguments):p.arguments;
      tool=a.left==='red'&&a.right==='blue';
      return {success:tool,contentItems:[{type:'inputText',text:tool?'VISION_TOOL_OK':'Incorrect image colors'}]};
    }});
  let report={passed:false,model:launch.model,provider:launch.provider,effort:launch.effort,
    actual_provider_model_verified:false,
    context_window:launch.context_window,auto_compact_token_limit:launch.auto_compact_token_limit,
    context_stress_tested:false,scope:'Small real API image/function/compaction test; no robot trial'};
  const deadline=(promise,ms)=>new Promise((resolve,reject)=>{
    const t=setTimeout(()=>reject(new Error('Preflight deadline exceeded')),ms);
    promise.then(v=>{clearTimeout(t);resolve(v);},e=>{clearTimeout(t);reject(e);});
  });
  try {
    await rpc.initialize();
    const a=await rpc.call('thread/start',{cwd:folder,model:launch.model,modelProvider:launch.provider,
      approvalPolicy:'never',sandbox:'read-only',dynamicTools:[{type:'function',name:'protocol_check',
        description:'Submit the two dominant image colors from left to right.',inputSchema:{type:'object',
        properties:{left:{type:'string'},right:{type:'string'}},required:['left','right']}}]});id=a.thread.id;
    await rpc.call('thread/name/set',{threadId:id,name:'API preflight · '+launch.model});
    await rpc.call('turn/start',{threadId:id,model:launch.model,effort:launch.effort,input:[
      {type:'text',text:'Inspect the attached image. Call protocol_check with the dominant color in the left half and the right half, using lowercase English color names. Then reply with the tool result. Do not use shell or other tools.'},
      {type:'localImage',path:path.join(folder,'colors.png')} ]});
    const turn=await deadline(finished,180000);
    if(turn.status!=='completed')throw new Error(JSON.stringify(turn.error));
    await rpc.call('thread/compact/start',{threadId:id});
    await deadline(compacted,180000);
    report={...report,passed:tool&&compact,vision_and_function:tool,compaction:compact,thread_id:id};
  } catch(e) {report.error=e.message;}
  finally {
    if(id)try{await rpc.call('thread/archive',{threadId:id});}catch{}
    await rpc.close();
    fs.writeFileSync(path.join(folder,'report.json'),JSON.stringify(report,null,2)+'\n');
  }
  return report;
}
if(require.main===module)preflight(path.resolve(process.argv[2])).then(r=>{console.log(JSON.stringify(r));process.exitCode=r.passed?0:1;});
module.exports={preflight};
