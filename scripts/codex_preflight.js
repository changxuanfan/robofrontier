// Explicitly invoked paid connectivity test, separate from scored robot trials.
const fs=require('node:fs'),path=require('node:path');
const {CodexRpc}=require('./codex_rpc');
async function preflight(folder, Rpc=CodexRpc, {smokeOnly=false}={}) {
  const launch=JSON.parse(fs.readFileSync(path.join(folder,'launch.json')));
  const waits=new Map();
  let resolveCompact;
  const compacted=new Promise(r=>resolveCompact=r);
  const tools=new Map(); let compact=false,id,childId;
  const serverLog=fs.openSync(path.join(folder,'server.log'),'a');
  const rpc=new Rpc(launch.args,{stderr:serverLog,
    onEvent:m=>{
      if(m.method==='turn/completed') waits.get(m.params.threadId)?.(m.params.turn);
      if(m.method==='thread/compacted' || m.method==='item/completed' && m.params?.item?.type==='contextCompaction') {compact=true;resolveCompact(true);}
      if(m.method==='error' && !m.params?.willRetry)resolveCompact(false);
    },onRequest:async(m,p)=>{
      if(m!=='item/tool/call' || p.tool!=='protocol_check')throw new Error('Unsupported preflight request');
      const a=typeof p.arguments==='string'?JSON.parse(p.arguments):p.arguments;
      const valid=a.left==='red'&&a.right==='blue'; tools.set(p.threadId,valid);
      return {success:valid,contentItems:[{type:'inputText',text:valid?'VISION_TOOL_OK':'Incorrect image colors'}]};
    }});
  fs.closeSync(serverLog);
  let report={passed:false,model:launch.model,provider:launch.provider,effort:launch.effort,
    actual_provider_model_verified:false,context_window:launch.context_window,
    auto_compact_token_limit:launch.auto_compact_token_limit,context_stress_tested:false,
    automatic_compaction_tested:false,smoke_only:smokeOnly,
    scope:smokeOnly?'Small real API image/function round trip; no robot trial or run qualification':
      'Small real API image/function/manual-compaction/fork test; no robot trial'};
  const deadline=(promise,ms)=>new Promise((resolve,reject)=>{
    const t=setTimeout(()=>reject(new Error('Preflight deadline exceeded')),ms);
    promise.then(v=>{clearTimeout(t);resolve(v);},e=>{clearTimeout(t);reject(e);});
  });
  const nextTurn=thread=>new Promise(resolve=>waits.set(thread,resolve));
  try {
    await rpc.initialize();
    const a=await rpc.call('thread/start',{cwd:folder,model:launch.model,modelProvider:launch.provider,
      approvalPolicy:'never',sandbox:'read-only',dynamicTools:[{type:'function',name:'protocol_check',
        description:'Submit the two dominant image colors from left to right.',inputSchema:{type:'object',
        properties:{left:{type:'string'},right:{type:'string'}},required:['left','right']}}]});id=a.thread.id;
    await rpc.call('thread/name/set',{threadId:id,name:'API preflight · '+launch.model});
    const finished=nextTurn(id);
    await rpc.call('turn/start',{threadId:id,model:launch.model,effort:launch.effort,input:[
      {type:'text',text:'Inspect the attached image. Call protocol_check with the dominant color in the left half and the right half, using lowercase English color names. Then reply with the tool result. Do not use shell or other tools.'},
      {type:'localImage',path:path.join(folder,'colors.png')} ]});
    const turn=await deadline(finished,180000);
    if(turn.status!=='completed' || !tools.get(id))throw new Error('Parent image/tool turn failed: '+JSON.stringify(turn.error));
    if(smokeOnly) {
      report={...report,passed:true,vision_and_function:true,thread_id:id};
      return report;
    }
    await rpc.call('thread/compact/start',{threadId:id});
    if(!await deadline(compacted,180000))throw new Error('Compaction failed');
    const fork=await rpc.call('thread/fork',{threadId:id,cwd:folder,model:launch.model,
      modelProvider:launch.provider,excludeTurns:true,ephemeral:false});
    childId=fork.thread.id;
    if(fork.thread.forkedFromId!==id)throw new Error('Fork parent mismatch');
    const childFinished=nextTurn(childId);
    await rpc.call('turn/start',{threadId:childId,model:launch.model,effort:launch.effort,input:[
      {type:'text',text:'Using the image and tool result from our prior conversation, call protocol_check again with the same left and right colors. Do not use shell or other tools.'}]});
    const child=await deadline(childFinished,180000);
    if(child.status!=='completed' || !tools.get(childId))throw new Error('Fork did not inherit usable image history and tool');
    report={...report,passed:true,vision_and_function:true,compaction:compact,
      fork_parent_verified:true,fork_history_and_tool:true,thread_id:id,child_thread_id:childId};
  } catch(e) {report.error=e.message;}
  finally {
    for(const thread of [childId,id])if(thread)try{await rpc.call('thread/archive',{threadId:thread});}catch{}
    await rpc.close();
    fs.writeFileSync(path.join(folder,'report.json'),JSON.stringify(report,null,2)+'\n');
  }
  return report;
}
if(require.main===module)preflight(path.resolve(process.argv[2]),CodexRpc,{smokeOnly:process.argv.includes('--smoke-only')})
  .then(r=>{console.log(JSON.stringify(r));process.exitCode=r.passed?0:1;});
module.exports={preflight};
