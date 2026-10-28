// Fake native/session/runtime adapter only. No Codex or MuJoCo connection.
const assert = require('node:assert/strict');
const {runMetaDispatcher} = require('../scripts/meta_dispatcher.js');
const response = value => ({content:[{type:'text',text:JSON.stringify(value)}]});
async function scenario({busy=false,failSend=false,wrongThread=false}={}) {
  const calls=[];let sendCount=0;
  const command={wake_id:'w1',actor:'A',thread_id:'fake-A',prompt:'fixed notification',reasons:['pending']};
  const tools={
    exec_command:async ({cmd})=>{
      calls.push(cmd);
      let value;
      if(cmd.includes(' snapshot '))value={lifecycle:'running',paused:false,config:{poll_interval_s:0},agents:{A:{kind:'worker',thread_id:'fake-A'}}};
      else if(cmd.includes(' observe-sessions '))value={observed:1};
      else if(cmd.includes(' plan '))value={commands:[command]};
      else if(cmd.includes(' claim '))value={command:busy?null:command};
      else if(cmd.includes(' dispatch-result '))value={recorded:true};
      else throw Error('unexpected runtime operation');
      return {exit_code:0,output:JSON.stringify(value)};
    },
    mcp__codex_app__wait_threads:async()=>response({polls:[{cursor:'fake',thread:{status:{type:busy?'active':'idle'}},latestTurn:{id:'old'}}]}),
    mcp__codex_app__send_message_to_thread:async p=>{
      sendCount++;assert.equal(p.prompt,'fixed notification');
      if(failSend)throw Error('injected lost acknowledgement');
      return response({threadId:wrongThread?'other':'fake-A'});
    }
  };
  await runMetaDispatcher({runDir:'/fake run',python:'/fake/python',cwd:'/fake',tools,maxCycles:1});
  assert.equal(sendCount,busy?0:1);
  const reports=calls.filter(c=>c.includes(' dispatch-result '));
  if(!busy)assert(reports[0].includes(failSend||wrongThread?'uncertain':'sent'));
  assert(!calls.some(c=>c.includes('pipeline_client')||c.includes(' move ')||c.includes(' gripper ')));
}
async function startup(armed,ready) {
  let starts=0,sends=0;
  const tools={
    exec_command:async({cmd})=>{
      let value;
      if(cmd.includes(' snapshot '))value={lifecycle:'prepared',paused:false,config:{poll_interval_s:0},agents:{A:{kind:'worker',thread_id:'fake-A',ready,recovery_required:false}}};
      else if(cmd.includes(' observe-sessions '))value={observed:1};
      else if(cmd.includes(' request-start ')){starts++;value={started:true};}
      else throw Error('unexpected operation before START');
      return {exit_code:0,output:JSON.stringify(value)};
    },
    mcp__codex_app__wait_threads:async()=>response({polls:[{thread:{status:{type:'idle'}},latestTurn:{id:'t0'}}]}),
    mcp__codex_app__send_message_to_thread:async()=>{sends++;throw Error('must not notify before run starts')}
  };
  await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:1,startWhenReady:armed});
  assert.equal(starts,armed&&ready?1:0);assert.equal(sends,0);
}
async function interrupted(){let calls=0;const tools={exec_command:async()=>{calls++;return {exit_code:0,output:JSON.stringify({lifecycle:'interrupted',paused:false})}}};const r=await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:1});assert.equal(r.lifecycle,'interrupted');assert.equal(calls,1)}
async function delegation({failFork=false,failBind=false,abandoned=false,lostStartAck=false,returnedBeforeAck=true,label=false,titleFailure=false}={}) {
  let forks=0,starts=0,activations=0,reviews=0;const titles=[];
  const batch={id:'scope-call',source_thread_id:'parent',status:'creating',claimed_by:abandoned?'old-manager':null,
    children:[{actor:'D1',robots:['A'],goal:'agent selected A outcome',done_when:'own evidence'},
              {actor:'D2',robots:['B'],goal:'agent selected B outcome',done_when:'own evidence'}]};
  const tools={exec_command:async({cmd})=>{
    let value={recorded:true};
    if(cmd.includes(' snapshot '))value={lifecycle:'running',paused:false,config:{poll_interval_s:0,model:'gpt-6-luna',effort:'xhigh'},
      agents:{Controller:{kind:'controller',thread_id:'parent'}},delegations:batch.status==='creating'?[batch]:[]};
    else if(cmd.includes(' delegate-claim '))value={delegation:structuredClone(batch)};
    else if(cmd.includes(' delegate-bind ') && failBind)return {exit_code:1,output:'different native source cutoff'};
    else if(cmd.includes(' delegate-activate ')){activations++;batch.status='active';}
    else if(cmd.includes(' delegate-uncertain ')){reviews++;batch.status='needs_review';}
    else if(cmd.includes(' delegate-started ') && cmd.includes('"outcome":"uncertain"')) {
      if(!returnedBeforeAck){reviews++;batch.status='needs_review';}
      value={recorded:true,status:batch.status};
    }
    else if(cmd.includes(' plan '))value={commands:[]};
    else assert(cmd.includes('observe-sessions')||cmd.includes('delegate-bind')||cmd.includes('delegate-started'));
    assert(!cmd.includes(' move ')&&!cmd.includes(' gripper '));
    return {exit_code:0,output:JSON.stringify(value)};
  },mcp__codex_app__wait_threads:async()=>response({polls:[{thread:{status:{type:'active'}},latestTurn:{id:'t1'}}]}),
  mcp__codex_app__fork_thread:async p=>{
    forks++;assert.equal(p.threadId,'parent');assert.equal(p.environment.type,'same-directory');
    if(failFork)throw Error('creation acknowledgement lost');
    return response({status:'created',sourceThreadId:'parent',threadId:'child'+forks});
  },mcp__codex_app__send_message_to_thread:async p=>{
    starts++;assert.equal(p.model,'gpt-6-luna');assert.equal(p.thinking,'xhigh');
    assert(p.prompt.includes('agent selected'));assert(p.prompt.includes('return_task'));
    assert(!p.prompt.includes('old-manager'));
    if(lostStartAck&&starts===1)throw Error('lost initial input acknowledgement');
    return response({threadId:p.threadId});
  }};
  if(label)tools.mcp__codex_app__set_thread_title=async p=>{
    titles.push(p);if(titleFailure)throw Error('Optional title unavailable');
    return response({threadId:p.threadId,title:p.title});
  };
  await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:2});
  if(label){assert.equal(titles.length,2);assert(titles[0].title.startsWith('Fork [A]'));assert(titles[1].title.startsWith('Fork [B]'));}
  if(abandoned||failFork||failBind){assert.equal(reviews,1);assert.equal(starts,0);assert.equal(activations,0);assert.equal(forks,abandoned?0:1);}
  else if(lostStartAck&&!returnedBeforeAck){assert.equal(forks,2);assert.equal(starts,1);assert.equal(activations,1);assert.equal(reviews,1);}
  else {assert.equal(forks,2);assert.equal(starts,2);assert.equal(activations,1);assert.equal(reviews,0);}
}
async function statusScenario(mode) {
  let paused=false,pauses=0,reads=0;const polled=[],sent=[],invalidated=[];
  const parent={kind:'controller',thread_id:'parent'};
  const agents={Controller:parent};
  if(mode==='finished')agents.Done={kind:'delegate',thread_id:'done',delegation_state:'finished'};
  if(mode==='isolate')agents.Good={kind:'worker',thread_id:'good'};
  const commands=mode==='finished'||mode==='persistent'?[]:[
    {wake_id:'bad',actor:'Controller',thread_id:'parent',prompt:'fixed notification'},
    ...(mode==='isolate'?[{wake_id:'good',actor:'Good',thread_id:'good',prompt:'fixed notification'}]:[])];
  const tools={exec_command:async({cmd})=>{
    let value={recorded:true};
    if(cmd.includes(' snapshot '))value={lifecycle:'running',paused,config:{poll_interval_s:0},agents,delegations:[]};
    else if(cmd.includes(' session-unavailable '))invalidated.push(cmd);
    else if(cmd.includes(' pause ')){paused=true;pauses++;}
    else if(cmd.includes(' plan '))value={commands};
    else if(cmd.includes(' claim '))value={command:cmd.includes('"good"')?commands.find(c=>c.actor==='Good'):commands[0]};
    else assert(cmd.includes('observe-sessions')||cmd.includes('dispatch-result'));
    return {exit_code:0,output:JSON.stringify(value)};
  },mcp__codex_app__wait_threads:async p=>{
    const id=p.targets[0].threadId;polled.push(id);
    if(id==='parent'){
      reads++;
      if(mode==='persistent'||mode==='isolate'||mode==='transient'&&reads===1)return response({polls:[]});
    }
    return response({polls:[{thread:{id,status:{type:mode==='finished'?'active':'idle'}},latestTurn:{id:'turn'}}]});
  },mcp__codex_app__send_message_to_thread:async p=>{sent.push(p.threadId);return response({threadId:p.threadId});}};
  const result=await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:mode==='persistent'?5:1});
  if(mode==='transient'){assert.deepEqual(sent,['parent']);assert(reads>1);assert.equal(pauses,0);}
  if(mode==='isolate'){assert.deepEqual(sent,['good']);assert(invalidated.length>0);assert.equal(pauses,0);}
  if(mode==='persistent'){assert.equal(result.paused,true);assert.equal(pauses,1);assert.equal(sent.length,0);assert(reads>1&&reads<15);}
  if(mode==='finished'){assert.deepEqual(polled,['parent']);}
}
async function endedDelegateCallback({failed=false}={}) {
  let notified=false,sent=0,cycleObserved=false;
  const command={wake_id:'end-event',actor:'Controller',thread_id:'parent',prompt:'fixed callback notice'};
  const tools={exec_command:async({cmd})=>{
    let value={recorded:true};
    if(cmd.includes(' snapshot '))value={lifecycle:'running',paused:false,config:{poll_interval_s:0},delegations:[],
      agents:{Controller:{kind:'controller',thread_id:'parent'},Child:{kind:'delegate',thread_id:'child',delegation_state:'active'},
              Other:{kind:'delegate',thread_id:'other',delegation_state:'active'}}};
    else if(cmd.includes(' observe-sessions ')){
      if(cmd.includes('"actor":"Child"')){
        assert(cmd.includes(failed?'"status":"error"':'"status":"idle"'));
        if(failed)assert(cmd.includes('"turn_status":"failed"'));
        notified=true;
      }
      if(cmd.includes('"cycle":1'))cycleObserved=true;
    }else if(cmd.includes(' plan '))value={commands:notified?[command]:[]};
    else if(cmd.includes(' claim '))value={command};
    else assert(cmd.includes(' dispatch-result '));
    return {exit_code:0,output:JSON.stringify(value)};
  },mcp__codex_app__wait_threads:async({targets})=>{
    const id=targets[0].threadId;
    return response({polls:[{thread:{id,status:{type:id==='other'?'active':failed&&id==='child'?'systemError':'idle'}},latestTurn:{id:'turn-'+id,status:failed&&id==='child'?'failed':id==='other'?'inProgress':'completed'},cursor:'cursor-'+id}]});
  },mcp__codex_app__send_message_to_thread:async p=>{
    assert.equal(p.threadId,'parent');assert.equal(p.prompt,command.prompt);sent++;
    return response({threadId:p.threadId});
  }};
  await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:1});
  assert(notified&&cycleObserved);assert.equal(sent,1);
}
async function platformErrorDetails() {
  let failure='';const tools={exec_command:async({cmd})=>{
    let value={recorded:true};
    if(cmd.includes(' snapshot '))value={lifecycle:'running',paused:false,config:{poll_interval_s:0},delegations:[],agents:{Controller:{kind:'controller',thread_id:'parent'}}};
    else if(cmd.includes(' session-unavailable '))failure=cmd;
    else if(cmd.includes(' plan '))value={commands:[]};
    else throw Error('unexpected operation');
    return {exit_code:0,output:JSON.stringify(value)};
  },mcp__codex_app__wait_threads:async()=>({isError:true,content:[{type:'text',text:'Completed host turn cannot query. authorization: Bearer secret-value token=other-secret sk-private_key'}]})};
  await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,maxCycles:1});
  assert(failure.includes('Completed host turn cannot query'));
  assert(!failure.includes('secret-value')&&!failure.includes('other-secret')&&!failure.includes('sk-private_key'));
}
// The same loop must run on a non-Codex adapter: no Codex tool objects, no Codex envelope.
async function neutralAdapter() {
  let forked=0;const sent=[],observed=[];
  const batch={id:'b',source_thread_id:'parent',status:'creating',claimed_by:null,
    children:[{actor:'D1',robots:['A'],goal:'agent selected goal',done_when:'own evidence'}]};
  const tools={exec_command:async({cmd})=>{
    let value={recorded:true};
    if(cmd.includes(' snapshot '))value={lifecycle:'running',paused:false,config:{poll_interval_s:0,model:'m',effort:'high'},
      agents:{Controller:{kind:'controller',thread_id:'parent'}},delegations:batch.status==='creating'?[batch]:[]};
    else if(cmd.includes(' delegate-claim '))value={delegation:structuredClone(batch)};
    else if(cmd.includes(' delegate-activate '))batch.status='active';
    else if(cmd.includes(' delegate-started '))value={recorded:true,status:'active'};
    else if(cmd.includes(' observe-sessions '))observed.push(cmd);
    else if(cmd.includes(' plan '))value={commands:[]};
    return {exit_code:0,output:JSON.stringify(value)};
  }};
  const adapter={
    observe:async p=>({thread:{id:p.threadId,status:{type:'idle'}},latestTurn:{id:'t0',status:'completed'}}),
    fork:async p=>{forked++;assert.equal(p.threadId,'parent');return {threadId:'child-1',sourceThreadId:'parent',status:'created'};},
    send:async p=>{sent.push(p);return {threadId:p.threadId};}};
  await runMetaDispatcher({runDir:'/fake',python:'/fake/python',cwd:'/fake',tools,adapter,maxCycles:1,observationSource:'claude.host'});
  assert.equal(forked,1);assert.equal(sent.length,1);
  assert.equal(sent[0].threadId,'child-1');assert.equal(sent[0].model,'m');assert(!('hostId' in sent[0]));
  assert(observed.length>0&&observed.every(c=>c.includes('"source":"claude.host"')));
}
(async()=>{
 await neutralAdapter();
 await scenario();await scenario({busy:true});await scenario({failSend:true});await scenario({wrongThread:true});
 await interrupted();await startup(false,true);await startup(true,false);await startup(true,true);
 await delegation();await delegation({failFork:true});await delegation({failBind:true});await delegation({abandoned:true});
 await delegation({lostStartAck:true});await delegation({lostStartAck:true,returnedBeforeAck:false});
 await statusScenario('transient');await statusScenario('isolate');await statusScenario('persistent');await statusScenario('finished');
 await endedDelegateCallback();await endedDelegateCallback({failed:true});await platformErrorDetails();
 await delegation({label:true});await delegation({label:true,titleFailure:true});
 console.log('PASS: 24 dispatcher scenarios: neutral engine adapter, fork lifecycle, first-result callback, failed-turn evidence, best-effort robot titles, error detail redaction, pinned settings, bounded status retries, visible pause, no finished-role polling');
})().catch(e=>{console.error(e);process.exit(1)});
