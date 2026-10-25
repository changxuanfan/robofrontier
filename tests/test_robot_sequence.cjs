const assert=require('node:assert/strict');
const bridge=require('../scripts/role_tools.js');

async function main(){
  assert.equal(typeof bridge.robotSequence,'function','Missing finite action sequence adapter');
  const steps=[
    {op:'move',data:{target_m:[.4,0,.55]},callId:'move-1'},
    {op:'gripper',data:{state:'close'},callId:'grip-2'},
    {op:'move',data:{target_m:[.4,0,.65]},callId:'move-3'}
  ];
  const base={python:'/python',connection:'/connection',text:()=>{},image:()=>{}};
  const fixture=(responses)=>{
    const commands=[];let n=0;
    return {commands,tools:{exec_command:async({cmd})=>{
      commands.push(cmd);return {exit_code:0,output:JSON.stringify(responses[n++])};
    },view_image:async()=>({image_url:'data:image/png;base64,fake'})}};
  };
  const ok=()=>({execution:{status:'completed'},messages:[],system_events:[]});
  let f=fixture([ok(),ok(),ok()]);
  let r=await bridge.robotSequence({...base,...f,steps});
  assert.equal(f.commands.length,3);assert.equal(r.sequence.completed_steps,3);
  assert.equal(r.sequence.stop_reason,'complete');assert.equal(r.sequence.next_step,3);
  assert(f.commands[0].includes('move-1'));assert(f.commands[1].includes('grip-2'));
  for(const [reason,changed] of [
    ['events',{messages:[{message_id:'m1'}]}],
    ['events',{system_events:[{event:'new'}]}],
    ['failed',{execution:{status:'failed'}}],
    ['execution_pending',{execution_status:'running',execution:{status:'running'}}],
    ['execution_pending',{execution_status:'uncertain'}],
    ['paused',{run_paused:true}],
    ['ended',{run_ended:true}],
    ['error',{error:'executor unavailable'}],
    ['unconfirmed_result',{execution:undefined}]
  ]){
    f=fixture([ok(),{...ok(),...changed},ok()]);
    r=await bridge.robotSequence({...base,...f,steps});
    assert.equal(f.commands.length,2,reason);assert.equal(r.sequence.stop_reason,reason);
    assert.equal(r.sequence.next_step,2);assert.equal(r.results.length,2);
  }
  // Loading an intermediate image is a model decision boundary, not interpretation by JS.
  f=fixture([{...ok(),observation:{images:{A_front:'/frame'}}},ok()]);
  r=await bridge.robotSequence({...base,...f,steps:[{...steps[0],views:['A_front']},steps[1]]});
  assert.equal(f.commands.length,1);assert.equal(r.sequence.stop_reason,'observation_boundary');
  // The Agent can keep an already-decided finite segment together, retaining mail.
  f=fixture([{...ok(),messages:[{message_id:'m1',text:'progress'}]},ok(),ok()]);
  r=await bridge.robotSequence({...base,...f,steps,messageBoundary:'sequence'});
  assert.equal(f.commands.length,3);assert.equal(r.results[0].messages[0].message_id,'m1');
  assert.equal(r.sequence.stop_reason,'complete');
  for(const changed of [{run_paused:true},{system_events:[{event:'new'}]},{execution:{status:'failed'}}]){
    f=fixture([{...ok(),...changed},ok()]);
    r=await bridge.robotSequence({...base,...f,steps,messageBoundary:'sequence'});
    assert.equal(f.commands.length,1);
  }
  f=fixture([ok()]);
  await assert.rejects(()=>bridge.robotSequence({...base,...f,steps,messageBoundary:'ignore'}),/messageBoundary/);
  assert.equal(f.commands.length,0);
  // Invalid or duplicate identifiers are rejected before any physical call.
  f=fixture([ok()]);
  await assert.rejects(()=>bridge.robotSequence({...base,...f,steps:[steps[0],steps[0]]}),/distinct/);
  await assert.rejects(()=>bridge.robotSequence({...base,...f,steps:[{op:'send',callId:'x'}]}),/move.*gripper/);
  assert.equal(f.commands.length,0);
  console.log('PASS: finite sequences preserve IDs and stop for events, execution, pause/end and model observation boundaries');
}
main().catch(e=>{console.error(e);process.exitCode=1});
