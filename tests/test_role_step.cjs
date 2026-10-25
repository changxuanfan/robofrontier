// Deterministic mailbox delivery at a local/native tool decision boundary.
const assert=require('node:assert/strict');
const {roleStep}=require('../scripts/role_tools.js');
async function main(){
  assert.equal(typeof roleStep,'function','A local-analysis boundary currently has no role inbox delivery');
  const order=[],printed=[],original={output:'computed from permitted RGB'};
  let response={call_id:'inbox-1',messages:[{message_id:'m1',from:'Child',text:'milestone'}],system_events:[],remaining_messages:0,remaining_events:0};
  const tools={exec_command:async({cmd})=>{
    order.push(cmd.includes('pipeline_client')?'inbox':'local');
    if(cmd.includes('pipeline_client')){
      assert(cmd.includes("'inbox'"));assert(!cmd.includes("'observe'")&&!cmd.includes("'move'"));
      assert(!cmd.includes('--call-id'),'A new local boundary must not reuse a cached inbox request ID');
      return {exit_code:0,output:JSON.stringify(response)};
    }
    return original;
  }};
  const base={tools,python:'/python',connection:'/own/role.json',callId:'stale-request',text:x=>printed.push(x)};
  let invalidWork=0;
  await assert.rejects(()=>roleStep({...base,python:'/same',connection:'/same',
    operation:async()=>{invalidWork++;}}),/Python.*connection/);
  assert.equal(invalidWork,0,'Invalid role configuration must be rejected before local work starts');
  const value=await roleStep({...base,operation:async()=>{
    const result=await tools.exec_command({cmd:'local analysis'});order.push('local finished');return result;
  }});
  assert.equal(value,original);assert.deepEqual(order,['local','local finished','inbox']);
  assert.equal(printed[0].role_events.messages[0].message_id,'m1');
  printed.length=0;order.length=0;
  response={call_id:'inbox-2',messages:[],system_events:[{event:'delegation_returned',actor:'Child',result_ref:'/report'}]};
  await roleStep({...base,operation:async()=>42});assert.equal(printed[0].role_events.system_events[0].actor,'Child');
  printed.length=0;response={call_id:'inbox-3',messages:[],system_events:[],remaining_messages:0,remaining_events:0};
  await roleStep({...base,operation:async()=>42});assert.equal(printed.length,0,'Empty inbox must not flood model context');
  response={...response,run_paused:true};
  await roleStep({...base,operation:async()=>42});assert.equal(printed[0].role_events.run_paused,true);
  printed.length=0;response={call_id:'inbox-4',messages:[{message_id:'m2'}],system_events:[]};
  await assert.rejects(()=>roleStep({...base,operation:async()=>{throw Error('local calculation failed');}}),/local calculation failed/);
  assert.equal(printed[0].role_events.messages[0].message_id,'m2','Local errors must not hide new mail');
  let ran=0;printed.length=0;
  const failure={...base,tools:{exec_command:async()=>{throw Error('mail transport unavailable');}}};
  const preserved=await roleStep({...failure,operation:async()=>{ran++;return original;}});
  assert.equal(ran,1);assert.equal(preserved,original);assert(printed[0].role_event_delivery_error);
  await assert.rejects(()=>roleStep({...base,operation:null}),/operation/);
  assert.equal(ran,1);
  console.log('PASS: local role step delivers mail/events after selected operation, preserves output/errors, never replays work or moves robots');
}
main().catch(e=>{console.error(e);process.exitCode=1;});
