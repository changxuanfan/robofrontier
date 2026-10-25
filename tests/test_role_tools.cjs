const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const {robotTool} = require('../scripts/role_tools.js');

async function main() {
  const calls=[], texts=[], images=[];
  const output={call_id:'original',execution:{status:'completed'},observation:{images:{A_front:'/rgb/front.png',A_side:'/rgb/side.png'}}};
  const tools={
    exec_command:async args=>{calls.push(args);return {session_id:7,output:''};},
    write_stdin:async args=>{calls.push(args);return {exit_code:0,output:JSON.stringify(output)};},
    view_image:async args=>{calls.push(args);return {image_url:'data:image/png;base64,fake'};}
  };
  const base={tools,python:'/a path/python',connection:'/a path/role.json',op:'move',
              data:{target_m:[.4,0,.6]},callId:'original',text:x=>texts.push(x),image:x=>images.push(x)};
  let invalidSubmissions=0;
  const invalidTools={exec_command:async()=>{invalidSubmissions++;return {exit_code:0,output:'{}'};}};
  for (const args of [{python:undefined},{connection:undefined},{python:''},{connection:''},
                      {python:'/same',connection:'/same'}]) {
    await assert.rejects(()=>robotTool({...base,...args,tools:invalidTools,views:[]}),/Python.*connection/);
  }
  assert.equal(invalidSubmissions,0,'Malformed client arguments must fail before any process/action submission');
  const result=await robotTool({...base,views:['A_front']});
  const physicalArgs=spawnSync('/bin/sh',['-c',`set -- ${calls[0].cmd}; printf '%s\\0' "$@"`],{encoding:'utf8'}).stdout.split('\0');
  assert.deepEqual(JSON.parse(physicalArgs[physicalArgs.indexOf('--data')+1]).cameras,['A_front']);
  assert.equal(result.execution.status,'completed');assert.equal(images.length,1);
  assert.deepEqual(calls[2],{path:'/rgb/front.png'});
  assert.equal(calls.filter(c=>c.cmd).length,1); // No hidden observe or robot action.
  calls.length=0;texts.length=0;images.length=0;
  await robotTool({...base,views:['global']});
  assert.equal(texts[0].view_errors[0].camera,'global');assert.equal(images.length,0);
  assert.equal(calls.length,2); // Missing view cannot trigger another request.
  calls.length=0;texts.length=0;
  await robotTool({...base,views:[]});assert.equal(calls.length,2);
  assert.equal(texts[0].call_id,'original');
  await assert.rejects(()=>robotTool({...base,views:['A_front','A_front']}),/distinct/);
  const bad={...tools,exec_command:async()=>({exit_code:0,output:'{truncated'})};
  await assert.rejects(()=>robotTool({...base,tools:bad}),/pending/);
  const failure={call_id:'held',execution_status:'uncertain',error:'owner lost'};
  const failed={...tools,exec_command:async()=>({exit_code:1,output:JSON.stringify(failure)})};
  assert.deepEqual(await robotTool({...base,tools:failed,views:[]}),failure);
  const missingImage={...tools,view_image:async()=>{throw Error('file missing');}};
  const partial=await robotTool({...base,tools:missingImage,views:['A_front']});
  assert.equal(partial.execution.status,'completed');assert.equal(partial.view_errors.length,1);
  // A job result reuses its original timestamped image, without another observe.
  calls.length=0;texts.length=0;images.length=0;
  const terminal={call_id:'job-query',execution:{status:'completed'},original_result:{
    call_id:'original',observation:{captured_at:123.5,sim_time_s:4,images:{A_front:'/rgb/saved-front.png'}}}};
  const jobTools={...tools,exec_command:async args=>{calls.push(args);return {exit_code:0,output:JSON.stringify(terminal)};}};
  const jobResult=await robotTool({...base,tools:jobTools,op:'job',data:{call_id:'original'},views:['A_front']});
  assert.deepEqual(jobResult,terminal);assert.equal(images.length,1);
  assert.equal(calls.filter(c=>c.cmd).length,1);
  assert.deepEqual(calls[1],{path:'/rgb/saved-front.png'});
  assert.deepEqual(texts[1],{camera:'A_front',captured_at:123.5,sim_time_s:4});
  calls.length=0;texts.length=0;images.length=0;
  const running={call_id:'job-query',execution:{status:'running'},original_result:{execution_status:'running'}};
  const runningTools={...tools,exec_command:async args=>{calls.push(args);return {exit_code:0,output:JSON.stringify(running)};}};
  const noFrame=await robotTool({...base,tools:runningTools,op:'job',views:['A_front']});
  assert.equal(noFrame.execution.status,'running');assert.equal(noFrame.view_errors.length,1);
  assert.equal(images.length,0);assert.equal(calls.length,1);
  // Exercise shell tokenization: quotes and substitutions must remain data.
  const payload={text:"O'Brien $HOME $(printf injected) `printf injected`\nsecond line"};
  let command;
  const quoting={exec_command:async({cmd})=>{command=cmd;return {exit_code:0,output:'{}'};}};
  await robotTool({...base,tools:quoting,op:'send',data:payload,views:[]});
  const split=spawnSync('/bin/sh',['-c',`set -- ${command}; printf '%s\\0' "$@"`],{encoding:'utf8'});
  assert.equal(split.status,0);
  const argv=split.stdout.split('\0').slice(0,-1);
  assert.equal(argv[0],'/a path/python');
  assert.equal(argv[argv.indexOf('--connection')+1],'/a path/role.json');
  assert.deepEqual(JSON.parse(argv[argv.indexOf('--data')+1]),payload);
  let batchCommand;
  const batchTools={exec_command:async({cmd})=>{batchCommand=cmd;return {exit_code:0,output:'{}'};}};
  await robotTool({...base,tools:batchTools,op:'action_batch',data:{actions:[],wait:false},views:['global']});
  const batchArgv=spawnSync('/bin/sh',['-c',`set -- ${batchCommand}; printf '%s\\0' "$@"`],{encoding:'utf8'}).stdout.split('\0');
  assert.deepEqual(JSON.parse(batchArgv[batchArgv.indexOf('--data')+1]).cameras,['global']);
  // Compact printing must preserve the complete returned data, outcomes, mail,
  // timestamps and unexpected fields. Full printing needs no new service call.
  const observed={call_id:'feedback',execution:{status:'failed',error:'tracking'},
    messages:[{id:'m7',text:'changed plan'}],system_events:[{id:'e3'}],
    observation:{captured_at:456,sim_time_s:12,images:{},robot:{A:{
      tcp_m:[.4,0,.42],status:'holding',gripper_width_m:.08,
      joint_rad:[.1,.2],joint_velocity_rad_s:[-.3,.2],unexpected:'keep'}}}};
  const observedCopy=JSON.stringify(observed);
  const feedbackTools={exec_command:async()=>({exit_code:0,output:observedCopy})};
  texts.length=0;
  const raw=await robotTool({...base,tools:feedbackTools,views:[]});
  assert.deepEqual(raw,observed);
  assert.equal(JSON.stringify(observed),observedCopy);
  const printed=texts[0];
  assert.equal(printed.observation.robot.A.joint_rad,undefined);
  assert.equal(printed.observation.robot.A.joint_velocity_rad_s,undefined);
  assert.equal(printed.observation.robot.A.max_abs_joint_velocity_rad_s,.3);
  assert.equal(printed.observation.robot.A.unexpected,'keep');
  assert.deepEqual(printed.messages,observed.messages);
  assert.deepEqual(printed.system_events,observed.system_events);
  assert.deepEqual(printed.execution,observed.execution);
  assert.equal(printed.observation.captured_at,456);
  assert.equal(printed.observation.robot.A.gripper_width_m,.08);
  texts.length=0;
  await robotTool({...base,tools:feedbackTools,views:[],detail:'full'});
  assert.deepEqual(texts[0],observed);
  let submitted=0;
  await assert.rejects(()=>robotTool({...base,tools:{exec_command:async()=>{submitted++;}},detail:'guess'}),/detail/);
  assert.equal(submitted,0);
  // Saved asynchronous results receive the same display policy without losing raw data.
  texts.length=0;
  const nested={call_id:'query',original_result:observed};
  const nestedTools={exec_command:async()=>({exit_code:0,output:JSON.stringify(nested)})};
  const nestedRaw=await robotTool({...base,tools:nestedTools,op:'job',views:[]});
  assert.deepEqual(nestedRaw,nested);
  assert.equal(texts[0].original_result.observation.robot.A.joint_rad,undefined);
  console.log('PASS: result/image bridge (15 scenarios including batch-view selection), early client validation, compact/full preserves raw results and events, no implicit observation, motion or session management');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
