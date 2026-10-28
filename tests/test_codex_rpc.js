const test=require('node:test'),assert=require('node:assert/strict');
const {capacityFailure}=require('../scripts/codex_rpc');
const {PolicyHost,robotTool}=require('../scripts/codex_policy_host');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
test('capacity retry distinguishes quota/auth/permanent errors',()=>{
  for(const message of ['model_capacity_limit','429 Too Many Requests','server_overloaded'])assert.equal(capacityFailure({message}),true);
  for(const message of ['insufficient_quota 429','invalid_api_key','Bad Request 400'])assert.equal(capacityFailure({message}),false);
});
test('an active turn cannot be steered by the scheduler',async()=>{
  let called=false;
  await assert.rejects(PolicyHost.prototype.send.call({observe:async()=>({thread:{status:{type:'active'}}}),rpc:{call:()=>{called=true;}}},'t','wake'),/active/);
  assert.equal(called,false);
});
test('robot transport cannot select another role connection',()=>{
  assert.equal(robotTool.name,'robot');
  assert.equal(robotTool.inputSchema.properties.connection,undefined);
});
test('capacity recovery resumes a started task without a READY barrier',async()=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'capacity-retry-'));
  try {
    const fakePython=path.join(dir,'python');
    fs.writeFileSync(fakePython,'#!/bin/sh\nprintf "false\\n"\n',{mode:0o755});
    const prompts=[];
    const host={stopped:false,began:Date.now(),run:dir,cwd:dir,python:fakePython,
      actors:new Map([['thread','Controller']]),failures:new Map([['thread:turn',Date.now()-11000]]),retries:new Set(),
      admin:async()=>({paused:false,lifecycle:'running'}),
      observe:async()=>({thread:{status:{type:'systemError'}},latestTurn:{id:'turn',status:'failed',error:{message:'429 Too Many Requests'}}}),
      send:async(id,prompt)=>prompts.push(prompt),event:()=>{}};
    await PolicyHost.prototype.retryCapacity.call(host);
    assert.equal(prompts.length,1);
    assert.match(prompts[0],/实验已经开始/);
    assert.doesNotMatch(prompts[0],/READY\/保存等待协议/);
    await PolicyHost.prototype.retryCapacity.call(host);
    assert.equal(prompts.length,1);
  } finally {fs.rmSync(dir,{recursive:true,force:true});}
});
