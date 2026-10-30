const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {PolicyHost}=require('../scripts/codex_policy_host');

test('host lifetime never consumes task time, while explicit STOP still works',()=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'rf-clock-'));
  try {
    const host={run:dir,stopped:false,began:0};
    assert.equal(PolicyHost.prototype.stopRequested.call(host),false);
    fs.writeFileSync(path.join(dir,'STOP'),'');
    assert.equal(PolicyHost.prototype.stopRequested.call(host),true);
    fs.unlinkSync(path.join(dir,'STOP'));host.stopped=true;
    assert.equal(PolicyHost.prototype.stopRequested.call(host),true);
  } finally {fs.rmSync(dir,{recursive:true,force:true});}
});

test('capacity recovery checks authoritative run state even after 24 hours',async()=>{
  let snapshots=0;
  const host={stopped:false,began:0,actors:new Map(),admin:async()=>{snapshots++;return {lifecycle:'running',paused:false};}};
  await PolicyHost.prototype.retryCapacity.call(host);
  assert.equal(snapshots,1);
});
