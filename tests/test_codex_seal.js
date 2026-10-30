const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {PolicyHost}=require('../scripts/codex_policy_host');

test('seal failure propagates after cleanup and never reports host success',async()=>{
  const dir=fs.mkdtempSync(path.join(os.tmpdir(),'rf-seal-'));
  try {
    fs.mkdirSync(path.join(dir,'private/inputs'),{recursive:true});
    fs.writeFileSync(path.join(dir,'private/inputs/manifest.json'),JSON.stringify({model_requested:'m',effort_requested:'high'}));
    let closed=false;const events=[];
    const host={run:dir,launch:{model:'m',effort:'high'},actors:new Map(),
      startServer:async()=>{throw new Error('startup failure');},
      admin:async()=>{throw new Error('unresolved physical action');},
      event:e=>events.push(e),rpc:{close:async()=>{closed=true;}}};
    await assert.rejects(PolicyHost.prototype.runTask.call(host),/seal incomplete/);
    assert.equal(closed,true);assert.equal(events[0].event,'seal_error');
  } finally {fs.rmSync(dir,{recursive:true,force:true});}
});
