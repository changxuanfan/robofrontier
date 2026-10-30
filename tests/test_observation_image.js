const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {PolicyHost,observationMime}=require('../scripts/codex_policy_host');
const png=Buffer.from([137,80,78,71,13,10,26,10,1]);
const jpg=Buffer.from([255,216,255,224,1]);
test('observation MIME uses image bytes and supports old PNG and new JPEG',()=>{
  assert.equal(observationMime(png),'image/png');assert.equal(observationMime(jpg),'image/jpeg');
  assert.throws(()=>observationMime(Buffer.from('invalid')),/not PNG or JPEG/);
});
test('Codex bridge keeps original action result and emits the proper image MIME',async()=>{
 const root=fs.mkdtempSync(path.join(os.tmpdir(),'rf-observation-'));
 try {
  const dir=path.join(root,'public','observations');fs.mkdirSync(dir,{recursive:true});
  const file=path.join(dir,'mismatched.png');fs.writeFileSync(file,jpg);
  const result={call_id:'call-1',execution:{status:'completed'},observation:{images:{top:file}}};
  const fake=path.join(root,'python');fs.writeFileSync(fake,'#!/usr/bin/env node\nprocess.stdout.write('+JSON.stringify(JSON.stringify(result))+');\n',{mode:0o755});
  const host={actors:new Map([['thread','Controller']]),run:root,cwd:root,python:fake,event:()=>{}};
  const req={threadId:'thread',tool:'robot',callId:'id',arguments:{op:'move',call_id:'call-1',data:{},views:['top']}};
  let reply=await PolicyHost.prototype.toolRequest.call(host,'item/tool/call',req);
  assert.equal(reply.success,true);assert.match(reply.contentItems[1].imageUrl,/^data:image\/jpeg;base64,/);
  assert.equal(JSON.parse(reply.contentItems[0].text).execution.status,'completed');
  fs.writeFileSync(file,png);reply=await PolicyHost.prototype.toolRequest.call(host,'item/tool/call',req);
  assert.match(reply.contentItems[1].imageUrl,/^data:image\/png;base64,/);
  fs.writeFileSync(file,'bad');reply=await PolicyHost.prototype.toolRequest.call(host,'item/tool/call',req);
  assert.equal(JSON.parse(reply.contentItems[0].text).call_id,'call-1');
  assert.match(reply.contentItems[1].text,/could not be loaded/);
 } finally {fs.rmSync(root,{recursive:true,force:true});}
});
