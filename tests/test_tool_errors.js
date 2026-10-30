const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {PolicyHost}=require('../scripts/codex_policy_host');

async function failedReply(stdout) {
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'rf-tool-error-'));
  try {
    const python=path.join(root,'python');
    fs.writeFileSync(python,'#!/usr/bin/env node\nprocess.stdout.write('+JSON.stringify(stdout)+');process.stderr.write("sk-test-private-token");process.exit(1);\n',{mode:0o755});
    const events=[],host={actors:new Map([['thread','A']]),run:root,cwd:root,python,event:x=>events.push(x)};
    const reply=await PolicyHost.prototype.toolRequest.call(host,'item/tool/call',{
      tool:'robot',threadId:'thread',callId:'native-1',arguments:{op:'move',call_id:'original-id',data:{}}});
    assert.equal(reply.success,false);
    assert.doesNotMatch(JSON.stringify({reply,events}),/sk-test-private-token/);
    return {value:JSON.parse(reply.contentItems[0].text),events};
  } finally {fs.rmSync(root,{recursive:true,force:true});}
}
function conflict(id='original-id') {return {tool_error_schema:1,call_id:id,error_code:'CALL_ID_CONFLICT',request_status:'rejected',original_call_status:'unknown'};}
test('host preserves a validated conflict instead of classifying it as transport failure',async()=>{
  const {value,events}=await failedReply(JSON.stringify({...conflict(),private_detail:'sk-other-private'}));
  assert.equal(value.error_code,'CALL_ID_CONFLICT');assert.equal(value.request_status,'rejected');
  assert.equal(value.original_call_status,'unknown');assert.equal(value.call_id,'original-id');
  assert.doesNotMatch(JSON.stringify(value),/private_detail|sk-other-private/);
  assert.equal(events.at(-1).durable_call_id,'original-id');
});
test('malformed, unrecognized or mismatched error output stays uncertain',async()=>{
  for(const output of ['broken json',JSON.stringify(conflict('another-id')),JSON.stringify({...conflict(),request_status:'unknown'}),JSON.stringify({error_code:'OTHER'})]) {
    const {value}=await failedReply(output);
    assert.equal(value.error_code,'TRANSPORT_UNCERTAIN');assert.equal(value.request_status,'unknown');
    assert.equal(value.original_call_status,'unknown');assert.equal(value.call_id,'original-id');
  }
});
