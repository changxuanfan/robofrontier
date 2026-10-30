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

test('conflict guidance distinguishes query request ID from original action ID',async()=>{
  const {value}=await failedReply(JSON.stringify(conflict()));
  assert.match(value.note,/data\.call_id/);
  assert.match(value.note,/omit.*outer.*call_id/i);
});

test('omitted outer ID generates a distinct request ID and preserves action target',async()=>{
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'rf-tool-id-'));
  try {
    const python=path.join(root,'python');
    const source=`#!/usr/bin/env node
const a=process.argv.slice(2),id=a[a.indexOf('--call-id')+1],data=JSON.parse(a[a.indexOf('--data')+1]);
process.stdout.write(JSON.stringify({call_id:id,execution:{call_id:data.call_id,status:'completed'}}));`;
    fs.writeFileSync(python,source,{mode:0o755});
    const events=[],host={actors:new Map([['thread','Controller']]),run:root,cwd:root,python,event:x=>events.push(x)};
    const reply=await PolicyHost.prototype.toolRequest.call(host,'item/tool/call',{
      tool:'robot',threadId:'thread',callId:'native-query-1',arguments:{op:'job',data:{call_id:'action-A'}}});
    const value=JSON.parse(reply.contentItems[0].text);
    assert.equal(reply.success,true);
    assert.equal(value.call_id,'Controller-native-query-1');
    assert.equal(value.execution.call_id,'action-A');
    assert.notEqual(value.call_id,value.execution.call_id);
    assert.deepEqual(events.map(e=>e.event),['robot_tool_started','robot_tool_completed']);
    assert.equal(events[1].op,'job');
    assert.ok(events[1].duration_s>=0);
  } finally {fs.rmSync(root,{recursive:true,force:true});}
});
