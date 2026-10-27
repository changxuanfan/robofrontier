const test=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {preflight}=require('../scripts/codex_preflight');
function fakeRpc({badParent=false,badChild=false}={}) {
  return class {
    constructor(args,handlers){this.handlers=handlers;}
    async initialize(){}
    async close(){}
    async call(method,p={}) {
      if(method==='thread/start')return {thread:{id:'parent'}};
      if(method==='thread/fork')return {thread:{id:'child',forkedFromId:badParent?'other':'parent'}};
      if(method==='thread/compact/start') {
        this.handlers.onEvent({method:'thread/compacted',params:{threadId:'parent'}});return {};
      }
      if(method==='turn/start') {
        const child=p.threadId==='child';
        await this.handlers.onRequest('item/tool/call',{tool:'protocol_check',threadId:p.threadId,
          arguments:JSON.stringify({left:child&&badChild?'green':'red',right:'blue'})});
        this.handlers.onEvent({method:'turn/completed',params:{threadId:p.threadId,turn:{status:'completed'}}});
        return {};
      }
      return {};
    }
  };
}
async function run(opts) {
  const folder=fs.mkdtempSync(path.join(os.tmpdir(),'rf-preflight-'));
  try {
    fs.writeFileSync(path.join(folder,'launch.json'),JSON.stringify({args:[],model:'sample',provider:'sample',effort:'high',context_window:272000,auto_compact_token_limit:245000}));
    return await preflight(folder,fakeRpc(opts));
  } finally {fs.rmSync(folder,{recursive:true,force:true});}
}
test('preflight verifies fork parent and inherited tool result',async()=>{
  const result=await run();assert.equal(result.passed,true);assert.equal(result.fork_history_and_tool,true);
  assert.equal(result.automatic_compaction_tested,false);assert.equal(result.context_stress_tested,false);
});
test('preflight fails closed on incorrect fork parent',async()=>{
  const result=await run({badParent:true});assert.equal(result.passed,false);assert.match(result.error,/parent mismatch/);
});
test('preflight fails closed when child cannot recover image colors',async()=>{
  const result=await run({badChild:true});assert.equal(result.passed,false);assert.match(result.error,/inherit/);
});
