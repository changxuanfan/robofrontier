// One independent CLI host per task. Reuses the existing deterministic Meta.
const fs = require('node:fs');
const path = require('node:path');
const {execFile,exec:execShell,spawn} = require('node:child_process');
const {promisify} = require('node:util');
const execFileAsync = promisify(execFile), execAsync = promisify(execShell);
const {CodexRpc,capacityFailure} = require('./codex_rpc');
const {runMetaDispatcher} = require('./meta_dispatcher');
const sleep = ms => new Promise(resolve=>setTimeout(resolve,ms));
const read = file=>JSON.parse(fs.readFileSync(file,'utf8'));
function toolFailure(callId, stdout) {
  let detail;
  try { if (typeof stdout==='string' && stdout.length<=16384) detail=JSON.parse(stdout); } catch {}
  const conflict=detail?.tool_error_schema===1 && detail.call_id===callId &&
    detail.error_code==='CALL_ID_CONFLICT' && detail.request_status==='rejected' &&
    detail.original_call_status==='unknown';
  // Rebuild an allowlisted response; never pass through raw stderr, command
  // lines, server strings or another request's details.
  return {tool_error_schema:1,call_id:callId,
    error_code:conflict?'CALL_ID_CONFLICT':'TRANSPORT_UNCERTAIN',
    error:conflict?'This call ID is already associated with a different request.':
      'The tool result could not be confirmed; execution must not be inferred.',
    request_status:conflict?'rejected':'unknown',original_call_status:'unknown',
    note:conflict?
      'The conflicting request was rejected. Inspect pending and, for your physical actions, job with the original ID. Use a new ID only for an intentionally new request after reconciling existing work; never replay uncertain motion.':
      'Inspect pending and, for your physical actions, job with the original ID. Do not create a new ID to replay an uncertain operation.'};
}
function observationMime(data) {
  if (data.subarray(0,8).equals(Buffer.from([137,80,78,71,13,10,26,10]))) return 'image/png';
  if (data.length>=3 && data[0]===255 && data[1]===216 && data[2]===255) return 'image/jpeg';
  throw new Error('Observation is not PNG or JPEG');
}
const robotTool = {type:'function',name:'robot',description:
  'Call your assigned RoboFrontier role API. Same operations/data as the frozen skill. Selected views return actual RGB images. Never manage sessions or use another role connection.',
  inputSchema:{type:'object',properties:{op:{type:'string'},data:{type:'object'},
    call_id:{type:'string'},views:{type:'array',items:{type:'string'}}},required:['op','data']}};

class PolicyHost {
  constructor(run, cwd, python) {
    this.run=path.resolve(run); this.cwd=cwd; this.python=python;
    this.launch=read(path.join(this.run,'private/inputs/codex.json'));
    this.wallLimit=read(path.join(this.run,'private/inputs/task.json')).run_protocol.total_wall_limit_s;
    this.actors=new Map(); this.turns=new Map(); this.failures=new Map(); this.retries=new Set();
    this.stopped=false;
  }
  event(value) {
    fs.appendFileSync(path.join(this.run,'private/outputs/codex-host.jsonl'),JSON.stringify({at:Date.now()/1000,...value})+'\n');
  }
  async admin(op,data={}) {
    const {stdout}=await execFileAsync(this.python,['-m','robofrontier.runtime_cli',
      '--run-dir',this.run,op,'--data',JSON.stringify(data)],{cwd:this.cwd,maxBuffer:8*1024*1024});
    return JSON.parse(stdout);
  }
  async toolRequest(method,p) {
    if (method!=='item/tool/call' || p.tool!=='robot') throw new Error('Unsupported host tool');
    const actor=this.actors.get(p.threadId); if (!actor) throw new Error('Unbound policy role');
    const a=typeof p.arguments==='string'?JSON.parse(p.arguments):p.arguments;
    if (['move','gripper','action_batch'].includes(a.op) && a.data?.cameras===undefined)
      a.data={...(a.data||{}),cameras:a.views||[]};
    // Native call IDs are scoped by actor and stable across transport retries.
    const callId=a.call_id || (actor+'-'+p.callId).replace(/[^A-Za-z0-9_-]/g,'_').slice(0,128);
    const args=['-m','robofrontier.pipeline_client','--quiet','--connection',
      path.join(this.run,'private/state/connections',actor+'.json'),'--call-id',callId,a.op,
      '--data',JSON.stringify(a.data||{})];
    try {
      const {stdout}=await execFileAsync(this.python,args,{cwd:this.cwd,maxBuffer:8*1024*1024});
      const result=JSON.parse(stdout),contentItems=[{type:'inputText',text:JSON.stringify(result)}];
      const observation=result.observation || result.original_result?.observation;
      for (const view of a.views||[]) {
        const file=observation?.images?.[view];
        if (!file) {contentItems.push({type:'inputText',text:'Selected view unavailable: '+view});continue;}
        try {
          const real=fs.realpathSync(file),publicRoot=fs.realpathSync(path.join(this.run,'public'))+path.sep;
          if (!real.startsWith(publicRoot)) throw new Error('Observation outside this run public directory');
          const bytes=fs.readFileSync(real),mime=observationMime(bytes);
          contentItems.push({type:'inputImage',imageUrl:'data:'+mime+';base64,'+bytes.toString('base64')});
        } catch (error) {
          // The physical action already ran: retain the original result and call_id.
          contentItems.push({type:'inputText',text:'Selected view could not be loaded: '+view});
        }
      }
      return {success:true,contentItems};
    } catch(error) {
      const failure=toolFailure(callId,error.stdout);
      this.event({event:'robot_tool_failed',actor,thread_id:p.threadId,op:a.op,call_id:p.callId,
        durable_call_id:callId,error_code:failure.error_code,request_status:failure.request_status});
      return {success:false,contentItems:[{type:'inputText',text:JSON.stringify(failure)}]};
    }
  }
  async startServer() {
    const log=fs.openSync(path.join(this.run,'private/outputs/codex-server.log'),'a');
    this.rpc=new CodexRpc(this.launch.args,{stderr:log,onRequest:(m,p)=>this.toolRequest(m,p),onEvent:m=>{
      const p=m.params||{};
      if (m.method==='turn/completed') {
        this.turns.set(p.threadId,p.turn);
        if (p.turn.status==='failed' && capacityFailure(p.turn.error)) {
          const key=p.threadId+':'+p.turn.id;
          if (!this.failures.has(key)) this.failures.set(key,Date.now());
          this.event({event:'capacity_wait',thread_id:p.threadId,turn_id:p.turn.id,retry_after_s:10});
        }
      }
      if (['turn/started','turn/completed','thread/status/changed','thread/tokenUsage/updated','error'].includes(m.method))
        this.event({event:m.method,thread_id:p.threadId,turn_id:p.turn?.id,status:p.turn?.status,
          token_usage:p.tokenUsage,error:p.error});
    }});
    fs.closeSync(log); await this.rpc.initialize();
  }
  async observe(threadId) {
    const result=await this.rpc.call('thread/read',{threadId});
    const latest=await this.rpc.call('thread/turns/list',{threadId,limit:1,sortDirection:'desc',itemsView:'notLoaded'});
    const turn=latest.data?.[0] || this.turns.get(threadId);
    let type=result.thread.status.type;
    if (type==='idle' && turn?.status==='failed') type='systemError';
    return {thread:{...result.thread,status:{type}},latestTurn:turn};
  }
  async send(threadId,prompt) {
    const state=await this.observe(threadId);
    if (state.thread.status.type==='active') throw new Error('Refusing to steer an active policy turn');
    const result=await this.rpc.call('turn/start',{threadId,model:this.launch.model,effort:this.launch.effort,
      input:[{type:'text',text:prompt}],turnTrigger:'robofrontier'});
    this.turns.set(threadId,result.turn); return {threadId};
  }
  stopRequested() { return this.stopped || fs.existsSync(path.join(this.run,'STOP')); }
  async retryCapacity() {
    if (this.stopped) return;
    const snapshot=await this.admin('snapshot');
    if (snapshot.paused || ['ended','interrupted','recovery_required'].includes(snapshot.lifecycle)) return;
    for (const [id,actor] of this.actors) {
      const state=await this.observe(id),t=state.latestTurn;
      if (state.thread.status.type!=='systemError' || t?.status!=='failed' || !capacityFailure(t.error)) continue;
      const key=id+':'+t.id;
      if (!this.failures.has(key)) this.failures.set(key,Date.now());
      if (this.retries.has(key) || Date.now()-this.failures.get(key)<10000) continue;
      // Physical uncertainty blocks retries; retrying a model never replays its tools.
      const code="from robofrontier.runtime_store import RuntimeStore;import sys,json;s=RuntimeStore(sys.argv[1]);a=sys.argv[2];print(json.dumps(any(v['status'] in ('pending','running','uncertain') for v in s.actions(a).values()) or any(True for r in s.db.execute(\"SELECT 1 FROM calls WHERE actor=? AND status='accepted' AND op IN ('move','gripper','action_batch','delegate')\",(a,)))));s.close()";
      const {stdout}=await execFileAsync(this.python,['-c',code,path.join(this.run,'private/state/runtime.sqlite'),actor],{cwd:this.cwd});
      if (JSON.parse(stdout)) continue;
      this.retries.add(key); // An unknown turn/start acknowledgement is never repeated.
      await this.send(id,'容量或速率限制导致上一模型轮次失败。继续同一任务与已有工作笔记；先核对 pending/job 原编号及当前收件，不重放已提交动作，不重置世界，不延长期限。实验已经开始；获取必要的能力信息后继续，不要等待另一条 START。');
      this.event({event:'capacity_retry',thread_id:id,failed_turn_id:t.id});
    }
  }
  bootstrap(actor) {
    return `执行新的独立 RoboFrontier 试验。运行目录：${this.run}。当前身份 ${actor}。
先读取本 run 的 public/inputs/common.md、policy-rules.md、task.json、roles/${actor}.md、skill/SKILL.md、skill/references/tools.md、communication.md 与 adaptive.md（仅 adaptive）。只读取本任务公开输入、自己的工作笔记和被允许的图像。
CLI 传输说明：使用本会话的 robot 工具，op/data 与冻结 Skill 的角色 API 完全相同；call_id 为你选择的稳定编号，views 明确选择本次返回的相机图像。robot 工具自动绑定自己的连接，返回完整 JSON 和真实 RGB。它等价于 Desktop 的 robotTool，无需装载 functions.exec 或 role_tools.js。做本地图像分析等非机器人工作后，用 robot inbox 刷新收件，等价于 roleStep。路径不是看过图片。
实验已经 START，计时已开始。先阅读任务并查看公开初始图或 robot observe，然后自行决定计划和机器人动作。可调用 robot ready 获取能力与限制，调用后继续执行，不需要等待另一条 START。任务内可按 adaptive 规则通过 robot delegate 请求 fork；不要直接管理会话。
本次评分版本 terminal_outcome_v2：完成由当前任务公布的条件决定；任务支持的过程事件按各自检测规则单独报告，记录不自动否决完成。发生问题后可自行观察、恢复并继续。Relay 的最终目标是实际释放并支撑在 GOAL 容差内，空中交接记录不作为最终完成门槛。
从 START 起最多 ${this.wallLimit} 秒，包括思考、通信、fork、容量等待与动作。收到 run_ended/预算结束立即停止新工作。结束只由任务规则和真实工具结果判断。
不读取 Oracle、其他试验、私有场景/XML、私有评估或工程源码；不修改代码、物理或 harness，不重启世界；不从工程主会话继承答案。自己的连接仅用于认证。所有经验仅属于本次 run。`;
  }
  async runTask() {
    const manifest=read(path.join(this.run,'private/inputs/manifest.json'));
    if (manifest.model_requested!==this.launch.model || manifest.effort_requested!==this.launch.effort)
      throw new Error('Frozen model/harness mismatch');
    try {
      await this.startServer();
      const ownerLog=fs.openSync(path.join(this.run,'private/outputs/owner.log'),'a');
      this.owner=spawn(this.python,['-m','robofrontier.pipeline','--run-dir',this.run,'--hidden'],
        {cwd:this.cwd,stdio:['ignore',ownerLog,ownerLog]}); fs.closeSync(ownerLog);
      this.owner.on('error',error=>{this.ownerError=error;});
      for (let n=0;;n++) {
        if (this.ownerError || this.owner.exitCode!==null || n>=120) throw new Error('Physical owner failed to publish its connection');
        if (read(path.join(this.run,'private/state/connections/Meta.json')).url) break;
        await sleep(1000);
      }
      const roles=read(path.join(this.run,'public/inputs/roles.json')).roles;
      for (const [actor,info] of Object.entries(roles)) {
        if (info.kind==='manager') continue;
        const result=await this.rpc.call('thread/start',{model:this.launch.model,modelProvider:this.launch.provider,
          cwd:this.run,approvalPolicy:'never',sandbox:'workspace-write',ephemeral:false,dynamicTools:[robotTool],
          baseInstructions:'You are a RoboFrontier policy agent running in Codex. Follow the frozen public task, current role, tool protocol and evidence boundaries. Use your assigned robot tools. Preserve original call IDs across uncertain results. Do not manage the model host, modify the benchmark, or read private evaluation data.',
          threadSource:'robofrontier-benchmark',config:{web_search:'disabled','features.multi_agent':false}});
        const id=result.thread.id; this.actors.set(id,actor);
        this.event({event:'role_created',actor,thread_id:id,provider:this.launch.provider,model:this.launch.model});
        await this.rpc.call('thread/name/set',{threadId:id,name:`${this.launch.model} · ${path.basename(this.run)} · ${actor}`});
        const conn=path.join(this.run,'private/state/connections/Meta.json');
        await execFileAsync(this.python,['-m','robofrontier.pipeline_client','--quiet','--connection',conn,
          '--call-id','bind-'+actor,'bind','--data',JSON.stringify({actor,thread_id:id})],{cwd:this.cwd});
        if (read(path.join(this.run,'private/inputs/harness.json')).startup_protocol !== 'direct_controller_v2')
          throw new Error('CLI host requires direct_controller_v2');
        const started=await this.admin('request-start');
        if (!started.started) throw new Error('Direct START was not confirmed');
        this.event({event:'direct_start',actor,thread_id:id});
        await this.send(id,this.bootstrap(actor));
      }
      const wrap=value=>({content:[{type:'text',text:JSON.stringify(value)}]});
      const tools={
        exec_command:async a=>{const r=await execAsync(a.cmd,{cwd:this.cwd,maxBuffer:16*1024*1024});return {output:r.stdout,exit_code:0};},
        mcp__codex_app__wait_threads:async({targets})=>{await this.retryCapacity();return wrap({polls:await Promise.all(targets.map(t=>this.observe(t.threadId)))});},
        mcp__codex_app__send_message_to_thread:async p=>wrap(await this.send(p.threadId,p.prompt)),
        mcp__codex_app__set_thread_title:async p=>{await this.rpc.call('thread/name/set',{threadId:p.threadId,name:p.title});return wrap({threadId:p.threadId});},
        mcp__codex_app__fork_thread:async p=>{
          const result=await this.rpc.call('thread/fork',{threadId:p.threadId,cwd:this.run,model:this.launch.model,
            modelProvider:this.launch.provider,excludeTurns:true,ephemeral:false});
          // Persist the known ID before lineage checks or other fallible work.
          this.event({event:'fork_created',thread_id:result.thread.id,parent:p.threadId});
          const snapshot=await this.admin('snapshot');
          const child=snapshot.delegations.flatMap(d=>d.children).find(c=>!c.thread_id);
          if (!child) throw new Error('No pending runtime delegation for fork');
          this.actors.set(result.thread.id,child.actor);
          return wrap({threadId:result.thread.id,sourceThreadId:result.thread.forkedFromId,status:'created'});
        }};
      return await runMetaDispatcher({runDir:this.run,python:this.python,cwd:this.cwd,tools,
        startWhenReady:true,observationSource:'codex.app-server',onEvent:e=>this.event(e),
        stopRequested:()=>this.stopRequested()});
    } finally {
      this.stopped=true;
      // Seal the existing owner; never recreate its physical world.
      let sealError;
      try {const result=await this.admin('seal',{timeout_s:30});
        if (!result.sealed) throw new Error('Seal did not confirm closure');
      } catch(error) {sealError=error;this.event({event:'seal_error',message:error.message});}
      for (const id of this.actors.keys()) {
        try {const s=await this.observe(id);if(s.thread.status.type==='active')
          await this.rpc.call('turn/interrupt',{threadId:id,turnId:s.latestTurn.id});} catch {}
      }
      if (this.rpc) await this.rpc.close();
      if (sealError) throw new Error('Run seal incomplete; manual review required: '+sealError.message);
    }
  }
}
if (require.main===module) {
  const [run,cwd,python]=process.argv.slice(2);
  const host=new PolicyHost(run,cwd,python);
  process.on('SIGTERM',()=>{host.stopped=true;});process.on('SIGINT',()=>{host.stopped=true;});
  host.runTask().then(r=>process.stdout.write(JSON.stringify(r)+'\n')).catch(e=>{process.stderr.write(e.stack+'\n');process.exitCode=1;});
}
module.exports={PolicyHost,robotTool,observationMime};
