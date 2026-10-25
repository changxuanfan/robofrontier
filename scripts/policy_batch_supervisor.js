// Engineering owner of the user-authorized12 independent policy trials.
// Loaded in functions.exec. Never chooses robot actions or delegation scopes.
async function runRemainingPolicyLevels({base,tools,onEvent=()=>{}}) {
  const py='/Users/fanchangxuan/Repos/roomscript2/.venv/bin/python';
  const projectId='c1df1b5d-e189-4c55-a3fd-a57fe4baa0bb';
  const quote=s=>"'"+String(s).replaceAll("'","'\\''")+"'";
  const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
  async function command(cmd,max_output_tokens=3000){
    const r=await tools.exec_command({cmd,max_output_tokens});
    if(r.exit_code!==0)throw new Error(r.output.slice(-2000));return r.output;
  }
  const batch=JSON.parse(await command('cat '+quote(base+'/batch.json'),12000));
  let writes=Promise.resolve();
  function patch(id,value){
    const code="import json;from pathlib import Path;from robofrontier.run_setup import write_json;p=Path("+JSON.stringify(base+'/batch.json')+
      ");d=json.loads(p.read_text());r=next(x for x in d['cases'] if x['case_id']=="+JSON.stringify(id)+
      ");r.update(json.loads("+JSON.stringify(JSON.stringify(value))+"));write_json(p,d)";
    writes=writes.then(()=>command(quote(py)+' -c '+quote(code)));return writes;
  }
  async function state(row){
    const code="import json,fcntl;from pathlib import Path;from robofrontier.runtime_store import RuntimeStore;p=Path("+JSON.stringify(row.run_dir)+
      ");s=RuntimeStore(p/'private/state/runtime.sqlite');v={k:s.get(k) for k in ('lifecycle','start_at','execution_owner_attached','termination_reason')};v['actors']={a:{k:x.get(k) for k in ('thread_id','kind','delegation_state')} for a,x in s.agents().items()};s.close();f=(p/'private/state/world.lock').open('a+');v['owner_closed']=False\ntry:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);v['owner_closed']=True\nexcept BlockingIOError:pass\nf.close();print(json.dumps(v))";
    return JSON.parse(await command(quote(py)+' -c '+quote(code)));
  }
  const prompt=row=>`执行新的 RoboFrontier Adaptive Controller 试验：${row.run_dir}。你可自行决定直接并行控制还是通过项目delegate请求有限fork；本消息授权Meta按你的项目请求fork本Controller并投递分身范围。不要直接创建或管理会话。
先读取本run的public/inputs/roles/Controller.md、common.md、task.json、冻结skill/SKILL.md及工具/Adaptive/通信参考。所有这些相对路径均相对本run。使用${py}、本run的public/inputs/role_tools.js和private/state/connections/Controller.json。先实际查看允许的图像，READY、保存等待并结束启动轮，等Meta的START通知；未START不运动。
只按本轮公开输入、RGB、机器人本体和动作反馈操作；不读Oracle、其他run、私有评估/真值、场景XML或工程脚本，不改代码、物理、Harness，不重置世界，不执行MuJoCo或管理接口。自己的连接文件仅作认证。主/分身只经项目通信；策略、并行、分身边界、姿态和观察均由你决定，必要经验只保存到本轮工作笔记/允许的经验文件。
这是用户授权的bounded_policy_probe，允许在原场景Oracle尚未全面认证时尝试，不能当工程认证。每项从START起最多20分钟墙钟，包括思考、通信、fork和工具等待。执行服务负责到期禁止新动作、制动与封存。成功可按正常finish提前结束；收到预算结束、run_ended或运行已封存立即停止所有新工作并结束本轮，不循环、不重启、不接下一Level，不复用其他运行经验。`;
  function decode(r){if(r.isError)throw new Error('Native platform rejected operation');return JSON.parse(r.content.find(c=>c.type==='text').text);}
  async function stopNative(row,snapshot){
    for(const a of Object.values(snapshot.actors)){
      if(!a.thread_id||a.kind==='manager')continue;
      try{
        const p=decode(await tools.mcp__codex_app__wait_threads({targets:[{threadId:a.thread_id,hostId:'local'}],timeoutMs:0})).polls?.[0];
        if(p?.thread?.status?.type==='active'){
          await tools.mcp__codex_app__send_message_to_thread({threadId:a.thread_id,hostId:'local',model:'gpt-6.1-sol',thinking:'high',
            prompt:'运行已结束并停止物理执行。请立即结束当前原生轮次，不再调用工具、重试请求、发送协作消息或创建分身。无需启动后续任务。此为用户要求的试验收尾通知。'});
          onEvent({event:'native_end_notice',case_id:row.case_id,thread_id:a.thread_id});
        }
      }catch(error){onEvent({event:'native_end_notice_error',case_id:row.case_id,message:String(error).slice(0,300)});}
    }
  }
  async function awaitClosed(row){
    const began=Date.now();
    for(;;){
      const s=await state(row);
      if(s.owner_closed&&s.execution_owner_attached&&['ended','interrupted','recovery_required'].includes(s.lifecycle)){
        await stopNative(row,s);
        await patch(row.case_id,{status:'closed',termination_reason:s.termination_reason??s.lifecycle});
        onEvent({event:'case_closed',case_id:row.case_id,lifecycle:s.lifecycle});return;
      }
      if((s.lifecycle==='prepared'&&Date.now()-began>180000)||s.start_at&&Date.now()/1000>s.start_at+1245){
        await command(quote(py)+' -m robofrontier.runtime_cli --run-dir '+quote(row.run_dir)+' pause --data '+quote(JSON.stringify({reason:'Bounded trial setup/closure watchdog'})));
        onEvent({event:'engineering_attention',case_id:row.case_id,reason:'Setup or owner closure deadline exceeded; no automatic physical restart'});
        throw new Error('Owner requires engineering review '+row.case_id);
      }
      await sleep(3000);
    }
  }
  async function start(row){
    const launch=await tools.exec_command({cmd:quote(py)+' -m robofrontier.policy_probe --run-dir '+quote(row.run_dir)+' > '+quote(row.run_dir+'/private/outputs/owner.log')+' 2>&1',yield_time_ms:1000,max_output_tokens:1000});
    if(!launch.session_id)throw new Error('Owner launch failed '+row.case_id);
    await patch(row.case_id,{status:'owner_starting',owner_exec_session:launch.session_id});
    for(let n=0;;n++){
      const ready=JSON.parse(await command(quote(py)+' -c '+quote("import json;from pathlib import Path;p=Path("+JSON.stringify(row.run_dir+'/private/state/connections/Meta.json')+");print(json.dumps(bool(json.loads(p.read_text()).get('url'))))")));
      if(ready)break;if(n>=30)throw new Error('Owner did not publish a connection');await sleep(1000);
    }
    const t=decode(await tools.mcp__codex_app__create_thread({title:`Main · ${{transport:'物品传递',mosaic:'拼图',relay:'空中接力',dial:'旋钮'}[row.family]} L${row.level} · ${row.family==='dial'?'4盘':''}${row.arms}臂 · Adaptive ·20m`,
      model:'gpt-6.1-sol',thinking:'high',prompt:prompt(row),target:{type:'project',projectId,environment:{type:'local'}}}));
    if(!t.threadId)throw new Error('Native creation outcome needs review');
    row.thread_id=t.threadId;await patch(row.case_id,{thread_id:t.threadId,status:'initializing'});
    onEvent({event:'controller_created',case_id:row.case_id,thread_id:t.threadId});
    await command(quote(py)+' -m robofrontier.pipeline_client --quiet --connection '+quote(row.run_dir+'/private/state/connections/Meta.json')+
      ' --call-id '+quote('bind-controller-'+row.case_id)+' --data '+quote(JSON.stringify({actor:'Controller',thread_id:t.threadId}))+' bind');
    const source=await command('cat '+quote(row.run_dir+'/private/inputs/frozen-source/scripts/meta_dispatcher.js'),13000);
    const dispatcher=eval(source+'\nrunMetaDispatcher;');
    // Keep this promise alive, and await it after closure; no detached task.
    const running=dispatcher({runDir:row.run_dir,python:py,cwd:'/Users/fanchangxuan/Repos/roomscript2',tools,startWhenReady:true,
      onEvent:e=>{if(e.event!=='cycle')onEvent({case_id:row.case_id,...e});}});
    const outcomes=await Promise.allSettled([running,awaitClosed(row)]);
    const error=outcomes.find(x=>x.status==='rejected');if(error)throw error.reason;
  }
  const outcomes=await Promise.allSettled(['transport','mosaic','relay','dial'].map(async family=>{
    const rows=batch.cases.filter(c=>c.family===family).sort((a,b)=>a.level-b.level);
    await awaitClosed(rows[0]);
    for(const row of rows.slice(1)){
      const stop=await command(quote(py)+' -c '+quote("from pathlib import Path;print('stop' if Path("+JSON.stringify(base+'/STOP')+").exists() else 'continue')"));
      if(stop.trim()==='stop')return;
      await start(row);
    }
    onEvent({event:'family_closed',family});
  }));
  await writes;
  const errors=outcomes.flatMap((x,i)=>x.status==='rejected'?[{family:['transport','mosaic','relay','dial'][i],error:String(x.reason)}]:[]);
  return {phase:errors.length?'engineering_attention':'all12_closed',errors};
}
