"""Historical v1 three-arm/six-slot exporter; not a current generic evaluator.

Writes into the explicitly selected old run. Do not use on v3 runs.
"""
from pathlib import Path
import argparse
from collections import Counter
import hashlib
import json
import subprocess

p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);args=p.parse_args();d=args.run_dir.resolve()
if (d/'private/inputs/manifest.json').exists():raise SystemExit('Historical v1 exporter only; current runs use their own runtime outputs')
s=json.loads((d/'private/runtime.json').read_text())
if not s['ended']:raise SystemExit('Trial still running; no completion claim exported')
events=[json.loads(line) for line in (d/'private/events.jsonl').read_text().splitlines() if line]
start=next(e for e in events if e['kind']=='START');end=next(e for e in events if e['kind']=='TEAM_FINISH_PROPOSED')
threads=json.loads((d/'private/threads.json').read_text())['threads'];models={}
sessionroot=Path.home()/'.codex/sessions'
for a,t in threads.items():
    files=list(sessionroot.rglob('*'+t+'.jsonl'));turns=[];calls=[]
    for file in files:
        for line in file.open():
            r=json.loads(line);v=r.get('payload',{})
            if r.get('type')=='turn_context':turns.append({k:v.get(k) for k in ('turn_id','model','effort')})
            if r.get('type')=='response_item' and v.get('type')=='function_call':calls.append(v.get('name'))
    models[a]={'thread_id':t,'turns':turns,'tool_names':dict(Counter(calls)),
               'all_luna_xhigh':bool(turns) and all(v['model']=='gpt-6-luna' and v['effort']=='xhigh' for v in turns)}
(d/'private/model-verification.json').write_text(json.dumps(models,ensure_ascii=False,indent=2)+'\n')
manifest=json.loads((d/'private/manifest.json').read_text());unchanged=all(Path(f).exists() and hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in manifest['source_sha256'].items())
evaluation=json.loads((d/'private/evaluation.json').read_text());jobs=list(s['jobs'].values())
allowed=all((m['from']=='Meta' and all(t in 'ABC' for t in m['to'])) or (m['from'] in 'ABC' and m['to']==['Meta']) for m in s['messages'])
research_motion=[e for e in events if e['kind']=='ACTION_STARTED' and e['actor']=='research']
started={e['action']['action_id']:e for e in events if e['kind']=='ACTION_STARTED'}
finished={e['call_id']:e for e in events if e['kind']=='TOOL_RETURN' and 'execution' in e.get('result',{}) and e['result']['execution'].get('status') in ('completed','failed')}
intervals=[]
for key,e in started.items():
    if key in finished:intervals.append((max(0,e['wall_s']-start['wall_s']),max(0,finished[key]['wall_s']-start['wall_s'])))
merged=[]
for lo,hi in sorted(intervals):
    if merged and lo<=merged[-1][1]+.3:merged[-1]=(merged[-1][0],max(merged[-1][1],hi))
    else:merged.append((max(0,lo-.15),hi+.15))
observations=[e['result']['observation'] for e in events if e['kind']=='TOOL_RETURN' and 'observation' in e.get('result',{})]
last_observation=observations[-1]
released_and_still=all(r['gripper_width_m']>.07 and r['tcp_m'][2]>.55 and max(abs(v) for v in r['joint_velocity_rad_s'])<.005 for r in last_observation['robot'].values())
checks={'final_grippers_open_arms_raised_and_still':released_and_still,'six_slots_correct':evaluation['all_goals_met'],'all_sessions_luna_xhigh':all(x['all_luna_xhigh'] for x in models.values()),
        'engineering_files_unchanged':unchanged,'tree_messages_only':allowed,'research_did_not_control_arms':not research_motion,
        'no_running_actions':all(j['status']!='running' for j in jobs),
        'video_encoder_ok':bool(s['video']) and s['video']['error'] is None,'video_no_dropped_frames':bool(s['video']) and s['video']['dropped_frames']==0}
# Actual overlap of action execution across arms, using wall clock events.
changes=[]
for key,e in started.items():
    if key in finished:changes.extend([(e['wall_s'],1),(finished[key]['wall_s'],-1)])
active=peak=0
for _,delta in sorted(changes,key=lambda x:(x[0],x[1])):active+=delta;peak=max(peak,active)
checks['three_arms_executed_concurrently']=peak>=3
report={'checks':checks,'evaluation':evaluation,'wall_seconds':round(end['wall_s']-start['wall_s'],3),
        'simulation_seconds':round(end['sim_s']-start['sim_s'],3),'messages':len(s['messages']),
        'actions_by_arm':dict(Counter(j['arm'] for j in jobs)),'action_status':dict(Counter(j['status'] for j in jobs)),
        'turns_by_role':{a:len(v['turns']) for a,v in models.items()},'peak_simultaneous_actions':peak,
        'action_clip_wall_seconds':round(sum(hi-lo for lo,hi in merged),3),
        'video':s['video'],'final_observation':last_observation,'scope':'One controlled desktop-session pilot. Public initial positions allowed; no policy dynamic object-state endpoint. Research session drives native lifecycle. Not an unattended daemon or OS sandbox.'}
(d/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
video=d/'global-realtime.mp4'
if video.exists() and merged:
    select='+'.join(f'between(t,{lo:.3f},{hi:.3f})' for lo,hi in merged)
    filt=d/'private/action-only.filter';filt.write_text("select='"+select+"',setpts=N/15/TB")
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(video),'-filter_script:v',str(filt),'-an','-c:v','libx264','-preset','veryfast','-crf','22',str(d/'actions-only.mp4')],check=True)
rows=['# Meta + 三个 Worker：真实 MuJoCo 试点','',f"六槽结果：{evaluation['all_goals_met']}。实验墙钟 {report['wall_seconds']:.1f} 秒，仿真推进 {report['simulation_seconds']:.1f} 秒。",'',
      '统一使用 gpt-6-luna / xhigh；实际逐轮验证见 private/model-verification.json。公开初始坐标；后续 RGB + 机器人状态。', '',
      f"动作 {len(jobs)} 次，消息 {len(s['messages'])} 条，最大同时执行 {peak} 台机械臂。",'',
      '## 验收','']
rows += [f"- {'PASS' if ok else 'FAIL'}：{key}" for key,ok in checks.items()]
rows += ['','## 关键协作过程','',
         'B 最初报告撤离，但 TCP 仍在棋盘块正上方。A 根据允许的机器人状态指出冲突并等待；Meta 要求 B 抬高并回到本区，A 核对后再接取。相关消息 m18–m31。没有由 research 会话提供修正方案。', '',
         '## 录像质量','',f"编码错误：{s['video']['error']}；丢帧 {s['video']['dropped_frames']} 帧。原片并非逐帧无损墙钟记录，动作片段按事件时间裁切，丢帧会带来约秒级边界偏差。",'','## 文件','', '- global-realtime.mp4：完整原速录像。','- actions-only.mp4：保留任一机械臂工具动作执行区间，移除主要无动作等待，播放速度仍为原速。','- report.json：机器可读验收与统计。','- private/events.jsonl：原始工具、消息和时间线。','- private/driver.jsonl：真实会话状态与唤醒。','',
         '## 范围','',report['scope'],'',
         '槽位评估仅检查属性、中心位置与高度；最终释放/撤离需结合图像和工具记录核对。单次试验不能推断总体成功率或架构优劣。']
(d/'report.md').write_text('\n'.join(rows)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
