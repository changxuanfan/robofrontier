"""Read-only audit of a finished v3 run; writes derived outputs only."""
import argparse
from collections import Counter
import json
from pathlib import Path

from robofrontier.runtime_store import RuntimeStore
from robofrontier.run_setup import write_json, verify_inputs
from robofrontier.action_batch import external_wait


def occupancy(intervals, start, end):
    changes=[(start,0),(end,0)]
    for low,high in intervals:
        low,high=max(start,low),min(end,high)
        if high>low:changes.extend([(low,1),(high,-1)])
    active=peak=0;previous=start;durations=Counter()
    for instant,delta in sorted(changes,key=lambda pair:(pair[0],pair[1])):
        durations[active]+=instant-previous
        active+=delta;peak=max(peak,active);previous=instant
    return {'seconds_by_active_arm_count':dict(sorted(durations.items())), 'peak':peak,
            'overlap_seconds':sum(value for key,value in durations.items() if key>=2)}


def export(directory):
    directory=Path(directory).resolve();output=directory/'private/outputs'
    manifest=directory/'private/inputs/manifest.json'
    if not manifest.is_file() or json.loads(manifest.read_text()).get('schema_version')!=3 or not (directory/'private/state/runtime.sqlite').is_file():
        raise ValueError('Existing v3 run required; historical layouts are not migrated')
    store=RuntimeStore(directory/'private/state/runtime.sqlite')
    try:
        lifecycle=store.get('lifecycle')
        if lifecycle not in ('ended','interrupted','recovery_required'):
            raise ValueError('Do not export a final report for a running/prepared trial')
        events=store.events();actions=store.actions();agents=store.agents();config=store.get('config')
        start_sim_time=store.get('start_sim_time');run_id=store.get('run_id');robot_count=len(store.get('robots',[]))
        messages=[store._message(row) for row in store.db.execute('SELECT * FROM messages ORDER BY seq')]
        async_actions=sum('started_sim_s' in a and not external_wait(store,a) for a in actions.values())
    finally:store.close()
    start=next((e for e in events if e['kind']=='START'),None)
    end=next((e for e in events if e['kind']=='TEAM_FINISH_PROPOSED'),None)
    starts={e['details']['action_id']:e['created_at'] for e in events if e['kind']=='ACTION_STARTED'}
    finishes={e['details']['action_id']:e['created_at'] for e in events if e['kind']=='ACTION_FINISHED'}
    models={}
    for actor,agent in agents.items():
        if agent['kind']=='manager':continue
        thread=agent['thread_id'];turns=[]
        paths=list((Path.home()/'.codex/sessions').rglob('*'+thread+'*.jsonl')) if thread else []
        for path in paths:
            for line in path.open():
                # Avoid decoding image payloads and unrelated model content.
                if '"turn_context"' not in line:continue
                record=json.loads(line)
                if record.get('type')=='turn_context':
                    value=record['payload'];turns.append({k:value.get(k) for k in ('turn_id','model','effort')})
        models[actor]={'thread_id':thread,'turns':turns,
                       'verified':bool(turns) and all(t['model']==config['model'] and t['effort']==config['effort'] for t in turns)}
    evaluation_path=output/'evaluation.json'
    evaluation=json.loads(evaluation_path.read_text()) if evaluation_path.exists() else None
    metric_evaluation=evaluation
    if metric_evaluation is None and (output/'interruption-evaluation.json').exists():
        metric_evaluation=json.loads((output/'interruption-evaluation.json').read_text()).get('evaluation')
    report={'lifecycle':lifecycle,'evaluation':evaluation,
            'scope':'One run, potentially multiple native sessions; public initial information, RGB and robot feedback. No population or architecture superiority claim.',
            'topology':config['topology'],'robot_count':robot_count,
            'run_id':run_id,'start_sim_time_s':start_sim_time,
            'sim_seconds':end['details']['sim_time_s']-start_sim_time if end and start_sim_time is not None else None,
            'wall_seconds':end['created_at']-start['created_at'] if start and end else None,
            'actions_by_arm':dict(Counter(a['arm'] for a in actions.values())),
            'executed_actions_by_arm':dict(Counter(a['arm'] for a in actions.values() if 'started_sim_s' in a)),
            'unstarted_members':sum('started_sim_s' not in a for a in actions.values()),
            'action_status':dict(Counter(a['status'] for a in actions.values())),
            'messages':len(messages), 'turns_by_role':{a:len(v['turns']) for a,v in models.items()},
            'all_requested_models_verified':bool(models) and all(v['verified'] for v in models.values()),
            'changed_inputs':verify_inputs(directory),
            'late_messages':sum(e['kind']=='LATE_MESSAGE_AFTER_FINISH' for e in events),
            'wait_returns':dict(Counter(e['details'].get('result',{}).get('wait_status') for e in events
                                if e['kind']=='TOOL_RETURN' and e['details'].get('op')=='wait_events')),
            'async_actions':async_actions,
            'interpretation':'Action overlap covers active tool executions, excluding idle pose/grasp holding. It does not establish independent useful work or an architecture advantage.'}
    if isinstance(metric_evaluation,dict) and 'metrics' in metric_evaluation:
        report['task_metrics']={**metric_evaluation['metrics'],
            'robot_action_count':sum('started_sim_s' in a for a in actions.values()),
            'source':'final evaluation' if evaluation is not None else 'interruption snapshot; not a completed run'}
    if start and end:
        report['execution_occupancy']=occupancy([(starts[i],finishes[i]) for i in starts.keys() & finishes.keys()],start['created_at'],end['created_at'])
    snapshot=output/'runtime.json'
    if snapshot.exists():report['video']=json.loads(snapshot.read_text()).get('video')
    write_json(directory/'private/state/model-verification.json',models)
    write_json(output/'messages.json',messages);write_json(output/'report.json',report)
    lines=['# RoboFrontier 运行审计','',f"生命周期：{lifecycle}；目标评估：{evaluation}。",'',
           f"墙钟：{report['wall_seconds']} 秒；动作：{sum(report['actions_by_arm'].values())}；消息：{len(messages)}。",'',
           f"实际模型设置核验：{report['all_requested_models_verified']}；固定输入变更：{report['changed_inputs']}。",'',
           '动作重叠不包含空闲保持；需结合任务进展与视频解释，一次试点不支持架构优劣结论。']
    if 'task_metrics' in report:
        lines+=['',f"任务指标：{report['task_metrics']}。"]
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path)
    print(json.dumps(export(p.parse_args().run_dir),ensure_ascii=False,indent=2))
