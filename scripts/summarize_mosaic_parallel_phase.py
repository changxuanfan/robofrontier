"""Read-only observed overlap and fixed-plan critical-path diagnostics.

The graph bound is conditional on this conservative candidate's dependencies
and measured action durations. It is never a global robot-planning optimum or
a separate serial physical experiment.
"""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import mujoco
import numpy as np
from robofrontier.motion_recording import MotionReader
from robofrontier.run_setup import write_json


def interval_profile(intervals):
    events=defaultdict(int)
    for start,end in intervals:
        if end<start:raise ValueError('Negative interval')
        events[start]+=1;events[end]-=1
    hist=defaultdict(float);active=0;previous=None
    for stamp,delta in sorted(events.items()):
        if previous is not None:hist[active]+=stamp-previous
        active+=delta;previous=stamp
    return dict(sorted(hist.items()))


def critical_path(costs,pred):
    if len(costs)!=len(pred):raise ValueError('Graph/cost size differs')
    longest=[];parent=[]
    for i,(cost,ps) in enumerate(zip(costs,pred)):
        if cost<0 or any(p>=i or p<0 for p in ps):raise ValueError('Require nonnegative costs and source-order DAG')
        best=max(ps,key=lambda p:longest[p]) if ps else None
        longest.append(cost+(longest[best] if best is not None else 0.));parent.append(best)
    end=max(range(len(longest)),key=longest.__getitem__);path=[];node=end
    while node is not None:path.append(node);node=parent[node]
    return longest[end],list(reversed(path))


def summarize(trial,prefix):
    trial=Path(trial).resolve();folder=trial/'source-prefixes'/f'prefix-{prefix:04d}'
    plan=json.loads((folder/'concurrency-plan.json').read_text());label=plan['operation_prefix']+'-';records={}
    for line in (trial/'trace.jsonl').read_text().splitlines():
        try:e=json.loads(line)
        except json.JSONDecodeError:continue
        if not e.get('label','').startswith(label):continue
        i=int(e['label'].rsplit('-',1)[1]);r=records.setdefault(i,{})
        if e['event']=='action_start':r.update(start=e['sim_time_s'],commands=e['commands'])
        elif e['event']=='action_end':r.update(end=e['sim_time_s'],results=e['results'])
    if len(records)!=len(plan['jobs']) or any('end' not in r or any(v['status']!='completed' for v in r['results'].values()) for r in records.values()):
        raise ValueError('Require this complete successful operation phase')
    if any(len(r['commands'])!=1 for r in records.values()):raise ValueError('This diagnostic requires atomic single-arm operation nodes')
    rows=[records[i] for i in range(len(records))];waits=[j.get('wait_after_s',max(0.,j['end_sim_s']-j['actions'][0]['end_sim_s'])) for j in plan['jobs']]
    durations=[r['end']-r['start'] for r in rows];costs=[d+w for d,w in zip(durations,waits)]
    bound,path=critical_path(costs,plan['predecessors']);start=min(r['start'] for r in rows);end=max(r['end']+w for r,w in zip(rows,waits));hist=interval_profile([(r['start'],r['end']) for r in rows])
    busy=sum(v for k,v in hist.items() if k>=1);overlap=sum(v for k,v in hist.items() if k>=2)
    categories=Counter();park_s=0.;per_robot=Counter();refine_count=0;refine_s=0.;hover_refine_count=0
    for i,r in enumerate(rows):
        robot,(op,params)=next(iter(r['commands'].items()));per_robot[robot]+=durations[i]
        category=('force_grasp' if params.get('mode')=='grasp' else 'jaw_width') if op=='gripper' else ('joint_move' if 'joint_rad' in params else 'cartesian_move')
        categories[category]+=1
        if 'park' in plan['jobs'][i]['actions'][0]['label']:park_s+=durations[i]
        source_label=plan['jobs'][i]['actions'][0]['label']
        if 'refine' in source_label:
            refine_count+=1;refine_s+=durations[i]
            if 'hover' in source_label:hover_refine_count+=1
    model=mujoco.MjModel.from_xml_path(str(trial/'scene.xml'));task=json.loads((trial/'task.json').read_text());robots=list(task['arms'])
    ids=np.asarray([[model.jnt_qposadr[model.joint(f'{r}_joint{k}').id] for k in range(1,8)] for r in robots])
    reader=MotionReader(trial/'motion');reader.meta['chunks']=[c for c in reader.meta['chunks'] if c['end_sim_s']>=start-.02 and c['start_sim_s']<=end+.02]
    previous=None;motion_hist=defaultdict(float);robot_motion=Counter()
    for stamp,qpos in reader.frames():
        if previous is not None:
            old_t,old_q=previous;dt=stamp-old_t;span=max(0.,min(stamp,end)-max(old_t,start))
            if span:
                moving=np.max(abs(qpos[ids]-old_q)/dt,axis=1)>.02;motion_hist[int(sum(moving))]+=span
                for r,m in zip(robots,moving):
                    if m:robot_motion[r]+=span
        previous=(stamp,qpos[ids].copy())
        if stamp>end:break
    any_motion=sum(v for k,v in motion_hist.items() if k>=1);overlap_motion=sum(v for k,v in motion_hist.items() if k>=2)
    result=dict(scope='Completed phase only. Observed overlap; fixed conservative-DAG bound with measured durations, not a global optimum or serial physics experiment.',
        trial=str(trial),prefix=prefix,candidate_goals=plan['total_goals'],operations=len(rows),operation_counts=dict(categories),
        measured_action_duration_sum_s=sum(durations),declared_wait_sum_s=sum(waits),observed_operation_window_s=end-start,
        busy_union_s=busy,overlap_action_s=overlap,overlap_action_fraction_of_busy=overlap/busy if busy else 0.,
        action_degree_time_s=dict(hist),ordinary_action_peak=max(k for k,v in hist.items() if v>0),
        fixed_graph_critical_bound_s=bound,observed_to_fixed_graph_bound=(end-start)/bound,critical_path_nodes=len(path),
        fixed_duration_work_and_wait_sum_s=sum(costs),fixed_duration_schedule_compression=1-(end-start)/sum(costs),
        critical_path_barrier_nodes=sum(plan['jobs'][i].get('barrier',False) for i in path),
        empty_parking_action_s=park_s,per_robot_action_s=dict(per_robot),
        cartesian_refinement_actions=refine_count,cartesian_refinement_action_s=refine_s,hover_refinement_actions=hover_refine_count,
        motion_detection='Approximate60Hz finite difference of seven controlled joints; >0.02rad/s. Initial parking and phase-external settle excluded; distinct from exact20ms monitor statistics.',
        sampled_any_motion_s=any_motion,sampled_overlap_motion_s=overlap_motion,
        sampled_overlap_motion_fraction=overlap_motion/any_motion if any_motion else 0.,sampled_motion_degree_time_s=dict(motion_hist),
        sampled_peak_moving_robots=max(k for k,v in motion_hist.items() if v>0),sampled_per_robot_motion_s=dict(robot_motion),
        omitted_source_human_deliberation_s=plan.get('review_deliberation_schedule',{}).get('saved_source_idle_s',0.),
        limitation='Bounds retain chosen object assignments, paths, source order, conservative swept boxes and repair barriers; loosen these or reduce steps before claiming a globally better solution.')
    write_json(trial/'analysis'/f'parallel-prefix-{prefix:04d}.json',result)
    print(json.dumps(result),flush=True);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path);p.add_argument('--prefix',type=int,default=0)
    a=p.parse_args();summarize(a.trial,a.prefix)
