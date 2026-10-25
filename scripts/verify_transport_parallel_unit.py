"""Two qualified disjoint local jobs, one canonical world, serial or batched.

Replay real actuator commands, never saved qpos. No full36-goal qualification.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[1]


def execute_independent_sequences(t,sequences):
    """One physics writer, disjoint arm programs, no per-stage team barrier.

    This commissioning executor follows already qualified commands and waits;
    it does not choose tasks, handover partners or recovery actions.
    """
    index={r:0 for r in sequences};ready={r:t.time+seq[0]['delay'] for r,seq in sequences.items()}
    pending={}
    while pending or any(index[r]<len(seq) for r,seq in sequences.items()):
        t.check_stop()
        for robot,seq in sequences.items():
            if robot in pending or index[robot]>=len(seq) or t.time+1e-9<ready[robot]:continue
            op=seq[index[robot]];command=op['command'];label=robot+'/'+op['label']
            prepared=t.arms[robot].validate(*command,t.cfg,t.task['workspace_world_m']['lower'],t.task['workspace_world_m']['upper'])
            t.event('action_start',label=label,commands={robot:command})
            t.arms[robot].start(f'independent-{robot}-{index[robot]}',prepared,t.time)
            pending[robot]=dict(label=label,commands={robot:command},start_sim_s=t.time)
        t.step()
        if t.tick%t.stride:continue
        for robot in list(pending):
            result=t.arms[robot].poll(t.time,15)
            if result is None:continue
            rec=pending.pop(robot);rec.update(end_sim_s=t.time,results={robot:result});t.actions.append(rec)
            t.event('action_end',label=rec['label'],results=rec['results'])
            if result['status']!='completed':raise RuntimeError(rec['label']+': '+str(result))
            index[robot]+=1
            if index[robot]<len(sequences[robot]):ready[robot]=t.time+sequences[robot][index[robot]]['delay']


def run(a):
    from robofrontier.suite.oracle import Trial
    from robofrontier.suite.placement_oracle import clear_idle_arms
    from robofrontier.run_setup import load_harness
    from robofrontier.task_config import physical_control_fingerprint
    from robofrontier.suite.source_snapshot import MARKER
    frozen=Path(os.environ[MARKER]);batch=a.unit_batch or ROOT/'runs/oracle/transport-r13-units-20261025'
    files=list((frozen/'robofrontier').rglob('*.py'))+list((frozen/'scripts').rglob('*.py'))+[frozen/'inputs/harness.json',frozen/'inputs/catalog.json']
    scene=ROOT/f'configs/environments/cube_transport/frontier-v2-r13-transport-a12-d{a.level}'
    task=json.loads((scene/'task.json').read_text());cfg=load_harness(frozen/'inputs/harness.json')
    frozen_sources={str(p.relative_to(frozen)):p.read_bytes() for p in files}
    expected_control=physical_control_fingerprint({name:hashlib.sha256(payload).hexdigest() for name,payload in frozen_sources.items()},cfg)
    actors=[('F',f'F_item{1 if a.level==1 else 2}','F_slot2',f'a12-l1-cube-1' if a.level==1 else 'parallel-a12-l2-cube-side'),
            ('H',f'H_item{1 if a.level==1 else 2}','H_slot3',f'a12-l{a.level}-cylinder-{1 if a.level==1 else 2}')]
    sources=[];sequences={}
    for robot,obj,goal,unit in actors:
        source=batch/'units'/unit/'attempt-001';r=json.loads((source/'report.json').read_text())
        assert r['local_passed'] and r['evaluation']['assignments'].get(goal)==obj
        assert r['scene_sha256']==task['scene_sha256']
        assert r['physical_control_sha256']==expected_control
        sequence=[];prior=None
        for action in json.loads((source/'actions.json').read_text()):
            if action['label']=='clear-idle-arms':prior=action['end_sim_s'];continue
            assert set(action['commands'])=={robot};assert all(x['status']=='completed' for x in action['results'].values())
            sequence.append(dict(label=action['label'],delay=max(0.,action['start_sim_s']-prior),command=action['commands'][robot]));prior=action['end_sim_s']
        sequences[robot]=sequence
        sources.append(dict(robot=robot,object=obj,goal=goal,reference=str(source),
            action_sha256=hashlib.sha256((source/'actions.json').read_bytes()).hexdigest()))
    t=Trial(scene,a.output,controller_cfg=cfg,sources=frozen_sources)
    t.live_preview=True;t.abort_on_process_violation=True
    t.stop_requested=lambda:time.monotonic()-t.started>240 or t.time>600 or len(t.actions)>500
    report=dict(case_id=t.task['id'],passed=False,commissioning_only=True,local_passed=False,
        scope=__doc__,mode=a.mode,canonical_initialization=True,actuator_command_sources=sources,
        state_replay=False,budgets=dict(wall_s=240,sim_s=600,action_batches=500))
    try:
        t.wait(.6);clear_idle_arms(t);start=t.time;before=t.monitor.result()
        if a.mode=='serial':
            for robot in sequences:
                for op in sequences[robot]:
                    if op['delay']:t.wait(op['delay'])
                    t.execute(robot+'/'+op['label'],{robot:op['command']})
        elif a.mode=='parallel':
            for index in range(max(map(len,sequences.values()))):
                selected={r:seq[index] for r,seq in sequences.items() if index<len(seq)}
                delay=max(op['delay'] for op in selected.values())
                if delay:t.wait(delay)
                t.execute('parallel-unit/'+str(index),{r:op['command'] for r,op in selected.items()})
        else:execute_independent_sequences(t,sequences)
        t.wait(3.2);after=t.monitor.result();report['task_motion_sim_s']=t.time-start
        report['transport_overlap_sim_s']=after['motion_overlap_sim_s']-before['motion_overlap_sim_s']
        report['transport_any_motion_sim_s']=after['any_motion_sim_s']-before['any_motion_sim_s']
        report['local_passed']=all(after.get('assignments',{}).get(goal)==obj for _,obj,goal,_ in actors) and not t.monitor.violations
        if not report['local_passed']:report['error']='Two original goals were not jointly accepted without violations'
    except (Exception,KeyboardInterrupt) as error:report['error']=type(error).__name__+': '+str(error)
    finally:
        try:t.snapshot('final')
        except Exception as error:report['snapshot_error']=str(error)
        report['evaluation']=t.monitor.result();report['unresolved_robot_actions']=[r for r,arm in t.arms.items() if arm.job is not None]
        t.close(report)
    print(json.dumps({k:report.get(k) for k in ['mode','local_passed','task_motion_sim_s','transport_overlap_sim_s','error']}),flush=True)
    return report['local_passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['serial','parallel','asynchronous'],required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--level',type=int,choices=(1,2),default=1);p.add_argument('--unit-batch',type=Path);a=p.parse_args()
    from robofrontier.suite.source_snapshot import reexec_entry,MARKER
    batch=ROOT/'runs/oracle/transport-r13-units-20261025'
    if not os.environ.get(MARKER):reexec_entry(ROOT,a.output,Path(__file__),__import__('sys').argv[1:],batch/'harness.json',batch/'catalog.json')
    raise SystemExit(0 if run(a) else 2)
