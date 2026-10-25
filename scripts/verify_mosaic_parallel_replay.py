"""Two local jobs: declared finite-command concurrency engineering probe.

Build per-arm sequences from one successful canonical serial commissioning run.
Screen sampled full-arm/payload swept bounds before starting another canonical
world, then execute the same ordinary commands in parallel. No physics replay,
object writes, public pick primitive or optimal-speed claim.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

import mujoco
import numpy as np

from robofrontier.run_setup import load_harness,write_json
from robofrontier.motion_recording import MotionReader
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.oracle import Trial,ReferenceStopped
from robofrontier.suite.v2 import ROOT
from robofrontier.suite.source_snapshot import reexec_entry,MARKER


def serial_plan(source):
    report=json.loads((source/'report.json').read_text())
    if not report.get('commissioning_passed') or report.get('error') or report.get('unresolved_robot_actions'):
        raise ValueError('Require a clean serial commissioning witness')
    events=[json.loads(line) for line in (source/'trace.jsonl').read_text().splitlines()]
    jobs=[e for e in events if e['event']=='goal_attempt']
    if len(jobs)!=2 or any(len(e['path'])!=1 for e in jobs) or len({e['path'][0] for e in jobs})!=2:
        raise ValueError('Exactly two distinct one-arm local jobs required')
    payloads={e['path'][0]:e['object'] for e in jobs}
    actions=json.loads((source/'actions.json').read_text())
    initial=[a for a in actions if a['label']=='clear-idle-arms']
    if len(initial)!=1:raise ValueError('One canonical initial parking required')
    streams={robot:[] for robot in payloads}
    for action in actions:
        if action['label']=='clear-idle-arms':continue
        if len(action['commands'])!=1:raise ValueError('Unexpected coupled serial operation')
        robot=next(iter(action['commands']))
        if robot not in streams or any(r['status']!='completed' for r in action['results'].values()):
            raise ValueError('Only clean selected-arm command streams supported')
        streams[robot].append(action)
    return report,jobs,payloads,initial[0],streams


def swept_screen(source,payloads):
    model=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));data=mujoco.MjData(model)
    task=json.loads((source/'task.json').read_text());monitor=Monitor(SimpleNamespace(model=model,data=data,task=task,arms=task['arms']))
    ids={}
    physical=(model.geom_contype!=0)|(model.geom_conaffinity!=0)
    for robot,name in payloads.items():
        ids[robot]=np.asarray(sorted(set(np.flatnonzero((monitor.geom_owner==monitor.robot_index[robot])&physical))|set(monitor.geoms[name])),int)
    bounds={robot:[np.full(3,np.inf),np.full(3,-np.inf)] for robot in payloads}
    count=0
    for _,qpos in MotionReader(source/'motion').frames():
        data.qpos[:]=qpos;mujoco.mj_kinematics(model,data)
        for robot,geoms in ids.items():
            rotation=data.geom_xmat[geoms].reshape(-1,3,3)
            centers=data.geom_xpos[geoms]+np.einsum('nij,nj->ni',rotation,model.geom_aabb[geoms,:3])
            extent=np.einsum('nij,nj->ni',abs(rotation),model.geom_aabb[geoms,3:])
            bounds[robot][0]=np.minimum(bounds[robot][0],np.min(centers-extent,axis=0))
            bounds[robot][1]=np.maximum(bounds[robot][1],np.max(centers+extent,axis=0))
        count+=1
    a,b=bounds.values();reserve=.02
    gaps=np.maximum(a[0]-b[1],b[0]-a[1])
    return dict(passed=bool(np.max(gaps)>2*reserve),frames=count,
        bounds_m={r:[v.tolist() for v in pair] for r,pair in bounds.items()},
        separating_axis_gap_m=gaps.tolist(),reserve_per_actor_m=reserve,
        scope='Recorded sampled full-arm/payload envelopes plus reserve; not a continuous or dynamic certificate')


def run(a):
    source=a.source.resolve();output=a.output.resolve()
    source_report,jobs,payloads,initial,streams=serial_plan(source)
    screen=swept_screen(source,payloads)
    if not screen['passed']:
        output.mkdir(parents=True,exist_ok=False)
        write_json(output/'planning-rejection.json',dict(screen=screen,physical_execution_started=False,
            reason='Conservative envelopes overlap; not physical impossibility'))
        print(json.dumps(dict(screen_passed=False,physical_execution_started=False,screen=screen)),flush=True)
        return 1
    sources={str(p.relative_to(ROOT)):p.read_bytes() for folder in ('robofrontier','scripts') for p in (ROOT/folder).rglob('*.py')}
    case=next(c for c in json.loads(a.catalog.read_text())['cases'] if c['id']==source_report['case_id'])
    t=Trial(ROOT/case['directory'],output,record_hz=60,sources=sources,controller_cfg=load_harness(a.harness))
    t.live_preview=True;t.abort_on_process_violation=True
    t.stop_requested=lambda:t.time>600 or time.monotonic()-t.started>600
    def stop(signum,frame):raise ReferenceStopped('External stop during parallel commission')
    signal.signal(signal.SIGTERM,stop)
    report=dict(passed=False,commissioning_only=True,parallel_commissioning_passed=False,
        case_id=case['id'],source_serial=str(source),source_report_sha256=hashlib.sha256((source/'report.json').read_bytes()).hexdigest(),
        declared_jobs=[dict(robot=e['path'][0],object=e['object'],goal=e['goal']) for e in jobs],
        sampled_swept_screen=screen,scope='Two-cell command concurrency fixture; not full artwork or GPT performance')
    try:
        if (t.physical_control_sha256!=source_report['physical_control_sha256'] or
                t.environment_logic_sha256!=source_report['environment_logic_sha256'] or
                t.source_bundle['sha256']!=source_report['scene_bundle_sha256']):
            raise ValueError('Serial and concurrent runs must share physical control, evaluator and source assets')
        t.wait(.6);t.snapshot('initial')
        t.execute('clear-idle-arms',deepcopy(initial['commands']));t.wait(.2)
        began=t.time
        for index in range(max(map(len,streams.values()))):
            commands={r:deepcopy(actions[index]['commands'][r]) for r,actions in streams.items() if index<len(actions)}
            t.execute(f'parallel-local-{index:03d}',commands)
            delays=[]
            for actions in streams.values():
                if index+1<len(actions):delays.append(max(0.,actions[index+1]['start_sim_s']-actions[index]['end_sim_s']))
            if delays:t.wait(max(delays))
        final_wait=source_report['evaluation']['sim_time_s']-max(actions[-1]['end_sim_s'] for actions in streams.values())
        t.wait(final_wait)
        result=t.monitor.result()
        report.update(evaluation=result,execution_sim_s=t.time-began,
            serial_execution_sim_s=source_report['evaluation']['sim_time_s']-initial['end_sim_s']-.2,
            parallel_commissioning_passed=bool(all(result.get('assignments',{}).get(e['goal'])==e['object'] for e in jobs) and not t.monitor.violations))
        t.snapshot('final')
    except Exception as error:
        report.update(error=f'{type(error).__name__}: {error}',evaluation=t.monitor.result())
        try:t.snapshot('failure')
        except Exception as image_error:report['snapshot_error']=str(image_error)
    finally:
        report['unresolved_robot_actions']=[r for r,arm in t.arms.items() if arm.job is not None]
        if report['unresolved_robot_actions']:report['parallel_commissioning_passed']=False
        t.close(report)
    print(json.dumps({k:report.get(k) for k in ('parallel_commissioning_passed','error','execution_sim_s','serial_execution_sim_s')}),flush=True)
    return 0 if report['parallel_commissioning_passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--harness',type=Path,default=ROOT/'configs/harness-frontier-v2-servo2.json')
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-mosaic-r13.json');a=p.parse_args()
    if os.environ.get(MARKER)!=str(ROOT):
        reexec_entry(ROOT,a.output,Path(__file__),sys.argv[1:],a.harness,a.catalog,cli_config=True)
    raise SystemExit(run(a))
