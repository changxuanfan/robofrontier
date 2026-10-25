"""Private source-bound finite-job concurrency, with conservative dependencies.

This is a bounded engineering strategy, not a policy API or optimal planner.
Conflicting swept envelopes retain source order. Any number of independent
ready scopes can run; physical feedback/evaluation remain authoritative.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import mujoco
import numpy as np

from ..motion_recording import MotionReader
from .evaluate import Monitor


def overlap(a,b,reserve=.02):
    """A separating coordinate proves these conservative boxes disjoint."""
    return bool(np.max(np.maximum(np.asarray(a[0])-b[1],np.asarray(b[0])-a[1]))<=2*reserve)


def dependencies(jobs):
    result=[]
    for j,job in enumerate(jobs):
        result.append([i for i,earlier in enumerate(jobs[:j])
            if set(job['robots'])&set(earlier['robots']) or overlap(job['bounds_m'],earlier['bounds_m'])])
    return result


def ready_scopes(predecessors,completed,active=()):
    return [i for i,pred in enumerate(predecessors) if i not in completed and i not in active and set(pred)<=set(completed)]


def build_plan(source):
    source=Path(source).resolve();began=time.monotonic()
    report=json.loads((source/'report.json').read_text())
    if (not report.get('passed') or report.get('error') or report.get('unresolved_robot_actions') or
            report['evaluation']['violations']):raise ValueError('A complete clean physical source witness is required')
    events=[json.loads(line) for line in (source/'trace.jsonl').read_text().splitlines()]
    ends=[e for e in events if e['event']=='goal_attempt']
    actions=json.loads((source/'actions.json').read_text())
    initial=[a for a in actions if a['label']=='clear-idle-arms']
    if len(initial)!=1 or len(ends)!=report['evaluation']['total_goals']:
        raise ValueError('Require one initial parking and one completed scope per cell')
    jobs=[];cursor=1
    for event in ends:
        commands=[]
        while cursor<len(actions) and actions[cursor]['end_sim_s']<=event['sim_time_s']+1e-8:
            action=actions[cursor];cursor+=1
            if any(r['status']!='completed' for r in action['results'].values()):
                raise ValueError('Source needs explicit treatment of failed actions, never blind replay')
            commands.append(action)
        if not commands or report['evaluation']['assignments'].get(event['goal'])!=event['object']:
            raise ValueError('Source scope and actual object pairing disagree')
        robots=sorted({r for action in commands for r in action['commands']})
        jobs.append(dict(goal_id=event['goal'],object=event['object'],path=event['path'],robots=robots,
            actions=commands,end_sim_s=event['sim_time_s'],bounds_m=[np.full(3,np.inf),np.full(3,-np.inf)]))
    if cursor!=len(actions):raise ValueError('Unassigned trailing physical actions')
    model=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));data=mujoco.MjData(model)
    task=json.loads((source/'task.json').read_text());monitor=Monitor(SimpleNamespace(model=model,data=data,task=task,arms=task['arms']))
    physical=(model.geom_contype!=0)|(model.geom_conaffinity!=0)
    geom_sets=[]
    for job in jobs:
        ids=set(monitor.geoms[job['object']])
        for robot in job['robots']:
            ids.update(np.flatnonzero((monitor.geom_owner==monitor.robot_index[robot])&physical))
        geom_sets.append(np.asarray(sorted(ids),int))
    index=0;frames=0
    for stamp,qpos in MotionReader(source/'motion').frames():
        while index+1<len(jobs) and stamp>jobs[index]['end_sim_s']:
            index+=1
        data.qpos[:]=qpos;mujoco.mj_kinematics(model,data)
        ids=geom_sets[index];rotation=data.geom_xmat[ids].reshape(-1,3,3)
        center=data.geom_xpos[ids]+np.einsum('nij,nj->ni',rotation,model.geom_aabb[ids,:3])
        half=np.einsum('nij,nj->ni',abs(rotation),model.geom_aabb[ids,3:])
        box=jobs[index]['bounds_m'];box[0]=np.minimum(box[0],np.min(center-half,axis=0));box[1]=np.maximum(box[1],np.max(center+half,axis=0))
        frames+=1
        if frames%30000==0:print(json.dumps(dict(phase='concurrency-envelope-planning',frames=frames,scope=index+1)),flush=True)
    for job in jobs:job['bounds_m']=[v.tolist() for v in job['bounds_m']]
    pred=dependencies(jobs)
    critical=[0.]*len(jobs)
    for i in range(len(jobs)-1,-1,-1):
        duration=jobs[i]['end_sim_s']-jobs[i]['actions'][0]['start_sim_s']
        critical[i]=duration+max([critical[j] for j in range(i+1,len(jobs)) if i in pred[j]] or [0.])
    return dict(source=str(source),case_id=report['case_id'],jobs=jobs,predecessors=pred,critical_path_weights=critical,
        initial_commands=initial[0]['commands'],source_physical_control_sha256=report['physical_control_sha256'],
        source_environment_logic_sha256=report['environment_logic_sha256'],source_scene_bundle_sha256=report['scene_bundle_sha256'],
        source_report_sha256=hashlib.sha256((source/'report.json').read_bytes()).hexdigest(),
        source_actions_sha256=hashlib.sha256((source/'actions.json').read_bytes()).hexdigest(),
        frames_screened=frames,planning_wall_s=time.monotonic()-began,reserve_per_scope_m=.02,
        scope='Source-order dependencies for shared robots or overlapping sampled full-scope envelopes; no continuous safety or optimality certificate')


def execute_plan(trial,plan):
    if (trial.physical_control_sha256!=plan['source_physical_control_sha256'] or
            trial.environment_logic_sha256!=plan['source_environment_logic_sha256'] or
            trial.source_bundle['sha256']!=plan['source_scene_bundle_sha256']):
        raise ValueError('Concurrent world must match source control/evaluator/assets')
    trial.execute('clear-idle-arms',deepcopy(plan['initial_commands']));trial.wait(.2)
    trial.parking_q={r:np.asarray(c[1]['joint_rad']) for r,c in plan['initial_commands'].items()}
    completed=set();active={};peak=0;rounds=0
    jobs=plan['jobs'];trial.mosaic_protected_assignments={}
    while len(completed)<len(jobs):
        ready=ready_scopes(plan['predecessors'],completed,active)
        for index in sorted(ready,key=lambda i:-plan['critical_path_weights'][i]):
            active[index]=0
        if not active:raise RuntimeError('No enabled scope in acyclic concurrency plan')
        peak=max(peak,len(active))
        trial.mosaic_active_jobs=[dict(goal_id=jobs[i]['goal_id'],object=jobs[i]['object'],path=jobs[i]['path']) for i in active]
        trial.event('mosaic_concurrent_frontier',active=trial.mosaic_active_jobs,completed=len(completed))
        commands={};owners={}
        for i,cursor in active.items():
            for robot,command in jobs[i]['actions'][cursor]['commands'].items():
                if robot in commands:raise RuntimeError('Conflicting robot reservation in concurrency plan')
                commands[robot]=deepcopy(command);owners[robot]=i
        try:
            trial.execute(f'mosaic-concurrent-{rounds:04d}',commands)
        finally:
            # Bookkeeping follows measured grasp/release, never attaches a load.
            for robot,(operation,parameters) in commands.items():
                name=jobs[owners[robot]]['object']
                if operation=='gripper' and parameters.get('mode')=='grasp' and trial.arms[robot].grasp_force is not None and trial.finger_contacts(robot,name):
                    trial.held[robot]=name
                if robot in trial.held and trial.arms[robot].grasp_force is None and trial.monitor.released(trial.monitor.geoms[trial.held[robot]]):
                    trial.held.pop(robot)
        rounds+=1;delays=[];finished=[]
        for i,cursor in active.items():
            actions=jobs[i]['actions'];end=actions[cursor]['end_sim_s'];cursor+=1;active[i]=cursor
            if cursor==len(actions):delays.append(max(0.,jobs[i]['end_sim_s']-end));finished.append(i)
            else:delays.append(max(0.,actions[cursor]['start_sim_s']-end))
        trial.wait(max(delays or [0.]))
        current=trial.monitor.result().get('assignments',{})
        for goal,name in trial.mosaic_protected_assignments.items():
            if current.get(goal)!=name:raise RuntimeError('Concurrent motion disturbed completed pairing '+goal)
        for i in finished:
            if current.get(jobs[i]['goal_id'])!=jobs[i]['object']:
                raise RuntimeError('Concurrent finite command scope did not satisfy original evaluator '+jobs[i]['goal_id'])
            completed.add(i);active.pop(i)
            trial.mosaic_protected_assignments=dict(current)
            print(f"{trial.task['id']} concurrent {len(completed)}/{len(jobs)} validated",flush=True)
            callback=getattr(trial,'mosaic_after_goal',None)
            if callback is not None:callback(trial)
    trial.mosaic_active_jobs=[];trial.wait(2.4)
    return dict(reference_placed=len(completed),total_goals=len(trial.task['goals']),
        commissioning_only=False,concurrent_scope_peak=peak,concurrent_rounds=rounds,
        strategy='Conservative source-bound dependency list scheduling; not proven optimal')
