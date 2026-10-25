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
from .mosaic_source_scopes import source_scopes


def overlap(a,b,reserve=.02):
    """A separating coordinate proves these conservative boxes disjoint."""
    return bool(np.max(np.maximum(np.asarray(a[0])-b[1],np.asarray(b[0])-a[1]))<=2*reserve)


def dependencies(jobs):
    result=[]
    for j,job in enumerate(jobs):
        result.append([i for i,earlier in enumerate(jobs[:j])
            if (job.get('barrier') or earlier.get('barrier') or
                set(job['robots'])&set(earlier['robots']) or
                (job.get('object') is not None and job.get('object')==earlier.get('object')) or
                overlap(job['bounds_m'],earlier['bounds_m']))])
    return result


def ready_scopes(predecessors,completed,active=()):
    completed=set(completed);active=set(active)
    return [i for i,pred in enumerate(predecessors) if i not in completed and i not in active and set(pred)<=completed]


class ReadyFrontier:
    """Visit each dependency edge once, never rescan it each physics step."""
    def __init__(self,predecessors):
        self.remaining=[len(p) for p in predecessors]
        self.successors=[[] for _ in predecessors]
        for node,pred in enumerate(predecessors):
            for parent in pred:self.successors[parent].append(node)
        self.ready={i for i,n in enumerate(self.remaining) if n==0}
        self.admitted=set();self.completed=set()

    def claim_ready(self):
        result=list(self.ready);self.ready.clear();self.admitted.update(result)
        return result

    def complete(self,node):
        if node not in self.admitted or node in self.completed:
            raise ValueError('Completion does not match an admitted unfinished operation')
        self.completed.add(node)
        for child in self.successors[node]:
            self.remaining[child]-=1
            if self.remaining[child]==0:self.ready.add(child)


def build_plan(source):
    source=Path(source).resolve();began=time.monotonic()
    report=json.loads((source/'report.json').read_text())
    if (not report.get('passed') or report.get('error') or report.get('unresolved_robot_actions') or
            report['evaluation']['violations']):raise ValueError('A complete clean physical source witness is required')
    if len(report['evaluation']['assignments'])!=report['evaluation']['total_goals']:
        raise ValueError('Every original source goal must have a physical assignment')
    events=[json.loads(line) for line in (source/'trace.jsonl').read_text().splitlines()]
    actions=json.loads((source/'actions.json').read_text())
    task=json.loads((source/'task.json').read_text())
    initial,scopes=source_scopes(actions,events,report['evaluation']['assignments'],[o['id'] for o in task['objects']])
    # Reserve individual finite operations rather than a whole multi-arm job.
    # Object order retains the actual custody/staging chain; disjoint robots
    # can prepare/execute other jobs between its operations.
    last_nodes={};last_pairs={};jobs=[]
    for scope in scopes:
        for i,action in enumerate(scope['actions']):
            end=(scope['actions'][i+1]['start_sim_s'] if i+1<len(scope['actions']) else scope['end_sim_s'])
            jobs.append(dict(goal_id=scope['goal_id'],object=scope['object'],path=scope['path'],
                objects=scope['objects'],barrier=scope['barrier'],
                robots=sorted(action['commands']),actions=[action],end_sim_s=end,
                bounds_m=[np.full(3,np.inf),np.full(3,-np.inf)]))
        last_nodes[len(jobs)-1]=scope['goal_id']
        last_pairs[len(jobs)-1]=scope['expected_assignments']
    model=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));data=mujoco.MjData(model)
    task=json.loads((source/'task.json').read_text());monitor=Monitor(SimpleNamespace(model=model,data=data,task=task,arms=task['arms']))
    physical=(model.geom_contype!=0)|(model.geom_conaffinity!=0)
    geom_sets=[]
    for job in jobs:
        ids={g for name in job['objects'] for g in monitor.geoms[name]}
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
    durations=[j['end_sim_s']-j['actions'][0]['start_sim_s'] for j in jobs]
    critical=list(durations)
    for j in range(len(jobs)-1,-1,-1):
        for i in pred[j]:critical[i]=max(critical[i],durations[i]+critical[j])
    return dict(source=str(source),case_id=report['case_id'],jobs=jobs,predecessors=pred,critical_path_weights=critical,
        table_buffers=json.loads((source/'supply-plan.json').read_text())['table_buffers'],
        initial_commands=initial['commands'],source_physical_control_sha256=report['physical_control_sha256'],
        source_environment_logic_sha256=report['environment_logic_sha256'],source_scene_bundle_sha256=report['scene_bundle_sha256'],
        source_report_sha256=hashlib.sha256((source/'report.json').read_bytes()).hexdigest(),
        source_assisted=bool(report.get('assisted') or report.get('llm_used')),
        source_actions_sha256=hashlib.sha256((source/'actions.json').read_bytes()).hexdigest(),
        last_goal_nodes=last_nodes,last_goal_assignments=last_pairs,total_goals=report['evaluation']['total_goals'],
        repair_barrier_count=sum(s['barrier'] for s in scopes),planning_granularity='ordinary_finite_action',
        frames_screened=frames,planning_wall_s=time.monotonic()-began,reserve_per_scope_m=.02,
        scope='Source-order dependencies for shared robots, object custody or overlapping sampled operation envelopes; no continuous safety or optimality certificate')


def execute_plan(trial,plan):
    if (trial.physical_control_sha256!=plan['source_physical_control_sha256'] or
            trial.environment_logic_sha256!=plan['source_environment_logic_sha256'] or
            trial.source_bundle['sha256']!=plan.get('target_scene_bundle_sha256',plan['source_scene_bundle_sha256'])):
        raise ValueError('Concurrent world must match source control/evaluator/assets')
    trial.execute('clear-idle-arms',deepcopy(plan['initial_commands']));trial.wait(.2)
    trial.parking_q={r:np.asarray(c[1]['joint_rad']) for r,c in plan['initial_commands'].items()}
    trial.mosaic_edges={frozenset(row['robots']):np.asarray(row['position_m']) for row in plan.get('table_buffers',[])}
    frontier=ReadyFrontier(plan['predecessors'])
    completed=frontier.completed;active={};peak=0;rounds=0;qualified=set();started={}
    jobs=plan['jobs'];trial.mosaic_protected_assignments={}
    last_goal_nodes={int(i):goal for i,goal in plan['last_goal_nodes'].items()}
    while len(completed)<len(jobs):
        ready=frontier.claim_ready()
        for index in sorted(ready,key=lambda i:-plan['critical_path_weights'][i]):
            active[index]=0
            job=jobs[index];started[job['goal_id']]=dict(goal_id=job['goal_id'],object=job['object'],path=job['path'])
        if not active:raise RuntimeError('No enabled scope in acyclic concurrency plan')
        peak=max(peak,len(active))
        trial.mosaic_active_jobs=list(started.values())
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
                if name is not None and operation=='gripper' and parameters.get('mode')=='grasp' and trial.arms[robot].grasp_force is not None and trial.finger_contacts(robot,name):
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
            frontier.complete(i);active.pop(i)
            if i in last_goal_nodes:
                goal=last_goal_nodes[i]
                pairs=plan.get('last_goal_assignments',{}).get(str(i),plan.get('last_goal_assignments',{}).get(i,{goal:jobs[i]['object']}))
                if any(current.get(g)!=name for g,name in pairs.items()):
                    raise RuntimeError('Concurrent finite command scope did not satisfy original evaluator '+goal)
                qualified.update(pairs);started.pop(goal,None)
                trial.mosaic_protected_assignments=dict(current)
                print(f"{trial.task['id']} concurrent {len(qualified)}/{plan['total_goals']} validated",flush=True)
                callback=getattr(trial,'mosaic_after_goal',None)
                if callback is not None:callback(trial)
    trial.mosaic_active_jobs=[];trial.wait(2.4)
    return dict(reference_placed=len(qualified),total_goals=len(trial.task['goals']),
        commissioning_only=False,concurrent_scope_peak=peak,concurrent_rounds=rounds,
        strategy='Conservative source-bound dependency list scheduling; not proven optimal')
