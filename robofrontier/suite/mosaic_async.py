"""Single physics owner, independently polled finite Mosaic operations.

Starts only the source plan's enabled nonconflicting operations. A shorter
operation can enable its successor while unrelated arms are still executing.
Known failures stop new admission and drain admitted work before owner review;
uncertain execution is fenced and never replayed.
"""
from copy import deepcopy
import numpy as np


def execute_plan(trial,plan,*,initialize=True):
    if (trial.physical_control_sha256!=plan['source_physical_control_sha256'] or
            trial.environment_logic_sha256!=plan['source_environment_logic_sha256'] or
            trial.source_bundle['sha256']!=plan.get('target_scene_bundle_sha256',plan['source_scene_bundle_sha256'])):
        raise ValueError('Async world must match source control/evaluator/assets')
    from .mosaic_concurrency import ReadyFrontier
    if initialize:
        trial.execute('clear-idle-arms',deepcopy(plan['initial_commands']));trial.wait(.2)
        trial.mosaic_concurrency_started_sim_s=trial.time
        trial.parking_q={r:np.asarray(c[1]['joint_rad']) for r,c in plan['initial_commands'].items()}
    elif not hasattr(trial,'parking_q') or trial.held or any(a.job is not None or a.grasp_force is not None for a in trial.arms.values()):
        raise ValueError('Continuation requires known idle released arms; never reset or repark')
    trial.mosaic_edges={frozenset(row['robots']):np.asarray(row['position_m']) for row in plan.get('table_buffers',[])}
    jobs=plan['jobs'];last_nodes={int(i):g for i,g in plan['last_goal_nodes'].items()}
    if any(len(j['actions'])!=1 for j in jobs):raise ValueError('Async admission requires operation-level nodes')
    frontier=ReadyFrontier(plan['predecessors'])
    completed=frontier.completed;active={};waiting={};qualified=set();started={};failures=[];peak=0
    if initialize:trial.mosaic_protected_assignments={}
    lower=np.asarray(trial.task['workspace_world_m']['lower']);upper=np.asarray(trial.task['workspace_world_m']['upper'])

    def bookkeeping(robot,command,node):
        operation,parameters=command;name=jobs[node]['object']
        if name is not None and operation=='gripper' and parameters.get('mode')=='grasp' and trial.arms[robot].grasp_force is not None and trial.finger_contacts(robot,name):
            trial.held[robot]=name
        if robot in trial.held and trial.arms[robot].grasp_force is None and trial.monitor.released(trial.monitor.geoms[trial.held[robot]]):
            trial.held.pop(robot)

    while len(completed)<len(jobs):
        trial.check_stop()
        current=trial.monitor.result().get('assignments',{})
        for goal,name in trial.mosaic_protected_assignments.items():
            if current.get(goal)!=name:
                failures.append('Async motion disturbed completed pairing '+goal)
        for node,ready_time in list(waiting.items()):
            if trial.time+1e-9<ready_time:continue
            waiting.pop(node);frontier.complete(node)
            if node in last_nodes:
                goal=last_nodes[node]
                pairs=plan.get('last_goal_assignments',{}).get(str(node),plan.get('last_goal_assignments',{}).get(node,{goal:jobs[node]['object']}))
                if any(current.get(g)!=name for g,name in pairs.items()):
                    failures.append('Async scope did not satisfy original evaluator '+goal)
                else:
                    qualified.update(pairs);started.pop(goal,None)
                    trial.mosaic_protected_assignments=dict(current)
                    print(f"{trial.task['id']} async {len(qualified)}/{plan['total_goals']} validated",flush=True)
                    callback=getattr(trial,'mosaic_after_goal',None)
                    if callback is not None:callback(trial)
        trial.mosaic_active_jobs=list(started.values())
        if failures and not active:
            raise RuntimeError('Known async failure after draining admitted work: '+failures[0])
        if not failures:
            ready=frontier.claim_ready()
            prepared={};robots=set()
            try:
                for node in sorted(ready,key=lambda i:-plan['critical_path_weights'][i]):
                    command=jobs[node]['actions'][0]['commands'];validated={}
                    for robot,(operation,parameters) in command.items():
                        if robot in robots or trial.arms[robot].job is not None:
                            raise RuntimeError('Source dependency plan admitted a busy robot')
                        validated[robot]=trial.arms[robot].validate(operation,parameters,trial.cfg,lower,upper)
                        robots.add(robot)
                    prepared[node]=validated
            except (ValueError,RuntimeError) as error:
                failures.append('Admission: '+str(error));prepared={}
                trial.event('mosaic_async_admission_rejected',reason=str(error))
            for node,validated in prepared.items():
                job=jobs[node];command=deepcopy(job['actions'][0]['commands']);label=f"{plan.get('operation_prefix','mosaic-async')}-{node:04d}"
                active[node]=dict(commands=command,label=label,start_sim_s=trial.time,pending=set(command),results={})
                started[job['goal_id']]=dict(goal_id=job['goal_id'],object=job['object'],path=job['path'])
                trial.event('action_start',label=label,commands=command)
                for robot,parameters in validated.items():
                    call_id=f'{label}-{robot}'
                    try:trial.arms[robot].start(call_id,parameters,trial.time)
                    except Exception as error:
                        result=dict(call_id=call_id,status='uncertain',reason='Async start acknowledgement failed: '+str(error))
                        trial.actions.append(dict(label=label,commands=command,results={robot:result},start_sim_s=trial.time,end_sim_s=trial.time))
                        trial.event('mosaic_async_unknown_start',robot=robot,result=result)
                        raise RuntimeError('Unknown async startup; original IDs retained') from error
            peak=max(peak,len(active))
            trial.mosaic_active_jobs=list(started.values())
        if not active and not waiting and len(completed)<len(jobs):
            raise RuntimeError('No enabled async operation')
        if len(completed)==len(jobs):break
        trial.step()
        if trial.tick%trial.stride:continue
        for node,record in list(active.items()):
            for robot in list(record['pending']):
                result=trial.arms[robot].poll(trial.time,15)
                if result is None:continue
                record['results'][robot]=deepcopy(result);record['pending'].remove(robot)
                bookkeeping(robot,record['commands'][robot],node)
                if result['status']=='uncertain':
                    trial.actions.append(dict(label=record['label'],commands=record['commands'],results=deepcopy(record['results']),start_sim_s=record['start_sim_s'],end_sim_s=trial.time))
                    trial.event('action_end',label=record['label'],results=record['results'])
                    raise RuntimeError('Unknown async execution; no replay permitted')
                if result['status']!='completed':failures.append(str(result))
            if record['pending']:continue
            trial.actions.append(dict(label=record['label'],commands=record['commands'],results=record['results'],start_sim_s=record['start_sim_s'],end_sim_s=trial.time))
            trial.event('action_end',label=record['label'],results=record['results'])
            active.pop(node)
            source=jobs[node]['actions'][0]
            waiting[node]=trial.time+jobs[node].get('wait_after_s',max(0.,jobs[node]['end_sim_s']-source['end_sim_s']))
    trial.mosaic_active_jobs=[];trial.wait(2.4)
    return dict(reference_placed=len(qualified),total_goals=len(trial.task['goals']),commissioning_only=False,
        concurrent_operation_peak=peak,strategy='Independent native controller polling with source-bound dependencies; not proven optimal')
