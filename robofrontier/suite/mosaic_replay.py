"""Fresh canonical L3 actuator execution of a qualified equivalent L2 recipe."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np

from .mosaic_equivalence import check


def prepare(source,target):
    source=Path(source).resolve();target=Path(target).resolve()
    report=json.loads((source/'report.json').read_text());payload=(source/'actions.json').read_bytes();actions=json.loads(payload)
    if (not report.get('passed') or report.get('error') or report.get('unresolved_robot_actions') or
            report.get('failed_robot_actions')):raise ValueError('Require a complete clean L2 command witness')
    previous=0.
    for action in actions:
        if action['start_sim_s']<previous-1e-8 or any(r['status']!='completed' for r in action['results'].values()):
            raise ValueError('Sequential replay does not silently serialize overlapping/failed source actions')
        previous=action['end_sim_s']
    equivalence=check(source,target)
    if not equivalence['equivalent_for_command_candidate']:raise ValueError('L2/L3 physical equivalence failed: '+str(equivalence['differences']))
    ends=[json.loads(line) for line in (source/'trace.jsonl').read_text().splitlines() if '"event": "goal_attempt"' in line]
    jobs=[];cursor=0
    for action in actions:
        while cursor<len(ends) and action['start_sim_s']>=ends[cursor]['sim_time_s']-1e-8:cursor+=1
        event=ends[cursor] if cursor<len(ends) else None
        jobs.append(None if action['label']=='clear-idle-arms' or event is None else dict(goal_id=event['goal'],object=event['object'],path=event['path']))
    buffers=json.loads((source/'supply-plan.json').read_text())['table_buffers']
    return dict(source=str(source),source_report=report,actions=actions,jobs=jobs,buffers=buffers,equivalence=equivalence,
        actions_sha256=hashlib.sha256(payload).hexdigest(),scope='New live actuator execution with source inter-command waits; no qpos replay, success copying or checkpoint restoration')


def execute(trial,recipe):
    report=recipe['source_report']
    if trial.physical_control_sha256!=report['physical_control_sha256'] or trial.environment_logic_sha256!=report['environment_logic_sha256']:
        raise ValueError('Equivalent replay controller/evaluator changed')
    trial.mosaic_edges={frozenset(row['robots']):np.asarray(row['position_m']) for row in recipe['buffers']}
    trial.parking_q={r:np.asarray(command[1]['joint_rad']) for r,command in recipe['actions'][0]['commands'].items()}
    trial.mosaic_protected_assignments={};previous=recipe['actions'][0]['start_sim_s'];count=0
    for index,action in enumerate(recipe['actions']):
        trial.mosaic_current_job=recipe['jobs'][index]
        trial.wait(max(0.,action['start_sim_s']-previous))
        commands=deepcopy(action['commands'])
        try:trial.execute('equivalent-'+action['label'],commands)
        finally:
            job=trial.mosaic_current_job
            for robot,(operation,parameters) in commands.items():
                if job and operation=='gripper' and parameters.get('mode')=='grasp' and trial.arms[robot].grasp_force is not None and trial.finger_contacts(robot,job['object']):
                    trial.held[robot]=job['object']
                if robot in trial.held and trial.arms[robot].grasp_force is None and trial.monitor.released(trial.monitor.geoms[trial.held[robot]]):
                    trial.held.pop(robot)
        previous=action['end_sim_s']
        current=trial.monitor.result().get('assignments',{})
        for goal,name in trial.mosaic_protected_assignments.items():
            if current.get(goal)!=name:raise RuntimeError('Equivalent replay disturbed completed pairing '+goal)
        trial.mosaic_protected_assignments=dict(current)
        if len(current)>count:
            count=len(current);print(f"{trial.task['id']} equivalent {count}/{len(trial.task['goals'])} validated",flush=True)
            callback=getattr(trial,'mosaic_after_goal',None)
            if callback is not None:callback(trial)
    trial.wait(max(0.,report['evaluation']['sim_time_s']-previous))
    trial.mosaic_current_job=None
    return dict(reference_placed=len(trial.monitor.result().get('assignments',{})),total_goals=len(trial.task['goals']),
        commissioning_only=False,fresh_canonical_command_replay=True)
