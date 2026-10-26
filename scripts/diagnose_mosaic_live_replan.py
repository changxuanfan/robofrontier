"""Read-only reproduction of an older owner's next matching/cache generation.

No request, physics step or live-state assignment. Uses that trial's archived
reference module with measured parked robot/object poses in a scratch mirror.
This checks logical graph consistency, not future physical manipulation safety.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np

from robofrontier.run_setup import write_json
from scripts.plan_mosaic_repair_pick import observed_trial


def inspect(a):
    directory=a.trial.resolve();t,o,payload=observed_trial(directory)
    if o['held'] or any(r['grasp_force_setpoint_n'] is not None for r in o['robots'].values()):
        raise ValueError('First establish actual released free grippers')
    original=directory/'executor-source/robofrontier/suite/mosaic_oracle_v2.py'
    name='robofrontier.suite._recorded_replan_probe'
    spec=importlib.util.spec_from_file_location(name,original);reference=importlib.util.module_from_spec(spec)
    sys.modules[name]=reference;spec.loader.exec_module(reference)
    initial=next(json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()
        if '"event": "action_start"' in line and '"label": "clear-idle-arms"' in line)
    for robot,command in initial['commands'].items():
        q=np.asarray(command[1]['joint_rad'])
        if np.max(abs(np.asarray(o['robots'][robot]['joint_rad'])-q))>.005:
            raise ValueError('Measured arm is not at its original parked reference: '+robot)
        t.arms[robot].qref=q.copy()
    t.actions=[];t.event=lambda *args,**kwargs:None
    t.monitor.result=lambda:dict(o['evaluation']);t.monitor.violations=list(o['evaluation']['violations'])
    old=json.loads((directory/'supply-plan.json').read_text())
    angles={frozenset(row['robots']):row['angled_grasp'] for row in old['table_buffers'] if row.get('angled_grasp')}
    t.mosaic_angled_edges=dict(angles);t.mosaic_edges={frozenset(row['robots']):np.asarray(row['position_m']) for row in old['table_buffers']}
    remaining,available,protected=reference.protected_remaining_inputs(t,o['evaluation']['assignments'])
    began=time.monotonic();print('READ ONLY REPLAN',len(remaining),'remaining',flush=True)
    coverage=reference.goal_endpoint_candidates(t);pickups=reference.pickup_endpoint_candidates(t)
    neighbors,edges=reference.helpers.graph(t.env);mode='direct'
    try:jobs=reference.assign_supply(t,remaining,coverage,pickups,neighbors,available_objects=available)
    except ValueError:
        mode='augmented';neighbors,edges=reference.buffer_graph(t,neighbors,edges)
        jobs=reference.assign_supply(t,remaining,coverage,pickups,neighbors,available_objects=available)
    missing=[sorted(key) for key in angles if key not in edges]
    result=dict(scope='Read-only logical cache consistency prediction from actual parked poses; no request, dynamics or task success certificate',
        world_id=o['world_id'],review_id=o['review_id'],source_sim_s=o['sim_time_s'],
        observation_sha256=hashlib.sha256(payload).hexdigest(),archived_reference_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
        source_observed_robots=o['robots'],source_observed_objects=o['objects'],matching_mode=mode,
        remaining=len(remaining),matched_jobs=len(jobs),old_angled_keys=[sorted(k) for k in angles],
        proposed_keys=[sorted(k) for k in edges],missing_old_angled_keys=missing,
        cache_consistent=not missing,wall_s=time.monotonic()-began)
    write_json(directory/'review/engineering'/f'{a.id}-replan.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_observed_robots','source_observed_objects')}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--id',required=True);inspect(p.parse_args())
