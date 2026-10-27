"""Actuate an empty reviewed arm back to its original parked joint reference."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from scripts.plan_mosaic_repair_pick import observed_trial
from robofrontier.suite.oracle import joint_edge_clear
from robofrontier.suite.joint_route import connect_route
from robofrontier.run_setup import write_json


def run(a):
    directory=a.trial.resolve();review=directory/'review';t,o,payload=observed_trial(directory);robot=a.robot
    if o['held'] or any(v['grasp_force_setpoint_n'] is not None for v in o['robots'].values()):
        raise ValueError('Park only with actually released, known idle arms')
    initial=next(json.loads(l) for l in (directory/'trace.jsonl').read_text().splitlines()
        if '"event": "action_start"' in l and '"label": "clear-idle-arms"' in l)
    goal=np.asarray(initial['commands'][robot][1]['joint_rad']);arm=t.arms[robot];start=t.data.qpos[arm.qids].copy()
    edge=lambda x,y:joint_edge_clear(x,y,lambda q:t.pose_clear(robot,q),step_rad=.005)
    path=[start,goal] if edge(start,goal) else connect_route(start,goal,arm.ik.limits,edge,np.random.default_rng(9901),iterations=800)
    if path is None:raise ValueError('No screened empty-arm parking route; no request emitted')
    points=[]
    for x,y in zip(path,path[1:]):
        count=max(1,int(np.ceil(np.max(abs(y-x))/1.5)))
        points.extend(x+(y-x)*fraction for fraction in np.linspace(0,1,count+1)[1:])
    if len(points)>16:raise ValueError('Split this reviewed route explicitly before issuing more than16 batches')
    request=review/'requests'/f'park-{a.id}.json'
    if request.exists():raise ValueError('Existing physical parking ID must be inspected, never replayed')
    source=Path(__file__).read_bytes();archive=review/'engineering/sources'/(hashlib.sha256(source).hexdigest()+'.py')
    archive.parent.mkdir(parents=True,exist_ok=True);archive.write_bytes(source)
    write_json(review/'engineering'/f'{a.id}-park-plan.json',dict(source_sim_s=o['sim_time_s'],observation_sha256=hashlib.sha256(payload).hexdigest(),
        robot=robot,path=[q.tolist() for q in points],joint_sample_step_rad=.005,max_command_delta_rad=1.5,
        scope='Scratch full-arm route followed by original native moves; never a pose reset'))
    write_json(request,dict(id=a.id,world_id=o['world_id'],review_id=o['review_id'],operation='commands',
        batches=[{robot:['move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.2)]} for q in points]))
    while True:
        status=json.loads((directory/'runner-status.json').read_text())
        if status['phase']!='review':raise RuntimeError('Owner review ended')
        result=review/'results'/f'{a.id}.json'
        if result.exists():
            r=json.loads(result.read_text())
            if r['status']!='running':
                if r['status']!='completed':raise RuntimeError('Original parking feedback retained: '+str(r))
                current=json.loads((review/'observation.json').read_text())
                if current['sim_time_s']>=r['finished_sim_s']-1e-8:break
        time.sleep(.1)
    if any(current['evaluation']['assignments'].get(g)!=v for g,v in o['evaluation']['assignments'].items()):
        raise RuntimeError('Parking changed an accepted pair')
    if np.max(abs(np.asarray(current['robots'][robot]['joint_rad'])-goal))>.005:raise RuntimeError('Actual parked joint target not verified')
    print('ACTUAL PARK VERIFIED',robot,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--robot',required=True);p.add_argument('--id',required=True);run(p.parse_args())
