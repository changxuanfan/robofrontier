"""Owner-authored finite recovery: observed pickup -> real lift -> reviewed goal.

The live process only receives normal commands and its existing frozen helper
operations. Stop on any failed/rejected/unknown feedback; never replay a physical
step under a new ID. The owner chooses the object, robot and final target.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

from robofrontier.run_setup import write_json
from scripts.plan_mosaic_repair_pick import plan as plan_pick


def run(a):
    directory=a.trial.resolve();review=directory/'review'
    identity=json.loads((review/'observation.json').read_text());world=identity['world_id'];epoch=identity['review_id']
    task=json.loads((directory/'task.json').read_text())
    target=next(g for g in task['goals'] if g['goal_id']==a.goal) if a.goal else dict(position_m=a.target_m)
    color=next(o['color'] for o in task['objects'] if o['id']==a.object)
    if a.goal and target['requires']['color']!=color:raise ValueError('Recovery object must match the owner-selected goal')
    if a.goal and identity['evaluation']['assignments'].get(a.goal)==a.object:
        print('Goal already physically satisfied; no action submitted',flush=True);return
    if a.goal and a.goal in identity['evaluation']['assignments']:raise ValueError('Do not replace an already completed pairing')
    initial_pairs=dict(identity['evaluation']['assignments'])
    for source in (Path(__file__),Path(__file__).with_name('plan_mosaic_repair_pick.py')):
        payload=source.read_bytes();dest=review/'engineering/sources'/(hashlib.sha256(payload).hexdigest()+'.py');dest.parent.mkdir(parents=True,exist_ok=True)
        if not dest.exists():dest.write_bytes(payload)

    def observation(after=None):
        while True:
            status=json.loads((directory/'runner-status.json').read_text())
            if status['phase']!='review':raise RuntimeError('Owner review ended; do not submit more repair work')
            o=json.loads((review/'observation.json').read_text())
            if o['world_id']!=world or o['review_id']!=epoch:raise RuntimeError('Owner review identity changed')
            if after is None or o['sim_time_s']>=after-1e-8:return o
            time.sleep(.1)

    def wait_result(ident,path):
        while True:
            result=review/'results'/f'{ident}.json'
            if result.exists():
                r=json.loads(result.read_text())
                if r['status']!='running':
                    if r['status']!='completed':raise RuntimeError('Repair did not complete; retained original feedback: '+str(r))
                    return observation(r['finished_sim_s'])
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            rejected=review/'rejected'/f'{digest}.json'
            if rejected.exists():raise RuntimeError('Repair request rejected: '+rejected.read_text())
            observation();time.sleep(.1)

    def submit(suffix,operation,**fields):
        ident=a.id+'-'+suffix;path=review/'requests'/f'owner-{ident}.json'
        payload=dict(id=ident,world_id=world,review_id=epoch,operation=operation,**fields)
        if path.exists() and json.loads(path.read_text())!=payload:raise ValueError('Existing request ID must never change parameters')
        if not path.exists():write_json(path,payload)
        return wait_result(ident,path)

    ident=a.id+'-pick';path=review/'requests'/f'owner-{ident}.json'
    if path.exists():raise ValueError('Do not rerun a partially executed recovery; inspect its original IDs')
    plan_pick(SimpleNamespace(trial=directory,robot=a.robot,object=a.object,id=ident,request=path,
        flat_only=getattr(a,'flat_only',False)))
    wait_result(ident,path)
    o=submit('adopt','adopt_hold',robot=a.robot,object=a.object)
    tcp=list(o['robots'][a.robot]['tcp_m']);tcp[2]+=.14
    o=submit('lift','commands',batches=[{a.robot:['move',dict(target_m=tcp,
        quaternion_wxyz=o['robots'][a.robot]['tcp_quaternion_wxyz'],speed_m_s=.035,angular_speed_rad_s=.6)]}])
    if o['objects'][a.object]['position_m'][2]<task['table_height_m']+.08:
        raise RuntimeError('Actual payload lift did not retain the object; leave world for owner review')
    o=submit('place','finish_held',robot=a.robot,target_m=target['position_m'])
    if a.goal and o['evaluation']['assignments'].get(a.goal)!=a.object:raise RuntimeError('Original evaluator did not establish requested goal')
    if not a.goal:
        import numpy as np
        if a.robot in o['held'] or np.linalg.norm(np.asarray(o['objects'][a.object]['position_m'])-target['position_m'])>.01:
            raise RuntimeError('Requested staging release/position not established')
    if any(o['evaluation']['assignments'].get(g)!=name for g,name in initial_pairs.items()):
        raise RuntimeError('Recovery disturbed an earlier completed pairing')
    result=dict(passed=True,robot=a.robot,object=a.object,goal=a.goal,target_m=target['position_m'],
        scope='Owner-selected recovery goal' if a.goal else 'Temporary staging only; not goal completion',matched=o['evaluation']['matched_goals'],
        same_world_id=world,review_id=epoch,previous_pairings_retained=True)
    write_json(review/'engineering'/f'{a.id}-result.json',result);print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--robot',required=True);p.add_argument('--object',required=True);p.add_argument('--id',required=True)
    p.add_argument('--flat-only',action='store_true')
    target=p.add_mutually_exclusive_group(required=True);target.add_argument('--goal');target.add_argument('--target-m',nargs=3,type=float)
    run(p.parse_args())
