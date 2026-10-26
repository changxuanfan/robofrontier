"""Finish reviewed unique jobs through finite native commands in the same world.

This avoids rerunning an older live executor's stale planning graph. Only the
original complete evaluator can authorize final resume; it then takes the
empty-remaining branch. No live code or physics state is changed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

from robofrontier.run_setup import write_json


def legs(job,ports,goal):
    path=job['path'];result=[]
    for i,robot in enumerate(path):
        if i==len(path)-1:result.append((robot,'goal',goal['goal_id']))
        else:
            key=frozenset(path[i:i+2])
            if key not in ports:raise ValueError('A reviewed normal port is required for this job')
            result.append((robot,'stage',ports[key]))
    return result


def run(a):
    directory=a.trial.resolve();review=directory/'review';source=a.supply_plan.resolve()
    task=json.loads((directory/'task.json').read_text());source_task=json.loads((source.parent/'task.json').read_text())
    if task['scene_sha256']!=source_task['scene_sha256']:raise ValueError('Unique source job witness must bind the same scene')
    data=json.loads(source.read_text());jobs=data['jobs'];goals={g['goal_id']:g for g in task['goals']}
    if len({j['goal'] for j in jobs})!=len(jobs) or len({j['object'] for j in jobs})!=len(jobs):
        raise ValueError('Original goal/object jobs must be unique')
    ports={frozenset(row['robots']):row['position_m'] for row in data['table_buffers'] if not row.get('angled_grasp')}
    o=json.loads((review/'observation.json').read_text());world=o['world_id'];epoch=o['review_id'];protected=dict(o['evaluation']['assignments'])
    if any(json.loads(p.read_text()).get('id','').startswith(a.id+'-') for p in (review/'requests').glob('*.json')):
        raise ValueError('Do not restart a partially executed physical completion batch')
    payload=Path(__file__).read_bytes();archive=review/'engineering/sources'/(hashlib.sha256(payload).hexdigest()+'.py')
    archive.parent.mkdir(parents=True,exist_ok=True);archive.write_bytes(payload)
    write_json(review/'engineering'/f'{a.id}-input.json',dict(world_id=world,review_id=epoch,
        original_supply_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),protected_pairs=protected,
        scope='Same-world finite native commands; final resume only after original full task success'))

    def observation(free=True):
        status=json.loads((directory/'runner-status.json').read_text());o=json.loads((review/'observation.json').read_text())
        if status['phase']!='review' or o['world_id']!=world or o['review_id']!=epoch:raise RuntimeError('Physical owner review changed')
        if o['evaluation']['violations'] or any(o['evaluation']['assignments'].get(g)!=name for g,name in protected.items()):
            raise RuntimeError('A retained completed pairing changed; no next job admitted')
        if free and (o['held'] or any(r['grasp_force_setpoint_n'] is not None for r in o['robots'].values())):
            raise RuntimeError('Inspect actual custody before starting another job')
        return o

    def child(command,ident):
        r=subprocess.run(command,text=True,capture_output=True)
        (review/'engineering'/f'{ident}-helper.log').write_text(r.stdout+'\n'+r.stderr)
        return r

    def correct(job,robot,ident):
        for height in ((.038,.040) if task['level']==1 else (.035,.038,.040)):
            cid=ident+f'-correct{round(height*1000)}'
            r=child([sys.executable,str(Path(__file__).with_name('execute_mosaic_flat_pose_correction.py')),
                '--trial',str(directory),'--robot',robot,'--object',job['object'],'--goal',job['goal'],
                '--id',cid,'--contact-height',str(height)],cid)
            if r.returncode==0:return
            issued=any(json.loads(p.read_text()).get('id','').startswith(cid+'-') for p in (review/'requests').glob('*.json'))
            if issued or 'No finite screened repair pickup candidate; no command emitted' not in r.stderr:
                raise RuntimeError('Correction needs owner review: '+str(review/'engineering'/f'{cid}-helper.log'))
        raise RuntimeError('No screened high correction candidate; keep physical review open')

    pending=[j for j in jobs if j['goal'] not in protected]
    if set(goals)-set(protected)-{j['goal'] for j in pending}:raise ValueError('Source job list does not cover every remaining goal')
    round_index=0
    while pending:
        progress=False;deferred=[]
        for job in sorted(pending,key=lambda j:len(j['path'])):
            o=observation()
            if job['goal'] in o['evaluation']['assignments']:
                if o['evaluation']['assignments'][job['goal']]!=job['object']:raise ValueError('Pairing differs from the unique source witness')
                continue
            if job['object'] in o['evaluation']['assignments'].values():raise ValueError('Never recycle a completed object')
            ident=a.id+f'-r{round_index}-{job["goal"]}';began=False;blocked=False
            for index,(robot,kind,target) in enumerate(legs(job,ports,goals[job['goal']])):
                o=observation()
                if kind=='stage' and any(name!=job['object'] and np.linalg.norm(np.asarray(b['position_m'])[:2]-np.asarray(target)[:2])<.08 and abs(b['position_m'][2]-target[2])<.15 for name,b in o['objects'].items()):
                    if began:raise RuntimeError('Intermediate port occupied after partial work')
                    blocked=True;break
                leg=ident+f'-leg{index}'
                command=[sys.executable,str(Path(__file__).with_name('execute_mosaic_repair_transfer.py')),
                    '--trial',str(directory),'--robot',robot,'--object',job['object'],'--id',leg,'--flat-only']
                if a.expanded_pickups:command+=['--expanded']
                command+=['--goal',target] if kind=='goal' else ['--target-m',*map(str,target)]
                r=child(command,leg)
                if r.returncode:
                    observation()
                    issued=(review/'requests'/f'owner-{leg}-pick.json').exists()
                    if not began and not issued and 'No finite screened repair pickup candidate; no command emitted' in r.stderr:
                        blocked=True;break
                    if kind=='goal' and 'Original evaluator did not establish requested goal' in r.stderr:
                        correct(job,robot,leg);began=True;continue
                    raise RuntimeError('Physical leg needs owner review; original IDs retained: '+str(review/'engineering'/f'{leg}-helper.log'))
                began=True
            if blocked:deferred.append(job);continue
            o=observation()
            if o['evaluation']['assignments'].get(job['goal'])!=job['object']:raise RuntimeError('Original evaluator did not accept the full job')
            protected=dict(o['evaluation']['assignments']);progress=True
            write_json(review/'engineering'/f'{a.id}-progress.json',dict(world_id=world,review_id=epoch,matched=len(protected),
                completed_pairs=protected,remaining=[j['goal'] for j in pending if j['goal'] not in protected]))
            print('SAME WORLD VERIFIED',job['goal'],len(protected),'/',len(goals),flush=True)
        if deferred and not progress:raise RuntimeError('No new executable prefix in this finite proposal set; owner replan needed')
        pending=deferred;round_index+=1
    o=observation()
    if not o['evaluation'].get('task_success') or len(o['evaluation']['assignments'])!=len(goals):
        raise RuntimeError('Only the original complete evaluator can authorize final resume')
    write_json(review/'engineering'/f'{a.id}-result.json',dict(passed=True,world_id=world,review_id=epoch,
        matched=len(protected),original_task_success=True,scope='All original goals physically verified before final no-remaining resume'))
    if a.resume_at_full:
        ident=a.id+'-final-resume';write_json(review/'requests'/f'zz-{ident}.json',
            dict(id=ident,world_id=world,review_id=epoch,operation='resume'))
    print('ALL ORIGINAL GOALS VERIFIED',len(protected),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--supply-plan',required=True,type=Path);p.add_argument('--id',required=True)
    p.add_argument('--expanded-pickups',action='store_true')
    p.add_argument('--resume-at-full',action='store_true');run(p.parse_args())
