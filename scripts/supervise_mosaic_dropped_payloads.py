"""Continue reviewed empty-hand payload loss or measured loaded-retract errors.

This external owner helper emits the same finite normal commands as the reviewed
manual recoveries. Frozen native owners never import it. Unexpected feedback,
partial execution, occupied ports or inconsistent old caches leave review open.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from types import SimpleNamespace
import numpy as np

from scripts.execute_mosaic_repair_route import choose
from scripts.plan_mosaic_repair_pick import plan,observed_trial
from robofrontier.suite.oracle import joint_edge_clear
from robofrontier.run_setup import write_json


def select(status,o,task):
    if status['phase']!='review' or (status['world_id'],status['review_id'])!=(o['world_id'],o['review_id']):
        raise ValueError('Require matching current review identity')
    if o['evaluation']['violations']:raise ValueError('Retained process violation requires owner review')
    match=re.fullmatch(r'RuntimeError: Payload (?:contact missing before loaded transit|still drifting during bounded re-observation) ([A-Z]_stock\d+)',status['review_history'][-1]['error'])
    if not match:raise ValueError('New error class requires owner review')
    name=match.group(1);job=o['job']
    if not job or job['object']!=name or list(o['held'].values())!=[name]:raise ValueError('Current job and single tracked payload must agree')
    robot=next(iter(o['held']));arm=o['robots'][robot];body=o['objects'][name]
    if (arm['grasp_force_setpoint_n'] is None or not -.001<=arm['gripper_width_m']<=.012 or
            np.linalg.norm(np.array(arm['tcp_m'])-body['position_m'])<.15):
        raise ValueError('Expected an actually separated empty force-held hand')
    if any(v['grasp_force_setpoint_n'] is not None for r,v in o['robots'].items() if r!=robot):raise ValueError('Another force-held gripper requires owner review')
    if abs(body['position_m'][2]-task['table_height_m']-task['cube_size_m']/2)>.001:
        raise ValueError('Known recovery covers only actual stable table-height cube')
    if max(abs(np.asarray(body['rotation'])[2]))<.999:raise ValueError('Cube not resting on a flat face')
    if any(r['status']!='completed' for a in o['last_actions'] for r in a['results'].values()):raise ValueError('Original native feedback needs owner review')
    if name in o['evaluation']['assignments'].values() or job['goal_id'] in o['evaluation']['assignments']:
        raise ValueError('Do not reuse a completed pairing')
    return robot,name,job['goal_id']



def select_settled_drop(status,o,task):
    if status['phase']!='review' or (status['world_id'],status['review_id'])!=(o['world_id'],o['review_id']) or o['evaluation']['violations']:
        raise ValueError('Require matching compliant review')
    match=re.fullmatch(r'RuntimeError: Payload (?:contact missing before loaded transit|still drifting during bounded re-observation) ([A-Z]_stock\d+)',status['review_history'][-1]['error'])
    if not match:raise ValueError('Not a reviewed dropped-payload error')
    name=match.group(1);job=o['job'];matches=[g for g,n in o['evaluation']['assignments'].items() if n==name]
    if not job or job['object']!=name or list(o['held'].values())!=[name] or len(matches)!=1:raise ValueError('Require the actual tracked object already matched to one original goal')
    robot=next(iter(o['held']));arm=o['robots'][robot]
    if arm['grasp_force_setpoint_n'] is None or not -.001<=arm['gripper_width_m']<=.012 or np.linalg.norm(np.array(arm['tcp_m'])-o['objects'][name]['position_m'])<.15:
        raise ValueError('The matched payload must be separated from an actually empty gripper')
    if any(v['grasp_force_setpoint_n'] is not None for r,v in o['robots'].items() if r!=robot):raise ValueError('Another force gripper remains active')
    if any(v['status']!='completed' for a in o['last_actions'] for v in a['results'].values()):raise ValueError('Original feedback needs review')
    return robot,name,matches[0]


def select_high_receiver(status,o,task):
    if status['phase']!='review' or (status['world_id'],status['review_id'])!=(o['world_id'],o['review_id']) or o['evaluation']['violations']:
        raise ValueError('Require matching compliant review')
    match=re.fullmatch(r'RuntimeError: No transfer pickup with usable continuation ([A-Z])/([A-Z]_stock\d+)',status['review_history'][-1]['error'])
    if not match:raise ValueError('Not the reviewed final-receiver admission class')
    robot,name=match.groups();job=o['job']
    if not job or job['object']!=name or len(job['path'])<2 or job['path'][-1]!=robot:raise ValueError('Require the actual final receiver of the current job')
    if o['held'] or any(v['grasp_force_setpoint_n'] is not None for v in o['robots'].values()):raise ValueError('Require released free grippers')
    if any(v['status']!='completed' for a in o['last_actions'] for v in a['results'].values()):raise ValueError('Original feedback needs review')
    body=o['objects'][name];goal=next(g for g in task['goals'] if g['goal_id']==job['goal_id'])
    if abs(body['position_m'][2]-task['table_height_m']-task['cube_size_m']/2)>.001 or max(abs(np.asarray(body['rotation'])[2]))<.999:
        raise ValueError('High-receiver template covers a flat cube on the actual table')
    if np.linalg.norm(np.array(body['position_m'])[:2]-np.array(goal['position_m'])[:2])<=.01:raise ValueError('Use a local correction for a cube already near its goal')
    if name in o['evaluation']['assignments'].values() or job['goal_id'] in o['evaluation']['assignments']:raise ValueError('Do not rework an accepted pairing')
    return robot,name,job['goal_id']


def select_retract(status,o,task):
    if status['phase']!='review' or (status['world_id'],status['review_id'])!=(o['world_id'],o['review_id']):
        raise ValueError('Require matching current review identity')
    if o['evaluation']['violations']:raise ValueError('Retained process violation requires owner review')
    match=re.fullmatch(r'RuntimeError: Transfer retract infeasible with measured payload ([A-Z]_stock\d+)',status['review_history'][-1]['error'])
    if not match:raise ValueError('Not the reviewed measured-retract class')
    name=match.group(1);job=o['job']
    if not job or job['object']!=name or list(o['held'].values())!=[name]:raise ValueError('Current job and single tracked payload must agree')
    robot=next(iter(o['held']));arm=o['robots'][robot];body=o['objects'][name]
    if (arm['grasp_force_setpoint_n'] is None or not .052<=arm['gripper_width_m']<=.068 or
            np.linalg.norm(np.array(arm['tcp_m'])-body['position_m'])>.07 or
            body['position_m'][2]<=task['table_height_m']+task['cube_size_m']):
        raise ValueError('Require an actually raised close-held cube before the native bilateral support check')
    if any(v['grasp_force_setpoint_n'] is not None for r,v in o['robots'].items() if r!=robot):raise ValueError('Another force-held gripper requires owner review')
    if any(r['status']!='completed' for a in o['last_actions'] for r in a['results'].values()):raise ValueError('Original native feedback needs owner review')
    if name in o['evaluation']['assignments'].values() or job['goal_id'] in o['evaluation']['assignments']:raise ValueError('Do not reuse a completed pairing')
    return robot,name,job['goal_id']


def vertical_clearance_plan(directory,robot,name):
    t,o,payload=observed_trial(directory);arm=t.arms[robot];start=t.data.site_xpos[arm.site].copy();rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()
    q=t.data.qpos[arm.qids].copy();target=start+[0,0,.10];minimum=1.
    for fraction in np.linspace(0,1,101)[1:]:
        n=t.screen_ik(robot,start+(target-start)*fraction,rotation,seed=q)
        if n is None or not joint_edge_clear(q,n,lambda value:t.pose_clear(robot,value,(name,)),step_rad=.002):
            raise ValueError('Measured vertical payload clearance has no screened route; no move emitted')
        minimum=min(minimum,float(np.min(np.minimum(n-arm.ik.limits[:,0],arm.ik.limits[:,1]-n))));q=n
    if minimum<.04:raise ValueError('Measured vertical path lacks the existing private correction margin')
    return dict(source_sim_s=o['sim_time_s'],observation_sha256=hashlib.sha256(payload).hexdigest(),robot=robot,object=name,
        target_m=target.tolist(),quaternion_wxyz=o['robots'][robot]['tcp_quaternion_wxyz'],minimum_joint_margin_rad=minimum,
        full_scene_payload_path_screened=True,joint_step_rad=.002)


def full_original_goal_set(o,task):
    return bool(o['evaluation'].get('task_success') and set(o['evaluation']['assignments'])=={g['goal_id'] for g in task['goals']} and
        not o['held'] and not o['evaluation']['violations'] and all(v['grasp_force_setpoint_n'] is None for v in o['robots'].values()))


def run(a):
    p=a.trial.resolve();review=p/'review';identity=json.loads((p/'process.json').read_text());task=json.loads((p/'task.json').read_text())
    if not identity['git_commit'].startswith(a.expected_source_commit):raise ValueError('Unexpected frozen owner source')
    if task['family']!='mosaic' or len(task['arms'])!=8 or abs(task['cube_size_m']-.06)>1e-10:
        raise ValueError('This reviewed recovery is scoped to the eight-arm60mm Mosaic cases')
    source=Path(__file__).read_bytes();f=review/'engineering/sources'/(hashlib.sha256(source).hexdigest()+'.py');f.parent.mkdir(parents=True,exist_ok=True);f.write_bytes(source)
    world=None;handled=set()
    print('WATCHING REVIEWED PAYLOAD-LOSS AND RETRACT CLASSES',p,flush=True)
    while True:
        s=json.loads((p/'runner-status.json').read_text())
        if world is None:world=s['world_id']
        if s['world_id']!=world:raise ValueError('Owner identity changed')
        if s['phase'] not in ('running','review'):print('OWNER TERMINAL',s['phase'],flush=True);return
        os.kill(identity['pid'],0)
        if s['phase']=='running':time.sleep(1);continue
        o=json.loads((review/'observation.json').read_text())
        if o['review_id']!=s['review_id']:time.sleep(.1);continue
        epoch=o['review_id']
        if epoch in handled:time.sleep(.2);continue
        error=s['review_history'][-1]['error']
        drop_match=re.fullmatch(r'RuntimeError: Payload (?:contact missing before loaded transit|still drifting during bounded re-observation) ([A-Z]_stock\d+)',error)
        settled=bool(drop_match and drop_match.group(1) in o['evaluation']['assignments'].values())
        high_receiver=bool(re.fullmatch(r'RuntimeError: No transfer pickup with usable continuation ([A-Z])/([A-Z]_stock\d+)',error))
        retract=bool(re.fullmatch(r'RuntimeError: Transfer retract infeasible with measured payload ([A-Z]_stock\d+)',error))
        selector=select_settled_drop if settled else select_high_receiver if high_receiver else select_retract if retract else select
        robot,name,goal=selector(s,o,task)
        strategy='settled' if settled else 'high-receiver' if high_receiver else 'retract' if retract else 'drop'
        prefix=f'known-{strategy}-normal-{epoch:03d}';protected=dict(o['evaluation']['assignments'])
        goal_entry=next(g for g in task['goals'] if g['goal_id']==goal)
        if next(v['color'] for v in task['objects'] if v['id']==name)!=goal_entry['requires']['color']:raise ValueError('Current object and original goal colors differ')
        for request in (review/'requests').glob('*.json'):
            q=json.loads(request.read_text())
            if q.get('review_id')!=epoch:continue
            result=review/'results'/f"{q['id']}.json"
            if not result.exists() or json.loads(result.read_text())['status']=='running':raise ValueError('Another request owns this review')
            if q['id'].startswith(prefix):raise ValueError('Never restart a partially executed recovery prefix')

        def observed(after=-1):
            while True:
                state=json.loads((p/'runner-status.json').read_text());v=json.loads((review/'observation.json').read_text())
                if state['phase']!='review' or (v['world_id'],v['review_id'])!=(world,epoch):raise ValueError('Owner review changed')
                if v['evaluation']['violations'] or any(v['evaluation']['assignments'].get(g)!=n for g,n in protected.items()):raise ValueError('Protected original state changed')
                if v['sim_time_s']>=after-1e-8:return v
                time.sleep(.1)

        def submit(suffix,operation,**fields):
            ident=prefix+'-'+suffix;path=review/'requests'/f'owner-{ident}.json'
            if path.exists():raise ValueError('Existing physical ID requires inspection')
            observed();write_json(path,dict(id=ident,world_id=world,review_id=epoch,operation=operation,**fields))
            while True:
                result=review/'results'/f'{ident}.json'
                if result.exists():
                    r=json.loads(result.read_text())
                    if r['status']!='running':
                        if r['status']!='completed':raise ValueError('Original feedback retained: '+str(r))
                        return r if operation=='resume' else observed(r['finished_sim_s'])
                rejected=review/'rejected'/(hashlib.sha256(path.read_bytes()).hexdigest()+'.json')
                if rejected.exists():raise ValueError('Request rejected: '+rejected.read_text())
                if operation!='resume':observed()
                else:
                    state=json.loads((p/'runner-status.json').read_text())
                    if state['world_id']!=world or state['phase'] not in ('running','review'):raise ValueError('Owner ended before resume feedback')
                time.sleep(.1)

        if settled:
            submit('open-empty','commands',batches=[{robot:['gripper',dict(mode='move',width_m=.072,speed_m_s=.04)]}])
            submit('reconcile-qualified','reconcile_released',robot=robot,object=name)
            subprocess.run([sys.executable,'-m','scripts.park_mosaic_review_robot','--trial',str(p),'--robot',robot,'--id',prefix+'-park'],check=True)
            path=[] # No payload motion: retain the actual evaluator's matching.
        elif high_receiver:
            subprocess.run([sys.executable,'-m','scripts.execute_mosaic_repair_transfer','--trial',str(p),'--robot',robot,'--object',name,
                '--goal',goal,'--id',prefix+'-high-pick','--flat-only','--expanded','--aperture','.064','--contact-height','.035'],check=True)
            path=[robot]
        else:
            if retract:
                submit('verify-load','adopt_hold',robot=robot,object=name)
                vertical=vertical_clearance_plan(p,robot,name);write_json(review/'engineering'/f'{prefix}-vertical-clearance-plan.json',vertical)
                submit('vertical-clear','commands',batches=[{robot:['move',dict(target_m=vertical['target_m'],quaternion_wxyz=vertical['quaternion_wxyz'],speed_m_s=.025,angular_speed_rad_s=.2)]}])
                current=observed()
                if current['objects'][name]['position_m'][2]<task['table_height_m']+.08:raise ValueError('Actual vertical lift did not retain payload')
                submit('verify-raised-load','adopt_hold',robot=robot,object=name)
            else:
                submit('open-empty','commands',batches=[{robot:['gripper',dict(mode='move',width_m=.072,speed_m_s=.04)]}])
                submit('reconcile-table','reconcile_released',robot=robot,object=name)
                subprocess.run([sys.executable,'-m','scripts.park_mosaic_review_robot','--trial',str(p),'--robot',robot,'--id',prefix+'-park'],check=True)
            o=observed()
            if name in o['evaluation']['assignments'].values() or goal in o['evaluation']['assignments']:
                raise ValueError('A newly settled pairing already satisfies a goal; rematch before moving it')
            rows=json.loads((p/'supply-plan.json').read_text())['table_buffers'];coverage=json.loads((p/'coverage.json').read_text())['goals'][goal]
            blocked=[frozenset(row['robots']) for row in rows if any(n!=name and np.linalg.norm(np.array(v['position_m'])[:2]-np.array(row['position_m'])[:2])<.08 and abs(v['position_m'][2]-row['position_m'][2])<.15 for n,v in o['objects'].items())]
            choices=[];available=list(o['robots'])
            for candidate in ([robot] if retract else available):
                distance=np.linalg.norm(np.array(task['arms'][candidate]['base_position_m'])[:2]-np.array(o['objects'][name]['position_m'])[:2])
                if not retract and distance>.90:continue
                try:path,ports=choose(rows,candidate,coverage,available,blocked,pickup_position=o['objects'][name]['position_m'],target_position=goal_entry['position_m'])
                except ValueError:continue
                choices.append((len(path),distance,candidate,path,ports))
            chosen=None
            for _,_,candidate,path,ports in sorted(choices,key=lambda x:(x[0],x[1],x[2])):
                if retract:chosen=path,ports;break
                try:
                    plan(SimpleNamespace(trial=p,robot=candidate,object=name,id=prefix+'-candidate-'+candidate,
                        request=review/'engineering'/f'{prefix}-candidate-{candidate}.json',flat_only=True,expanded=True))
                except ValueError as error:
                    print('NO READONLY PICK CANDIDATE',candidate,str(error),flush=True);continue
                chosen=path,ports;break
            if chosen is None:raise ValueError('No screened known normal recovery route; review remains open')
            path,ports=chosen;write_json(review/'engineering'/f'{prefix}-route.json',dict(world_id=world,review_id=epoch,object=name,goal=goal,path=path,
                protected_pairs=protected,scope='Finite observed-state shortest normal-port route among screened initial pickups; not global optimum'))
            locations=[np.asarray(o['objects'][name]['position_m'])[:2]]+[np.asarray(ports[frozenset(edge)])[:2] for edge in zip(path,path[1:])]+[np.asarray(goal_entry['position_m'])[:2]]
            write_json(review/'engineering'/f'{prefix}-transport-objective.json',dict(path=path,minimum_normal_handoffs=len(path)-1,estimated_payload_xy_distance_m=sum(float(np.linalg.norm(y-x)) for x,y in zip(locations,locations[1:])),scope='Shortest normal-port hop count, then less payload travel among destination alternatives; not a global makespan optimum'))
            for index,candidate in enumerate(path):
                target=['--goal',goal] if index==len(path)-1 else ['--target-m',*map(str,ports[frozenset(path[index:index+2])])]
                if retract and index==0:
                    point=goal_entry['position_m'] if len(path)==1 else ports[frozenset(path[:2])]
                    submit('stage-raised-load','finish_held',robot=candidate,target_m=point)
                else:
                    subprocess.run([sys.executable,'-m','scripts.execute_mosaic_repair_transfer','--trial',str(p),'--robot',candidate,'--object',name,
                        '--id',prefix+f'-leg{index}','--flat-only','--expanded',*target],check=True)
        o=observed()
        if o['evaluation']['assignments'].get(goal)!=name or o['held']:raise ValueError('Recovery goal and release must be physically established')
        # The archived solve returns before graph construction when none remain.
        if not full_original_goal_set(o,task):
            subprocess.run([sys.executable,'-m','scripts.diagnose_mosaic_live_replan','--trial',str(p),'--id',prefix+'-verified'],check=True)
            d=json.loads((review/'engineering'/f'{prefix}-verified-replan.json').read_text());o=observed()
            if not d['cache_consistent'] or d['matched_jobs']!=d['remaining']:raise ValueError('Old source cache inconsistent; do not resume')
            if any(v['grasp_force_setpoint_n'] is not None for v in o['robots'].values()):raise ValueError('A force gripper remains active')
            if max(np.linalg.norm(np.array(v['position_m'])-d['source_observed_objects'][n]['position_m']) for n,v in o['objects'].items())>.0005:raise ValueError('Object inputs changed since cache probe')
            if max(np.max(abs(np.array(v['joint_rad'])-d['source_observed_robots'][n]['joint_rad'])) for n,v in o['robots'].items())>.001:raise ValueError('Joint inputs changed since cache probe')
        write_json(review/'engineering'/f'{prefix}-result.json',dict(passed=True,strategy=strategy,original_job_goal=o['job']['goal_id'] if o.get('job') else None,goal=goal,object=name,path=path,matched=o['evaluation']['matched_goals'],previous_pairings_retained=True))
        submit('resume','resume');handled.add(epoch);print('RECOVERED AND RESUMED',epoch,goal,name,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trial',required=True,type=Path);parser.add_argument('--expected-source-commit',required=True)
    run(parser.parse_args())
