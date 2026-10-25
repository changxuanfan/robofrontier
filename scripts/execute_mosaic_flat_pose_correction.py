"""Finite observed-state correction of a slightly yawed, seated planar cube.

Only ordinary native operations go to the existing frozen owner. Every segment
is screened and its actual feedback checked. Failure leaves the owner review
open; no physical request is replayed and no world state is assigned.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np

from robofrontier.robot_control import matrix_quat
from robofrontier.run_setup import write_json
from robofrontier.suite.oracle import joint_edge_clear
from scripts.plan_mosaic_repair_pick import observed_trial,plan as pick


def rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def original_support_contact(name,distance,normal):
    return name in ('table_0','pocket_bottom') and distance>=-.0002 and abs(normal[2])>.99


def run(a):
    directory=a.trial.resolve();review=directory/'review';t,initial,_=observed_trial(directory)
    goal=next(g for g in t.task['goals'] if g['goal_id']==a.goal);name=a.object;robot=a.robot
    if t.task.get('family')!='mosaic' or abs(t.task['cube_size_m']-.06)>1e-10:
        raise ValueError('This finite correction covers only the declared60mm Mosaic cubes')
    if any(arm['grasp_force_setpoint_n'] is not None for arm in initial['robots'].values()) or initial['held']:
        raise ValueError('Begin this correction only with released free grippers')
    if a.goal in initial['evaluation']['assignments']:raise ValueError('Do not rework an accepted goal')
    body=initial['objects'][name];rotation=np.asarray(body['rotation']);position=np.asarray(body['position_m'])
    if (np.linalg.norm(position[:2]-np.asarray(goal['position_m'])[:2])>.005 or
            abs(position[2]-goal['position_m'][2])>.003 or rotation[2,2]<.999):
        raise ValueError('This finite in-place correction requires the actual seated planar pose')
    color=next(o['color'] for o in t.task['objects'] if o['id']==name)
    if color!=goal['requires']['color']:raise ValueError('Selected object color differs from the original goal')
    yaw=math.atan2(rotation[1,0],rotation[0,0]);delta=round(yaw/(math.pi/2))*math.pi/2-yaw
    if not math.radians(8)<abs(delta)<math.radians(12):raise ValueError('Select the demonstrated small yaw correction class')
    world=initial['world_id'];epoch=initial['review_id'];protected=dict(initial['evaluation']['assignments'])
    if any(json.loads(p.read_text()).get('id','').startswith(a.id+'-') for p in (review/'requests').glob('*.json')):
        raise ValueError('Existing physical request prefix: inspect its original results, never restart it')
    for source in (Path(__file__),Path(__file__).with_name('plan_mosaic_repair_pick.py')):
        payload=source.read_bytes();dest=review/'engineering/sources'/(hashlib.sha256(payload).hexdigest()+'.py')
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(payload)

    def observation(after=-1.):
        while True:
            status=json.loads((directory/'runner-status.json').read_text());o=json.loads((review/'observation.json').read_text())
            if status['phase']!='review' or o['world_id']!=world or o['review_id']!=epoch:raise RuntimeError('Actual owner review changed')
            if o['evaluation']['violations'] or any(o['evaluation']['assignments'].get(g)!=v for g,v in protected.items()):
                raise RuntimeError('Earlier verified state changed; leave owner review open')
            if o['sim_time_s']>=after-1e-8:return o
            time.sleep(.1)

    def wait(ident,path):
        while True:
            f=review/'results'/f'{ident}.json'
            if f.exists():
                r=json.loads(f.read_text())
                if r['status']!='running':
                    if r['status']!='completed':raise RuntimeError('Original physical feedback retained: '+str(r))
                    return observation(r['finished_sim_s'])
            rejected=review/'rejected'/(hashlib.sha256(path.read_bytes()).hexdigest()+'.json')
            if rejected.exists():raise RuntimeError('Original request rejected: '+rejected.read_text())
            observation();time.sleep(.1)

    def submit(suffix,operation,**fields):
        observation();ident=a.id+'-'+suffix;path=review/'requests'/f'pose-{ident}.json'
        if path.exists():raise ValueError('Never repeat a partial physical correction')
        write_json(path,dict(id=ident,world_id=world,review_id=epoch,operation=operation,**fields))
        return wait(ident,path)

    def move(suffix,point,rotation,speed=.012,omega=.1,support=False):
        mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];start=mirror.data.site_xpos[arm.site].copy()
        old_r=mirror.data.site_xmat[arm.site].reshape(3,3).copy();q=mirror.data.qpos[arm.qids].copy()
        # Rotation is either unchanged or a small yaw about the measured body.
        angle=math.atan2((rotation@old_r.T)[1,0],(rotation@old_r.T)[0,0])
        gids=set(mirror.monitor.geoms[name]);rid=mirror.monitor.robot_index[robot]
        def clear(value,allow_support):
            if mirror.pose_clear(robot,value,(name,)):return True
            if not allow_support or robot not in mirror.held:return False
            for c in mirror.screen_data.contact:
                if c.dist>=.003:continue
                x,y=map(int,c.geom)
                if x not in gids and y not in gids:continue
                other=y if x in gids else x
                if other in gids:continue
                owner=mirror.model.body(int(mirror.model.geom_bodyid[other])).name
                if mirror.monitor.geom_owner[other]==rid and owner in (robot+'_left_finger',robot+'_right_finger'):continue
                if not original_support_contact(mirror.model.geom(other).name,c.dist,c.frame.reshape(3,3)[0]):return False
            held=mirror.held.pop(robot)
            try:return mirror.pose_clear(robot,value,(name,))
            finally:mirror.held[robot]=held
        for fraction in np.linspace(0,1,51)[1:]:
            p=start+(np.asarray(point)-start)*fraction;r=rz(angle*fraction)@old_r
            n=mirror.screen_ik(robot,p,r,seed=q)
            allow=support and p[2]-start[2]<.004
            if n is None or not joint_edge_clear(q,n,lambda v:clear(v,allow),step_rad=.002):
                raise ValueError('No complete screened correction segment: '+suffix)
            q=n
        write_json(review/'engineering'/f'{a.id}-{suffix}-plan.json',dict(source_sim_s=o['sim_time_s'],target_m=list(point),
            rotation=rotation.tolist(),sampled_full_arm_payload_clear=True,initial_support_exception=bool(support)))
        return submit(suffix,'commands',batches=[{robot:['move',dict(target_m=list(point),quaternion_wxyz=matrix_quat(rotation).tolist(),speed_m_s=speed,angular_speed_rad_s=omega)]}])

    ident=a.id+'-pick';path=review/'requests'/f'pose-{ident}.json'
    pick(SimpleNamespace(trial=directory,robot=robot,object=name,id=ident,request=path,aperture=.064,contact_height=.035,flat_only=True))
    wait(ident,path);o=submit('adopt','adopt_hold',robot=robot,object=name)
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot]
    o=move('lift',mirror.data.site_xpos[arm.site]+[0,0,.014],mirror.data.site_xmat[arm.site].reshape(3,3).copy(),support=True)
    if o['objects'][name]['position_m'][2]<goal['position_m'][2]+.008:raise RuntimeError('Actual small lift not retained')
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];body=mirror.data.body(name);centre=body.xpos.copy();r=body.xmat.reshape(3,3)
    yaw=math.atan2(r[1,0],r[0,0]);delta=round(yaw/(math.pi/2))*math.pi/2-yaw
    if abs(delta)>.22:raise RuntimeError('Actual grasp changed the correction class')
    turn=rz(delta);tcp=mirror.data.site_xpos[arm.site].copy();rot=mirror.data.site_xmat[arm.site].reshape(3,3).copy()
    move('rotate',centre+turn@(tcp-centre),turn@rot)
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];target=np.asarray(goal['position_m'])+[0,0,.0038]
    tcp=mirror.data.site_xpos[arm.site].copy();body=mirror.data.body(name).xpos.copy()
    move('lower',tcp+target-body,mirror.data.site_xmat[arm.site].reshape(3,3).copy(),speed=.008)
    submit('open','commands',batches=[{robot:['gripper',dict(mode='move',width_m=.064,speed_m_s=.02)]}])
    submit('release','reconcile_released',robot=robot,object=name)
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot]
    move('withdraw',mirror.data.site_xpos[arm.site]+[0,0,.10],mirror.data.site_xmat[arm.site].reshape(3,3).copy(),speed=.025,omega=.2)
    o=observation()
    if o['evaluation']['assignments'].get(a.goal)!=name:raise RuntimeError('Original evaluator did not accept the correction')
    write_json(review/'engineering'/f'{a.id}-result.json',dict(passed=True,world_id=world,review_id=epoch,goal=a.goal,object=name,
        matched=o['evaluation']['matched_goals'],previous_pairings_retained=len(protected),actual_body=o['objects'][name],
        scope='Same-world ordinary native pose correction; no task/controller changes'))
    if a.resume:
        ident=a.id+'-resume';path=review/'requests'/f'pose-{ident}.json'
        if path.exists():raise ValueError('Never repeat a resume request with this correction ID')
        write_json(path,dict(id=ident,world_id=world,review_id=epoch,operation='resume'))
    print('POSE CORRECTION COMPLETE',a.goal,o['evaluation']['matched_goals'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--robot',required=True);p.add_argument('--object',required=True);p.add_argument('--goal',required=True)
    p.add_argument('--id',required=True);p.add_argument('--resume',action='store_true');run(p.parse_args())
