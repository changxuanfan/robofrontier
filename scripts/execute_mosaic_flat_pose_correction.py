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

from robofrontier.robot_control import matrix_quat,rotation_vector
from robofrontier.run_setup import write_json
from robofrontier.suite.oracle import joint_edge_clear
from robofrontier.suite.oracle import CUBE_ROTATIONS
from scripts.plan_mosaic_repair_pick import observed_trial,plan as pick


def rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def original_support_contact(name,distance,normal):
    return name in ('table_0','pocket_bottom') and distance>=-.0002 and abs(normal[2])>.99


def rotation_step(vector):
    angle=float(np.linalg.norm(vector))
    if angle<1e-12:return np.eye(3)
    x,y,z=np.asarray(vector)/angle;cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)


def run(a):
    release_height=float(getattr(a,'release_height_m',.0038))
    if not np.isfinite(release_height) or not .0038<=release_height<=.015:
        raise ValueError('Select a release clearance within the demonstrated3.8..15mm interval')
    directory=a.trial.resolve();review=directory/'review';t,initial,_=observed_trial(directory)
    goal=next(g for g in t.task['goals'] if g['goal_id']==a.goal);name=a.object;robot=a.robot
    if t.task.get('family')!='mosaic' or abs(t.task['cube_size_m']-.06)>1e-10:
        raise ValueError('This finite correction covers only the declared60mm Mosaic cubes')
    if any(arm['grasp_force_setpoint_n'] is not None for arm in initial['robots'].values()) or initial['held']:
        raise ValueError('Begin this correction only with released free grippers')
    if a.goal in initial['evaluation']['assignments']:raise ValueError('Do not rework an accepted goal')
    body=initial['objects'][name];rotation=np.asarray(body['rotation']);position=np.asarray(body['position_m'])
    xy_error=float(np.linalg.norm(position[:2]-np.asarray(goal['position_m'])[:2]))
    allowed_xy=.005 if t.task['level']==1 else .010
    if (xy_error>allowed_xy or
            abs(position[2]-goal['position_m'][2])>.003 or rotation[2,2]<.999):
        raise ValueError('This finite in-place correction requires the actual seated planar pose')
    color=next(o['color'] for o in t.task['objects'] if o['id']==name)
    if color!=goal['requires']['color']:raise ValueError('Selected object color differs from the original goal')
    yaw=math.atan2(rotation[1,0],rotation[0,0]);delta=round(yaw/(math.pi/2))*math.pi/2-yaw
    angular_case=math.radians(8)<abs(delta)<math.radians(12)
    position_case=t.task['level']>1 and .005<xy_error<=.010 and abs(delta)<math.radians(12)
    if not (angular_case or position_case):raise ValueError('Select a seated small-yaw or5..10mm flat position correction')
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
        vector=rotation_vector(rotation,old_r)
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
            p=start+(np.asarray(point)-start)*fraction;r=rotation_step(vector*fraction)@old_r
            n=mirror.screen_ik(robot,p,r,seed=q)
            allow=support and p[2]-start[2]<.004
            if n is None or not joint_edge_clear(q,n,lambda v:clear(v,allow),step_rad=.002):
                raise ValueError('No complete screened correction segment: '+suffix)
            q=n
        write_json(review/'engineering'/f'{a.id}-{suffix}-plan.json',dict(source_sim_s=o['sim_time_s'],target_m=list(point),
            rotation=rotation.tolist(),sampled_full_arm_payload_clear=True,initial_support_exception=bool(support)))
        return submit(suffix,'commands',batches=[{robot:['move',dict(target_m=list(point),quaternion_wxyz=matrix_quat(rotation).tolist(),speed_m_s=speed,angular_speed_rad_s=omega)]}])

    ident=a.id+'-pick';path=review/'requests'/f'pose-{ident}.json'
    pick(SimpleNamespace(trial=directory,robot=robot,object=name,id=ident,request=path,aperture=a.aperture,contact_height=a.contact_height,
        flat_only=True,contact_offset_m=a.contact_offset_m,tilts_deg=a.grasp_tilts_deg,
        force_n=a.force_n,centering_contact_m=a.centering_contact_m,defer_grasp=a.centering_contact_m>0))
    o=wait(ident,path)
    if a.centering_contact_m>0:
        mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];actual_r=mirror.data.body(name).xmat.reshape(3,3)
        closing=mirror.data.site_xmat[arm.site].reshape(3,3)[:,1]
        if t.task['cube_size_m']*sum(abs(actual_r.T@closing))>.068:raise RuntimeError('Observed approach tipped the cube outside the original grasp-width band; no grasp issued')
        if o['evaluation']['assignments'].get(a.goal)==name:
            move('already-qualified-withdraw',mirror.data.site_xpos[arm.site]+[0,0,.10],mirror.data.site_xmat[arm.site].reshape(3,3).copy(),speed=.025)
            if observation()['evaluation']['assignments'].get(a.goal)!=name:raise RuntimeError('Actual centering result changed during withdrawal')
            write_json(review/'engineering'/f'{a.id}-result.json',dict(passed=True,world_id=world,review_id=epoch,goal=a.goal,object=name,scope='Declared centering contact satisfied the original goal; unnecessary grasp/lift omitted'))
            print('GOAL VERIFIED WITHOUT EXTRA GRASP',a.goal,flush=True);return
        submit('grasp','commands',batches=[{robot:['gripper',dict(mode='grasp',width_m=.06,force_n=a.force_n,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008)]}])
    o=submit('adopt','adopt_hold',robot=robot,object=name)
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot]
    o=move('lift',mirror.data.site_xpos[arm.site]+[0,0,.014],mirror.data.site_xmat[arm.site].reshape(3,3).copy(),support=True)
    if o['objects'][name]['position_m'][2]<goal['position_m'][2]+.008:raise RuntimeError('Actual small lift not retained')
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];body=mirror.data.body(name);centre=body.xpos.copy();r=body.xmat.reshape(3,3)
    aligned=min(CUBE_ROTATIONS,key=lambda candidate:np.linalg.norm(rotation_vector(candidate,r)))
    if np.linalg.norm(rotation_vector(aligned,r))>.22:raise RuntimeError('Actual grasp changed the correction class')
    turn=aligned@r.T;tcp=mirror.data.site_xpos[arm.site].copy();rot=mirror.data.site_xmat[arm.site].reshape(3,3).copy()
    move('rotate',centre+turn@(tcp-centre),turn@rot)
    mirror,o,_=observed_trial(directory);arm=mirror.arms[robot];target=np.asarray(goal['position_m'])+[0,0,release_height]
    tcp=mirror.data.site_xpos[arm.site].copy();body=mirror.data.body(name).xpos.copy()
    move('lower',tcp+target-body,mirror.data.site_xmat[arm.site].reshape(3,3).copy(),speed=.008)
    submit('open','commands',batches=[{robot:['gripper',dict(mode='move',width_m=.064,speed_m_s=.02)]}])
    if release_height>.0038:observation(observation()['sim_time_s']+.7)
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
    p.add_argument('--contact-height',type=float,default=.035)
    p.add_argument('--contact-offset-m',nargs=3,type=float,default=(0.,0.,0.))
    p.add_argument('--aperture',type=float,default=.064)
    p.add_argument('--grasp-tilts-deg',nargs='+',type=float,default=(0.,))
    p.add_argument('--centering-contact-m',type=float,default=0.)
    p.add_argument('--release-height-m',type=float,default=.0038)
    p.add_argument('--force-n',type=float,default=50.)
    p.add_argument('--id',required=True);p.add_argument('--resume',action='store_true');run(p.parse_args())
