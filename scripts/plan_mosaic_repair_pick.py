"""Private read-only observed-state planning, emitting ordinary repair commands.

No physics step or state assignment occurs in the live Trial. Its owner receives
only the finite native commands; contact/retention must be checked afterwards.
"""
import argparse
from copy import copy
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.run_setup import write_json
from robofrontier.suite.oracle import Trial,joint_edge_clear
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.mosaic_oracle_v2 import grasp_rotations,loaded_ik,screen_transfer_segments


def observed_trial(directory):
    """Reconstruct a read-only planning mirror; never a resumed live world."""
    directory=directory.resolve();payload=(directory/'review/observation.json').read_bytes();o=json.loads(payload)
    if o['evaluation']['violations']:raise ValueError('Do not plan past retained irreversible violations')
    t=Trial.__new__(Trial);t.model=mujoco.MjModel.from_xml_path(str(directory/'scene.xml'));t.data=mujoco.MjData(t.model)
    mujoco.mj_resetDataKeyframe(t.model,t.data,t.model.key('initial').id)
    t.task=json.loads((directory/'task.json').read_text());t.env=json.loads((directory/'environment.json').read_text())
    for robot,observed in o['robots'].items():
        for i,value in enumerate(observed['joint_rad'],1):
            t.data.joint(f'{robot}_joint{i}').qpos[0]=value;t.data.ctrl[t.model.actuator(f'{robot}_actuator{i}').id]=value
        for suffix in ('finger_joint1','finger_joint2'):t.data.joint(robot+'_'+suffix).qpos[0]=observed['gripper_width_m']/2
    for name,observed in o['objects'].items():
        t.data.joint(name+'_free').qpos[:]=np.r_[observed['position_m'],matrix_quat(np.asarray(observed['rotation']))]
    mujoco.mj_forward(t.model,t.data)
    t.arms={r:PandaArm(t.model,t.data,r) for r in t.task['arms']};t.home_q={r:t.model.key_qpos[t.model.key('initial').id][arm.qids].copy() for r,arm in t.arms.items()}
    t.held=dict(o['held']);t.carry_geoms={};t.hand_geoms={};t.stop_requested=None
    t.screen_model=copy(t.model);t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
    t.screen_data=mujoco.MjData(t.screen_model);t.monitor=Monitor(t)
    def forbidden(*args,**kwargs):raise RuntimeError('Repair planning cannot step or execute a world')
    t.step=t.execute=t.wait=forbidden
    return t,o,payload


def plan(a):
    directory=a.trial.resolve();t,o,payload=observed_trial(directory)
    if a.robot in o['held']:raise ValueError('Receiver must have a free gripper')
    robot=a.robot;name=a.object;body=t.data.body(name);position=body.xpos.copy();rotation=body.xmat.reshape(3,3).copy()
    aperture=getattr(a,'aperture',.072)
    if not t.task['cube_size_m']+.002<aperture<=t.arms[robot].grip_max:raise ValueError('Select a legal approach opening with actual cube clearance')
    for suffix in ('finger_joint1','finger_joint2'):t.data.joint(robot+'_'+suffix).qpos[0]=aperture/2
    mujoco.mj_forward(t.model,t.data)
    up_axis=int(np.argmax(abs(rotation[2])));up=rotation[:,up_axis]*np.sign(rotation[2,up_axis])
    axes=[rotation[:,axis]*sign for axis in range(3) if axis!=up_axis for sign in (1,-1)]
    base=np.asarray(t.task['arms'][robot]['base_position_m']);arm=t.arms[robot]
    selected=None
    height_override=getattr(a,'contact_height',None)
    heights=(height_override,) if height_override is not None else (t.task['cube_size_m']/2-.005,t.task['cube_size_m']/3,t.task['cube_size_m']*.2)
    if any(not 0<height<=t.task['cube_size_m']/2+.01 for height in heights):raise ValueError('Unsupported finite top-grasp contact height')
    for height in heights:
        contact=position+up*height
        for r,tilt in grasp_rotations(up,axes,position-base):
            seat=loaded_ik(t,robot,contact,r,(name,))
            if seat is None:continue
            for rise in (.14,.10,.08):
                hover=contact+[0,0,rise];q=loaded_ik(t,robot,hover,r)
                if q is None or not joint_edge_clear(t.data.qpos[arm.qids],q,lambda value:t.pose_clear(robot,value),step_rad=.005):continue
                if not screen_transfer_segments(t,robot,name,r,q,[('repair-contact',contact)],path_joint_margin_rad=0.):continue
                selected=(contact,r,q,tilt);break
            if selected is not None:break
        if selected is not None:break
    if selected is None:raise ValueError('No finite screened repair pickup candidate; no command emitted')
    contact,r,q,tilt=selected
    commands=[{robot:('gripper',dict(mode='move',width_m=aperture,speed_m_s=.05))},
        {robot:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.2))},
        {robot:('move',dict(target_m=contact.tolist(),quaternion_wxyz=matrix_quat(r).tolist(),speed_m_s=.035,angular_speed_rad_s=.6))},
        {robot:('gripper',dict(mode='grasp',width_m=.06,force_n=50.,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008))}]
    color=next(item['color'] for item in t.task['objects'] if item['id']==name)
    coverage=json.loads((directory/'coverage.json').read_text())['goals']
    goals=[g for g in t.task['goals'] if g['requires']['color']==color and g['goal_id'] not in o['evaluation']['assignments'] and robot in coverage[g['goal_id']]]
    goal=min(goals,key=lambda g:np.linalg.norm(np.asarray(g['position_m'])[:2]-base[:2])) if goals else None
    request=dict(id=a.id,world_id=o['world_id'],review_id=o['review_id'],operation='commands',batches=commands)
    evidence=dict(scope='Observed-state kinematic candidate only; actual grasp/lift/placement must be verified',
        observation_sha256=hashlib.sha256(payload).hexdigest(),source_sim_s=o['sim_time_s'],robot=robot,object=name,
        candidate_tilt_deg=tilt,approach_width_m=aperture,contact_m=contact.tolist(),rotation=r.tolist(),suggested_unfilled_goal=goal,
        pending_held_payloads=o['held'],finger_state_reconstruction='Observed total opening split symmetrically for scratch screening')
    write_json(directory/'review/engineering'/f'{a.id}-plan.json',evidence)
    write_json(a.request,request)
    print(json.dumps(evidence),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--robot',required=True);p.add_argument('--object',required=True);p.add_argument('--id',required=True)
    p.add_argument('--request',required=True,type=Path);p.add_argument('--aperture',type=float,default=.072)
    p.add_argument('--contact-height',type=float)
    plan(p.parse_args())
