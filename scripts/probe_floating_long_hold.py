"""Private 60s contact diagnosis. Never changes the live policy world or its inputs."""
import json,math
from pathlib import Path
from types import SimpleNamespace
import mujoco,numpy as np
from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.task_config import load_harness
from robofrontier.floating_dial_task import FloatingState
from scripts.verify_floating_dial_oracle import axis_rotation

RUN=Path('runs/floating-dial-060-tight-adaptive-sol-high-20261023-r1')
OUT=Path('runs/floating-long-hold-diagnosis-20261024');OUT.mkdir(exist_ok=True)

def probe(noslip,bearing,friction=None,duration_s=60.,friction_after_lift=None):
    model=mujoco.MjModel.from_xml_path(str((RUN/'private/inputs/scene/scene.xml').resolve()));model.opt.noslip_iterations=noslip
    if friction is not None:
        for g in range(model.ngeom):
            body=model.body(model.geom_bodyid[g]).name
            if body in ('B_left_finger','B_right_finger') or model.geom(g).name=='dial_A_module_stem':model.geom_friction[g,0]=friction
    data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    arms={a:PandaArm(model,data,a) for a in 'ABCDEFGH'};cfg=load_harness();task=json.loads((RUN/'private/inputs/task.json').read_text())
    app=FloatingState(SimpleNamespace(model=model,data=data,task=task));arm=arms['B'];stem=model.geom('dial_A_module_stem').id
    target=data.geom_xpos[stem].copy()+data.geom_xmat[stem].reshape(3,3)@np.array([0,0,.07]);rot=data.site_xmat[arm.site].reshape(3,3).copy()
    def step():
        for a in arms.values():a.control(.02)
        for _ in range(20):mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
    def act(label,op,p):
        command=arm.validate(op,p,cfg,np.array(task['workspace_world_m']['lower']),np.array(task['workspace_world_m']['upper']));arm.start(label,command,data.time)
        for _ in range(6000):
            step();result=arm.poll(data.time,15)
            if result:
                if result['status']!='completed':raise RuntimeError(str(result))
                return
        raise RuntimeError(label+' timeout')
    def pose(p,r=rot):return dict(target_m=p.tolist(),quaternion_wxyz=matrix_quat(r).tolist(),speed_m_s=.05,angular_speed_rad_s=.6)
    act('approach','move',pose(target+np.array([0,0,.10])));act('descend','move',pose(target))
    act('grasp','gripper',dict(mode='grasp',width_m=.016,force_n=70,speed_m_s=.04))
    lift=np.array(arm.observe()['tcp_m'])+np.array([0,0,.18]);act('lift','move',pose(lift))
    if bearing:act('bearing','move',pose(lift+np.array([0,0,.10]),axis_rotation([0,1,0],math.pi/2)@rot))
    if friction_after_lift is not None:
        for g in range(model.ngeom):
            body=model.body(model.geom_bodyid[g]).name
            if body in ('B_left_finger','B_right_finger') or model.geom(g).name=='dial_A_module_stem':model.geom_friction[g,0]=friction_after_lift
    ids=[model.jnt_qposadr[model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'];rows=[];started=float(data.time);first_loss=None
    finger_ids={g for g in range(model.ngeom) if model.body(model.geom_bodyid[g]).name in ('B_left_finger','B_right_finger')}
    for i in range(round(duration_s/.02)):
        step()
        if i%5:continue
        app.sample(data.qpos[ids],audit=False);normal=0.;tangent=0.;cone=0.;contact_count=0
        for j,c in enumerate(data.contact):
            if stem not in (c.geom1,c.geom2) or not (c.geom1 in finger_ids or c.geom2 in finger_ids):continue
            force=np.zeros(6);mujoco.mj_contactForce(model,data,j,force);normal+=float(force[0]);tangent+=float(np.linalg.norm(force[1:3]));cone+=float(force[0]*c.friction[0]);contact_count+=1
        clearance=app.snapshot['clearance_m'][0];age=float(data.time)-started
        if clearance<.02 and first_loss is None:first_loss=age
        rows.append(dict(held_s=age,clearance_m=clearance,module_z_m=float(data.body('dial_A_module').xpos[2]),tcp_z_m=float(data.site_xpos[arm.site,2]),width_m=arm.observe()['gripper_width_m'],actuator_force_n=float(data.actuator_force[arm.grip]),normal_sum_n=normal,tangent_sum_n=tangent,sliding_cone_sum_n=cone,contacts=contact_count))
    result=dict(noslip_iterations=noslip,bearing_reorientation=bearing,hold_duration_s=duration_s,friction_override=friction,friction_after_lift=friction_after_lift,first_clearance_loss_s=first_loss,start=rows[0],at_5s=min(rows,key=lambda r:abs(r['held_s']-5)),end=rows[-1],scope='Oracle diagnostic with exact grasp geometry. Sum of contact-frame force magnitudes is not a calibrated hardware force measurement; this is not a policy trial.')
    name=f'noslip-{noslip}-bearing-{int(bearing)}'+('' if friction is None else f'-friction-{friction}')+('' if friction_after_lift is None else f'-after-lift-friction-{friction_after_lift}');(OUT/(name+'-samples.json')).write_text(json.dumps(rows));(OUT/(name+'.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);return result

if __name__=='__main__':
    result=[probe(*v) for v in ((0,False),(20,False),(0,True))]
    (OUT/'summary.json').write_text(json.dumps(result,indent=2))
