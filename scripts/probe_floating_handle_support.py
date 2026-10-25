"""Private diagnostic: vary only the grasp point along the blue handle; no policy/world resume."""
import json,math
from pathlib import Path
import mujoco,numpy as np
from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.task_config import load_harness
from robofrontier.floating_dial_task import FloatingState

RUN=Path('runs/floating-dial-adaptive-sol-high-20261023-r1')

def probe(offset):
    model=mujoco.MjModel.from_xml_path(str((RUN/'private/inputs/scene/scene.xml').resolve()));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    arms={a:PandaArm(model,data,a) for a in 'ABCDEFGH'};cfg=load_harness();task=json.loads((RUN/'private/inputs/task.json').read_text())
    from types import SimpleNamespace
    world=SimpleNamespace(model=model,data=data,task=task);app=FloatingState(world)
    gid=model.geom('dial_A_module_stem').id;g=data.geom(gid);target=g.xpos.copy()+g.xmat.reshape(3,3)@np.array([0,0,offset]);rot=data.site_xmat[arms['B'].site].reshape(3,3).copy();records=[];tick=0
    def step():
        nonlocal tick
        if tick%20==0:
            for arm in arms.values():arm.control(.02)
        mujoco.mj_step(model,data);tick+=1
    def act(label,op,p):
        c=arms['B'].validate(op,p,cfg,np.array([-3.45,-1.03,.401]),np.array([3.45,1.03,1.5]));arms['B'].start(label,c,data.time)
        for _ in range(90000):
            step()
            if tick%20:continue
            result=arms['B'].poll(data.time,15)
            if result:records.append(dict(label=label,result=result));return result
        raise RuntimeError('probe action timed out')
    pose=lambda p:dict(target_m=p.tolist(),quaternion_wxyz=matrix_quat(rot).tolist(),speed_m_s=.05)
    act('approach','move',pose(target+np.array([0,0,.10])));act('descend','move',pose(target))
    grasp=act('grasp','gripper',dict(mode='grasp',width_m=.016,force_n=70,speed_m_s=.04))
    before=data.body('dial_A_module').xquat.copy()
    lift=np.array(arms['B'].observe()['tcp_m'])+np.array([0,0,.13]);act('lift','move',pose(lift))
    qids=[model.jnt_qposadr[model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'];clearances=[]
    for _ in range(5000):
        step()
        if tick%20==0:
            app.sample(data.qpos[qids]);clearances.append(app.snapshot['clearance_m'][0])
    q=data.body('dial_A_module').xquat.copy();angle=math.degrees(2*math.acos(float(np.clip(abs(q@before),0,1))))
    return dict(handle_axis_offset_m=offset,grasp_point_world_m=target.tolist(),grasp_status=grasp['status'],grasp_actual_width_m=grasp.get('actual_width_m'),minimum_clearance_during_final_5s_m=min(clearances),final_clearance_m=clearances[-1],orientation_change_after_lift_deg=angle,final_gripper_width_m=arms['B'].observe()['gripper_width_m'],scope='20ms sampled diagnosis; exact geometry used privately. Same force, orientation, lift and object; only along-handle grasp point varies.')

if __name__=='__main__':
    results=[probe(x) for x in (-.02,.07)]
    out=RUN/'private/outputs/handle-support-probe.json';out.write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
    assert results[0]['grasp_status']=='completed' and results[0]['minimum_clearance_during_final_5s_m']<.02
    assert results[1]['grasp_status']=='completed' and results[1]['minimum_clearance_during_final_5s_m']>.02
    assert results[0]['orientation_change_after_lift_deg']>20 and results[1]['orientation_change_after_lift_deg']<10
