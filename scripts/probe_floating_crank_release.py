"""Private counterfactual release probe from recorded qpos, not a full Oracle trial."""
import argparse,json
from pathlib import Path
import mujoco,numpy as np
from scripts.build_floating_crank import DIRECTORY
from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.task_config import load_harness

def probe(recording,output):
    m=mujoco.MjModel.from_xml_path(str(DIRECTORY/'scene.xml'));d=mujoco.MjData(m)
    d.qpos[:]=np.load(Path(recording)/'frame-states.npz')['qpos'][-1]
    for a in 'ABCDEFGH':
        for j in range(1,8):d.ctrl[m.actuator(f'{a}_actuator{j}').id]=d.joint(f'{a}_joint{j}').qpos[0]
        d.ctrl[m.actuator(a+'_actuator8').id]=255
    mujoco.mj_forward(m,d);arms={a:PandaArm(m,d,a) for a in 'ABCDEFGH'};cfg=load_harness();lower=np.array([-2,-2,.4]);upper=np.array([2,2,1.3]);tick=0;results={}
    def batch(label,commands):
        nonlocal tick
        pending=set(commands)
        for a,(op,p) in commands.items():arms[a].start(label,arms[a].validate(op,p,cfg,lower,upper),d.time)
        while pending and d.time<30:
            if tick%20==0:
                for arm in arms.values():arm.control(.02)
            mujoco.mj_step(m,d);tick+=1
            if tick%20:continue
            for a in list(pending):
                r=arms[a].poll(d.time,15)
                if r:results[label+'-'+a]=r;pending.remove(a)
    batch('support',{a:('gripper',dict(mode='grasp',width_m=.016,force_n=70.,speed_m_s=.02)) for a in 'BDFH'})
    batch('establish-crank-hold',{a:('gripper',dict(mode='grasp',width_m=.022,force_n=50.,speed_m_s=.025,epsilon_inner_m=.006,epsilon_outer_m=.006)) for a in 'ACEG'})
    batch('relax',{a:('gripper',dict(mode='move',width_m=arms[a].observe()['gripper_width_m'],speed_m_s=.04)) for a in 'ACEG'})
    commands={}
    for i,a in enumerate('ACEG'):
        mr=d.body('dial_'+'ABCD'[i]+'_module').xmat.reshape(3,3);arm=arms[a]
        commands[a]=('move',dict(target_m=(d.site_xpos[arm.site]+mr[:,2]*.08).tolist(),quaternion_wxyz=matrix_quat(d.site_xmat[arm.site].reshape(3,3)).tolist(),speed_m_s=.04,angular_speed_rad_s=.4))
    batch('withdraw',commands)
    batch('open-clear',{a:('gripper',dict(mode='move',width_m=.08,speed_m_s=.04)) for a in 'ACEG'})
    for _ in range(3000):
        if tick%20==0:
            for arm in arms.values():arm.control(.02)
        mujoco.mj_step(m,d);tick+=1
    report=dict(full_trial=False,replica_from_recorded_qpos=True,velocity_and_solver_state_reset=True,noslip_iterations=m.opt.noslip_iterations,results=results,angles_deg=[float(np.degrees(d.joint('dial_'+a+'_hinge').qpos[0])) for a in 'ABCD'],widths={a:arms[a].observe()['gripper_width_m'] for a in 'ACEG'})
    Path(output).write_text(json.dumps(report,indent=2));print(json.dumps(report))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('recording');p.add_argument('output');a=p.parse_args();probe(a.recording,a.output)
