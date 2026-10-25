"""Read-only FK reproduction of an interrupted Relay terminal guard.

Saved motion/observations are diagnostic geometry, never a physics restart.
This program neither creates a Trial directory nor steps a physical world.
"""
import argparse
from copy import copy
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm, matrix_quat, rotation_vector
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite import relay_oracle_v2 as ref


def diagnostic_world(run):
    run=Path(run); observation=json.loads((run/'recovery/observation.json').read_text())
    t=Trial.__new__(Trial);t.model=mujoco.MjModel.from_xml_path(str(run/'scene.xml'))
    t.data=mujoco.MjData(t.model)
    mujoco.mj_resetDataKeyframe(t.model,t.data,t.model.key('initial').id)
    mujoco.mj_forward(t.model,t.data)
    t.task=json.loads((run/'task.json').read_text());t.stop_requested=None;t.live_preview=False
    t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
    t.home_q={a:t.data.qpos[arm.qids].copy() for a,arm in t.arms.items()}
    t.home={a:(t.data.site_xpos[arm.site].copy(),t.data.site_xmat[arm.site].reshape(3,3).copy()) for a,arm in t.arms.items()}
    stream=json.loads((run/'motion/stream.json').read_text());chunk=stream['chunks'][-1]
    with np.load(run/'motion'/chunk['file'],allow_pickle=False) as z:
        bits=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base'])
        t.data.qpos[:]=bits.view('<f4')[-1]
    # Restore observed doubles for arm joints and body poses; remaining tiny
    # finger asymmetry comes from the final float32 visual record.
    for a,arm in t.arms.items():
        t.data.qpos[arm.qids]=observation['robots'][a]['joint_rad']
        t.data.ctrl[arm.act]=t.data.qpos[arm.qids]
        arm.qref=t.data.qpos[arm.qids].copy()
    for ident,body in observation['payloads'].items():
        t.data.joint(ident+'_free').qpos[:]=np.r_[body['position_m'],matrix_quat(np.array(body['rotation']))]
    t.data.time=observation['sim_time_s'];mujoco.mj_forward(t.model,t.data)
    t.screen_model=copy(t.model);t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
    t.screen_data=mujoco.MjData(t.screen_model);t.held=dict(observation['held']);t.carry_geoms={};t.hand_geoms={}
    t.monitor=Monitor(t)
    errors={a:float(np.linalg.norm(t.data.site_xpos[arm.site]-observation['robots'][a]['tcp_m'])) for a,arm in t.arms.items()}
    if max(errors.values())>1e-8:raise ValueError('Recorded FK does not match observation: '+str(errors))
    return t,observation


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--scan',action='store_true');a=p.parse_args()
    started=time.monotonic();t,obs=diagnostic_world(a.run);ident='payload_02';team=['G','H']
    commands=ref.landing_increment_clear(t,ident,team,.005)
    result=dict(scope='Read-only saved-state FK; no physical restart or stepping',
        world_id=obs['world_id'],descent_admitted=commands is not None,robots={})
    for robot in team:
        arm=t.arms[robot];q=t.data.qpos[arm.qids].copy()
        point=t.data.site_xpos[arm.site].copy();rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()
        goal=point-np.array([0,0,.005]);solved,error=arm.ik.solve(goal,rotation,q,220)
        reached,frame=arm.ik.pose(solved)
        result['robots'][robot]=dict(position_error_m=error,angle_error_rad=float(np.linalg.norm(rotation_vector(rotation,frame))),
            current_soft_margin_rad=float(np.min(np.minimum(q-arm.ik.limits[:,0],arm.ik.limits[:,1]-q))),
            reached_soft_margin_rad=float(np.min(np.minimum(solved-arm.ik.limits[:,0],arm.ik.limits[:,1]-solved))),
            current_pose_passes_reach_bound=bool(ref.potentially_reachable(t,robot,point,rotation)),
            descent_passes_reach_bound=bool(ref.potentially_reachable(t,robot,goal,rotation)),
            raw_ik_admitted=bool(error<.0018 and np.linalg.norm(rotation_vector(rotation,frame))<.02))
    if a.scan:
        result['candidates']=[];root=t.data.body(ident).xpos.copy();rotation=t.data.body(ident).xmat.reshape(3,3).copy()
        yaw=float(np.arctan2(rotation[1,0],rotation[0,0]))
        for retain in [True,False]:
            for dx,dy,dz in [(-.005,0,0),(-.01,0,0),(-.02,0,0),(0,0,-.005),(0,0,.005),(.005,0,0),(0,.005,0),(0,-.005,0),(-.02,0,-.01),(-.05,0,-.02)]:
                target=root+np.array([dx,dy,dz]);clear=ref.carry_clear(t,ident,team,target,yaw,target_rotation=rotation if retain else None)
                result['candidates'].append(dict(delta_m=[dx,dy,dz],retain_attitude=retain,clear=bool(clear),reason=t.relay_screen_reason))
    result['wall_s']=time.monotonic()-started
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['descent_admitted'] else 1


if __name__=='__main__':raise SystemExit(main())
