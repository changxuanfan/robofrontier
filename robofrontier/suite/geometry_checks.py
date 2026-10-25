"""Static engineering probes on scratch data, never runtime robot commands."""
import mujoco
import numpy as np
from ..robot_control import matrix_quat,quat_matrix


def owner_map(model,robots):
    roots={model.body(r+'_link0').id:r for r in robots};result={}
    for g in range(model.ngeom):
        body=int(model.geom_bodyid[g])
        while body and body not in roots:body=int(model.body_parentid[body])
        result[g]=roots.get(body)
    return result


def clear_robot_pose(model,initial_qpos,scratch,robot,qids,q,owners,source_id=None):
    """Screen a 70 mm preshape against real initial-scene collision geometry.

    Intended fingertip/source contact is allowed. Other penetrations over
    0.5 mm, including robot self-contact, reject the candidate. No dynamics
    advance and no initial/live state or actuator parameters are modified.
    This does not certify the trajectory between screened poses or a grasp.
    """
    scratch.qpos[:]=initial_qpos;scratch.qpos[qids]=q
    for suffix in ('finger_joint1','finger_joint2'):scratch.joint(robot+'_'+suffix).qpos[0]=.035
    mujoco.mj_forward(model,scratch)
    for contact in scratch.contact:
        if contact.dist>=-.0005:continue
        a,b=map(int,contact.geom)
        if owners[a]!=robot and owners[b]!=robot:continue
        own,other=(a,b) if owners[a]==robot else (b,a)
        own_body=model.body(int(model.geom_bodyid[own])).name
        other_body=model.body(int(model.geom_bodyid[other])).name
        if other_body==source_id and own_body.endswith(('_left_finger','_right_finger')):continue
        return False
    return True


def clear_pick_candidate(model,initial_qpos,scratch,robot,ik,seed,point,rotation,owners,source_id=None):
    """Rotate the empty wrist at its initial high pose, then approach a cube.

    Two rotation samples and three approach samples are checked. This remains
    a geometric witness, not continuous path certification or a physics run.
    """
    q=seed.copy();home,initial_rotation=ik.pose(q)
    first=matrix_quat(initial_rotation);last=matrix_quat(rotation)
    if first@last<0:last=-last
    stages=[]
    for fraction in (.5,1.):
        quat=(1-fraction)*first+fraction*last;quat/=np.linalg.norm(quat)
        stages.append((home,quat_matrix(quat)))
    stages.extend((point+[0,0,height],rotation) for height in (.12,.06,0))
    for target,orientation in stages:
        q,error=ik.solve(target,orientation,q,140)
        if error>.0025 or not clear_robot_pose(model,initial_qpos,scratch,robot,ik.qids,q,owners,source_id):return False
    return True
