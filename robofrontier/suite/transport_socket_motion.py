"""Bounded private socket planning, separate from controller and task rules.

Candidates are checked against the current board and full arm geometry. All
state writes below belong to independent kinematic scratch data. Screens are
finite samples, not a contact-dynamics or continuous-collision certificate.
"""
from collections import Counter
import math
import time

import mujoco
import numpy as np

from ..robot_control import advance_rotation, matrix_quat, rotation_vector
from . import transport_oracle_v2 as ref
from .oracle import joint_edge_clear
from .scenes import rz


class SocketPlanningError(RuntimeError):
    def __init__(self, stage, diagnostics):
        self.stage, self.diagnostics = stage, diagnostics
        super().__init__(f'Socket planning rejected at {stage}: {diagnostics}')


def require_scratch(t):
    if not getattr(t, '_reference_planning_only', False) or t.data is getattr(t, '_live_data', None):
        raise RuntimeError('Socket screening requires independent planning data')


def rejection_contacts(t, robot, limit=4, allowed_object=None):
    """Small diagnostic excerpt from the last screen, not all contacts."""
    rid=list(t.arms).index(robot); rows=[]
    for c in t.screen_data.contact:
        a,b=map(int,c.geom)
        oa,ob=t.monitor.geom_owner[a],t.monitor.geom_owner[b]
        if rid not in (oa,ob) or c.dist>=.003:
            continue
        names=[t.model.body(int(t.model.geom_bodyid[g])).name for g in (a,b)]
        if oa==ob==rid:
            if c.dist>=-.001 or all('finger' in n for n in names):continue
        else:
            other,own=(a,b) if ob==rid else (b,a)
            body=t.model.body(int(t.model.geom_bodyid[own])).name
            if body==robot+'_link0':continue
            if allowed_object is not None and other in t.monitor.geoms[allowed_object] and body in (robot+'_left_finger',robot+'_right_finger'):continue
        rows.append(dict(bodies=names,distance_m=float(c.dist)))
    return sorted(rows,key=lambda r:r['distance_m'])[:limit]


def sample_clear(t, robot, name, q, position, rotation, board, arm_clearance_m=.003,source_support=()):
    """Check arm and payload; only shallow intended board seating is allowed.

    With held={} pose_clear does not move the scratch payload from the
    explicitly proposed pose. Its arm checks still allow only target finger
    contact. Payload collisions are checked separately, including neighbors.
    """
    require_scratch(t)
    held=t.held
    try:
        t.held={}
        t.data.joint(name+'_free').qpos[:]=np.r_[position,matrix_quat(rotation)]
        mujoco.mj_forward(t.model,t.data)
        if not t.pose_clear(robot,q,(name,),clearance_m=arm_clearance_m):
            return False,dict(reason='arm_clearance',contacts=rejection_contacts(t,robot,allowed_object=name))
        payload=set(t.monitor.geoms[name]); board_geoms=set(t.monitor.descendant_geoms(t.model.body(board).id))
        rid=list(t.arms).index(robot)
        for c in t.screen_data.contact:
            a,b=map(int,c.geom)
            if a not in payload and b not in payload:
                continue
            part,other=(a,b) if a in payload else (b,a)
            if other in payload:
                continue
            body=t.model.body(int(t.model.geom_bodyid[other])).name
            if t.monitor.geom_owner[other]==rid and body in (robot+'_left_finger',robot+'_right_finger'):
                continue
            # The final support plane and pin/socket surfaces may touch, but
            # they may not intersect by more than the existing 0.3 mm fit
            # tolerance used by the mechanical unit diagnostic.
            if other in board_geoms and c.dist>=-.0003:
                continue
            if other in source_support and c.dist>=-.0003 and abs(float(c.frame[2]))>.9:
                continue
            if c.dist<.001:
                return False,dict(reason='payload_clearance',bodies=[name,body],distance_m=float(c.dist))
        return True,None
    finally:
        t.held=held


def trace_segment(t, robot, name, start, end, start_rotation, end_rotation,
                  seed, offset, relative, board, linear_step=.002, angular_step=.025,arm_clearance_m=.003,source_support=(),fixed_object_pose=None):
    """Follow one IK branch, checking intermediate joint edges as well.

    offset/relative describe the grasp, not a virtual attachment in physics.
    The payload pose at each sample is reconstructed solely in scratch.
    """
    require_scratch(t)
    distance=float(np.linalg.norm(np.asarray(end)-start))
    angle=float(np.linalg.norm(rotation_vector(end_rotation,start_rotation)))
    count=max(1,math.ceil(distance/linear_step),math.ceil(angle/angular_step))
    q=np.asarray(seed).copy();rows=[]
    for i in range(count+1):
        t.check_stop();f=i/count
        point=np.asarray(start)+(np.asarray(end)-start)*f
        tool=advance_rotation(start_rotation,end_rotation,angle*f)
        following=t.screen_ik(robot,point,tool,q)
        if following is None:
            return None,dict(reason='ik',fraction=f)
        if np.max(abs(following-q))>.20:
            return None,dict(reason='joint_branch_jump',fraction=f,max_joint_delta_rad=float(np.max(abs(following-q))))
        edge_failure={}
        def check(candidate):
            tcp,r=t.arms[robot].ik.pose(candidate);obj_r=r@relative.T
            p,rot=fixed_object_pose if fixed_object_pose is not None else (tcp-obj_r@offset,obj_r)
            clear,why=sample_clear(t,robot,name,candidate,p,rot,board,arm_clearance_m,source_support)
            if not clear and why is not None:edge_failure.update(why)
            return clear
        if not joint_edge_clear(q,following,check,step_rad=.01):
            return None,dict(reason='joint_edge_clearance',fraction=f,detail=edge_failure)
        tcp,r=t.arms[robot].ik.pose(following);obj_r=r@relative.T
        p,rot=fixed_object_pose if fixed_object_pose is not None else (tcp-obj_r@offset,obj_r)
        clear,why=sample_clear(t,robot,name,following,p,rot,board,arm_clearance_m,source_support)
        if not clear:
            return None,dict(why,fraction=f)
        q=following;rows.append(q.copy())
    return rows,None


def payload_clear_of_board(trial,name,board,minimum_m=.015):
    """Exact geometry distance, not height above the board's infinite plane.

    A retained object beside a raised spring board can be lower than its top
    and still be safely separated. A partly inserted blade is near a wall
    and fails this check even if its center remains high.
    """
    geoms=trial.monitor.descendant_geoms(trial.model.body(board).id)
    return all(mujoco.mj_geomDistance(trial.model,trial.data,g,h,minimum_m,None)>=minimum_m
               for g in trial.monitor.geoms[name] for h in geoms)


def may_replan_in_air(trial,robot,name,board,stage):
    """Only known idle, retained, collision-free and pre-contact states."""
    if stage not in ('seat_release_descent','socket-descend') or trial.monitor.violations:
        return False
    if any(a.job is not None for a in trial.arms.values()) or trial.held.get(robot)!=name or not trial.finger_contacts(robot,name):
        return False
    return payload_clear_of_board(trial,name,board)


def joint_seeds(t,robot):
    arm=t.arms[robot]
    for seed in getattr(t,'socket_seed_hints',{}).get(robot,()):
        yield np.asarray(seed).copy()
    yield arm.qref.copy()
    yield t.home_q[robot].copy()
    for delta in (-1.2,-.6,.6,1.2):
        seed=t.home_q[robot].copy();seed[0]+=delta;seed[2]-=delta
        yield np.clip(seed,*arm.ik.limits.T)


def seating_candidate(t,robot,name,position,rotation,board,offset,relative,
                      budget_s=15.,arm_clearance_m=.003):
    """Plan final fit, release, retreat and reverse descent before transit.

    Return a preapproach joint branch; it still must be connected to the
    actual pickup by a screened route and confirmed by real execution.
    """
    require_scratch(t);started=time.monotonic();counts=Counter();examples={}
    arm=t.arms[robot];original=t.data.qpos.copy()
    width=float(t.task['cube_size_m']);opening=min(arm.grip_max,width+.012)
    def fail(stage,why=None):
        counts[stage]+=1
        if why is not None and stage not in examples:examples[stage]=why
    try:
        for yaw in (0.,math.pi):
            obj_r=np.asarray(rotation)@rz(yaw);tool=obj_r@relative
            target=np.asarray(position)+obj_r@offset
            for seed in joint_seeds(t,robot):
                t.check_stop()
                if time.monotonic()-started>budget_s:
                    fail('search_budget');break
                t.data.qpos[:]=original;ref._scratch_gap(t,robot,width)
                mujoco.mj_forward(t.model,t.data)
                q=t.screen_ik(robot,target,tool,seed)
                if q is None:fail('seat_ik');continue
                clear,why=sample_clear(t,robot,name,q,position,obj_r,board,arm_clearance_m)
                if not clear:fail('seat_clearance',why);continue
                # Sweep the aperture with the object seated, then withdraw
                # the hand without carrying the object along with it.
                released=True
                for gap in np.linspace(width,opening,5):
                    ref._scratch_gap(t,robot,float(gap))
                    clear,why=sample_clear(t,robot,name,q,position,obj_r,board,arm_clearance_m)
                    if not clear:released=False;fail('release_clearance',why);break
                if not released:continue
                retreat_q=q.copy()
                for h in np.linspace(0,.10,26):
                    following=t.screen_ik(robot,target+obj_r[:,2]*h,tool,retreat_q)
                    if following is None or np.max(abs(following-retreat_q))>.20:
                        released=False;fail('withdraw_ik');break
                    def retreat_check(candidate):
                        return sample_clear(t,robot,name,candidate,position,obj_r,board,arm_clearance_m)[0]
                    if not joint_edge_clear(retreat_q,following,retreat_check,step_rad=.01):
                        released=False;fail('withdraw_clearance',dict(contacts=rejection_contacts(t,robot)));break
                    retreat_q=following
                if not released:continue
                ref._scratch_gap(t,robot,width)
                pre=target+obj_r[:,2]*.14
                path,why=trace_segment(t,robot,name,target,pre,tool,tool,q,offset,relative,board,arm_clearance_m=arm_clearance_m)
                if path is None:fail('descent_path',why);continue
                counts['qualified_branch']+=1
                return dict(target=target,rotation=tool,object_rotation=obj_r,
                            pre=pre,pre_q=path[-1],seat_q=q),dict(counts=counts,examples=examples,wall_s=time.monotonic()-started)
        return None,dict(counts=counts,examples=examples,wall_s=time.monotonic()-started)
    finally:
        t.data.qpos[:]=original;mujoco.mj_forward(t.model,t.data)


def guard_contact_moves(trial,board):
    """Opt-in unit executor guard. The normal controller remains unchanged."""
    original=trial.execute
    def execute(label,commands,allow_failure=False):
        for robot,(operation,params) in commands.items():
            name=trial.held.get(robot)
            if name is not None and operation=='move' and 'joint_rad' not in params:
                s=ref._planning_copy(trial);arm=s.arms[robot]
                body=s.data.body(name);obj_r=body.xmat.reshape(3,3).copy()
                start=s.data.site_xpos[arm.site].copy();start_r=s.data.site_xmat[arm.site].reshape(3,3).copy()
                offset=obj_r.T@(start-body.xpos);relative=obj_r.T@start_r
                fixed=(body.xpos.copy(),obj_r) if getattr(trial,'socket_released_payload',{}).get(robot)==name else None
                from ..robot_control import quat_matrix
                target=np.asarray(params.get('target_m',start+np.asarray(params.get('delta_m',[0,0,0]))))
                rotation=quat_matrix(params['quaternion_wxyz']) if 'quaternion_wxyz' in params else start_r
                active_board=board(robot) if callable(board) else board
                with ref.socket_contact_clearance(s,name,active_board):
                    path,why=trace_segment(s,robot,name,start,target,start_r,
                        rotation,trial.arms[robot].qref,offset,relative,active_board,fixed_object_pose=fixed)
                if path is None:
                    trial.event('socket_motion_rejected',robot=robot,object=name,label=label,diagnostics=why)
                    raise SocketPlanningError(label,why)
        return original(label,commands,allow_failure=allow_failure)
    trial.execute=execute
    return original


def place_socket(trial,robot,position,rotation,board,pose_provider=None,budget_s=15.):
    """Choose a fit-capable branch, then use ordinary robot commands.

    Live board pose is reread between short contact stages. A failed stage
    returns control to the caller; no rollback or unknown-command replay.
    """
    name=trial.held[robot];arm=trial.arms[robot];body=trial.data.body(name)
    r=body.xmat.reshape(3,3).copy();offset=r.T@(trial.data.site_xpos[arm.site]-body.xpos)
    relative=r.T@trial.data.site_xmat[arm.site].reshape(3,3)
    s=ref._planning_copy(trial)
    with ref.socket_contact_clearance(s,name,board):
        plan,stats=seating_candidate(s,robot,name,position,rotation,board,offset,relative,budget_s)
    trial.event('socket_seating_screen',robot=robot,object=name,qualified=plan is not None,diagnostics=stats,
                scope='Sampled current-geometry fit/release/retreat; not dynamic certification')
    if plan is None:raise SocketPlanningError('seat_release_descent',stats)
    trial.move_free(robot,plan['pre'],plan['rotation'],'socket-transit',target_joints=plan['pre_q'])
    symmetry=np.asarray(rotation).T@plan['object_rotation']
    for height in (.07,.035,.015,.005,0.):
        if not trial.finger_contacts(robot,name):raise SocketPlanningError('payload_support',dict(label='descent'))
        p,br=pose_provider() if pose_provider is not None else (np.asarray(position),np.asarray(rotation))
        obj_r=br@symmetry
        # Re-estimate the real grasp transform after physical carrying.
        body=trial.data.body(name);r=body.xmat.reshape(3,3)
        offset=r.T@(trial.data.site_xpos[arm.site]-body.xpos);relative=r.T@trial.data.site_xmat[arm.site].reshape(3,3)
        trial.move(robot,p+obj_r@offset+obj_r[:,2]*height,obj_r@relative,'socket-descend',speed_m_s=.015)
    trial.wait(.15)
    release=trial.execute('socket-release',{robot:('gripper',dict(mode='move',width_m=min(arm.grip_max,float(trial.task['cube_size_m'])+.012),speed_m_s=.05))},allow_failure=True)[robot]
    known=release['status']=='completed' or (release['status']=='failed' and release.get('reason') in ('Requested opening not reached','Requested aperture exceeded'))
    released=trial.monitor.released(trial.monitor.geoms[name])
    if not known or arm.grasp_force is not None:
        raise SocketPlanningError('release',dict(result=release,physically_released=released,force_hold_active=arm.grasp_force is not None))
    if release['status']!='completed':
        trial.event('socket_release_aperture_fault',robot=robot,object=name,result=release,
                    physically_released=released,scope='Keep failed aperture feedback; release must be verified after screened withdrawal')
    # After removing the force hold, intended finger contact may persist
    # until the hand withdraws. Screen the fixed supported object rather
    # than carrying it virtually with the retreating hand. Real release and
    # final seating are checked after withdrawal, never inferred from width.
    tool=trial.data.site_xmat[arm.site].reshape(3,3).copy()
    start=trial.data.site_xpos[arm.site].copy()
    if not hasattr(trial,'socket_released_payload'):trial.socket_released_payload={}
    trial.socket_released_payload[robot]=name
    try:
        for height in (.02,.05,.10):
            trial.move(robot,start+obj_r[:,2]*height,tool,'socket-withdraw',speed_m_s=.015)
    finally:
        trial.socket_released_payload.pop(robot,None)
    trial.wait(.12)
    if not trial.monitor.released(trial.monitor.geoms[name]):
        raise SocketPlanningError('release',dict(reason='robot_contact_after_withdrawal'))
    trial.held.pop(robot)
    trial.event('placed',robot=robot,object=name,target_m=np.asarray(p).tolist())
