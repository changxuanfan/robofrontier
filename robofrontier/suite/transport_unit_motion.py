"""Finite private grasp candidates for direct transport-unit commissioning.

Scratch endpoint/approach screening is separate from live contact evidence.
No environment, live object state, controller or acceptance is changed.
"""
from contextlib import nullcontext
import math
import time
import numpy as np

from . import transport_oracle_v2 as ref
from .scenes import rz
from ..robot_control import matrix_quat


class DirectGraspPlanningError(RuntimeError):
    def __init__(self,robot,name,diagnostics):
        self.stage='grasp_selection';self.diagnostics=diagnostics
        super().__init__('No finite direct pickup/insertion candidate '+robot+'/'+name+' '+str(diagnostics))


def turn(axis,angle):
    axis=np.asarray(axis);x,y,z=axis;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)


def insertion_grasps(frames,object_rotation,goal_rotation):
    """Rank finite grips by hand space above both source and destination.

    This is a geometric ordering heuristic only. In particular, it does not
    assume a side approach is reachable or exempt it from collision checks.
    """
    candidates=[]
    for angle in (0,30,-30,45,-45,60,-60,75,-75,90,-90,105,-105,120,-120):
        for frame in frames:
            tool=turn(frame[:,1],math.radians(angle))@frame
            final_tool=goal_rotation@object_rotation.T@tool
            reserve=min(float(-tool[2,2]),float(-goal_rotation[:,2]@final_tool[:,2]))
            candidates.append((reserve,angle,tool))
    return [(angle,tool) for _,angle,tool in sorted(candidates,key=lambda c:-c[0])]


def direct_grasp_candidate(trial,robot,name,goal_pose,board,budget_s=45.,check_seating=False,
        axial_shift_m=0.,pickup_heights_m=None):
    t=ref._planning_copy(trial);start=time.monotonic();stats={}
    def count(key):stats[key]=stats.get(key,0)+1
    obj=t.data.body(name);p=obj.xpos.copy();rotation=obj.xmat.reshape(3,3).copy()
    shape=next(o['shape'] for o in t.env['objects'] if o['id']==name)
    up,frames=ref._pickup_frames(shape,rotation);gp,gr=goal_pose
    original=t.data.qpos.copy();arm=t.arms[robot];width=float(t.task['cube_size_m']);opening=min(arm.grip_max,width+.012)
    # Only the existing supporting tabletop may remain in shallow contact
    # while the pickup starts. This does not permit touching other objects.
    source_support=set();payload=set(t.monitor.geoms[name])
    for contact in t.data.contact:
        a,b=map(int,contact.geom)
        if a in payload or b in payload:
            other=b if a in payload else a
            if t.model.geom(other).name.startswith('table_') and contact.dist>=-.0003 and abs(float(contact.frame[2]))>.9:
                source_support.add(other)
    def seed_values():
        values=[arm.qref.copy(),t.home_q[robot].copy()]
        for angle in (-.7,.7):
            q=t.home_q[robot].copy();q[0]+=angle;q[2]-=angle
            values.append(np.clip(q,*arm.ik.limits.T))
        return values
    choices=insertion_grasps(frames,rotation,gr) if check_seating else [
        (angle,turn(frame[:,1],math.radians(angle))@frame)
        for angle in (90,-90,105,-105,120,-120,75,-75,60,-60,0) for frame in frames]
    for angle,tool in choices:
        for height in ((0.,-.006,-.012,.006,.012,.020) if pickup_heights_m is None else pickup_heights_m):
            if time.monotonic()-start>budget_s:
                return None,dict(stats,budget_exhausted=True,wall_s=time.monotonic()-start)
            t.data.qpos[:]=original;t.held={};ref._scratch_gap(t,robot,opening)
            import mujoco
            mujoco.mj_forward(t.model,t.data)
            point=p+up*height+rotation[:,2]*axial_shift_m;approach=point-tool[:,2]*.12
            seating=None
            pickup_candidates=[]
            # Cheap source checks avoid performing long reverse-path
            # sweeps for a grip the source arm cannot even approach.
            for seed in seed_values():
                q=t.screen_ik(robot,point,tool,seed)
                if q is None or not t.pose_clear(robot,q,(name,),clearance_m=.001):continue
                near=t.screen_ik(robot,approach,tool,q)
                if near is not None and t.pose_clear(robot,near):pickup_candidates.append((q,near))
            if not pickup_candidates:continue
            if check_seating and t.task['level']>1:
                from .transport_socket_motion import seating_candidate
                offset=rotation.T@(point-p);relative=rotation.T@tool
                with ref.socket_contact_clearance(t,name,board):
                    seating,diagnostic=seating_candidate(t,robot,name,gp,gr,board,offset,relative,
                        budget_s=max(.001,budget_s-(time.monotonic()-start)),arm_clearance_m=.006)
                for stage,total in diagnostic['counts'].items():stats[stage]=stats.get(stage,0)+total
                for stage,example in diagnostic['examples'].items():stats.setdefault('first_'+stage,example)
                if seating is None:continue
                count('insertion_first_candidate')
                t.data.qpos[:]=original;ref._scratch_gap(t,robot,opening);mujoco.mj_forward(t.model,t.data)
            for q,near in pickup_candidates:
                count('pickup_contact_clear')
                count('pickup_approach_clear')
                offset=rotation.T@(point-p);relative=rotation.T@tool
                if seating is not None:
                    # Also check the actual approach and lift, retaining
                    # the candidate object transform only in scratch.
                    from .transport_socket_motion import trace_segment
                    approach_path,why=trace_segment(t,robot,name,approach,point,tool,tool,near,offset,relative,board,
                        source_support=source_support,fixed_object_pose=(p,rotation))
                    t.data.qpos[:]=original;ref._scratch_gap(t,robot,opening);mujoco.mj_forward(t.model,t.data)
                    if approach_path is None:
                        count('pickup_descent_rejected');stats.setdefault('first_pickup_descent_rejected',why);continue
                    path,why=trace_segment(t,robot,name,point,point+np.array([0,0,.14]),tool,tool,q,offset,relative,board,source_support=source_support)
                    t.data.qpos[:]=original;ref._scratch_gap(t,robot,opening);mujoco.mj_forward(t.model,t.data)
                    if path is None:
                        count('pickup_lift_rejected');stats.setdefault('first_pickup_lift_rejected',why);continue
                    return dict(point=point,rotation=tool,approach=approach,approach_q=near,contact_q=q,
                        height=height,tilt_deg=angle,final_object_rotation=seating['object_rotation'],
                        final_contact_q=seating['seat_q'],final_pre_q=seating['pre_q']),dict(stats,wall_s=time.monotonic()-start)
                for yaw in (0,math.pi):
                    r=gr@rz(yaw);target=gp+r@offset;end_tool=r@relative
                    t.data.joint(name+'_free').qpos[:]=np.r_[gp,matrix_quat(r)]
                    mujoco.mj_forward(t.model,t.data)
                    for end_seed in seed_values():
                        end=t.screen_ik(robot,target,end_tool,end_seed)
                        if end is None:continue
                        count('final_ik')
                        context=ref.socket_contact_clearance(t,name,board) if t.task['level']>1 else nullcontext()
                        with context:
                            if not t.pose_clear(robot,end,(name,),clearance_m=.003):
                                count('final_seat_rejected')
                                if 'first_final_contacts' not in stats:
                                    rid=list(t.arms).index(robot);allowed=set(t.monitor.geoms[name]);rows=[]
                                    for c in t.screen_data.contact:
                                        a,b=map(int,c.geom)
                                        if c.dist>=.003:continue
                                        oa,ob=t.monitor.geom_owner[a],t.monitor.geom_owner[b]
                                        if rid not in (oa,ob):continue
                                        if oa==ob:continue
                                        own,other=(a,b) if oa==rid else (b,a)
                                        body=t.model.body(int(t.model.geom_bodyid[own])).name
                                        if other in allowed and body in (robot+'_left_finger',robot+'_right_finger'):continue
                                        rows.append(dict(bodies=[t.model.body(int(t.model.geom_bodyid[g])).name for g in (a,b)],geoms=[t.model.geom(g).name for g in (a,b)],distance_m=float(c.dist)))
                                    stats['first_final_contacts']=rows[:4]
                                continue
                            count('final_seat_clear')
                            clear=True
                            for gap in (width,opening,arm.grip_max):
                                ref._scratch_gap(t,robot,gap)
                                if not t.pose_clear(robot,end,(name,),clearance_m=.003):clear=False;break
                            ref._scratch_gap(t,robot,opening)
                            if not clear:continue
                            pre=t.screen_ik(robot,target+gr[:,2]*.14,end_tool,end)
                            if pre is None or not t.pose_clear(robot,pre):continue
                        count('final_seat_open_and_pre_clear')
                        return dict(point=point,rotation=tool,approach=approach,
                            approach_q=near,contact_q=q,height=height,tilt_deg=angle,
                            final_object_rotation=r,final_contact_q=end,final_pre_q=pre),dict(stats,wall_s=time.monotonic()-start)
                    t.data.qpos[:]=original;ref._scratch_gap(t,robot,opening);mujoco.mj_forward(t.model,t.data)
    return None,dict(stats,wall_s=time.monotonic()-start)


def direct_pick(t,robot,name,goal_pose,board,budget_s=45.,check_seating=False):
    plan,stats=direct_grasp_candidate(t,robot,name,goal_pose,board,budget_s,check_seating)
    t.event('unit_direct_grasp_screen',robot=robot,object=name,statistics=stats,
        candidate_found=plan is not None,scope='Private scratch endpoints, not live feasibility proof')
    if plan is None:raise DirectGraspPlanningError(robot,name,stats)
    t.event('unit_direct_grasp_choice',robot=robot,object=name,height_m=plan['height'],tilt_deg=plan['tilt_deg'])
    width=float(t.task['cube_size_m']);opening=min(t.arms[robot].grip_max,width+.012)
    t.execute('unit-side-open',{robot:('gripper',dict(mode='move',width_m=opening,speed_m_s=.05))})
    t.move_free(robot,plan['approach'],plan['rotation'],'unit-side-approach',target_joints=plan['approach_q'],
        continuation=(plan['point'],plan['rotation'],(name,)))
    t.move(robot,plan['point'],plan['rotation'],'unit-side-contact',precise=True)
    t.execute('pick-grasp',{robot:('gripper',dict(mode='grasp',width_m=width,force_n=min(getattr(t,'transport_grasp_force_n',70.),t.cfg['allowed_grasp_force_n'][1]),speed_m_s=.025,
        epsilon_inner_m=.008,epsilon_outer_m=.008))})
    t.wait(.15)
    if not t.finger_contacts(robot,name):raise RuntimeError('Direct grasp did not establish real bilateral contact')
    before=t.data.body(name).xpos.copy();arm=t.arms[robot]
    if getattr(t,'transport_guarded_pickups',False):
        from .transport_grasp_safety import guarded_pickup_lift
        guarded_pickup_lift(t,robot,name,before[2],plan['rotation'])
    else:
        t.move(robot,t.data.site_xpos[arm.site]+np.array([0,0,.14]),plan['rotation'],'pick-lift',speed_m_s=.025)
        t.wait(.1)
        if not t.finger_contacts(robot,name) or t.data.body(name).xpos[2]<before[2]+.08:
            raise RuntimeError('Direct side pickup lost real payload support')
    t.held[robot]=name;t.event('picked',robot=robot,object=name)
    if check_seating and t.task['level']>1:
        if not hasattr(t,'socket_seed_hints'):t.socket_seed_hints={}
        t.socket_seed_hints[robot]=[plan['final_contact_q']]
    return plan
