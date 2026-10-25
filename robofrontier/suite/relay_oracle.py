"""Private finite-horizon full-arm candidates and contact-based airborne relay."""
import math
from dataclasses import dataclass
import mujoco,numpy as np
from .scenes import rz
from ..robot_control import rotation_vector

@dataclass
class HandoffPlan:
    position: np.ndarray
    yaw: float
    receivers: list
    choices: dict | None
    early_release: str | None = None
    first_receiver: str | None = None

def grasp_rotation(t,robot,object_r,pair_yaw):
    if t.task['mechanism'].get('beam_alignment')=='tangent':
        pair=next(p for p in t.task['pairs'] if robot in p['robots']);sign=1 if pair['robots'][0]==robot else -1
        return object_r@np.array([[0,sign,0],[sign,0,0],[0,0,-1.]])
    return object_r@rz(-pair_yaw)@t.home[robot][1]

def targets(t,robots,offset,center=None,rotation=None,pair_yaw=None):
    p=t.data.body('payload').xpos.copy() if center is None else np.asarray(center);r=t.data.body('payload').xmat.reshape(3,3).copy() if rotation is None else rotation
    pair=next(x for x in t.task['pairs'] if set(x['robots'])==set(robots));yaw=pair['yaw_rad'] if pair_yaw is None else pair_yaw
    return {robot:(p+r@np.array([0,(-1 if j==0 else 1)*offset,.006]),grasp_rotation(t,robot,r,yaw)) for j,robot in enumerate(robots)}

def free_of_robot_contacts(t,qs,payload_pose=None,open_robots=(),check_payload=True):
    scratch=t.screen_data;scratch.qpos[:]=t.data.qpos
    if payload_pose is not None:
        from ..robot_control import matrix_quat
        p,r=payload_pose;scratch.joint('payload_free').qpos[:]=np.r_[p,matrix_quat(r)]
    for robot,q in qs.items():
        scratch.qpos[t.arms[robot].qids]=q
        for suffix in ('finger_joint1','finger_joint2'):scratch.joint(robot+'_'+suffix).qpos[0]=.04 if robot in open_robots else .0225
    mujoco.mj_kinematics(t.screen_model,scratch);mujoco.mj_comPos(t.screen_model,scratch);mujoco.mj_collision(t.screen_model,scratch)
    if not scratch.ncon:return True
    pairs=np.asarray(scratch.contact.geom);owner=t.monitor.geom_owner[pairs]
    if np.any((owner[:,0]>=0)&(owner[:,1]>=0)&(owner[:,0]!=owner[:,1])&(np.asarray(scratch.contact.dist)<.002)):return False
    if not check_payload:return True
    payload=set(t.monitor.geoms['payload'])
    for contact in scratch.contact:
        if contact.dist>=.001:continue
        for part,robot in ((int(contact.geom1),int(contact.geom2)),(int(contact.geom2),int(contact.geom1))):
            if part in payload and t.monitor.geom_owner[robot]>=0 and t.monitor.finger[robot]<0:return False
    return True

def variant_target(t,robots,offset,robot,roll=0.,turn=0.,center=None,rotation=None,pair_yaw=None):
    from scripts.verify_floating_dial_oracle import axis_rotation
    point,base=targets(t,robots,offset,center,rotation,pair_yaw)[robot];r=t.data.body('payload').xmat.reshape(3,3).copy() if rotation is None else rotation
    local=axis_rotation([0,1,0],roll);point=point+r@(local@np.array([0,0,.006])-np.array([0,0,.006]))
    return point,r@local@r.T@base@rz(turn)

def close_receiver(t,robot):
    for force in (50.,30.):
        result=t.execute('receive-grasp',{robot:('gripper',dict(mode='grasp',width_m=.045,force_n=force,speed_m_s=.04,epsilon_inner_m=.006,epsilon_outer_m=.006))},allow_failure=True)[robot]
        if result['status']=='uncertain':raise RuntimeError('Receiving grasp unresolved '+robot)
        t.wait(.25)
        if result['status']=='completed' and robot in t.monitor.bilateral(t.monitor.geoms['payload'],positive_force=True):return
        if t.arms[robot].observe()['gripper_width_m']<.039:break
    raise RuntimeError('Receiving force grasp did not settle '+robot)

def receive_one(t,robots,offset,robot,choice):
    t.execute('receive-preshape',{robot:('gripper',dict(mode='move',width_m=.060,speed_m_s=.08))})
    point,r=variant_target(t,robots,offset,robot,choice['roll'],choice['turn'])
    contact=t.screen_ik(robot,point,r,seed=np.asarray(choice['seed']));pre=None if contact is None else t.screen_ik(robot,point-r[:,2]*.05,r,seed=contact)
    if pre is None:raise RuntimeError('Selected receiving branch no longer reaches its approach '+robot)
    t.move_free(robot,point-r[:,2]*.05,r,'receive-selected-approach',continuation=(point,r,('payload',)),target_joints=pre)
    origin=t.data.site_xpos[t.arms[robot].site].copy();seed=pre
    # The chosen elbow branch was checked for the next carry. Do not let an
    # unrelated endpoint seed silently replace it during the last 5 cm.
    for f in np.linspace(0,1,6)[1:]:
        point,r=variant_target(t,robots,offset,robot,choice['roll'],choice['turn']);target=origin*(1-f)+point*f;q=t.screen_ik(robot,target,r,seed=seed)
        if q is None:raise RuntimeError('Selected receiving contact branch lost '+robot)
        t.execute('receive-selected-contact',{robot:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.20))});seed=q
    close_receiver(t,robot)

def grip(t,robots,offset,choices=None):
    if choices:
        t.execute('receive-preshape',{robot:('gripper',dict(mode='move',width_m=.060,speed_m_s=.08)) for robot in robots})
        for robot in reversed(robots):
            receive_one(t,robots,offset,robot,choices[robot])
        actual=t.monitor.bilateral(t.monitor.geoms['payload'],positive_force=True)
        if not set(robots)<=actual:raise RuntimeError('Selected receiving grasp missing '+str(set(robots)-actual))
        t.event('relay_grip',robots=robots,offset_m=offset,actual_support=sorted(actual),selected_grasps={a:{k:v for k,v in choice.items() if k!='seed'} for a,choice in choices.items()});return
    points=targets(t,robots,offset);relative={}
    for a,(p,r) in points.items():
        options=[]
        for turn in (0,math.pi):
            rr=r@rz(turn);q=t.screen_ik(a,p,rr,seed=t.home_q[a])
            if q is None:continue
            margin=float(np.min(np.minimum(q-t.arms[a].ik.limits[:,0],t.arms[a].ik.limits[:,1]-q)))
            options.append((-margin,rr,q))
        ordered=sorted(options,key=lambda v:v[0]) if t.task['mechanism'].get('beam_alignment')=='tangent' else options
        for _,rr,q in ordered:
            try:t.move_free(a,p+np.array([0,0,.05]),rr,'receive-approach',continuation=(p,rr,('payload',)),preferred_seed=q)
            except RuntimeError as error:
                if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                raise
            relative[a]=t.data.body('payload').xmat.reshape(3,3).T@rr;break
        if a not in relative:raise RuntimeError('No full-link receiving grasp '+a)
    points=targets(t,robots,offset)
    points={a:(p,t.data.body('payload').xmat.reshape(3,3)@relative[a]) for a,(p,r) in points.items()}
    t.execute('receive-contact',{a:t.pose_command(a,p,r,.025,.3) for a,(p,r) in points.items()})
    result=t.execute('receive-grasp',{a:('gripper',dict(mode='grasp',width_m=.045,force_n=50.,speed_m_s=.04,epsilon_inner_m=.006,epsilon_outer_m=.006)) for a in robots},allow_failure=True);t.wait(.45)
    for robot,value in result.items():
        if value['status']=='uncertain':raise RuntimeError('Receiving grasp unresolved '+robot)
        if value['status']!='completed':close_receiver(t,robot)
    actual=t.monitor.bilateral(t.monitor.geoms['payload'],positive_force=True)
    if not set(robots)<=actual:raise RuntimeError('Receiving bilateral force contact missing '+str(set(robots)-actual))
    t.event('relay_grip',robots=robots,offset_m=offset,actual_support=sorted(actual))

def carry(t,robots,point,yaw):
    p0=t.data.body('payload').xpos.copy();r0=t.data.body('payload').xmat.reshape(3,3).copy();r1=rz(yaw);vector=rotation_vector(r1,r0);angle=float(np.linalg.norm(vector));steps=max(1,math.ceil(np.linalg.norm(np.asarray(point)-p0)/.035),math.ceil(angle/math.radians(5)))
    from scripts.verify_floating_dial_oracle import axis_rotation
    for k in range(1,steps+1):
        f=k/steps;p=p0*(1-f)+np.asarray(point)*f;r=r0 if angle<1e-9 else axis_rotation(vector/angle,angle*f)@r0
        actual=t.data.body('payload');ar=actual.xmat.reshape(3,3).copy();commands={}
        for robot in robots:
            arm=t.arms[robot];off=ar.T@(t.data.site_xpos[arm.site]-actual.xpos);relative=ar.T@t.data.site_xmat[arm.site].reshape(3,3);commands[robot]=t.pose_command(robot,p+r@off,r@relative,.04,.35)
        results=t.execute('relay-carry-'+str(k),commands,allow_failure=True)
        for robot,result in results.items():
            if result['status']=='completed':continue
            if result['status']=='uncertain':raise RuntimeError('Unresolved carry action '+robot)
            goal=commands[robot][1];arm=t.arms[robot]
            from ..robot_control import quat_matrix
            distance=np.linalg.norm(t.data.site_xpos[arm.site]-goal['target_m']);angle_error=np.linalg.norm(rotation_vector(quat_matrix(goal['quaternion_wxyz']),t.data.site_xmat[arm.site].reshape(3,3)))
            if result.get('reason')!='Action timeout; stopped at current reference' or distance>.006 or angle_error>.06:raise RuntimeError('Carry waypoint not reached after controlled stop '+robot)
            t.event('near_waypoint_after_known_stop',robot=robot,distance_m=float(distance),angle_error_rad=float(angle_error),scope='Reference accepts approximate intermediate waypoint; the underlying action remains failed')
        support=t.monitor.bilateral(t.monitor.geoms['payload'],positive_force=True)
        if not set(robots)<=support:
            # Respect the evaluator's declared 200 ms contact-gap policy.
            # Re-check after a short physical settling interval; no state edits
            # or automatic regrasp. The continuous monitor remains active.
            t.wait(.12);support=t.monitor.bilateral(t.monitor.geoms['payload'],positive_force=True)
        if not set(robots)<=support:raise RuntimeError('Support lost during carry '+str(set(robots)-support))
        if t.monitor.violations:raise RuntimeError('Relay process violation '+str(t.monitor.violations[-1]))
    t.wait(.25)

def reachable_carry(t,robot,q,off,relative,p0,yaw0,p1,yaw1,return_seed=False):
    """Private continuous IK branch probe, including the following carry.

    Endpoint reach alone misses a wrist-limit trap immediately after handover.
    This probe is kinematic evidence only; execution remains contact driven.
    """
    delta=(yaw1-yaw0+math.pi)%(2*math.pi)-math.pi
    for f in np.linspace(0,1,13)[1:]:
        p=np.asarray(p0)*(1-f)+np.asarray(p1)*f;r=rz(yaw0+delta*f)
        q=t.screen_ik(robot,p+r@off,r@relative,seed=q)
        if q is None:return None if return_seed else False
        if min(q[-1]-t.arms[robot].ik.limits[-1,0],t.arms[robot].ik.limits[-1,1]-q[-1])<.05:return None if return_seed else False
    return q if return_seed else True

def handoff_candidate(t,index,sender,offset_receiver):
    a=t.task['route'][index];b=t.task['route'][index+1];receiver=t.task['pairs'][(index+1)%len(t.task['pairs'])]['robots'];pa=np.asarray(a['center_m']);pb=np.asarray(b['center_m']);delta=(b['yaw_rad']-a['yaw_rad']+math.pi)%(2*math.pi)-math.pi
    payload=t.data.body('payload');ar=payload.xmat.reshape(3,3);offsets={robot:(ar.T@(t.data.site_xpos[t.arms[robot].site]-payload.xpos),ar.T@t.data.site_xmat[t.arms[robot].site].reshape(3,3)) for robot in sender}
    proposals=[]
    for bias in ((.15,.20,.10,0,-.10) if len(t.task['pairs'])==8 else (0,)):
        for fraction in (.5,.4,.6,.3,.7):
            for turn in (.5,.25,.75,0,1):
                angle=a['yaw_rad']+delta*turn;r=rz(angle);p=pa*(1-fraction)+pb*fraction;p[2]=.80 if t.task['mechanism'].get('beam_alignment')=='tangent' else .84 if len(t.task['pairs'])>4 else .92
                p+=(r[:,0] if t.task['mechanism'].get('beam_alignment')=='tangent' else -r[:,1])*bias;qs={}
                for robot,(off,relative) in offsets.items():
                    q=t.screen_ik(robot,p+r@off,r@relative)
                    if q is None or not reachable_carry(t,robot,t.arms[robot].qref,off,relative,payload.xpos,math.atan2(ar[1,0],ar[0,0]),p,angle):break
                    qs[robot]=q
                else:proposals.append((p,r,angle,qs))
    if t.task['mechanism'].get('beam_alignment')!='tangent':
        # Preserve a simple common top grasp when it works. A receiving grasp
        # need not become a side grasp just because a broader search exists.
        for p,r,angle,send_q in proposals:
            qs=dict(send_q)
            for robot,(point,rr) in targets(t,receiver,offset_receiver,p,r,b['yaw_rad']).items():
                q=t.screen_ik(robot,point,rr)
                if q is None or t.screen_ik(robot,point+np.array([0,0,.05]),rr,seed=q) is None:break
                qs[robot]=q
            else:
                if free_of_robot_contacts(t,qs,(p,r)):return HandoffPlan(p,angle,receiver,None)
    if t.task['mechanism'].get('beam_alignment')=='tangent':
        # Two continuous supports do not require four simultaneous grasps.
        # Try top grasps in a three/two/three support sequence before using
        # awkward side grasps. The temporary bridge must span the load COM.
        stats=dict(proposals=len(proposals),grasp_ik_rejected=0,future_rejected=0,bridge_com_rejected=0,collision_rejected=0)
        for p,r,angle,send_q in proposals:
            for ordered in (receiver,list(reversed(receiver))):
                for flips in ((0.,0.),(0.,math.pi),(math.pi,0.),(math.pi,math.pi)):
                    qs={};choices={};points={};valid=True
                    for robot,flip in zip(ordered,flips):
                        point,rr=variant_target(t,ordered,offset_receiver,robot,0.,flip,p,r,b['yaw_rad']);q=t.screen_ik(robot,point,rr,seed=t.home_q[robot])
                        if q is None or t.screen_ik(robot,point-rr[:,2]*.05,rr,seed=q) is None:stats['grasp_ik_rejected']+=1;valid=False;break
                        off=r.T@(point-p);relative=r.T@rr;following=np.asarray(b['center_m']).copy();following[2]=p[2]
                        next_q=reachable_carry(t,robot,q,off,relative,p,angle,following,b['yaw_rad'],return_seed=True)
                        if next_q is None:stats['future_rejected']+=1;valid=False;break
                        if index+2<len(t.task['route']):
                            nxt=t.task['route'][index+2];lead=following*.65+np.asarray(nxt['center_m'])*.35;lead[2]=p[2];dy=(nxt['yaw_rad']-b['yaw_rad']+math.pi)%(2*math.pi)-math.pi
                            if not reachable_carry(t,robot,next_q,off,relative,following,b['yaw_rad'],lead,b['yaw_rad']+dy*.35):stats['future_rejected']+=1;valid=False;break
                        qs[robot]=q;points[robot]=point;choices[robot]=dict(roll=0.,turn=flip,seed=q.tolist())
                    if not valid:continue
                    com_y=float((ar.T@(t.data.subtree_com[t.model.body('payload').id]-payload.xpos))[1])
                    for first in ordered:
                        for early in sender:
                            remaining=next(robot for robot in sender if robot!=early);y0=float(offsets[remaining][0][1]);y1=float((r.T@(points[first]-p))[1])
                            if not min(y0,y1)-.01<=com_y<=max(y0,y1)+.01:stats['bridge_com_rejected']+=1;continue
                            before=dict(send_q);before[first]=qs[first]
                            after={remaining:send_q[remaining],early:t.home_q[early],**qs}
                            if free_of_robot_contacts(t,before,(p,r)) and free_of_robot_contacts(t,after,(p,r),open_robots=(early,)):
                                # Execution order is separate from the signed
                                # grasp-point mapping retained in ordered.
                                t.event('reference_staged_search',outcome='accepted',statistics=stats)
                                return HandoffPlan(p,angle,ordered,choices,early,first)
                            stats['collision_rejected']+=1
        t.event('reference_staged_search',outcome='sampled_candidates_exhausted',statistics=stats)
    variants=[(0.,0.),(0.,math.pi/2),(0.,-math.pi/2),(math.pi/2,0.),(-math.pi/2,0.),(math.pi/2,-math.pi/2),(-math.pi/2,math.pi/2),(math.pi/2,math.pi/2),(-math.pi/2,-math.pi/2),(math.pi,0.),(0.,math.pi)]
    # First preserve the simple top grasp. If that fails, consider real
    # square-section side/bottom grasps instead of declaring workspace overlap
    # an unavoidable conflict. These are private geometric candidates.
    for rolls in variants:
        for p,r,angle,send_q in proposals:
            for turns in ((0.,0.),(0.,math.pi),(math.pi,0.),(math.pi,math.pi)):
                qs=dict(send_q);choices={}
                for robot,roll,turn in zip(receiver,rolls,turns):
                    point,rr=variant_target(t,receiver,offset_receiver,robot,roll,turn,p,r,b['yaw_rad']);q=t.screen_ik(robot,point,rr)
                    if q is None:q=t.screen_ik(robot,point,rr,seed=t.home_q[robot])
                    margin=.8 if t.task['mechanism'].get('beam_alignment')=='tangent' else .12
                    if q is None or min(q[-1]-t.arms[robot].ik.limits[-1,0],t.arms[robot].ik.limits[-1,1]-q[-1])<margin or t.screen_ik(robot,point-rr[:,2]*.05,rr,seed=q) is None:break
                    following=np.asarray(b['center_m']).copy();following[2]=p[2]
                    off=r.T@(point-p);relative=r.T@rr
                    if not reachable_carry(t,robot,q,off,relative,p,angle,following,b['yaw_rad']):break
                    qs[robot]=q;choices[robot]=dict(roll=roll,turn=turn,seed=q.tolist())
                else:
                    if free_of_robot_contacts(t,qs,(p,r)):return HandoffPlan(p,angle,receiver,choices)
    raise RuntimeError('No sampled four-arm handoff pose '+str(index+1))

def release(t,robots):
    result=t.execute('send-open',{a:('gripper',dict(mode='move',width_m=.08,speed_m_s=.05)) for a in robots},allow_failure=True)
    for a,r in result.items():
        if r['status']=='uncertain' or (r['status']=='failed' and r.get('reason') not in ('Requested opening not reached','Requested aperture exceeded')):raise RuntimeError('Sender release unresolved '+a)
    t.wait(.3)
    for a in robots:
        arm=t.arms[a];origin=t.data.site_xpos[arm.site].copy();r=t.data.site_xmat[arm.site].reshape(3,3).copy()
        for direction in (-r[:,2],np.array([0,0,1.])):
            q=t.data.qpos[arm.qids].copy();valid=True
            for f in np.linspace(0,1,7)[1:]:
                q=t.screen_ik(a,origin+direction*.06*f,r,seed=q)
                if q is None or not free_of_robot_contacts(t,{a:q},open_robots=(a,),check_payload=False):valid=False;break
            if valid:
                t.event('sender_departure_screen',robot=a,scope='Robot-robot sampled clearance only; departing payload contact remains in live physics')
                t.execute('send-withdraw',{a:t.pose_command(a,origin+direction*.06,r,.025,.25)});break
        try:t.move_free(a,*t.home[a],label='send-clear',target_joints=t.home_q[a])
        except RuntimeError as error:
            if not str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):raise
            # Returning to HOME is not a task requirement. After a known jaw
            # opening, leave the real idle posture in the scene if no sampled
            # home route was found; later planning includes that exact posture
            # and the continuous monitor still rejects forbidden collisions.
            clear_width=float(np.linalg.norm(t.task['mechanism']['beam_cross_section_m']))+.008
            if arm.job is not None or arm.grasp_force is not None or arm.observe()['gripper_width_m']<clear_width or t.monitor.violations:raise
            t.event('sender_idle_without_home',robot=a,aperture_m=arm.observe()['gripper_width_m'],reason=str(error),scope='Known stopped/open hand; no guaranteed HOME or global clearance is claimed. Subsequent plans use the real state.')
    t.wait(.2)

def solve(t):
    pairs=t.task['pairs'];route=t.task['route'];sender=pairs[0]['robots'];tangent=t.task['mechanism'].get('beam_alignment')=='tangent';outer=.40 if tangent else .45;inner=.25 if tangent else .30 if len(pairs)==8 else .25 if len(pairs)==6 else .18
    # Select a physically reachable initial support spacing from finite candidates.
    chosen=None
    for candidate in ((.40,.45,.35,.50) if tangent else (.45,.40,.50,.53)):
        points=targets(t,sender,candidate)
        if all(t.screen_ik(a,p,r) is not None and t.screen_ik(a,p+np.array([0,0,.12]),r) is not None for a,(p,r) in points.items()):chosen=candidate;break
    if chosen is None:raise RuntimeError('No initial dual-arm support grasp')
    outer=chosen;grip(t,sender,outer);start=route[0];p=np.array(start['center_m']);p[2]=.80 if tangent else .95;carry(t,sender,p,start['yaw_rad']);t.wait(.4);t.snapshot('initial-lift')
    if len(pairs)>4 and not tangent:
        p=p+rz(start['yaw_rad'])@np.array([.14,0,0]);carry(t,sender,p,start['yaw_rad'])
    for index in range(len(route)-1):
        receiver=pairs[(index+1)%len(pairs)]['robots'];offset=inner if (index+1)%2 else outer
        plan=None
        for candidate in dict.fromkeys((offset,offset-.04,offset+.04,offset-.08,offset+.08,.18,.25,.35,.45)):
            if not .14<=candidate<=.52:continue
            try:plan=handoff_candidate(t,index,sender,candidate)
            except RuntimeError as error:
                if str(error).startswith('No sampled four-arm handoff pose'):continue
                raise
            offset=candidate;break
        if plan is None:raise RuntimeError('No receiving grasp spacing and four-arm handoff pose '+str(index+1))
        receiver=plan.receivers;carry(t,sender,plan.position,plan.yaw)
        if plan.early_release is not None:
            first=plan.first_receiver;second=next(a for a in receiver if a!=first);remaining=next(a for a in sender if a!=plan.early_release)
            t.event('staged_handover',first_receiver=first,early_sender_release=plan.early_release,remaining_sender=remaining,second_receiver=second,scope='At least two physical beam grasps throughout; no four-grasp requirement')
            receive_one(t,receiver,offset,first,plan.choices[first]);release(t,[plan.early_release])
            if not {remaining,first}<=t.monitor.bilateral(t.monitor.support_geoms,positive_force=True):raise RuntimeError('Temporary bridge support absent')
            receive_one(t,receiver,offset,second,plan.choices[second]);t.wait(.3)
            if not set(receiver+[remaining])<=t.monitor.bilateral(t.monitor.support_geoms,positive_force=True):raise RuntimeError('Receiving overlap support absent')
            t.snapshot(f'handoff-{index+1}-overlap',('global','global_rear'));release(t,[remaining])
        else:
            grip(t,receiver,offset,plan.choices)
            all_support=t.monitor.bilateral(t.monitor.support_geoms,positive_force=True)
            if not set(sender+receiver)<=all_support:raise RuntimeError('Four-arm overlap not established')
            t.wait(.3);t.snapshot(f'handoff-{index+1}-overlap',('global','global_rear'));release(t,sender)
        sender=receiver
        if t.monitor.route_index!=index+1:raise RuntimeError('Custody evaluator did not confirm handoff '+str(index+1))
        frame=route[index+1];p=np.asarray(frame['center_m']).copy();p[2]=.80 if tangent else .94 if index+1==len(route)-1 else .84 if len(pairs)>4 else .92;carry(t,sender,p,frame['yaw_rad'])
        print(f"{t.task['id']} verified handoff {index+1}/{len(route)-1}",flush=True)
    final=route[-1];carry(t,sender,np.asarray(final['center_m'])+np.array([0,0,.0001]),final['yaw_rad']);t.wait(.5)
    t.execute('final-open',{a:('gripper',dict(mode='move',width_m=.08,speed_m_s=.05)) for a in sender},allow_failure=True)
    t.execute('final-withdraw',{a:t.pose_command(a,t.data.site_xpos[t.arms[a].site]+np.array([0,0,.15]),t.data.site_xmat[t.arms[a].site].reshape(3,3),.04,.3) for a in sender});t.wait(1.5)
