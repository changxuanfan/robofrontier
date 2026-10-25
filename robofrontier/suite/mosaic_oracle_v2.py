"""Private actuator reference for v2 mosaics; no task-policy recipes.

The reference may read evaluator state. All live changes are ordinary Trial
commands; screening does not certify contact manipulation or final acceptance.
"""
from collections import Counter
import numpy as np
from . import placement_oracle as helpers


def validate_task(task):
    """Reject unsupported input before allocating a physical attempt."""
    if task.get('family') != 'mosaic' or task.get('suite_version') != 2:
        raise ValueError('Mosaic v2 reference requires a v2 mosaic task')
    if task.get('level') not in (1, 2, 3):
        raise ValueError('Unsupported mosaic level')
    size = float(task['cube_size_m'])
    if not np.isfinite(size) or size <= 0:
        raise ValueError('Cube size must be finite and positive')
    if not np.isfinite(task['table_height_m']):
        raise ValueError('Invalid table datum')
    goals = task['goals']
    if len({g['goal_id'] for g in goals}) != len(goals):
        raise ValueError('Duplicate mosaic goal')
    stock = Counter(o['color'] for o in task['objects'])
    demand = Counter(g['requires']['color'] for g in goals)
    if demand - stock:
        raise ValueError('Insufficient matching color supply')
    for g in goals:
        if len(g['position_m']) != 3 or not np.isfinite(g['position_m']).all():
            raise ValueError('Invalid goal position')
    return dict(goals=len(goals), cube_size_m=size,
                table_height_m=float(task['table_height_m']),
                approach_width_m=size + .012)


def approach_width(trial, robot):
    width = min(float(trial.arms[robot].grip_max),
                float(trial.task['cube_size_m']) + .012)
    if width <= float(trial.task['cube_size_m']) + .002:
        raise RuntimeError('Insufficient gripper aperture for cube approach')
    return width





def validate_motion_profile(profile):
    profile={} if profile is None else dict(profile)
    allowed={'loaded_joint_speed_rad_s','coarse_tcp_speed_m_s'}
    if set(profile)-allowed:raise ValueError('Unknown reference speed profile field')
    for key,value in profile.items():
        if not np.isfinite(value) or value<=0:raise ValueError('Finite positive reference speed required')
        profile[key]=float(value)
    return profile


def speed_kwargs(trial,key,parameter):
    value=getattr(trial,'mosaic_motion_profile',{}).get(key)
    return {} if value is None else {parameter:value}


def stable_relocation(samples):
    """A sampled half-second hold, never just two coincident endpoint frames."""
    from ..robot_control import rotation_vector
    if len(samples)<10 or samples[-1]['sim_time_s']-samples[0]['sim_time_s']<.4:return False
    if max(np.diff([r['sim_time_s'] for r in samples]))>.021:return False
    if not all(r['bilateral'] and r['airborne'] and r['free_support'] for r in samples):return False
    offsets=np.asarray([r['offset_m'] for r in samples]);rotations=np.asarray([r['relative_rotation'] for r in samples])
    speeds=np.asarray([[r['linear_speed_m_s'],r['angular_speed_rad_s']] for r in samples])
    if not np.isfinite(offsets).all() or not np.isfinite(rotations).all() or not np.isfinite(speeds).all():return False
    return bool(np.max(np.linalg.norm(offsets-offsets[0],axis=1))<.0005 and
                max(np.linalg.norm(rotation_vector(r,rotations[0])) for r in rotations)<.02 and
                np.max(speeds[:,0])<.005 and np.max(speeds[:,1])<.05)


def grip_observation(trial,robot):
    import mujoco
    name=trial.held[robot];arm=trial.arms[robot];body=trial.data.body(name)
    tool=trial.data.site_xmat[arm.site].reshape(3,3);velocity=np.empty(6)
    mujoco.mj_objectVelocity(trial.model,trial.data,mujoco.mjtObj.mjOBJ_BODY,trial.model.body(name).id,velocity,0)
    geoms=set(trial.monitor.geoms[name]);free=True
    for contact in trial.data.contact:
        a,b=map(int,contact.geom)
        if a not in geoms and b not in geoms:continue
        other=b if a in geoms else a
        if other in geoms:continue
        owner=trial.model.body(int(trial.model.geom_bodyid[other])).name
        if contact.dist<.0005 and owner not in (robot+'_left_finger',robot+'_right_finger'):free=False
    return dict(sim_time_s=trial.time,offset_m=(tool.T@(body.xpos-trial.data.site_xpos[arm.site])).tolist(),
                relative_rotation=(tool.T@body.xmat.reshape(3,3)).tolist(),
                linear_speed_m_s=float(np.linalg.norm(velocity[3:])),angular_speed_rad_s=float(np.linalg.norm(velocity[:3])),
                bilateral=trial.finger_contacts(robot,name),airborne=bool(body.xpos[2]>trial.task['table_height_m']+trial.task['cube_size_m']),free_support=free)


def relocalize_payload(trial,robot,old_offset,old_rotation):
    samples=[grip_observation(trial,robot)]
    for _ in range(25):
        trial.wait(.02);samples.append(grip_observation(trial,robot))
        if not all(samples[-1][key] for key in ('bilateral','airborne','free_support')):
            trial.event('grip_relocalization_rejected',robot=robot,samples=samples)
            raise RuntimeError('Payload lost independent support during re-observation '+trial.held[robot])
    if not stable_relocation(samples):
        trial.event('grip_relocalization_rejected',robot=robot,samples=samples)
        raise RuntimeError('Payload still drifting during bounded re-observation '+trial.held[robot])
    trial.event('grip_relocalized',robot=robot,object=trial.held[robot],
                old_grip=dict(offset_m=np.asarray(old_offset).tolist(),relative_rotation=np.asarray(old_rotation).tolist()),
                new_grip=dict(offset_m=samples[-1]['offset_m'],relative_rotation=samples[-1]['relative_rotation']),
                window_s=samples[-1]['sim_time_s']-samples[0]['sim_time_s'],samples=samples,
                position_drift_limit_m=.0005,rotation_drift_limit_rad=.02,linear_speed_limit_m_s=.005,angular_speed_limit_rad_s=.05,
                scope='Private reference observation; all subsequent paths require fresh current-grip screening')
    return np.asarray(samples[-1]['offset_m'])


def check_payload(trial, robot, reference_offset=None, label='feedback', relocalize=False, reference_rotation=None):
    """Never infer a carried transform from a payload already dropped on a support."""
    name=trial.held[robot]
    if not trial.finger_contacts(robot,name):
        raise RuntimeError('Payload contact missing before '+label+' '+name)
    arm=trial.arms[robot]
    tcp=trial.data.site_xpos[arm.site]
    tool_r=trial.data.site_xmat[arm.site].reshape(3,3)
    offset=tool_r.T@(trial.data.body(name).xpos-tcp)
    if reference_offset is not None and np.linalg.norm(offset-reference_offset)>trial.task['cube_size_m']*.5:
        if not relocalize or reference_rotation is None:raise RuntimeError('Payload slipped during '+label+' '+name)
        return relocalize_payload(trial,robot,reference_offset,reference_rotation)
    return offset.copy()


def align_payload(trial, robot, position, rotation, label, coarse=False):
    """Finite Cartesian feedback compensates physical grip slip, never state."""
    from ..robot_control import rotation_vector
    name = trial.held[robot]
    arm = trial.arms[robot]
    for attempt in range(3):
        check_payload(trial,robot,label=label)
        body = trial.data.body(name)
        actual_r = body.xmat.reshape(3, 3).copy()
        tcp = trial.data.site_xpos[arm.site].copy()
        tcp_r = trial.data.site_xmat[arm.site].reshape(3, 3).copy()
        offset = actual_r.T @ (tcp-body.xpos)
        relative = actual_r.T @ tcp_r
        trial.move(robot, np.asarray(position)+rotation@offset, rotation@relative,
                   label+'-'+str(attempt), precise=True,
                   **(speed_kwargs(trial,'coarse_tcp_speed_m_s','speed_m_s') if coarse else {}))
        trial.wait(.10)
        if not trial.finger_contacts(robot, name):
            raise RuntimeError('Payload contact lost in feedback placement '+name)
        body = trial.data.body(name)
        error = float(np.linalg.norm(body.xpos-position))
        angle = float(np.linalg.norm(rotation_vector(rotation, body.xmat.reshape(3,3))))
        trial.event('mosaic_payload_alignment', robot=robot, object=name,
                    label=label, attempt=attempt+1, position_error_m=error,
                    orientation_error_rad=angle)
        if error < .0015 and angle < .025:
            return
    if error > .003 or angle > .05:
        raise RuntimeError('Physical payload alignment did not converge '+name)



JOINT_MARGIN_RAD = .04


def joint_margin(trial, robot, q):
    limits = trial.arms[robot].ik.limits
    return float(np.min(np.minimum(np.asarray(q)-limits[:,0],limits[:,1]-np.asarray(q))))


def loaded_ik_candidates(trial, robot, target, rotation, allowed_objects=()):
    """Finite IK seeds with correction room; no endpoint is a route proof."""
    arm=trial.arms[robot]
    seeds=[arm.qref,trial.home_q[robot]]
    for delta in (-.5,.5):
        seed=trial.home_q[robot].copy();seed[0]+=delta;seed[2]-=delta
        seeds.append(np.clip(seed,*arm.ik.limits.T))
    choices=[]
    for seed in seeds:
        q=trial.screen_ik(robot,target,rotation,seed=seed)
        if q is None or joint_margin(trial,robot,q)<JOINT_MARGIN_RAD:continue
        if trial.pose_clear(robot,q,allowed_objects):
            choices.append(q)
    return sorted(choices,key=lambda q:(-joint_margin(trial,robot,q),np.linalg.norm(q-arm.qref)))


def loaded_ik(trial, robot, target, rotation, allowed_objects=()):
    choices=loaded_ik_candidates(trial,robot,target,rotation,allowed_objects)
    return choices[0] if choices else None


def pose_window(trial, robot, target, rotation, seed, radius_m=.004):
    """Screen a real target and six correction offsets on one nearby IK branch."""
    q=trial.screen_ik(robot,target,rotation,seed=seed)
    if q is None or joint_margin(trial,robot,q)<JOINT_MARGIN_RAD or not trial.pose_clear(robot,q):return None
    margin=joint_margin(trial,robot,q)
    for axis in range(3):
        for sign in (-1,1):
            point=np.asarray(target).copy();point[axis]+=radius_m*sign
            nearby=trial.screen_ik(robot,point,rotation,seed=q)
            if nearby is None or joint_margin(trial,robot,nearby)<JOINT_MARGIN_RAD or not trial.pose_clear(robot,nearby):return None
            margin=min(margin,joint_margin(trial,robot,nearby))
    return q,margin


def screen_release(trial,robot,position,rotation,offset,relative,hover_q):
    """Release height, correction room and withdrawal, using current geometry."""
    arm=trial.arms[robot];tool=rotation@relative
    # Reserve the same 4 mm vertical room that the correction screen explores.
    clearance=trial.release_clearance_height(robot,tool,float((rotation@offset)[2]))+.004
    if clearance>trial.task['cube_size_m']*1.25:return None
    target=np.asarray(position)+[0,0,clearance]+rotation@offset
    origin,_=arm.ik.pose(hover_q);q=hover_q;minimum=joint_margin(trial,robot,q)
    for fraction in (.25,.5,.75,1.):
        point=origin*(1-fraction)+target*fraction
        q=trial.screen_ik(robot,point,tool,seed=q)
        if q is None or joint_margin(trial,robot,q)<JOINT_MARGIN_RAD or not trial.pose_clear(robot,q):return None
        minimum=min(minimum,joint_margin(trial,robot,q))
    window=pose_window(trial,robot,target,tool,q)
    if window is None:return None
    q,margin=window;minimum=min(minimum,margin)
    for height in (.025,.05,.075,.10):
        q=trial.screen_ik(robot,target+[0,0,height],tool,seed=q)
        if q is None or joint_margin(trial,robot,q)<JOINT_MARGIN_RAD or not trial.pose_clear(robot,q):return None
        minimum=min(minimum,joint_margin(trial,robot,q))
    return dict(clearance_m=clearance,minimum_joint_margin_rad=minimum)


def pocket_release_clearances(trial, position, current_clearance):
    """Finite lower drops for final pocket goals, never a new task tolerance."""
    if (trial.task.get('level') != 1 or
            trial.task.get('grid', {}).get('depth_m', 0.) <= 0 or
            not any(np.linalg.norm(np.asarray(g['position_m'])-position) < 1e-8
                    for g in trial.task['goals'])):
        return ()
    # The existing correction window reserves4mm. The nominal search spacing
    # scales with the cube; retain the original height if none can be screened.
    reserve = .004
    step = float(trial.task['cube_size_m'])/12
    return tuple(step*index+reserve
                 for index in range(1, int(np.floor((current_clearance-reserve)/step))+1)
                 if step*index+reserve < current_clearance-.0005)


def screen_lower_pocket_release(trial, robot, position):
    """Check actual held geometry through lowering, jaw opening and exit.

    Uses scratch data only. The released object is approximated at its settled
    height for exit screening; this remains a candidate, not a dynamics proof.
    """
    from copy import copy
    import mujoco
    from ..robot_control import matrix_quat
    from .oracle import Trial, joint_edge_clear

    name = trial.held[robot]
    arm = trial.arms[robot]
    body = trial.data.body(name)
    current_clearance = float(body.xpos[2]-position[2])
    heights = pocket_release_clearances(trial, position, current_clearance)
    if not heights:
        return None
    start = trial.data.site_xpos[arm.site].copy()
    tool = trial.data.site_xmat[arm.site].reshape(3, 3).copy()
    body_rotation = body.xmat.reshape(3, 3).copy()
    landed_position = body.xpos.copy()
    landed_position[2] = position[2]
    initial_width = arm.observe()['gripper_width_m']
    opening = approach_width(trial, robot)

    def proxy(width, landed=False):
        candidate = copy(trial)
        candidate.data = mujoco.MjData(trial.model)
        candidate.data.qpos[:] = trial.data.qpos
        candidate.data.qvel[:] = trial.data.qvel
        candidate.data.time = trial.data.time
        for suffix in ('finger_joint1', 'finger_joint2'):
            candidate.data.joint(robot+'_'+suffix).qpos[0] = (width-arm.grip_min)/2
        if landed:
            candidate.data.joint(name+'_free').qpos[:] = np.r_[
                landed_position, matrix_quat(body_rotation)]
        mujoco.mj_forward(trial.model, candidate.data)
        candidate.held = dict(trial.held)
        if landed:
            candidate.held.pop(robot, None)
        candidate.screen_data = mujoco.MjData(trial.screen_model)
        candidate.carry_geoms = dict(trial.carry_geoms)
        return candidate

    opening_proxy = proxy(initial_width)
    widths = np.linspace(initial_width, opening, 13)

    def opening_clear(joints):
        # Reuse one scratch state instead of keeping13 complete MjData arenas.
        for width in widths:
            for suffix in ('finger_joint1', 'finger_joint2'):
                opening_proxy.data.joint(robot+'_'+suffix).qpos[0] = (width-arm.grip_min)/2
            mujoco.mj_kinematics(trial.model, opening_proxy.data)
            if not Trial.pose_clear(opening_proxy, robot, joints):
                return False
        return True

    exit_proxy = proxy(opening, landed=True)
    for height in heights:
        target = start+np.array([0., 0., height-current_clearance])
        q = arm.qref.copy()
        minimum = joint_margin(trial, robot, q)
        valid = True
        for fraction in np.linspace(0., 1., 17):
            point = start+(target-start)*fraction
            following = trial.screen_ik(robot, point, tool, seed=q)
            if (following is None or joint_margin(trial, robot, following) < JOINT_MARGIN_RAD or
                    not joint_edge_clear(q, following,
                        lambda joints: trial.pose_clear(robot, joints), step_rad=.005)):
                valid = False
                break
            q = following
            minimum = min(minimum, joint_margin(trial, robot, q))
        if not valid:
            continue
        if not opening_clear(q):
            continue
        for axis in range(3):
            for sign in (-1, 1):
                point = target.copy()
                point[axis] += sign*.004
                nearby = trial.screen_ik(robot, point, tool, seed=q)
                if (nearby is None or joint_margin(trial, robot, nearby) < JOINT_MARGIN_RAD or
                        not opening_clear(nearby)):
                    valid = False
                    break
                minimum = min(minimum, joint_margin(trial, robot, nearby))
            if not valid:
                break
        if not valid:
            continue
        for rise in np.linspace(0., .10, 21):
            following = trial.screen_ik(robot, target+[0., 0., rise], tool, seed=q)
            if (following is None or joint_margin(trial, robot, following) < JOINT_MARGIN_RAD or
                    not joint_edge_clear(q, following,
                        lambda joints: Trial.pose_clear(exit_proxy, robot, joints), step_rad=.005)):
                valid = False
                break
            q = following
            minimum = min(minimum, joint_margin(trial, robot, q))
        if valid:
            return dict(clearance_m=float(height), target_m=target, rotation=tool,
                        original_clearance_m=current_clearance,
                        minimum_joint_margin_rad=minimum,
                        scope='Scratch lowering/opening/correction/exit samples; not a dynamics certificate')
    return None


def lower_pocket_release(trial, robot, position):
    candidate = screen_lower_pocket_release(trial, robot, position)
    if candidate is None:
        return
    trial.event('mosaic_lower_release_candidate', robot=robot,
                object=trial.held[robot], **{
                    key: value for key, value in candidate.items()
                    if key not in ('target_m', 'rotation')})
    trial.move(robot, candidate['target_m'], candidate['rotation'],
               label='mosaic-lower-release', precise=True)
    trial.wait(.10)


def execute_loaded_transit(trial, robot, target, rotation, joints):
    """Recover a task outcome, never rewrite an original failed tool result.

    A controller can report a stopped timeout while the physical endpoint is
    subsequently satisfied. Only a clean terminal move, stable original joint
    target and continuously retained payload may pass this independent gate.
    """
    try:
        return trial.move_free(robot,target,rotation,'mosaic-feedback-transit',target_joints=joints,
            **speed_kwargs(trial,'loaded_joint_speed_rad_s','joint_speed_rad_s'))
    except RuntimeError:
        actions=getattr(trial,'actions',[]);arm=trial.arms[robot]
        last=actions[-1] if actions else {};result=last.get('results',{}).get(robot,{})
        if (result.get('status')!='failed' or result.get('reason')!='Action timeout; stopped at current reference' or
                not last.get('label','').startswith('mosaic-feedback-transit') or
                any(a.job is not None for a in trial.arms.values()) or trial.monitor.violations or
                trial.held.get(robot) is None or arm.grasp_force is None):raise
        name=trial.held[robot];observations=[]
        if not trial.finger_contacts(robot,name):raise
        for _ in range(20):
            trial.wait(.02)
            observations.append(dict(joint_error_rad=float(np.max(abs(trial.data.qpos[arm.qids]-joints))),
                joint_speed_rad_s=float(np.max(abs(trial.data.qvel[arm.dofs]))),
                bilateral=bool(trial.finger_contacts(robot,name)),position_m=trial.data.body(name).xpos.copy().tolist()))
        positions=np.asarray([x['position_m'] for x in observations])
        verified=bool(all(x['joint_error_rad']<.004 and x['joint_speed_rad_s']<.03 and x['bilateral'] for x in observations) and
            np.max(np.linalg.norm(positions-positions[0],axis=1))<.002 and
            positions[:,2].min()>trial.task['table_height_m']+.08 and
            not trial.monitor.violations and trial.pose_clear(robot,trial.data.qpos[arm.qids]))
        evidence=dict(robot=robot,object=name,original_call_id=result.get('call_id'),
            original_status='failed',original_feedback_preserved=True,target_joint_rad=np.asarray(joints).tolist(),
            observations=observations,verified=verified)
        trial.event('mosaic_stopped_motion_outcome_review',**evidence)
        if not verified:raise
        if not hasattr(trial,'mosaic_recovered_motion_failures'):trial.mosaic_recovered_motion_failures=[]
        trial.mosaic_recovered_motion_failures.append(evidence)
        return dict(task_endpoint_verified_after_stop=True,original_feedback=result)


def place_cube(trial, robot, position, _grip_replans=0):
    """Transit high, remeasure grip, align before release, then withdraw."""
    from .oracle import CUBE_ROTATIONS
    from ..robot_control import rotation_vector
    name = trial.held[robot]
    arm = trial.arms[robot]
    body = trial.data.body(name)
    carried_offset=check_payload(trial,robot,label='loaded transit')
    old_r = body.xmat.reshape(3,3).copy()
    offset = old_r.T @ (trial.data.site_xpos[arm.site]-body.xpos)
    relative = old_r.T @ trial.data.site_xmat[arm.site].reshape(3,3)
    chosen = None
    rotations = sorted(CUBE_ROTATIONS,
                       key=lambda r: np.linalg.norm(rotation_vector(r,old_r)))
    candidates=[]
    tilted_grasp=getattr(trial,'mosaic_tilted_grasps',{}).get(robot,False)
    for height in (.08,.12,.16):
        hover=np.asarray(position)+[0,0,height]
        for rotation in rotations:
            tool_r=rotation@relative
            if tool_r[2,2]>(-np.cos(np.deg2rad(30)) if tilted_grasp else -.98):continue
            target=hover+rotation@offset
            q=loaded_ik(trial,robot,target,tool_r)
            if q is None:continue
            release=screen_release(trial,robot,position,rotation,offset,relative,q)
            if release is None:continue
            candidates.append((min(joint_margin(trial,robot,q),release['minimum_joint_margin_rad']),np.linalg.norm(q-arm.qref),q,target,tool_r,rotation,hover))
    candidates.sort(key=lambda c:(-min(c[0],.30),c[1]))
    for margin,_,q,target,tool_r,rotation,hover in candidates:
        try:
            execute_loaded_transit(trial,robot,target,tool_r,q)
        except RuntimeError as error:
            if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
            raise
        actual_offset=check_payload(trial,robot,carried_offset,label='loaded transit',relocalize=True,reference_rotation=relative.T)
        if np.linalg.norm(actual_offset-carried_offset)>trial.task['cube_size_m']*.5:
            if _grip_replans>=2:raise RuntimeError('Repeated grip changes exceeded bounded replanning '+name)
            trial.event('replan_after_grip_change',robot=robot,object=name,attempt=_grip_replans+1)
            return place_cube(trial,robot,position,_grip_replans+1)
        trial.event('mosaic_loaded_candidate',robot=robot,object=name,joint_margin_rad=margin,hover_center_m=hover.tolist())
        chosen=rotation
        break
    if chosen is None:
        raise RuntimeError('No reachable loaded mosaic hover '+robot+'/'+name)
    align_payload(trial, robot, hover, chosen, 'mosaic-align-hover')
    # Keep the complete hand above neighbors/rim; gravity performs final seat.
    body = trial.data.body(name)
    actual_r = body.xmat.reshape(3,3).copy()
    offset = actual_r.T@(trial.data.site_xpos[arm.site]-body.xpos)
    relative = actual_r.T@trial.data.site_xmat[arm.site].reshape(3,3)
    release=screen_release(trial,robot,position,chosen,offset,relative,arm.qref)
    if release is None:raise RuntimeError('Current grip has no screened release and withdrawal '+name)
    clearance=release['clearance_m']
    for height in sorted(set((max(.08,clearance),clearance)),reverse=True):
        align_payload(trial,robot,np.asarray(position)+[0,0,height],chosen,'mosaic-align-release',coarse=height>=.08)
    lower_pocket_release(trial, robot, np.asarray(position))
    result = trial.execute('mosaic-release',{robot:('gripper',dict(
        mode='move',width_m=approach_width(trial,robot),speed_m_s=.05))},allow_failure=True)[robot]
    if arm.grasp_force is not None or result['status']=='uncertain':
        raise RuntimeError('Unresolved force release '+name)
    if arm.observe()['gripper_width_m'] < trial.task['cube_size_m']+.003:
        raise RuntimeError('Insufficient physical release aperture '+name)
    release_speed=speed_kwargs(trial,'coarse_tcp_speed_m_s','speed_m_s') if trial.monitor.released(trial.monitor.geoms[name]) else {}
    trial.move(robot,trial.data.site_xpos[arm.site]+[0,0,.10],label='mosaic-withdraw',**release_speed)
    trial.wait(.12)
    if not trial.monitor.released(trial.monitor.geoms[name]):
        raise RuntimeError('Cube still contacting hand after withdrawal '+name)
    trial.held.pop(robot,None)
    getattr(trial,'mosaic_tilted_grasps',{}).pop(robot,None)
    trial.event('placed',robot=robot,object=name,target_m=np.asarray(position).tolist())




def grasp_rotations(up,axes,forward,tilts=(0,15,25)):
    """Tilt about a horizontal closing axis, retaining parallel cube faces."""
    for angle_deg in tilts:
        angle=np.deg2rad(angle_deg)
        for y in axes:
            direction=np.asarray(forward)-up*np.dot(forward,up)-y*np.dot(forward,y)
            norm=np.linalg.norm(direction)
            if angle_deg and norm<.1:continue
            z=-up if not angle_deg else -up*np.cos(angle)+direction/norm*np.sin(angle)
            yield np.column_stack((np.cross(y,z),y,z)),angle_deg


def transfer_lift_segments(start,base,radial_sign=1):
    """One definition for the reference's real clear-support/retract commands."""
    clear=np.asarray(start,dtype=float)+[0,0,.03]
    direction=np.asarray(base,dtype=float)[:2]-clear[:2]
    direction/=max(np.linalg.norm(direction),1e-9)
    return [('transfer-pick-lift-clear',clear),
            ('transfer-pick-lift-retract',clear+np.r_[direction*.10*radial_sign,.10])]


def screen_transfer_segments(trial,robot,name,rotation,seed,segments,path_joint_margin_rad=None):
    """Sample actual segment endpoints and paths; not a dynamic grasp proof.

    Before lift-off only robot geometry is predicted and source finger contact
    is allowed. After lift-off Trial also predicts the measured carried body.
    """
    from .oracle import joint_edge_clear
    q=np.asarray(seed).copy();origin,_=trial.arms[robot].ik.pose(q)
    for _,target in segments:
        target=np.asarray(target)
        count=max(1,int(np.ceil(np.linalg.norm(target-origin)/.015)))
        for fraction in np.linspace(0,1,count+1)[1:]:
            point=origin*(1-fraction)+target*fraction
            next_q=trial.screen_ik(robot,point,rotation,seed=q)
            # A reserved correction endpoint and an interior path point are
            # different constraints. Ordinary picks may retain the original
            # shared IK bounds along the path while reserving endpoint room.
            margin=JOINT_MARGIN_RAD if path_joint_margin_rad is None or fraction==1 else path_joint_margin_rad
            if next_q is None or joint_margin(trial,robot,next_q)<margin:return False
            if not joint_edge_clear(q,next_q,lambda sample:trial.pose_clear(robot,sample,(name,)),step_rad=.005):return False
            q=next_q
        origin=target
    return True


def transfer_approach_branch(trial, robot, name, point, rotation, seat, height):
    """Bind arrival to a branch compatible with the already-screened seat."""
    approach=trial.screen_ik(robot,np.asarray(point)+[0,0,height],rotation,seed=seat)
    if (approach is None or joint_margin(trial,robot,approach)<JOINT_MARGIN_RAD or
            not trial.pose_clear(robot,approach)):
        return None
    if not screen_transfer_segments(trial,robot,name,rotation,approach,
                                    [('transfer-pick-contact',np.asarray(point))]):
        return None
    return approach


def transfer_continuation_available(trial, robot, name, target):
    """Current-grip hover/release witness; transit is rechecked after retract."""
    from .oracle import CUBE_ROTATIONS
    from ..robot_control import rotation_vector
    arm=trial.arms[robot];body=trial.data.body(name)
    actual=body.xmat.reshape(3,3).copy()
    offset=actual.T@(trial.data.site_xpos[arm.site]-body.xpos)
    relative=actual.T@trial.data.site_xmat[arm.site].reshape(3,3)
    tilted=getattr(trial,'mosaic_tilted_grasps',{}).get(robot,False)
    limit=-np.cos(np.deg2rad(30)) if tilted else -.98
    for rotation in sorted(CUBE_ROTATIONS,key=lambda r:np.linalg.norm(rotation_vector(r,actual))):
        tool=rotation@relative
        if tool[2,2]>limit:continue
        for height in (.08,.12,.16):
            point=np.asarray(target)+[0,0,height]+rotation@offset
            hover=loaded_ik(trial,robot,point,tool)
            if hover is not None and screen_release(trial,robot,target,rotation,offset,relative,hover) is not None:
                return True
    return False


def pick_transfer(trial,robot,name,target,aperture=None,required_tilt=False):
    """Finite angled top-grasp candidates, then clear support and retract."""
    from .oracle import CUBE_ROTATIONS
    from ..robot_control import rotation_vector
    arm=trial.arms[robot];width=trial.task['cube_size_m']
    default_aperture=approach_width(trial,robot)
    aperture=default_aperture if aperture is None else aperture
    first_action=len(getattr(trial,'actions',[]))
    if arm.grasp_force is not None or abs(arm.observe()['gripper_width_m']-aperture)>.0005:
        trial.execute('transfer-open',{robot:('gripper',dict(mode='move',width_m=aperture,speed_m_s=.05))})
    body=trial.data.body(name);position=body.xpos.copy();rotation=body.xmat.reshape(3,3).copy()
    up_index=int(np.argmax(abs(rotation[2])));up=rotation[:,up_index]*np.sign(rotation[2,up_index])
    axes=[rotation[:,axis]*sign for axis in range(3) if axis!=up_index for sign in (1,-1)]
    base=np.asarray(trial.task['arms'][robot]['base_position_m'])
    directions=[np.asarray(target)-base,position-base];candidate=None;chosen_tilt=0;chosen_radial_sign=1
    candidates=((position+up*height,r,tilt) for height in (width/2-.005,*reversed(normal_grasp_heights(trial)))
        for forward in directions for r,tilt in grasp_rotations(up,axes,forward) if not required_tilt or tilt>0)
    for point,r,tilt in candidates:
        seat=loaded_ik(trial,robot,point,r,(name,))
        if seat is None:continue
        # Close-to-base stock can need extension rather than further folding.
        # Both choices retain the same lift, distance and full-path margins.
        radial_sign=next((sign for sign in (1,-1) if screen_transfer_segments(
            trial,robot,name,r,seat,transfer_lift_segments(point,base,sign))),None)
        if radial_sign is None:continue
        relative=rotation.T@r;offset=rotation.T@(point-position)
        future=False
        for desired in sorted(CUBE_ROTATIONS,key=lambda rr:np.linalg.norm(rotation_vector(rr,rotation))):
            tool=desired@relative
            if tool[2,2]>-np.cos(np.deg2rad(30)):continue
            if loaded_ik(trial,robot,np.asarray(target)+[0,0,.08]+desired@offset,tool) is not None:future=True;break
        if not future:continue
        for height in (.04,.08,.12):
            approach=transfer_approach_branch(trial,robot,name,point,r,seat,height)
            if approach is None:continue
            try:trial.move_free(robot,point+[0,0,height],r,'transfer-pick-approach',
                continuation=(point,r,(name,)),target_joints=approach)
            except RuntimeError as error:
                if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                raise
            candidate=r;chosen_tilt=tilt;chosen_radial_sign=radial_sign;break
        if candidate is not None:break
    if candidate is None:
        # A narrower legal opening can clear adjacent stock. Only retry the
        # finite search before any approach/grasp, never a failed action.
        arm_idle=getattr(arm,'job',None) is None and arm.grasp_force is None and robot not in getattr(trial,'held',{})
        uncommitted=all(a['label']=='transfer-open' and all(r['status']=='completed' for r in a['results'].values())
            for a in getattr(trial,'actions',[])[first_action:])
        narrow=width+.004
        if aperture==default_aperture and narrow<aperture and arm_idle and uncommitted:
            trial.event('transfer_narrow_admission_search',robot=robot,object=name,aperture_m=narrow,
                scope='Empty-hand geometry search only; unchanged controller, full-arm checks and physical evaluation')
            return pick_transfer(trial,robot,name,target,aperture=narrow,required_tilt=required_tilt)
        raise RuntimeError('No transfer pickup with usable continuation '+robot+'/'+name)
    trial.move(robot,point,candidate,'transfer-pick-contact',precise=True)
    trial.execute('transfer-pick-grasp',{robot:('gripper',dict(mode='grasp',width_m=width,force_n=50.,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008))})
    trial.wait(.15)
    if not trial.finger_contacts(robot,name):raise RuntimeError('Transfer bilateral grasp missing '+name)
    lift_start=trial.data.site_xpos[arm.site].copy()
    segments=transfer_lift_segments(lift_start,base,chosen_radial_sign)
    # The source still rests on its support. Recheck the measured arm pose now;
    # predict the carried body's full geometry after the real takeoff below.
    if not screen_transfer_segments(trial,robot,name,candidate,trial.data.qpos[arm.qids],segments):
        raise RuntimeError('Transfer lift/retract changed after grasp '+name)
    trial.held[robot]=name
    if not hasattr(trial,'mosaic_tilted_grasps'):trial.mosaic_tilted_grasps={}
    trial.mosaic_tilted_grasps[robot]=chosen_tilt>0
    trial.move(robot,segments[0][1],candidate,segments[0][0])
    if not trial.finger_contacts(robot,name):raise RuntimeError('Transfer contact lost after lift '+name)
    if not screen_transfer_segments(trial,robot,name,candidate,trial.data.qpos[arm.qids],segments[1:]):
        alternate=transfer_lift_segments(lift_start,base,-chosen_radial_sign)[1:]
        if (not screen_transfer_segments(trial,robot,name,candidate,trial.data.qpos[arm.qids],alternate) or
                not transfer_continuation_available(trial,robot,name,target)):
            raise RuntimeError('Transfer retract infeasible with measured payload '+name)
        trial.event('transfer_measured_retract_replan',robot=robot,object=name,
            previous_radial_sign=chosen_radial_sign,new_radial_sign=-chosen_radial_sign,
            scope='Actual held geometry and original path/correction margins; current-grip release endpoints screened, future transit revalidated after real retract')
        chosen_radial_sign=-chosen_radial_sign
        segments[1:]=alternate
    trial.move(robot,segments[1][1],candidate,segments[1][0],
               **speed_kwargs(trial,'coarse_tcp_speed_m_s','speed_m_s'))
    trial.wait(.1)
    if trial.data.body(name).xpos[2]<position[2]+.08 or not trial.finger_contacts(robot,name):raise RuntimeError('Transfer payload not retained '+name)
    trial.event('picked',robot=robot,object=name,source='finite-transfer-grasp',tilt_deg=chosen_tilt,
                retract_radial_sign=chosen_radial_sign)


def normal_approach_candidates(trial, robot, point, rotation, expanded=False):
    """Try the nominal arrival first, then finite higher folded-arm arrivals."""
    for q in loaded_ik_candidates(trial,robot,point,rotation):
        yield np.asarray(point),q
    if not expanded:return
    arm=trial.arms[robot];step=float(trial.task['cube_size_m'])
    for rise in (step,2*step):
        higher=np.asarray(point)+[0,0,rise]
        for q in loaded_ik_candidates(trial,robot,higher,rotation):
            yield higher,q
        # Rear supplies can need a different elbow branch, not a larger limit.
        for sign in (-1,1):
            seed=trial.home_q[robot].copy();seed[0]=2*sign;seed[2]=2*sign
            seed=np.clip(seed,*arm.ik.limits.T)
            q=trial.screen_ik(robot,higher,rotation,seed=seed)
            if q is not None and joint_margin(trial,robot,q)>=JOINT_MARGIN_RAD and trial.pose_clear(robot,q):
                yield higher,q


def pick_for_transport(trial,robot,name,target):
    """Use the ordinary picker with this reference's full-path margins."""
    original=trial.pose_clear
    original_free=getattr(trial,'move_free',None)
    original_move=getattr(trial,'move',None)
    expanded=False
    def clear(robot,q,allowed_objects=(),clearance_m=.003):
        return original(robot,q,allowed_objects,clearance_m=max(.003,clearance_m))
    def free(robot,point,rotation,label='free-approach',**kwargs):
        if label=='pick-approach':
            seat,r,allowed=kwargs['continuation']
            for approach_point,approach in normal_approach_candidates(trial,robot,point,rotation,expanded=expanded):
                if not any(screen_transfer_segments(trial,robot,name,r,approach,
                    [('pick-contact',np.asarray(seat)),('pick-lift',lift)],path_joint_margin_rad=0.)
                    for lift in normal_lift_targets(trial,np.asarray(seat)+[0,0,.14])):continue
                try:return original_free(robot,approach_point,rotation,label,**dict(kwargs,target_joints=approach))
                except RuntimeError as error:
                    if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                    raise
            raise RuntimeError('No clear free-arm IK '+robot+'/'+label)
        return original_free(robot,point,rotation,label,**kwargs)
    def move(robot,point,rotation=None,label='move',**kwargs):
        if label in ('pick-contact','pick-lift'):
            if label=='pick-lift':
                if not trial.finger_contacts(robot,name):raise RuntimeError('Normal pickup contact lost before lift '+name)
                # The force grasp is real now: predict its measured payload,
                # rather than assuming it stays at the initial stock pose.
                trial.held[robot]=name
            arm=trial.arms[robot]
            r=trial.data.site_xmat[arm.site].reshape(3,3) if rotation is None else rotation
            candidates=normal_lift_targets(trial,point) if label=='pick-lift' else (np.asarray(point),)
            selected=next((candidate for candidate in candidates if screen_transfer_segments(
                trial,robot,name,r,trial.data.qpos[arm.qids],[(label,candidate)],path_joint_margin_rad=0.)),None)
            if selected is None:
                raise RuntimeError('Normal pickup measured segment rejected '+robot+'/'+name+'/'+label)
            if np.linalg.norm(selected-point)>1e-8:
                trial.event('normal_lift_endpoint_extended',robot=robot,object=name,
                    original_target_m=np.asarray(point).tolist(),target_m=selected.tolist(),
                    scope='Same full-path and endpoint correction screen; a higher finite endpoint, not relaxed limits')
            point=selected
        return original_move(robot,point,rotation,label,**kwargs)
    trial.pose_clear=clear
    if original_free is not None:trial.move_free=free
    if original_move is not None:trial.move=move
    try:
        for expanded in (False,True):
            for height in normal_grasp_heights(trial):
                try:
                    trial.pick(robot,name,height=height,force=50.,placement_hint=(target,np.eye(3)),aperture=approach_width(trial,robot))
                except RuntimeError as error:
                    if str(error).startswith('No pickup pose '):continue
                    raise
                trial.event('normal_grasp_height_selected',robot=robot,object=name,height_m=height,
                    expanded_approach=expanded,non_target_clearance_m=.003)
                return
        raise RuntimeError('No geometrically screened normal grasp '+robot+'/'+name)
    finally:
        trial.pose_clear=original
        if original_free is not None:trial.move_free=original_free
        if original_move is not None:trial.move=original_move


def normal_lift_targets(trial, target):
    """Retain nominal lift, then finite higher endpoints with the same checks."""
    step=float(trial.task['cube_size_m'])/3
    return tuple(np.asarray(target)+[0.,0.,rise] for rise in (0.,step,2*step))


def normal_grasp_heights(trial):
    return tuple(trial.task['cube_size_m']*ratio for ratio in (.1,.2,1/3))


def pickup_endpoint_candidates(trial):
    """Extend the generic map with the actual ordinary picker's finite grasps.

    Include the finite normal/transfer heights and transfer tilts, rather than
    losing a supply that the existing angled picker can use. This map still
    proves only endpoints, never aperture clearance, contact or whole routes.
    Provenance alone never adds an operating robot.
    """
    result=helpers.pickup_coverage(trial)
    for item in trial.env['objects']:
        name=item['id'];body=trial.data.body(name);position=body.xpos.copy()
        rotation=body.xmat.reshape(3,3);up_index=int(np.argmax(abs(rotation[2])))
        up=rotation[:,up_index]*np.sign(rotation[2,up_index])
        axes=[rotation[:,axis]*sign for axis in range(3) if axis!=up_index for sign in (1,-1)]
        for robot in trial.arms:
            if robot in result[name]:continue
            base=np.asarray(trial.task['arms'][robot]['base_position_m'])
            if np.linalg.norm(position[:2]-base[:2])>.88:continue
            found=False
            for height in (*normal_grasp_heights(trial),trial.task['cube_size_m']/2-.005):
                point=position+up*height
                for pose,_ in grasp_rotations(up,axes,position-base):
                    seat=trial.screen_ik(robot,point,pose,seed=trial.home_q[robot])
                    if seat is not None and trial.screen_ik(robot,point+[0,0,.14],pose,seed=seat) is not None:
                        result[name].append(robot);found=True;break
                if found:break
    return result


def goal_endpoint_candidates(trial):
    """Retain nominal access, then the actual reference's release/exit poses.

    A free-drop placement need not reach a fictitious seated TCP. A template
    does not prove grasp dynamics; execution rechecks the measured held offset.
    Tilt requirements remain explicit so allocation cannot silently pick an
    incompatible flat grasp for a destination that needs the angled template.
    """
    from copy import copy
    trial.mosaic_goal_tilted_coverage={}
    try:return helpers.goal_coverage(trial)
    except RuntimeError as error:
        if not str(error).startswith('No candidate goal approach/seat/withdraw endpoints '):raise
    result={};axes=[np.array(v,dtype=float) for v in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0))]
    for goal in trial.task['goals']:
        proxy=copy(trial);proxy.task=dict(trial.task,goals=[goal])
        try:
            result[goal['goal_id']]=helpers.goal_coverage(proxy)[goal['goal_id']]
            continue
        except RuntimeError as error:
            if not str(error).startswith('No candidate goal approach/seat/withdraw endpoints '):raise
        position=np.asarray(goal['position_m']);options=[];tilts={}
        robots=sorted(trial.arms,key=lambda r:np.linalg.norm(position[:2]-np.asarray(trial.task['arms'][r]['base_position_m'])[:2]))
        for robot in robots:
            base=np.asarray(trial.task['arms'][robot]['base_position_m'])
            if np.linalg.norm(position[:2]-base[:2])>.88:continue
            found=False
            for height in (trial.task['cube_size_m']*.2,trial.task['cube_size_m']/3,trial.task['cube_size_m']/2-.005,trial.task['cube_size_m']*.1):
                offset=np.array([0.,0.,height])
                for rotation,tilt in grasp_rotations(np.array([0.,0.,1.]),axes,position-base):
                    hover=loaded_ik(trial,robot,position+[0,0,.08]+offset,rotation)
                    if hover is None:continue
                    release=screen_release(trial,robot,position,np.eye(3),offset,rotation,hover)
                    if release is None:continue
                    options.append(robot);tilts[robot]=tilt>0;found=True
                    trial.event('mosaic_goal_release_access_template',goal=goal['goal_id'],robot=robot,
                        tilt_deg=tilt,capture_height_m=height,**release,
                        scope='Finite endpoint/lowering/opening/exit screen; not grasp or placement dynamics proof')
                    break
                if found:break
        if not options:raise RuntimeError('No finite Mosaic release/withdrawal access candidate '+goal['goal_id'])
        result[goal['goal_id']]=options;trial.mosaic_goal_tilted_coverage[goal['goal_id']]=tilts
    return result


def pickup_admission_uncommitted(trial, robot, name, first_action, error,
                                 aperture_labels=('open-for-pick','transfer-open')):
    """Recognize empty-hand admission, never a failed/unknown pickup action."""
    messages=('No geometrically screened normal grasp '+robot+'/'+name,
              'No transfer pickup with usable continuation '+robot+'/'+name)
    if str(error) not in messages:
        return False
    arm=trial.arms[robot]
    if (getattr(arm,'job',None) is not None or getattr(arm,'grasp_force',None) is not None or
            robot in getattr(trial,'held',{}) or trial.finger_contacts(robot,name)):
        return False
    for action in getattr(trial,'actions',[])[first_action:]:
        commands=action['commands'];results=action['results']
        if action['label'] not in aperture_labels or set(commands)!={robot} or set(results)!={robot}:
            return False
        operation,parameters=commands[robot]
        if operation!='gripper' or parameters.get('mode')!='move' or results[robot]['status']!='completed':
            return False
    return True


def normal_admission_uncommitted(trial, robot, name, first_action, error):
    return (str(error)=='No geometrically screened normal grasp '+robot+'/'+name and
            pickup_admission_uncommitted(trial,robot,name,first_action,error,('open-for-pick',)))


def deliver_cube(trial, name, path, position):
    edges = getattr(trial,'mosaic_edges',helpers.graph(trial.env)[1])
    for index, robot in enumerate(path):
        target = np.asarray(position) if index+1==len(path) else edges[frozenset(path[index:index+2])]
        current=trial.data.body(name).xpos
        angled_positions=[edges[key] for key in getattr(trial,'mosaic_angled_edges',{})]
        needs_angle=any(np.linalg.norm(np.asarray(p)[:2]-np.asarray(target)[:2])<.02 or np.linalg.norm(np.asarray(p)[:2]-current[:2])<.02 for p in angled_positions)
        goal_id=(getattr(trial,'mosaic_current_job',None) or {}).get('goal_id')
        required_tilt=bool(index+1==len(path) and getattr(trial,'mosaic_goal_tilted_coverage',{}).get(goal_id,{}).get(robot))
        if needs_angle or required_tilt:
            pick_transfer(trial,robot,name,target,**({'required_tilt':True} if required_tilt else {}))
        else:
            first_action=len(getattr(trial,'actions',[]))
            try:pick_for_transport(trial,robot,name,target)
            except RuntimeError as error:
                if not normal_admission_uncommitted(trial,robot,name,first_action,error):raise
                trial.event('ordinary_admission_transfer_fallback',robot=robot,object=name,
                    scope='Normal planning exhausted before approach/grasp/lift; one existing screened transfer template, not a failed-action replay')
                pick_transfer(trial,robot,name,target)
        place_cube(trial,robot,target)
        helpers.park(trial,robot)
        if np.linalg.norm(target-np.asarray(position))<1e-6:break


def candidate_route(trial, neighbors, starts, destination, name):
    """A disconnected alternative is not failure of every pickup candidate."""
    point = trial.data.body(name).xpos[:2]
    paths = []
    for source in starts:
        try:
            path = helpers.route(neighbors, source, destination)
        except ValueError:
            continue
        base = np.asarray(trial.task['arms'][source]['base_position_m'][:2])
        paths.append((len(path), float(np.linalg.norm(point-base)), path))
    return min(paths, key=lambda item: (item[0], item[1]))[2] if paths else None



def buffer_graph(trial, neighbors, edges):
    """Propose finite, empty table buffers, screened for both adjacent arms.

    Buffers are private reference choices, never new scene fixtures or changes
    to task rules. Endpoint screens are not physical handover certificates.
    """
    from itertools import combinations
    graph = {r:list(v) for r,v in neighbors.items()}
    buffers = dict(edges)
    angled = dict(getattr(trial,'mosaic_angled_edges',{}))
    size = trial.task['cube_size_m']
    goals = np.asarray([g['position_m'] for g in trial.task['goals']])
    lo, hi = goals[:,:2].min(axis=0)-size/2, goals[:,:2].max(axis=0)+size/2
    stock = np.asarray([trial.data.body(o['id']).xpos[:2] for o in trial.env['objects']])
    bases = {r:np.asarray(trial.task['arms'][r]['base_position_m'][:2]) for r in graph}
    rotations = []
    for y in (np.array([1.,0,0]),np.array([0.,1,0]),np.array([-1.,0,0]),np.array([0.,-1,0])):
        z=np.array([0.,0,-1.]);rotations.append(np.column_stack((np.cross(y,z),y,z)))
    def supported(point):
        for table in trial.env['tables']:
            center=np.asarray(table['center_m']);half=np.asarray(table['half_size_m'])
            if np.all(abs(point-center[:2])+size/2+.01 < half[:2]):return True
        return False
    def endpoints(robot,center):
        target=center+[0,0,size/2-.005]
        forward=center-np.r_[bases[robot],trial.task['table_height_m']]
        axes=[r[:,1] for r in rotations]
        for rotation,tilt in grasp_rotations(np.array([0.,0,1.]),axes,forward):
            near=loaded_ik(trial,robot,target,rotation)
            if near is None:continue
            for height in (.08,):
                high=loaded_ik(trial,robot,target+[0,0,height],rotation)
                if high is not None and screen_release(trial,robot,center,np.eye(3),np.array([0.,0.,size/2-.005]),rotation,high) is not None:return tilt
        return None
    for a,b in combinations(graph,2):
        key=frozenset((a,b))
        if key in buffers or np.linalg.norm(bases[a]-bases[b])>1.76:continue
        middle=(bases[a]+bases[b])/2
        candidates=[middle]
        delta=bases[b]-bases[a]
        normal=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)
        candidates.extend(middle+normal*size*factor for factor in (-7.5,-5,-2.5,2.5,5,7.5))
        for x in (lo[0]-.10,hi[0]+.10):
            candidates.extend(np.array([x,y]) for y in np.linspace(lo[1],hi[1],7))
        for y in (lo[1]-.10,hi[1]+.10):
            candidates.extend(np.array([x,y]) for x in np.linspace(lo[0],hi[0],9))
        candidates.sort(key=lambda p:float(np.linalg.norm(p-bases[a])+np.linalg.norm(p-bases[b])))
        for point in candidates:
            if np.all(point>lo-.06) and np.all(point<hi+.06):continue
            if not supported(point) or min(np.linalg.norm(point-p) for p in bases.values())<.25:continue
            if any(np.linalg.norm(point-p)<size+.02 for p in stock):continue
            if max(np.linalg.norm(point-bases[r]) for r in (a,b))>.88:continue
            center=np.r_[point,trial.task['table_height_m']+size/2]
            approaches=[endpoints(r,center) for r in (a,b)]
            if any(v is None for v in approaches):continue
            buffers[key]=center
            if any(v>0 for v in approaches):angled[key]=dict(tilt_deg_by_robot=dict(zip((a,b),approaches)))
            graph[a].append(b);graph[b].append(a)
            break
    trial.mosaic_angled_edges=angled
    return graph,buffers


def protected_remaining_inputs(trial, protected_assignments):
    """Plan from actual progress without recycling any completed goal's item.

    This prepares private planning inputs only. A lost or replaced completed
    pairing needs an explicit owner repair, never a silent matching reset.
    """
    current=dict(trial.monitor.result().get('assignments',{}))
    for goal,name in protected_assignments.items():
        if current.get(goal)!=name:
            raise RuntimeError('Previously completed pairing changed '+goal+'/'+name)
    used=set(current.values())
    goals=[goal for goal in trial.task['goals'] if goal['goal_id'] not in current]
    objects=[item for item in trial.env['objects'] if item['id'] not in used]
    return goals,objects,current


def assign_supply(trial, goals, coverage, pickups, neighbors, available_objects=None):
    """Match all unique supplies before execution, not greedy local depletion."""
    from scipy.optimize import linear_sum_assignment
    objects=list(trial.env['objects'] if available_objects is None else available_objects)
    costs=np.full((len(goals),len(objects)),np.inf)
    routes={}
    for i,goal in enumerate(goals):
        position=np.asarray(goal['position_m'])
        for j,item in enumerate(objects):
            if item['color']!=goal['requires']['color']:continue
            for destination in coverage[goal['goal_id']]:
                path=candidate_route(trial,neighbors,pickups[item['id']],destination,item['id'])
                if path is None:continue
                base=np.asarray(trial.task['arms'][destination]['base_position_m'][:2])
                pickup_base=np.asarray(trial.task['arms'][path[0]]['base_position_m'][:2])
                score=float(np.linalg.norm(base-position[:2])+np.linalg.norm(trial.data.body(item['id']).xpos[:2]-pickup_base)+2*(len(path)-1))
                if score<costs[i,j]:costs[i,j]=score;routes[i,j]=path
    if np.any(~np.isfinite(costs).any(axis=1)):
        missing=[goals[i]['goal_id'] for i in range(len(goals)) if not np.isfinite(costs[i]).any()]
        raise ValueError('No supply path for '+','.join(missing))
    try:rows,columns=linear_sum_assignment(costs)
    except ValueError as error:raise ValueError('No complete unique supply assignment') from error
    if len(rows)!=len(goals):raise ValueError('Incomplete supply matching')
    return [(goals[i],objects[j]['id'],routes[i,j]) for i,j in zip(rows,columns)]


def check_process(trial):
    if trial.monitor.violations:
        raise RuntimeError('Process violation: ' + str(trial.monitor.violations[-1]))


def install_fine_route_screen(trial):
    """Keep the private reference's narrow passages below the coarse step."""
    if getattr(trial,'mosaic_fine_route_installed',False):return
    previous=getattr(trial,'move_free',None)
    if previous is None:return
    def fine_route(*args,**kwargs):
        kwargs.setdefault('route_joint_step_rad',.005)
        robot=args[0] if args else kwargs.get('robot')
        q=kwargs.get('target_joints')
        if (q is not None and robot in getattr(trial,'arms',{}) and
                robot not in getattr(trial,'held',{}) and 'joint_speed_rad_s' not in kwargs and
                joint_margin(trial,robot,q)<.10):
            kwargs['joint_speed_rad_s']=.20
            trial.event('mosaic_empty_narrow_margin_speed',robot=robot,joint_margin_rad=joint_margin(trial,robot,q),
                joint_speed_rad_s=.20,scope='Private command choice based on actual joint margin; original limits and feedback unchanged')
        return previous(*args,**kwargs)
    trial.move_free=fine_route
    trial.mosaic_fine_route_installed=True


def install_process_guard(trial):
    """Check irreversible task violations at every existing control boundary."""
    if getattr(trial,'mosaic_process_guard_installed',False):return
    previous=getattr(trial,'check_stop',lambda:None)
    def guarded():
        check_process(trial)
        return previous()
    trial.check_stop=guarded
    trial.mosaic_process_guard_installed=True


def solve(trial, max_goals=None, goal_ids=None, motion_profile=None, resume=False):
    """Execute bounded physical placements; max_goals is commissioning only.

    A partial witness never constitutes task success. The caller independently
    reads the unchanged evaluator and enforces wall/simulation/action budgets.
    """
    validate_task(trial.task)
    install_process_guard(trial)
    install_fine_route_screen(trial)
    trial.mosaic_motion_profile=validate_motion_profile(motion_profile)
    trial.event('mosaic_reference_speed_profile',profile=trial.mosaic_motion_profile,normal_grasp_force_n=50.,repair_grasp_force_n=30.,contact_and_refine_unchanged=True)
    if max_goals is not None and (not isinstance(max_goals, int) or max_goals < 1):
        raise ValueError('max_goals must be a positive commissioning limit')
    protected={};available=None
    if resume:
        if trial.held:raise RuntimeError('Stage or finish the held payload before rematching remaining work')
        remaining,available,protected=protected_remaining_inputs(trial,getattr(trial,'mosaic_protected_assignments',{}))
        trial.event('mosaic_resume_from_actual_progress',protected_assignments=protected,
            remaining_goals=[g['goal_id'] for g in remaining],scope='Same live world; no reset, initial parking or completed-object reuse')
    else:
        helpers.clear_idle_arms(trial)
        remaining=trial.task['goals']
    trial.mosaic_protected_assignments=protected
    if not remaining:
        trial.wait(1.2)
        return dict(reference_placed=len(protected),total_goals=len(trial.task['goals']),commissioning_only=False,resumed=resume)
    neighbors, _ = helpers.graph(trial.env)
    coverage = goal_endpoint_candidates(trial)
    pickups = pickup_endpoint_candidates(trial)
    from ..run_setup import write_json
    write_json(trial.directory / 'coverage.json', dict(
        scope='Endpoint candidates only; actual motion and contact remain unproven',
        goals=coverage))
    goals = sorted(remaining,
                   key=lambda g: (np.linalg.norm(g['position_m'][:2]), g['goal_id']))
    try:
        jobs=assign_supply(trial,goals,coverage,pickups,neighbors,**({'available_objects':available} if available is not None else {}))
        edges=helpers.graph(trial.env)[1]
    except ValueError as direct_error:
        trial.event('direct_supply_incomplete',reason=str(direct_error))
        neighbors,edges=buffer_graph(trial,neighbors,helpers.graph(trial.env)[1])
        try:jobs=assign_supply(trial,goals,coverage,pickups,neighbors,**({'available_objects':available} if available is not None else {}))
        except ValueError as error:
            trial.event('mosaic_supply_plan_exhausted',reason=str(error),empty_pickup_candidates=[name for name,robots in pickups.items() if not robots])
            raise RuntimeError('Finite Mosaic supply plan needs owner review: '+str(error)) from error
    trial.mosaic_edges=edges
    jobs.sort(key=lambda job:(-len(job[2]),np.linalg.norm(job[0]['position_m'][:2]),job[0]['goal_id']))
    write_json(trial.directory/'supply-plan.json',dict(
        scope='All unique supplies matched; buffers have endpoint screens only, not physical proof',
        table_buffers=[dict(robots=sorted(key),position_m=value.tolist(),angled_grasp=getattr(trial,'mosaic_angled_edges',{}).get(key)) for key,value in edges.items()],
        jobs=[dict(goal=g['goal_id'],object=name,path=path) for g,name,path in jobs]))
    if goal_ids is not None:
        known={g['goal_id'] for g,_,_ in jobs}
        if not set(goal_ids)<=known:raise ValueError('Unknown commissioning goal')
        jobs=[job for job in jobs if job[0]['goal_id'] in goal_ids]
    selected=jobs if max_goals is None else jobs[:max_goals]
    completed=set(protected);pending=list(selected);deferred={};attempts={}
    while pending:
        goal,name,path=pending.pop(0)
        trial.mosaic_current_job=dict(goal_id=goal['goal_id'],object=name,path=list(path))
        position=np.asarray(goal['position_m'])
        first_action=len(getattr(trial,'actions',[]))
        try:deliver_cube(trial, name, path, position)
        except RuntimeError as error:
            check_process(trial)
            message=str(error);robot=next((r for r in path if message.endswith(' '+r+'/'+name)),None)
            allowed=(robot is not None and not getattr(trial,'held',{}) and
                     pickup_admission_uncommitted(trial,robot,name,first_action,error))
            key=goal['goal_id'];progress=len(completed)
            if (not allowed or deferred.get(key)==progress or attempts.get(key,0)>=2 or
                    completed-set(trial.monitor.result().get('assignments',{}))):
                raise
            deferred[key]=progress;attempts[key]=attempts.get(key,0)+1;pending.append((goal,name,path))
            trial.event('mosaic_goal_deferred',goal=key,object=name,robot=robot,
                reason=message,completed_goals=progress,attempt=attempts[key],
                pending_goals=[g['goal_id'] for g,_,_ in pending],
                scope='Known empty-hand admission only; unchanged object/goal pairing, independent jobs continue, bounded recheck after progress')
            continue
        trial.wait(.7)
        check_process(trial)
        for repair in range(2):
            if goal['goal_id'] in trial.monitor.result().get('assignments', {}):
                break
            robot = path[-1]
            trial.event('reference_placement_repair', goal=goal['goal_id'],
                        object=name, retry=repair+1,
                        scope='Physical regrasp; unchanged geometry and evaluator')
            trial.pick(robot, name, height=float(trial.task['cube_size_m'])/3,
                       force=30., placement_hint=(position, np.eye(3)),
                       aperture=approach_width(trial, robot))
            place_cube(trial, robot, position)
            helpers.park(trial, robot)
            trial.wait(.7)
            check_process(trial)
        assignments = set(trial.monitor.result().get('assignments', {}))
        if goal['goal_id'] not in assignments:
            raise RuntimeError('Released cube failed assembly goal after bounded repair ' + goal['goal_id'])
        lost = completed - assignments
        if lost:
            raise RuntimeError('Placement disturbed completed goals: ' + ','.join(sorted(lost)))
        completed.add(goal['goal_id'])
        trial.mosaic_protected_assignments=dict(trial.monitor.result().get('assignments',{}))
        trial.mosaic_current_job=None
        trial.event('goal_attempt', goal=goal['goal_id'], object=name, path=path,
                    progress=len(assignments), reference_completed=len(completed))
        print(f"{trial.task['id']} mosaic {len(completed)}/{len(selected)+len(protected)} validated", flush=True)
        after_goal=getattr(trial,'mosaic_after_goal',None)
        if after_goal is not None:after_goal(trial)
    trial.wait(1.2)
    return dict(reference_placed=len(completed), total_goals=len(trial.task['goals']),
                commissioning_only=len(selected) < len(goals),resumed=resume)
