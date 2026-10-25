"""Transport-owned actuator reference; planning reads never alter live state.

The shared placement implementation executes the same Panda commands as the
other references. This entry checks finite inventory and declared relay routes
before allowing it to move a robot; it does not certify dynamic feasibility.
"""
from collections import Counter
from contextlib import contextmanager,nullcontext
from copy import copy, deepcopy

import mujoco
import math
import time

import numpy as np

from .placement_oracle import graph, route


def validate_inputs(task, environment):
    """Check task contract without stepping physics or solving kinematics."""
    if task.get('family') != 'transport' or task.get('suite_version') != 2:
        raise ValueError('Transport reference requires a v2 transport task')
    if task.get('level') not in (1, 2, 3):
        raise ValueError('Unsupported transport level')
    height = task.get('table_height_m')
    if not isinstance(height, (int, float)) or not math.isfinite(height):
        raise ValueError('Finite task.table_height_m is required')
    robots = set(task['arms'])
    objects = environment['objects']
    goals = task['goals']
    if len({obj['id'] for obj in objects}) != len(objects):
        raise ValueError('Duplicate transport object ID')
    if len({goal['goal_id'] for goal in goals}) != len(goals):
        raise ValueError('Duplicate transport goal ID')
    supply = Counter(obj['identity'] for obj in objects)
    demand = Counter(goal['requires']['identity'] for goal in goals)
    if supply != demand:
        raise ValueError('Transport inventory and goal identities differ')
    neighbors, _ = graph(environment)
    if set(neighbors) != robots:
        raise ValueError('Declared relay stations differ from task robots')
    for obj in objects:
        if obj['source'] not in robots:
            raise ValueError('Unknown transport source: ' + obj['source'])
    for goal in goals:
        if goal['robot'] not in robots:
            raise ValueError('Unknown destination: ' + goal['robot'])
        if not goal.get('board') or len(goal.get('local_position_m', ())) != 3:
            raise ValueError('Goal must declare its current board-relative pose')
        for obj in objects:
            if obj['identity'] == goal['requires']['identity']:
                route(neighbors, obj['source'], goal['robot'])
    return dict(table_height_m=height, objects=len(objects), goals=len(goals),
                scope='Inventory and declared graph only; no execution certificate')


def cylinder_commands(commands, held, objects):
    """A finite reference motion policy, not a change to robot controller limits."""
    adjusted = deepcopy(commands)
    for robot, (operation, parameters) in adjusted.items():
        name = held.get(robot)
        if operation != 'move' or objects.get(name, {}).get('shape') != 'cylinder':
            continue
        for key, maximum in (('joint_speed_rad_s', .18), ('speed_m_s', .025),
                             ('angular_speed_rad_s', .25)):
            if key in parameters:
                parameters[key] = min(parameters[key], maximum)
    return adjusted


def _planning_copy(trial):
    """Independent scratch FK/collision data; never a live execution world."""
    from .evaluate import Monitor
    from ..robot_control import PandaArm
    scratch = copy(trial)
    scratch.live_preview = False
    scratch._reference_planning_only = True
    scratch._live_data = trial.data
    scratch.model = copy(trial.model)
    scratch.data = mujoco.MjData(scratch.model)
    scratch.data.time = trial.data.time
    def forbidden(*args, **kwargs):
        raise RuntimeError('Planning scratch cannot execute, step or publish events')
    scratch.execute = scratch.step = scratch.event = forbidden
    scratch.motion = None
    scratch.data.qpos[:] = trial.data.qpos
    scratch.data.ctrl[:] = trial.data.ctrl
    mujoco.mj_forward(scratch.model, scratch.data)
    scratch.arms = {r: PandaArm(scratch.model, scratch.data, r) for r in trial.arms}
    scratch.screen_model = copy(trial.screen_model)
    scratch.screen_data = mujoco.MjData(scratch.screen_model)
    scratch.held = dict(trial.held)
    scratch.carry_geoms, scratch.hand_geoms = {}, {}
    scratch.monitor = Monitor(scratch)
    return scratch


def _scratch_gap(trial, robot, width):
    if (not getattr(trial, '_reference_planning_only', False)
            or getattr(trial, 'data', None) is getattr(trial, '_live_data', None)):
        raise RuntimeError('Gripper geometry sweep requires planning-only data')
    arm = trial.arms[robot]
    half = (float(width)-arm.grip_min)/2
    for suffix in ('finger_joint1', 'finger_joint2'):
        trial.data.joint(robot+'_'+suffix).qpos[0] = half


def _release_sweep_clear(t, donor, receiver, name, receiver_q, width):
    """Screen both grippers' swept geometry and the donor's release space."""
    if (not getattr(t, '_reference_planning_only', False)
            or getattr(t, 'data', None) is getattr(t, '_live_data', None)):
        raise RuntimeError('Release sweep requires independent planning-only data')
    before, held = t.data.qpos.copy(), t.held
    try:
        donor_q = t.data.qpos[t.arms[donor].qids].copy()
        donor_width = t.arms[donor].observe()['gripper_width_m']
        t.data.qpos[t.arms[receiver].qids] = receiver_q
        # The candidate object pose is fixed in scratch for a dual grasp.
        # Neither planning arm may drag it through the other arm.
        t.held = {}
        receiver_open = min(t.arms[receiver].grip_max, width+.012)
        receiver_closed = max(t.arms[receiver].grip_min+.001, width-.004)
        for gap in np.linspace(receiver_open, receiver_closed, 5):
            _scratch_gap(t, receiver, gap)
            if not t.pose_clear(receiver, receiver_q, (name,), clearance_m=.003):
                return False
            if not t.pose_clear(donor, donor_q, (name,), clearance_m=.003):
                return False
        for gap in np.linspace(donor_width, t.arms[donor].grip_max, 5):
            _scratch_gap(t, donor, gap)
            if not t.pose_clear(donor, donor_q, (name,), clearance_m=.003):
                return False
            if not t.pose_clear(receiver, receiver_q, (name,), clearance_m=.003):
                return False
        # Fully open donor must clear every payload geom, not merely cease
        # bilateral grasp inference. Expected contact is receiver-only now.
        if not t.pose_clear(donor, donor_q, clearance_m=.003):
            return False
        mujoco.mj_kinematics(t.model, t.data)
        point = t.data.site_xpos[t.arms[donor].site].copy()
        tool = t.data.site_xmat[t.arms[donor].site].reshape(3, 3).copy()
        back, up = -tool[:, 2], np.array([0.,0.,1.])
        for direction in (back, back+.75*up, up):
            length = np.linalg.norm(direction)
            if length < .01:
                continue
            direction = direction/length
            for distance in (.16, .10, .06):
                for seed in (donor_q, t.home_q[donor]):
                    q = t.screen_ik(donor, point+direction*distance, tool, seed)
                    if q is not None and t.pose_clear(donor, q, clearance_m=.003):
                        return True
        return False
    finally:
        t.data.qpos[:] = before
        t.held = held
        mujoco.mj_kinematics(t.model, t.data)


def _hand_geometry_clear(t, donor, receiver, name, point, tool, width, diagnostics=None):
    """Reject incompatible relative hand/payload geometry before expensive IK.

    Only independent proxy geom transforms are changed for signed-distance
    queries. This does not assert a reachable arm pose, alter live state or
    replace the later full-arm opening/withdrawal screens. Pair geometry is
    invariant under the common rigid transform used by the carry candidates.
    """
    if not getattr(t, '_reference_planning_only', False):
        raise RuntimeError('Hand geometry prefilter requires planning-only data')
    model = t.model
    data = mujoco.MjData(model)
    data.qpos[:] = t.data.qpos
    def hand_geoms(robot):
        bodies = (robot+'_hand', robot+'_left_finger', robot+'_right_finger')
        return [g for g in range(model.ngeom)
                if model.body(int(model.geom_bodyid[g])).name in bodies
                and (model.geom_contype[g] or model.geom_conaffinity[g])]
    giver, taker = hand_geoms(donor), hand_geoms(receiver)
    payload = t.monitor.geoms[name]
    giver_palm = [g for g in giver if model.body(int(model.geom_bodyid[g])).name == donor+'_hand']
    taker_palm = [g for g in taker if model.body(int(model.geom_bodyid[g])).name == receiver+'_hand']
    giver_gap = t.arms[donor].observe()['gripper_width_m']
    open_gap = min(t.arms[receiver].grip_max, width+.012)
    closed_gap = max(t.arms[receiver].grip_min+.001, width-.004)
    gaps = [(giver_gap, g, False) for g in np.linspace(open_gap, closed_gap, 5)]
    gaps += [(g, closed_gap, i == 4) for i, g in enumerate(np.linspace(giver_gap, t.arms[donor].grip_max, 5))]
    for dg, rg, released in gaps:
        for robot, gap in ((donor, dg), (receiver, rg)):
            for suffix in ('finger_joint1', 'finger_joint2'):
                data.joint(robot+'_'+suffix).qpos[0] = (gap-t.arms[robot].grip_min)/2
        mujoco.mj_forward(model, data)
        site = t.arms[receiver].site
        old_p, old_r = data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()
        rotation = tool @ old_r.T
        for geom in taker:
            data.geom_xpos[geom] = point + rotation @ (data.geom_xpos[geom]-old_p)
            data.geom_xmat[geom] = (rotation @ data.geom_xmat[geom].reshape(3, 3)).ravel()
        pairs = [(a, b) for a in giver for b in taker]
        pairs += [(a, b) for a in taker_palm+giver_palm for b in payload]
        if released:
            pairs += [(a, b) for a in giver for b in payload]
        for a, b in pairs:
            distance = mujoco.mj_geomDistance(model, data, int(a), int(b), .003, None)
            if distance < .003:
                if diagnostics is not None:
                    diagnostics.update(donor_gap_m=float(dg), receiver_gap_m=float(rg),
                        pair_bodies=[model.body(int(model.geom_bodyid[g])).name for g in (a, b)],
                        pair_geoms=[model.geom(g).name for g in (a, b)], distance_m=float(distance))
                return False
    return True


def _robot_payload_contacts(trial, robot, name):
    owner = list(trial.arms).index(robot)
    geoms = set(trial.monitor.geoms[name])
    for contact in trial.data.contact:
        a, b = map(int, contact.geom)
        if contact.dist <= .0002 and (
                (a in geoms and trial.monitor.geom_owner[b] == owner) or
                (b in geoms and trial.monitor.geom_owner[a] == owner)):
            return True
    return False


def _receiver_tilt(tool,degrees):
    angle=math.radians(degrees);c,s=math.cos(angle),math.sin(angle)
    return np.column_stack((c*tool[:,0]-s*tool[:,2],tool[:,1],s*tool[:,0]+c*tool[:,2]))


def _receiver_successor_clear(t,donor,receiver,name,goal,point,tool,contact_q=None):
    """Private successor screen; optional sampled fit/release/descent check.

    Neither mode predicts physical slipping, tracking or the full transit.
    """
    if not getattr(t,'_reference_planning_only',False):
        raise RuntimeError('Successor screening requires planning-only data')
    from ..robot_control import matrix_quat
    before=t.data.qpos.copy();held=t.held
    try:
        body=t.data.body(name);object_r=body.xmat.reshape(3,3).copy()
        offset=object_r.T@(point-body.xpos);relative=object_r.T@tool
        if 'next_handover_center_m' in goal:
            # A middle carrier need not reach the final board. Check that
            # this proposed grasp can carry to its next transfer neighborhood.
            # The next pair's grasp, route and actual support are checked when
            # that handover is planned/executed, not assumed certified here.
            from .scenes import rz
            contact=contact_q if contact_q is not None else t.screen_ik(receiver,point,tool,t.data.qpos[t.arms[receiver].qids])
            if contact is None:return False
            t.data.qpos[t.arms[receiver].qids]=contact
            if donor in getattr(t,'parking_q',{}):t.data.qpos[t.arms[donor].qids]=t.parking_q[donor]
            mujoco.mj_kinematics(t.model,t.data);t.held={receiver:name}
            center=np.asarray(goal['next_handover_center_m'],dtype=float)
            for height in (.22,.30,.38):
                p=np.r_[center[:2],t.task['table_height_m']+height]
                for yaw in (0.,math.pi/2,-math.pi/2,math.pi):
                    r=rz(yaw)@object_r;end_tool=r@relative
                    q=t.screen_ik(receiver,p+r@offset,end_tool,contact)
                    if q is not None and t.pose_clear(receiver,q):return True
            return False
        board=t.data.body(goal['board']);board_r=board.xmat.reshape(3,3).copy()
        position=board.xpos+board_r@np.asarray(goal['local_position_m'])
        if t.task['level']==1:position=position+board_r[:,2]*float(goal.get('release_clearance_m',0.))
        if donor in getattr(t,'parking_q',{}):t.data.qpos[t.arms[donor].qids]=t.parking_q[donor]
        t.held={};width=float(t.task['cube_size_m'])
        if goal.get('check_socket_path') and t.task['level']>1:
            from .transport_socket_motion import seating_candidate
            with socket_contact_clearance(t,name,goal['board']):
                plan,_=seating_candidate(t,receiver,name,position,board_r,goal['board'],offset,relative,budget_s=2.)
            return plan is not None
        for yaw in ((0.,math.pi) if t.task['level']>1 else (0.,math.pi/2,math.pi,-math.pi/2)):
            from .scenes import rz
            r=board_r@rz(yaw);rotation=r@relative;target=position+r@offset
            t.data.joint(name+'_free').qpos[:]=np.r_[position,matrix_quat(r)]
            mujoco.mj_forward(t.model,t.data)
            context=socket_contact_clearance(t,name,goal['board']) if t.task['level']>1 else nullcontext()
            with context:
                for seed in (t.home_q[receiver],t.data.qpos[t.arms[receiver].qids]):
                    q=t.screen_ik(receiver,target,rotation,seed)
                    if q is None:continue
                    if min(np.min(q-t.arms[receiver].ik.limits[:,0]),np.min(t.arms[receiver].ik.limits[:,1]-q))<.02:continue
                    ref_ok=True
                    for gap in (width,min(t.arms[receiver].grip_max,width+.012)):
                        _scratch_gap(t,receiver,gap)
                        if not t.pose_clear(receiver,q,(name,),clearance_m=.003):ref_ok=False;break
                    if not ref_ok:continue
                    pre=t.screen_ik(receiver,target+board_r[:,2]*.14,rotation,q)
                    if pre is not None and t.pose_clear(receiver,pre):return True
        return False
    finally:
        t.data.qpos[:]=before;t.held=held;mujoco.mj_forward(t.model,t.data)


def plan_air_regrasp(trial, donor, name, diagnostics=None, planning_budget_s=120.,
                     *, allowed_receivers=None, candidate_centers=None,receiver_tilt_deg=0.,receiver_depths=None,future_goal=None):
    """Finite raised transfer candidates. Endpoints are not execution proof.

    The receiver filter selects this reference's declared transfer-center
    candidates; it does not define which robot pairs the task permits.
    """
    from .scenes import rz
    from ..robot_control import matrix_quat
    if not trial.finger_contacts(donor, name):
        raise RuntimeError('Air-regrasp planning requires retained donor payload '+name)
    stats = diagnostics if diagnostics is not None else {}
    if candidate_centers is None:
        declared, proposed_edges = graph(trial.env)
    else:
        declared={r:[] for r in trial.arms};proposed_edges={}
        for receiver,point in candidate_centers.items():
            point=np.asarray(point,dtype=float)
            if receiver==donor or receiver not in trial.arms or point.shape!=(3,) or not np.isfinite(point).all():
                raise ValueError('Invalid private handover candidate center')
            declared[donor].append(receiver);declared[receiver].append(donor)
            proposed_edges[frozenset((donor,receiver))]=point.copy()
    receivers = list(declared[donor])
    if allowed_receivers is not None:
        requested = set(allowed_receivers)
        if requested-set(receivers):
            raise ValueError('Receiver has no declared transfer-center candidate in this finite planner')
        receivers = [robot for robot in receivers if robot in requested]
        if not receivers:
            stats['receiver_filter_empty'] = True
            return None
    started = time.monotonic()
    def budget():
        stats['planning_wall_s'] = time.monotonic()-started
        if stats['planning_wall_s'] > planning_budget_s:
            stats['planning_budget_exhausted'] = True
            return True
        return False
    def count(key):
        stats[key] = stats.get(key, 0) + 1
    depths=(.10*float(trial.task['cube_size_m']),.25*float(trial.task['cube_size_m']),.40*float(trial.task['cube_size_m'])) if receiver_depths is None else tuple(receiver_depths)
    if not depths or not all(math.isfinite(v) and abs(v)<=float(trial.task['cube_size_m'])/2 for v in depths):
        raise ValueError('Receiver grasp offsets must lie within the physical object half-height')
    t = _planning_copy(trial)
    original = t.data.qpos.copy()
    obj = t.data.body(name)
    rotation0 = obj.xmat.reshape(3, 3).copy()
    arm = t.arms[donor]
    offset = rotation0.T @ (t.data.site_xpos[arm.site] - obj.xpos)
    relative = rotation0.T @ t.data.site_xmat[arm.site].reshape(3, 3)
    neighbors, edges = declared, proposed_edges
    width = float(t.task['cube_size_m'])

    def seeds(robot):
        a = t.arms[robot]
        result = [t.data.qpos[a.qids].copy(), t.home_q[robot].copy()]
        for delta in (-1., 1.):
            q = t.home_q[robot].copy()
            q[0] += delta
            q[2] -= delta
            result.append(np.clip(q, *a.ik.limits.T))
        return result

    # A pins-up plug cannot be received from below in every arm layout.
    # Screen a real 90-degree donor wrist turn before asking another arm to
    # grasp it. These are candidate orientations, not object state commands.
    tilts = [np.eye(3)]
    for sign in (1., -1.):
        tilts.extend((np.array([[1.,0,0],[0,0,-sign],[0,sign,0]]),
                      np.array([[0,0,sign],[0,1.,0],[-sign,0,0]])))
    if rotation0[2, 2] < -.8:
        tilts = tilts[1:] + tilts[:1]
    rotations = [rz(yaw) @ tilt @ rotation0 for tilt in tilts
                 for yaw in (0., math.pi/2, -math.pi/2, math.pi)]

    geometry = {}
    for receiver in receivers:
        if receiver in trial.held:
            continue
        for axis in (0, 1):
            for sign in (1, -1):
                y, z = rotation0[:, axis]*sign, -rotation0[:, 2]
                tool = _receiver_tilt(np.column_stack((np.cross(y, z), y, z)),receiver_tilt_deg)
                tangent = rotation0[:, 1-axis]
                for depth in depths:
                    for shift in (0., -.15*width, .15*width):
                        point = obj.xpos + rotation0[:, 2]*depth + tangent*shift
                        reason={}
                        allowed = _hand_geometry_clear(t, donor, receiver, name, point, tool, width,diagnostics=reason)
                        if not allowed and 'first_hand_geometry_rejection' not in stats:
                            stats['first_hand_geometry_rejection']=reason
                        geometry[receiver, axis, sign, depth, shift] = allowed
                        count('hand_geometry_compatible' if allowed else 'hand_geometry_rejected')
    if not any(geometry.values()):
        return None

    for receiver in receivers:
        if receiver in trial.held or not any(v for k, v in geometry.items() if k[0] == receiver):
            continue
        edge = edges[frozenset((donor, receiver))]
        for height in (.22, .30, .38):
            for rotation in rotations:
                for dx, dy in ((0, 0), (-.08, 0), (0, -.08), (.08, 0), (0, .08)):
                    if budget():
                        return None
                    t.data.qpos[:] = original
                    mujoco.mj_forward(t.model, t.data)
                    center = np.array([edge[0]+dx, edge[1]+dy,
                                       t.task['table_height_m']+height])
                    donor_tool = rotation @ relative
                    donor_point = center + rotation @ offset
                    donor_q = None
                    for seed in seeds(donor):
                        q = t.screen_ik(donor, donor_point, donor_tool, seed)
                        if q is not None and t.pose_clear(donor, q):
                            donor_q = q
                            break
                    if donor_q is None:
                        continue
                    count('donor_clear_endpoints')
                    # This placement is in independent scratch data only.
                    t.data.qpos[t.arms[donor].qids] = donor_q
                    t.data.joint(name+'_free').qpos[:] = np.r_[center, matrix_quat(rotation)]
                    mujoco.mj_forward(t.model, t.data)
                    _scratch_gap(t, receiver, min(t.arms[receiver].grip_max, width+.012))
                    for axis in (0, 1):
                        for sign in (1, -1):
                            y, z = rotation[:, axis]*sign, -rotation[:, 2]
                            tool = _receiver_tilt(np.column_stack((np.cross(y, z), y, z)),receiver_tilt_deg)
                            tangent = rotation[:, 1-axis]
                            for depth in depths:
                                for shift in (0., -.15*width, .15*width):
                                    if not geometry[receiver, axis, sign, depth, shift]:
                                        continue
                                    point = center + rotation[:, 2]*depth + tangent*shift
                                    approach = point-tool[:,2]*.15
                                    for seed in seeds(receiver):
                                        if budget():return None
                                        q = t.screen_ik(receiver, approach, tool, seed)
                                        if q is None:
                                            continue
                                        count('receiver_approach_ik')
                                        # Reserve 3 mm beyond execution's 3 mm
                                        # screen for held-pose/servo prediction error.
                                        if not t.pose_clear(receiver, q, clearance_m=.006):
                                            continue
                                        count('receiver_approach_clear')
                                        near = t.screen_ik(receiver, point, tool, q)
                                        if near is None:
                                            continue
                                        count('receiver_contact_ik')
                                        if not t.pose_clear(receiver, near, (name,), clearance_m=.001):
                                            continue
                                        count('receiver_contact_clear')
                                        track, continuous = q.copy(), True
                                        for fraction in np.linspace(0., 1., 9)[1:]:
                                            sample = approach*(1-fraction)+point*fraction
                                            track = t.screen_ik(receiver, sample, tool, track)
                                            if track is None or not t.pose_clear(receiver, track, (name,), clearance_m=.001):
                                                continuous = False
                                                break
                                        if not continuous:
                                            continue
                                        count('receiver_contact_continuation_clear')
                                        if not _release_sweep_clear(t, donor, receiver, name, near, width):
                                            continue
                                        count('gripper_sweeps_and_release_clear')
                                        if budget():return None
                                        if future_goal is not None and not _receiver_successor_clear(t,donor,receiver,name,future_goal,point,tool,contact_q=near):
                                            count('receiver_successor_rejected');continue
                                        if future_goal is not None:count('receiver_successor_endpoints_clear')
                                        return dict(receiver=receiver, donor_point=donor_point,
                                            donor_tool=donor_tool, donor_q=donor_q,
                                            receiver_approach_q=q,
                                            receiver_axis=axis, receiver_sign=sign,
                                            receiver_depth=depth, receiver_shift=shift,
                                            receiver_relative_rotation=rotation.T@tool,
                                            buffer_xy=np.asarray(edge[:2]).copy())
    return None


def unload_receiver_for_donor_release(trial,donor,receiver,name):
    """Try legal receiver pressures after a known, blocked donor opening.

    The receiver must retain real force-bearing support. Lower pressure can
    remove frictional preload without changing contact physics or drive limits.
    Every retry is a new ordinary command after a known result, never unknown
    action replay. The caller restores pressure after donor withdrawal.
    """
    arm=trial.arms[receiver];original=arm.grasp_force
    if original is None:raise RuntimeError('Receiver has no active grasp for release unloading')
    minimum=float(trial.cfg['allowed_grasp_force_n'][0]);width=float(trial.task['cube_size_m'])
    initial_z=float(trial.data.body(name).xpos[2])
    for force in sorted({(minimum+original)/2,minimum},reverse=True):
        if force>=original:continue
        if receiver not in trial.monitor.bilateral(trial.monitor.geoms[name],positive_force=True):
            raise RuntimeError('Receiver support unverified before release unloading')
        trial.execute('regrasp-receiver-unload',{receiver:('gripper',dict(mode='grasp',width_m=width,force_n=force,
            speed_m_s=.025,epsilon_inner_m=.004,epsilon_outer_m=.008))})
        trial.wait(.15)
        if receiver not in trial.monitor.bilateral(trial.monitor.geoms[name],positive_force=True) or float(trial.data.body(name).xpos[2])<initial_z-.01:
            raise RuntimeError('Receiver support changed during release unloading')
        result=trial.execute('regrasp-donor-release-unloaded',{donor:('gripper',dict(mode='move',
            width_m=trial.arms[donor].grip_max,speed_m_s=.005))},allow_failure=True)[donor]
        if trial.arms[donor].grasp_force is not None or result['status']=='uncertain':
            raise RuntimeError('Donor release unresolved after unloading')
        if result['status']!='completed' and result.get('reason') not in ('Requested opening not reached','Requested aperture exceeded'):
            raise RuntimeError('Donor release failed after unloading: '+str(result))
        trial.wait(.15)
        supported=receiver in trial.monitor.bilateral(trial.monitor.geoms[name],positive_force=True)
        contact=_robot_payload_contacts(trial,donor,name)
        trial.event('reference_release_unloading',donor=donor,receiver=receiver,object=name,
            receiver_force_setpoint_n=force,donor_result=result,donor_still_contacts=contact,receiver_support=supported)
        if not supported or float(trial.data.body(name).xpos[2])<initial_z-.01:
            raise RuntimeError('Receiver did not retain payload during donor release')
        if not contact:return original
    raise RuntimeError('Donor still contacts payload after legal pressure unloading')


def withdraw_released_donor(trial,donor,receiver,name,restore_force=None):
    """Complete a known released sender retreat before the carrier continues."""
    from .placement_oracle import park
    width=float(trial.task['cube_size_m'])
    trial.held.pop(donor,None)
    trial.held[receiver] = name
    pose = trial.data.site_xpos[trial.arms[donor].site].copy()
    donor_tool = trial.data.site_xmat[trial.arms[donor].site].reshape(3, 3).copy()
    # A tool-axis retreat is a candidate, not a whole-arm certificate.
    # Donor no longer carries the item: do not permit finger/object contact
    # in this route screen. All links and the receiver remain obstacles.
    back, up = -donor_tool[:, 2], np.array([0., 0., 1.])
    directions = []
    for direction in (back, back+.75*up, up):
        length = np.linalg.norm(direction)
        if length < .01:
            continue
        direction = direction/length
        if not any(np.dot(direction, previous) > .999 for previous in directions):
            directions.append(direction)
    cleared = False
    for direction in directions:
        for distance in (.16, .10, .06):
            try:
                trial.move_free(donor, pose+direction*distance, donor_tool,
                                'regrasp-send-clear')
            except RuntimeError as error:
                if str(error).startswith(('No clear free-arm IK',
                                          'No collision-free sampled free-arm route')):
                    trial.event('reference_retreat_candidate_rejected', robot=donor,
                                distance_m=distance, reason=str(error))
                    continue
                raise
            cleared = True
            break
        if cleared:
            break
    if not cleared:
        raise RuntimeError('No fully screened donor withdrawal '+donor+'/'+name)
    park(trial, donor)
    if restore_force is not None:
        trial.execute('regrasp-receiver-restore-pressure',{receiver:('gripper',dict(mode='grasp',width_m=width,
            force_n=restore_force,speed_m_s=.025,epsilon_inner_m=.004,epsilon_outer_m=.008))})
        trial.wait(.15)
    if not trial.finger_contacts(receiver, name):
        raise RuntimeError('Air-regrasp payload not retained after donor withdrawal')


def regrasp_held_plug(trial, donor, name, *, keep_receiver_held=False,
                     allowed_receivers=None,candidate_centers=None,receiver_tilt_deg=0.,receiver_depths=None,future_goal=None,planning_budget_s=120.):
    """Real handover, optionally retaining its actual carrier for a successor.

    Default staging behavior is retained. The optional branch returns only
    after real receiver support, donor release and withdrawal; its caller
    must plan/execute the next operation from that actual carrier.
    """
    from .placement_oracle import park
    diagnostics = {}
    if candidate_centers is not None or future_goal is not None:
        plan=plan_air_regrasp(trial,donor,name,diagnostics,planning_budget_s=planning_budget_s,
            allowed_receivers=allowed_receivers,candidate_centers=candidate_centers,receiver_tilt_deg=receiver_tilt_deg,receiver_depths=receiver_depths,future_goal=future_goal)
    elif allowed_receivers is None:
        plan = plan_air_regrasp(trial, donor, name, diagnostics)
    else:
        plan = plan_air_regrasp(trial, donor, name, diagnostics,
                               allowed_receivers=allowed_receivers)
    trial.event('reference_air_regrasp_search', donor=donor, object=name, diagnostics=diagnostics,
                scope='Bounded private candidate screening, not a feasibility certificate')
    if plan is None:
        raise RuntimeError('No bounded physical air-regrasp candidate '+donor+'/'+name+'; '+str(diagnostics))
    receiver = plan['receiver']
    trial.event('reference_air_regrasp_plan', donor=donor, receiver=receiver,
                object=name, scope='Candidate endpoints; actual route and contacts still required')
    trial.move_free(donor, plan['donor_point'], plan['donor_tool'],
                    'regrasp-donor-transit', target_joints=plan['donor_q'])
    if not trial.finger_contacts(donor, name):
        raise RuntimeError('Donor grasp lost during air-regrasp transit '+name)
    obj = trial.data.body(name)
    rotation = obj.xmat.reshape(3, 3).copy()
    normal = rotation[:, 2]
    axis, sign = plan['receiver_axis'], plan['receiver_sign']
    y, z = rotation[:, axis]*sign, -normal
    tool = obj.xmat.reshape(3,3)@plan['receiver_relative_rotation'] if 'receiver_relative_rotation' in plan else np.column_stack((np.cross(y, z), y, z))
    point = obj.xpos + normal*plan['receiver_depth'] + rotation[:, 1-axis]*plan['receiver_shift']
    width = float(trial.task['cube_size_m'])
    opening = min(trial.arms[receiver].grip_max, width+.012)
    trial.execute('regrasp-receiver-open', {receiver: ('gripper', dict(
        mode='move', width_m=opening, speed_m_s=.05))})
    trial.move_free(receiver, point-tool[:,2]*.15, tool, 'regrasp-receiver-approach',
                    continuation=(point, tool, (name,)), continuation_clearance=.001,
                    preferred_seed=plan.get('receiver_approach_q'))
    trial.move(receiver, point, tool, 'regrasp-contact', precise=True)
    force = 70.
    force = min(force, trial.cfg['allowed_grasp_force_n'][1])
    trial.execute('regrasp-receiver-close', {receiver: ('gripper', dict(
        mode='grasp', width_m=width, force_n=force, speed_m_s=.025,
        epsilon_inner_m=.004, epsilon_outer_m=.008))})
    trial.wait(.20)
    if not trial.finger_contacts(receiver, name) or not trial.finger_contacts(donor, name):
        raise RuntimeError('Air-regrasp requires actual bilateral support from both arms')
    released = trial.execute('regrasp-donor-release', {donor: ('gripper', dict(
        mode='move', width_m=trial.arms[donor].grip_max, speed_m_s=.005))}, allow_failure=True)[donor]
    if trial.arms[donor].grasp_force is not None or released['status'] == 'uncertain':
        raise RuntimeError('Air-regrasp donor release unresolved')
    if released['status'] == 'failed' and released.get('reason') not in (
            'Requested opening not reached', 'Requested aperture exceeded'):
        raise RuntimeError('Air-regrasp donor release failed: '+str(released))
    restore_force=None
    if _robot_payload_contacts(trial, donor, name):
        restore_force=unload_receiver_for_donor_release(trial,donor,receiver,name)
    trial.event('reference_donor_release_verified', robot=donor, object=name,
                actual_width_m=trial.arms[donor].observe()['gripper_width_m'])
    withdraw_released_donor(trial,donor,receiver,name,restore_force=restore_force)
    if keep_receiver_held:
        trial.event('reference_air_regrasp_carrier_retained', object=name,
                    donor=donor, receiver=receiver,
                    scope='Actual receiver holds after verified donor release; no buffer placement or donor repick')
        return receiver
    # Blades protrude 24 mm beyond the 60 mm body. The shared buffer release
    # stops 15 mm above its requested pose, so derive a one-mm release gap.
    blade = trial.env['mechanism']['blade_size_m'][2]
    center_height = trial.task['table_height_m'] + width/2 + blade + .001 - .015
    destination = np.r_[plan['buffer_xy'], center_height]
    trial.place(receiver, destination, np.eye(3), precise=False)
    park(trial, receiver)
    trial.wait(.4)
    if trial.data.body(name).xmat.reshape(3, 3)[2, 2] < .8:
        raise RuntimeError('Staged plug did not settle insertion-side down')
    trial.event('reference_air_regrasp_complete', object=name, donor=donor,
                receiver=receiver, scope='Released upright stage; original arm still must physically pick again')


@contextmanager
def socket_contact_clearance(trial, name, board):
    """Use real geometry for the intended pin/socket fit, only in scratch.

    Slots have a 1 mm side gap, smaller than the free-motion 3 mm safety
    clearance. Remove the artificial 4 mm contact-generation margin only from
    both mating surfaces. Arm/hand and other-object margins remain unchanged;
    negative-distance pin/wall collisions are still generated and rejected.
    """
    pins = [g for g in trial.monitor.geoms[name]
            if trial.model.geom(g).name.startswith(name+'_blade')]
    if len(pins) != 2:
        raise RuntimeError('Expected exactly two transport insertion blades')
    board_geoms = trial.monitor.descendant_geoms(trial.model.body(board).id)
    indices = np.array(sorted(set(pins) | set(board_geoms)), dtype=int)
    previous = trial.screen_model.geom_margin[indices].copy()
    trial.screen_model.geom_margin[indices] = trial.model.geom_margin[indices]
    try:
        yield
    finally:
        trial.screen_model.geom_margin[indices] = previous


def _pickup_height(task, rotation, default=.006, shape='cube'):
    if task['level'] <= 1 or rotation[2, 2] >= .8:
        return default
    fraction = .30 if shape == 'cylinder' and rotation[2, 2] > -.8 else .40
    return max(default, fraction*float(task['cube_size_m']))


def _pickup_frames(shape, rotation):
    """A cylinder's arbitrary radial clocking must not tilt its top grasp."""
    if shape == 'cylinder' and abs(rotation[2, 2]) < .8:
        axis = rotation[:, 2]
        up = np.array([0., 0., 1.])-axis*axis[2]
        up /= np.linalg.norm(up)
        closing = np.cross(axis, up)
        closing /= np.linalg.norm(closing)
        frames = []
        for sign in (1, -1):
            y, z = closing*sign, -up
            frames.append(np.column_stack((np.cross(y, z), y, z)))
        return up, frames
    index = int(np.argmax(abs(rotation[2])))
    up = rotation[:, index]*np.sign(rotation[2, index])
    frames = []
    for axis in (i for i in range(3) if i != index):
        for sign in (1, -1):
            y, z = rotation[:, axis]*sign, -up
            frames.append(np.column_stack((np.cross(y, z), y, z)))
    return up, frames


def _pick_side_cylinder(t, robot, name, height, force, tool_x_shift=0., lift_speed_m_s=.025):
    """Cylinder body pickup with a finite optional contact-patch shift."""
    arm = t.arms[robot]
    width = float(t.task['cube_size_m'])
    opening = min(arm.grip_max, width+.012)
    if arm.grasp_force is not None or abs(arm.observe()['gripper_width_m']-opening) > .0005:
        t.execute('open-for-pick', {robot: ('gripper', dict(mode='move', width_m=opening, speed_m_s=.05))})
    obj = t.data.body(name)
    up, frames = _pickup_frames('cylinder', obj.xmat.reshape(3, 3).copy())
    point = obj.xpos.copy()+up*height
    chosen = None
    for frame in frames:
        contact_point = point+frame[:, 0]*tool_x_shift
        try:
            t.move_free(robot, contact_point+up*.15, frame, 'pick-approach',
                        continuation=(contact_point, frame, (name,)), continuation_clearance=.001)
        except RuntimeError as error:
            if str(error).startswith(('No clear free-arm IK', 'No collision-free sampled free-arm route')):
                continue
            raise
        chosen = frame
        break
    if chosen is None:
        raise RuntimeError('No radial cylinder pickup pose '+robot+'/'+name)
    before = obj.xpos.copy()
    up, _ = _pickup_frames('cylinder', obj.xmat.reshape(3, 3).copy())
    t.move(robot, before+up*height+chosen[:, 0]*tool_x_shift, chosen, 'pick-contact', precise=True)
    t.execute('pick-grasp', {robot: ('gripper', dict(mode='grasp', width_m=width,
        force_n=force, speed_m_s=.04, epsilon_inner_m=.008, epsilon_outer_m=.008))})
    t.wait(.15)
    if not t.finger_contacts(robot, name):
        raise RuntimeError('Bilateral cylinder contact missing '+name)
    t.move(robot, t.data.site_xpos[arm.site]+np.array([0.,0.,.14]), chosen,
           'pick-lift', speed_m_s=lift_speed_m_s)
    t.wait(.1)
    if t.data.body(name).xpos[2] < before[2]+.08:
        raise RuntimeError('Cylinder payload not retained '+name)
    t.held[robot] = name
    t.event('picked', robot=robot, object=name)


def _pickup_coverage(trial, items=None):
    """Same pickup height/hand aperture as execution; endpoints remain candidates."""
    t = _planning_copy(trial)
    result = {}
    for item in (t.env['objects'] if items is None else items):
        name = item['id']
        body = t.data.body(name)
        p, r = body.xpos.copy(), body.xmat.reshape(3, 3).copy()
        up, frames = _pickup_frames(item.get('shape', 'cube'), r)
        point = p + up*_pickup_height(t.task, r, shape=item.get('shape', 'cube'))
        candidates = []
        for robot, arm in t.arms.items():
            if np.linalg.norm(p[:2]-np.asarray(t.task['arms'][robot]['base_position_m'][:2])) > .88:
                continue
            _scratch_gap(t, robot, min(arm.grip_max, float(t.task['cube_size_m'])+.012))
            for rotation in frames:
                # The actual free-path picker tries a current and home seed.
                # A failed home branch must not erase a valid current branch.
                for seed in (t.home_q[robot], t.data.qpos[arm.qids]):
                    q = t.screen_ik(robot, point, rotation, seed=seed)
                    if q is None or not t.pose_clear(robot, q, (name,), clearance_m=.001):
                        continue
                    near = t.screen_ik(robot, point+up*.15, rotation, seed=q)
                    if near is not None and t.pose_clear(robot, near):
                        candidates.append(robot)
                        break
                if candidates and candidates[-1] == robot:
                    break
        # Declared supply ownership is not geometric evidence. An empty map
        # stays empty; surviving endpoints still need the real pickup path.
        result[name] = list(dict.fromkeys(candidates))
    return result


def _repair_small_l1_placement(t, goal, name):
    """Finite actual regrasp/replacement, never tolerance or state correction."""
    from .placement_oracle import park
    for attempt in range(2):
        board = t.data.body(goal['board'])
        br = board.xmat.reshape(3, 3).copy()
        body = t.data.body(name)
        delta = br.T@(body.xpos-board.xpos)-np.asarray(goal['local_position_m'])
        if (np.linalg.norm(delta[:2]) > .020 or abs(delta[2]) > .003
                or (br.T@body.xmat.reshape(3, 3))[2, 2] < math.cos(math.radians(5))
                or not t.monitor.released(t.monitor.geoms[name])):
            return t.monitor.result()
        def pose():
            b = t.data.body(goal['board'])
            r = b.xmat.reshape(3, 3).copy()
            return b.xpos+r@np.asarray(goal['local_position_m']), r
        point, rotation = pose()
        robot = goal['robot']
        t.event('reference_goal_repair', goal=goal['goal_id'], object=name,
                attempt=attempt+1, method='Real regrasp and replacement; unchanged acceptance')
        t.pick(robot, name, placement_hint=(point, rotation))
        t.place(robot, point, rotation, precise=True, pose_provider=pose)
        park(t, robot)
        t.wait(.7)
        result = t.monitor.result()
        t.event('reference_goal_repair_result', goal=goal['goal_id'], object=name,
                attempt=attempt+1, progress=result.get('matched_goals', 0))
        if t.monitor.violations:
            raise RuntimeError('Process violation during goal repair: '+str(t.monitor.violations[-1]))
        if goal['goal_id'] in result.get('assignments', {}):
            return result
    return result


def _legacy_baseline(t):
    """Existing finite logistics order with transport-owned pickup coverage."""
    from .placement_oracle import clear_idle_arms, shortest_pickup_route, deliver
    clear_idle_arms(t)
    neighbors, _ = graph(t.env)
    pickups = _pickup_coverage(t)
    unused = {obj['id']: obj for obj in t.env['objects']}
    jobs = []
    for goal in t.task['goals']:
        obj = next(o for o in unused.values() if o['identity'] == goal['requires']['identity'])
        unused.pop(obj['id'])
        path = shortest_pickup_route(t, neighbors, pickups[obj['id']], goal['robot'], obj['id'])
        jobs.append((goal, obj, path))
    jobs.sort(key=lambda item: -len(item[2]))
    for goal, obj, path in jobs:
        # Stocks can roll or be displaced while earlier jobs execute. Keep
        # the logistics order but refresh this pickup's present candidates.
        current = _pickup_coverage(t, [obj])[obj['id']]
        path = shortest_pickup_route(t, neighbors, current, goal['robot'], obj['id'])
        t.event('reference_pickup_route_refresh', object=obj['id'], candidates=current, path=path,
                scope='Current-state pickup endpoint candidates; actual path still checked')
        def pose(goal=goal):
            board = t.data.body(goal['board'])
            rotation = board.xmat.reshape(3, 3).copy()
            return board.xpos+rotation@np.asarray(goal['local_position_m']), rotation
        point, rotation = pose()
        deliver(t, obj['id'], path, point, rotation, pose)
        t.wait(.7)
        result = t.monitor.result()
        t.event('goal_attempt', goal=goal['goal_id'], object=obj['id'], path=path, progress=result.get('matched_goals', 0))
        if t.monitor.violations:
            raise RuntimeError('Process violation: '+str(t.monitor.violations[-1]))
        if goal['goal_id'] not in result.get('assignments', {}) and t.task['level'] == 1:
            result = _repair_small_l1_placement(t, goal, obj['id'])
        if goal['goal_id'] not in result.get('assignments', {}):
            raise RuntimeError('Released object did not satisfy socket/board goal '+goal['goal_id'])
    t.wait(1.2)


def _occupied_transport_robots(t):
    """Actual contact/force ownership, not a nominal station reservation."""
    occupied=set(t.held)
    occupied.update(robot for robot,arm in t.arms.items() if arm.grasp_force is not None)
    payload={g for obj in t.env['objects'] for g in t.monitor.geoms[obj['id']]}
    for c in t.data.contact:
        if c.dist>.0002:continue
        a,b=map(int,c.geom)
        for obj,other in ((a,b),(b,a)):
            if obj in payload and int(t.monitor.geom_owner[other])>=0:
                occupied.add(list(t.arms)[int(t.monitor.geom_owner[other])])
    return occupied


def _safe_transport_deferral(t,error):
    """Only settled, known admission/tabletop failures can yield to other work."""
    if t.monitor.violations or any(arm.job is not None or getattr(arm,'uncertainty_reported',False)
                                  for arm in t.arms.values()):return False
    if any(arm.grasp_force is not None for arm in t.arms.values()):return False
    allowed=('Payload not physically released before withdrawal','No pickup pose',
             'No radial cylinder pickup pose','No clear free-arm IK',
             'No collision-free sampled free-arm route','Released object did not satisfy',
             'Requested pose has no verified IK solution')
    if not str(error).startswith(allowed):return False
    # A retained load in air cannot be ignored while other jobs execute.
    for name in set(t.held.values()):
        if not _table_supported_cylinder(t,name):return False
        velocity=t.data.joint(name+'_free').qvel
        if np.linalg.norm(velocity[:3])>.005 or np.linalg.norm(velocity[3:])>.05:return False
    return True


def _forward_baseline(t):
    """Bounded forward scheduling in one real world; no per-arm/state rewind.

    The saved queue is a task plan, not a physics checkpoint. A live recovery
    adapter may call this again on the same Trial after verified compensation.
    """
    from .placement_oracle import clear_idle_arms, shortest_pickup_route, deliver
    if any(getattr(arm,'job',None) is not None or getattr(arm,'uncertainty_reported',False)
           for arm in t.arms.values()):
        raise RuntimeError('Synchronous transport frontier requires settled verified action results')
    if not hasattr(t,'transport_pending_jobs'):
        selected=getattr(t,'transport_goal_ids',None)
        if selected is not None and set(selected)-{g['goal_id'] for g in t.task['goals']}:
            raise ValueError('Unknown transport goal selection')
        if not hasattr(t,'parking_q'):
            if getattr(t,'actions',[]) or t.held or any(arm.grasp_force is not None for arm in t.arms.values()):
                raise RuntimeError('Live transport resume needs saved parking targets; refusing a blanket robot reset')
            clear_idle_arms(t)
        neighbors,_=graph(t.env);pickups=_pickup_coverage(t)
        accepted=t.monitor.result().get('assignments',{})
        reserved=set(accepted.values())
        unused={o['id']:o for o in t.env['objects'] if o['id'] not in reserved};jobs=[]
        for goal in t.task['goals']:
            if goal['goal_id'] in accepted:continue
            obj=next(o for o in unused.values() if o['identity']==goal['requires']['identity'])
            unused.pop(obj['id'])
            if selected is not None and goal['goal_id'] not in selected:continue
            starts=pickups.get(obj['id'],[])
            try:path=shortest_pickup_route(t,neighbors,starts,goal['robot'],obj['id']) if starts else []
            except ValueError:path=[]
            jobs.append((goal,obj,path))
        jobs.sort(key=lambda job:-len(job[2]))
        t.transport_pending_jobs=jobs;t.transport_failed_jobs=[];t.transport_job_attempts={}
        t.transport_failed_revision={};t.transport_forward_revision=0
        t.transport_selected_goals=[g['goal_id'] for g in t.task['goals']
                                    if selected is None or g['goal_id'] in selected]
    t.transport_forward_revision+=1
    accepted_now=t.monitor.result().get('assignments',{})
    t.transport_failed_jobs=[job for job in t.transport_failed_jobs
                             if job[0]['goal_id'] not in accepted_now]
    failures=[]
    while t.transport_pending_jobs:
        t.check_stop()
        # Clear bookkeeping only from measured separation; never reset a robot.
        for robot,name in list(t.held.items()):
            if t.arms[robot].grasp_force is None and not _robot_payload_contacts(t,robot,name):
                t.held.pop(robot);t.event('reference_measured_carrier_clear',robot=robot,object=name)
        occupied=_occupied_transport_robots(t);neighbors,_=graph(t.env)
        free={r:[n for n in ns if n not in occupied] for r,ns in neighbors.items() if r not in occupied}
        chosen=None
        assignments=t.monitor.result().get('assignments',{})
        for job in list(t.transport_pending_jobs):
            goal,obj,_=job
            if goal['goal_id'] in assignments:
                t.transport_pending_jobs.remove(job);continue
            if obj['id'] in assignments.values():continue
            if t.transport_failed_revision.get(goal['goal_id'])==t.transport_forward_revision:continue
            if obj['id'] in t.held.values() or goal['robot'] not in free:continue
            starts=[r for r in _pickup_coverage(t,[obj])[obj['id']] if r in free]
            try:path=shortest_pickup_route(t,free,starts,goal['robot'],obj['id']) if starts else []
            except ValueError:path=[]
            if path:chosen=(job,path);break
        if chosen is None:break
        job,path=chosen;goal,obj,_=job;t.transport_pending_jobs.remove(job)
        t.event('reference_forward_job',goal=goal['goal_id'],object=obj['id'],path=path,
                occupied_robots=sorted(occupied),scope='Actual-world remaining work; no robot/state rollback')
        def pose(goal=goal):
            board=t.data.body(goal['board']);rotation=board.xmat.reshape(3,3).copy()
            return board.xpos+rotation@np.asarray(goal['local_position_m']),rotation
        try:
            p,r=pose();deliver(t,obj['id'],path,p,r,pose);t.wait(.7)
            result=t.monitor.result()
            if goal['goal_id'] not in result.get('assignments',{}) and t.task['level']==1:
                result=_repair_small_l1_placement(t,goal,obj['id'])
            if goal['goal_id'] not in result.get('assignments',{}):
                raise RuntimeError('Released object did not satisfy socket/board goal '+goal['goal_id'])
            t.event('goal_attempt',goal=goal['goal_id'],object=obj['id'],path=path,
                    progress=result.get('matched_goals',0))
            t.transport_forward_revision+=1
        except (RuntimeError,ValueError) as error:
            if not _safe_transport_deferral(t,error):
                # The caller may retain this live world for a separately
                # verified recovery. Do not lose the in-progress objective
                # merely because its operation raised; never replay it here.
                t.transport_pending_jobs.insert(0,job)
                t.transport_last_unhandled_boundary=dict(goal=goal['goal_id'],object=obj['id'],
                    reason=str(error),actual_carriers=dict(t.held))
                t.event('reference_forward_unhandled_boundary',**t.transport_last_unhandled_boundary)
                raise
            count=t.transport_job_attempts.get(goal['goal_id'],0)+1;t.transport_job_attempts[goal['goal_id']]=count
            t.transport_failed_revision[goal['goal_id']]=t.transport_forward_revision
            entry=dict(goal=goal['goal_id'],object=obj['id'],reason=str(error),attempt=count,
                       actual_carriers=dict(t.held),occupied_robots=sorted(_occupied_transport_robots(t)))
            failures.append(entry);t.event('reference_forward_deferral',**entry,
                scope='Known settled failure preserved; only verified independent remaining routes may continue')
            if count<2:t.transport_pending_jobs.append(job)
            else:t.transport_failed_jobs.append(job)
            # Prefer short enabled work after a block, not more waiting at it.
            t.transport_pending_jobs.sort(key=lambda item:len(item[2]))
            if getattr(t,'transport_yield_on_deferral',False):
                # One owner-requested handover; a later resume still has all
                # independent jobs and does not stop at every subsequent fault.
                t.transport_yield_on_deferral=False
                break
    t.wait(.2)
    pending=t.transport_pending_jobs+t.transport_failed_jobs
    result=dict(needs_recovery=bool(pending),remaining_jobs=[dict(goal=g['goal_id'],object=o['id']) for g,o,_ in pending],
                recoverable_failures=failures,actual_carriers=dict(t.held),
                commissioning_only=len(t.transport_selected_goals)<len(t.task['goals']),
                selected_goals=t.transport_selected_goals,
                scope='One continuous world; original task acceptance; pending queue is not a physics restart checkpoint')
    t.event('reference_forward_frontier',**result)
    return result


def _baseline(t):
    return _forward_baseline(t) if getattr(t,'transport_forward_recovery',False) else _legacy_baseline(t)


def _table_supported_cylinder(trial, name):
    body = trial.data.body(name)
    height = float(trial.task['table_height_m'])
    extension = (float(trial.env['mechanism']['blade_size_m'][2])
                 if trial.task['level'] > 1 else 0.)
    if (body.xmat.reshape(3, 3)[2, 2] < math.cos(math.radians(5))
            or abs(float(body.xpos[2]) - height - trial.task['cube_size_m']/2-extension) > .0015):
        return False
    payload = set(trial.monitor.geoms[name])
    for contact in trial.data.contact:
        a, b = map(int, contact.geom)
        if a not in payload:
            a, b = b, a
        if (a in payload and b not in payload and contact.dist <= .0002
                and abs(float(contact.pos[2])-height) < .002
                and abs(float(contact.frame[2])) > .8
                and trial.model.body(int(trial.model.geom_bodyid[b])).name == 'world'):
            return True
    return False


def _released_buffer_evidence(trial, robot, name, other_positions):
    """A buffer may shift; verify support and future access, not slot accuracy."""
    body = trial.data.body(name)
    position, rotation = body.xpos.copy(), body.xmat.reshape(3, 3).copy()
    radius = float(trial.task['cube_size_m'])/2
    footprints = []
    for geom in range(trial.model.ngeom):
        if (not trial.model.geom(geom).name.startswith('table_')
                or trial.model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_BOX):
            continue
        frame = trial.data.geom_xmat[geom].reshape(3, 3)
        local = frame.T@(position-trial.data.geom_xpos[geom])
        relative = frame.T@rotation
        extent = radius*np.sqrt(relative[:2, 0]**2+relative[:2, 1]**2)+radius*abs(relative[:2, 2])
        margins = trial.model.geom_size[geom, :2]-abs(local[:2])-extent
        if np.all(margins >= .003):
            footprints.append(dict(geom=trial.model.geom(geom).name, margin_m=margins.tolist()))
    trial.wait(.2)
    body = trial.data.body(name)
    drift = float(np.linalg.norm(body.xpos-position))
    rotation_drift = float(np.linalg.norm(body.xmat.reshape(3, 3)-rotation))
    other_drift = max((float(np.linalg.norm(trial.data.body(obj).xpos-before))
                       for obj, before in other_positions.items()), default=0.)
    evidence = dict(full_support_footprints=footprints, stable_position_drift_m=drift,
                    stable_rotation_matrix_drift=rotation_drift,
                    max_other_object_displacement_m=other_drift, pickup_candidates=[],
                    scope='Actual released tabletop buffer state; endpoint candidates are not a full pickup certificate')
    if (not footprints or drift > .0005 or rotation_drift > .01 or other_drift > .0005
            or not _table_supported_cylinder(trial, name)
            or not trial.monitor.released(trial.monitor.geoms[name])):
        return evidence
    arm = trial.arms[robot]
    point = trial.data.site_xpos[arm.site].copy()+np.array([0., 0., .10])
    frame = trial.data.site_xmat[arm.site].reshape(3, 3).copy()
    scratch = _planning_copy(trial)
    # Actual release was verified above. Do not attach the body to this
    # retreating hand in a planning proxy through stale held bookkeeping.
    scratch.held.pop(robot, None)
    q = scratch.screen_ik(robot, point, frame, seed=trial.data.qpos[arm.qids])
    if q is None or not scratch.pose_clear(robot, q):
        return evidence
    scratch.data.qpos[scratch.arms[robot].qids] = q
    mujoco.mj_forward(scratch.model, scratch.data)
    item = next(obj for obj in trial.env['objects'] if obj['id'] == name)
    evidence['pickup_candidates'] = _pickup_coverage(scratch, [item])[name]
    evidence['allowed'] = bool(evidence['pickup_candidates'])
    return evidence


def _supported_release_unload(trial, robot, name, objects, execute):
    """One physically validated L1-cylinder recovery; never waive release.

    Canonical recorded-command replay reproduces the 57mm capture exactly.
    A real 1mm support adjustment, then ordinary opening, cleared that state.
    Other shapes, sockets, free-held loads and grazing-only contacts do not
    inherit this witness. The old command failures remain in the trace.
    """
    arm = trial.arms[robot]
    if (trial.task['level'] != 1 or objects[name].get('shape') != 'cylinder'
            or arm.grasp_force is not None
            or arm.observe()['gripper_width_m'] >= arm.grip_max-.002
            or not trial.finger_contacts(robot, name)
            or not _table_supported_cylinder(trial, name)):
        return False
    initial = trial.data.body(name).xpos.copy()
    other_positions = {obj['id']: trial.data.body(obj['id']).xpos.copy()
                       for obj in trial.env['objects'] if obj['id'] != name}
    maximum = [0.]
    step = trial.step
    def measured_step():
        step()
        maximum[0] = max(maximum[0], float(np.linalg.norm(
            trial.data.body(name).xpos-initial)))
    trial.step = measured_step
    result = None
    released = False
    evidence = {}
    try:
        point = trial.data.site_xpos[arm.site].copy()
        rotation = trial.data.site_xmat[arm.site].reshape(3, 3).copy()
        point[2] -= .001
        trial.move(robot, point, rotation, 'reference-supported-unload',
                   precise=True, speed_m_s=.005)
        if not _table_supported_cylinder(trial, name):
            return False
        result = execute('reference-supported-unload-open', {robot: ('gripper', dict(
            mode='move', width_m=arm.grip_max, speed_m_s=.005))}, allow_failure=True)[robot]
        if result['status'] != 'completed' or arm.grasp_force is not None:
            return False
        trial.wait(.15)
        released = bool(trial.monitor.released(trial.monitor.geoms[name]))
        if not released:
            return False
        evidence = _released_buffer_evidence(trial, robot, name, other_positions)
        return bool(evidence.get('allowed'))
    finally:
        trial.step = step
        trial.event('reference_supported_release_unload', robot=robot, object=name,
                    adjustment_m=.001, max_object_displacement_m=maximum[0],
                    opening_result=result, physically_released=released, buffer_evidence=evidence,
                    scope='Single supported L1-cylinder buffer recovery; final goal scoring unchanged')


def _single_finger_axis(trial, robot, name, *, supporting_robots=()):
    """A departing hand moves away from its sole touching finger, not the body."""
    payload=set(trial.monitor.geoms[name]);touching=set()
    for contact in trial.data.contact:
        a,b=map(int,contact.geom)
        if a not in payload:a,b=b,a
        if a not in payload or b in payload or contact.dist>.0002:continue
        body=trial.model.body(int(trial.model.geom_bodyid[b])).name
        supporters={r+'_'+side+'_finger' for r in supporting_robots for side in ('left','right')}
        if body=='world' or body in supporters:continue
        if body not in (robot+'_left_finger',robot+'_right_finger'):return None
        touching.add(body)
    if len(touching)!=1:return None
    rotation=trial.data.site_xmat[trial.arms[robot].site].reshape(3,3)
    return rotation[:,1].copy()*(1 if robot+'_left_finger' in touching else -1)


def _supported_separation_clear(trial, robot, name, target, rotation):
    """Fixed supported payload, full-arm departure; never a carried-body proxy."""
    scratch=_planning_copy(trial);scratch.held.pop(robot,None)
    arm=scratch.arms[robot];start=scratch.data.site_xpos[arm.site].copy()
    q=scratch.data.qpos[arm.qids].copy();payload=set(scratch.monitor.geoms[name])
    # The screening model has its own contact margin and contact manifold.
    # Reconstruct the unchanged start there before comparing candidate depths;
    # a live-model depth is not a baseline for that different manifold.
    if not scratch.pose_clear(robot,q,(name,),clearance_m=.001):return False
    original=min((float(c.dist) for c in scratch.screen_data.contact
                  if any(int(g) in payload for g in c.geom)
                  and any(scratch.model.body(int(scratch.model.geom_bodyid[g])).name
                          in (robot+'_left_finger',robot+'_right_finger') for g in c.geom)),default=0.)
    count=max(2,math.ceil(float(np.linalg.norm(target-start))/.001))
    for fraction in np.linspace(0.,1.,count+1)[1:]:
        point=start+(target-start)*fraction
        q=scratch.screen_ik(robot,point,rotation,seed=q)
        if q is None or not scratch.pose_clear(robot,q,(name,),clearance_m=.001):return False
        for contact in scratch.screen_data.contact:
            if any(int(g) in payload for g in contact.geom) and any(
                    scratch.model.body(int(scratch.model.geom_bodyid[g])).name
                    in (robot+'_left_finger',robot+'_right_finger') for g in contact.geom):
                if contact.dist<original-1e-6:return False
    return True


def _supported_release_separate(trial, robot, name, objects):
    """Finite L1 single-finger departure after a verified maximum opening.

    A canonical hot-state replay validates 2mm then cumulative5mm separation.
    Opening, actual release, stable support and later access remain distinct.
    Socket payloads and force-held or bilaterally captured loads are excluded.
    """
    arm=trial.arms[robot]
    if (trial.task['level']!=1 or objects[name].get('shape')!='cylinder'
            or arm.grasp_force is not None
            or arm.observe()['gripper_width_m']<arm.grip_max-.002):return False
    if not _table_supported_cylinder(trial,name):return False
    direction=_single_finger_axis(trial,robot,name)
    if direction is None:return False
    start=trial.data.site_xpos[arm.site].copy()
    rotation=trial.data.site_xmat[arm.site].reshape(3,3).copy()
    initial=trial.data.body(name).xpos.copy()
    others={o['id']:trial.data.body(o['id']).xpos.copy() for o in trial.env['objects'] if o['id']!=name}
    maximum=[0.];step=trial.step;released=False;evidence={}
    def measured_step():
        step();maximum[0]=max(maximum[0],float(np.linalg.norm(trial.data.body(name).xpos-initial)))
    trial.step=measured_step
    try:
        for distance in (.002,.005):
            if trial.monitor.released(trial.monitor.geoms[name]):break
            target=start+direction*distance
            if not _supported_separation_clear(trial,robot,name,target,rotation):return False
            trial.move(robot,target,rotation,'reference-single-finger-separate',precise=True,speed_m_s=.005)
            if not _table_supported_cylinder(trial,name):return False
        released=bool(trial.monitor.released(trial.monitor.geoms[name]))
        if not released:return False
        evidence=_released_buffer_evidence(trial,robot,name,others)
        return bool(evidence.get('allowed'))
    finally:
        trial.step=step
        trial.event('reference_supported_single_finger_release',robot=robot,object=name,
                    separation_direction=direction.tolist(),max_object_displacement_m=maximum[0],
                    physically_released=released,buffer_evidence=evidence,
                    scope='Finite supported L1-cylinder release; no socket or final-score relaxation')


def _plan_supported_side_assist(trial, donor, name, budget_s=30.):
    """Finite receiver template; every opening and donor exit is rescreened.

    This does not predict contact dynamics. Its narrow L2-cylinder template
    was exercised in a real supported dual-grasp release; other poses still
    require the actual support/contact checks below.
    """
    neighbors, _ = graph(trial.env)
    center = trial.data.body(name).xpos.copy()
    started = time.monotonic()
    receivers = [r for r in neighbors[donor] if r not in trial.held
                 and trial.arms[r].job is None and trial.arms[r].grasp_force is None]
    receivers.sort(key=lambda r:np.linalg.norm(center[:2]-np.asarray(trial.task['arms'][r]['base_position_m'][:2])))
    for receiver in receivers:
        to_base=np.asarray(trial.task['arms'][receiver]['base_position_m'])-center
        directions=sorted((np.array([1.,0.,0.]),np.array([0.,1.,0.]),
                           np.array([-1.,0.,0.]),np.array([0.,-1.,0.])),
                          key=lambda v:-float(v@to_base))
        templates=((v,r,h) for v in directions for r in (.020,.024,.028)
                   for h in (-.006,-.012,-.018,.006,.018))
        for outward,radial,height in templates:
            for sign in (1,-1):
                if time.monotonic()-started>budget_s:return None
                z=-outward;y=np.cross(np.array([0.,0.,1.]),z)*sign
                tool=np.column_stack((np.cross(y,z),y,z))
                point=center+outward*radial+[0,0,height]
                approach=point-tool[:,2]*.12
                s=_planning_copy(trial);s.held.clear();arm=s.arms[receiver]
                _scratch_gap(s,receiver,.072);mujoco.mj_forward(s.model,s.data)
                seeds=[s.data.qpos[arm.qids].copy(),s.home_q[receiver].copy()]
                for delta in (-1.,1.):
                    seed=s.home_q[receiver].copy();seed[0]+=delta;seed[2]-=delta
                    seeds.append(np.clip(seed,*arm.ik.limits.T))
                def margin(q):
                    return float(np.min(np.minimum(q-arm.ik.limits[:,0],arm.ik.limits[:,1]-q)))
                for seed in seeds:
                    q=s.screen_ik(receiver,approach,tool,seed=seed)
                    if q is None or margin(q)<.04 or not s.pose_clear(receiver,q):continue
                    approach_q=q.copy();okay=True
                    for fraction in np.linspace(0,1,13)[1:]:
                        q=s.screen_ik(receiver,approach+(point-approach)*fraction,tool,seed=q)
                        if q is None or not s.pose_clear(receiver,q,(name,)):okay=False;break
                    if not okay or margin(q)<.04:continue
                    s.data.qpos[arm.qids]=q;mujoco.mj_forward(s.model,s.data)
                    main=s.model.geom(name+'_geom').id
                    blades=set(s.monitor.geoms[name])-{main}
                    fingers={side:[g for g in range(s.model.ngeom) if
                        s.model.body(int(s.model.geom_bodyid[g])).name==receiver+'_'+side+'_finger'
                        and (s.model.geom_contype[g] or s.model.geom_conaffinity[g])]
                        for side in ('left','right')}
                    def gaps(width):
                        _scratch_gap(s,receiver,width);mujoco.mj_forward(s.model,s.data)
                        return [min(float(mujoco.mj_geomDistance(s.model,s.data,g,main,.08,None))
                                    for g in fingers[side]) for side in ('left','right')]
                    low,high=.020,.072
                    for _ in range(16):
                        width=(low+high)/2
                        if max(gaps(width))>.0001:high=width
                        else:low=width
                    width=(low+high)/2;distances=gaps(width)
                    if min(distances)<-.0015 or max(distances)>.0003:continue
                    for gap in np.linspace(.072,width,7):
                        _scratch_gap(s,receiver,gap);mujoco.mj_forward(s.model,s.data)
                        if not s.pose_clear(receiver,q,(name,)):okay=False;break
                        if any(mujoco.mj_geomDistance(s.model,s.data,g,b,.003,None)<.001
                               for group in fingers.values() for g in group for b in blades):
                            okay=False;break
                    if not okay:continue
                    _scratch_gap(s,receiver,width);mujoco.mj_forward(s.model,s.data)
                    da=s.arms[donor];p=s.data.site_xpos[da.site].copy()
                    r=s.data.site_xmat[da.site].reshape(3,3).copy()
                    direction=_single_finger_axis(s,donor,name)
                    if direction is None or not _supported_separation_clear(s,donor,name,p+direction*.005,r):continue
                    dq=s.screen_ik(donor,p+direction*.005,r,seed=s.data.qpos[da.qids])
                    if dq is None:continue
                    s.data.qpos[da.qids]=dq;mujoco.mj_forward(s.model,s.data)
                    for f in np.linspace(0,1,21)[1:]:
                        dq=s.screen_ik(donor,p+direction*.005+[0,0,.10*f],r,seed=dq)
                        if dq is None or not s.pose_clear(donor,dq):okay=False;break
                    if not okay:continue
                    s.data.qpos[da.qids]=dq;mujoco.mj_forward(s.model,s.data)
                    for gap in np.linspace(width,.080,13):
                        _scratch_gap(s,receiver,gap);mujoco.mj_forward(s.model,s.data)
                        if not s.pose_clear(receiver,q,(name,)):okay=False;break
                    if not okay or not s.pose_clear(receiver,q):continue
                    return dict(receiver=receiver,point_m=point.tolist(),tool=tool.tolist(),
                        approach_m=approach.tolist(),approach_q=approach_q.tolist(),
                        expected_width_m=width,source_center_m=center.tolist(),
                        radial_m=radial,height_m=height)
    return None


def _supported_release_assist(trial, donor, name, objects):
    """Opt-in real receiver stabilization, release and bounded clear retreat.

    Target contact is intentional; other-object/arm contacts and lost accepted
    goals stop execution. Passive motion of distant supplies is recorded, not
    redefined as task failure by a velocity predictor.
    """
    arm=trial.arms[donor]
    if (trial.task['level']!=2 or objects[name].get('shape')!='cylinder'
            or arm.job is not None or arm.grasp_force is not None
            or arm.observe()['gripper_width_m']<arm.grip_max-.002
            or not _table_supported_cylinder(trial,name)
            or _single_finger_axis(trial,donor,name) is None):return False
    plan=_plan_supported_side_assist(trial,donor,name)
    if plan is None:return False
    receiver=plan['receiver'];ra=trial.arms[receiver]
    payload=set(trial.monitor.geoms[name]);before_goals=set(trial.monitor.result().get('assignments',{}))
    initial=trial.data.body(name).xpos.copy()
    others={o['id']:trial.data.body(o['id']).xpos.copy() for o in trial.env['objects'] if o['id']!=name}
    non_target={g for obj in others for g in trial.monitor.geoms[obj]}
    active={list(trial.arms).index(donor),list(trial.arms).index(receiver)}
    previous_step=trial.step;stopped=[];evidence={};success=False
    def guarded_step():
        previous_step()
        if trial.tick%trial.stride or stopped:return
        reasons=[]
        if (not _table_supported_cylinder(trial,name) and
                not (ra.grasp_force is not None and receiver in trial.monitor.bilateral(payload,positive_force=True))):
            reasons.append('target support absent')
        if np.linalg.norm(trial.data.body(name).xpos-initial)>.010:reasons.append('target motion envelope exceeded')
        if before_goals-set(trial.monitor.result().get('assignments',{})):reasons.append('accepted goal disturbed')
        for c in trial.data.contact:
            if c.dist>.0002:continue
            a,b=map(int,c.geom);oa,ob=map(int,trial.monitor.geom_owner[[a,b]])
            if ((a in non_target and (ob in active or b in payload)) or
                    (b in non_target and (oa in active or a in payload))):
                reasons.append('active manipulation contacted another item');break
            for robot_geom,other,owner in ((a,b,oa),(b,a,ob)):
                if owner not in active or int(trial.monitor.geom_owner[other])>=0:continue
                body=trial.model.body(int(trial.model.geom_bodyid[robot_geom])).name
                if other in payload and body.endswith(('_left_finger','_right_finger')):continue
                if body.endswith('_link0'):continue
                reasons.append('unexpected arm/static or palm/payload contact');break
            if reasons:break
        if reasons:
            stopped.extend(reasons)
            for who in (donor,receiver):
                if trial.arms[who].job is not None:trial.arms[who].request_stop(trial.time,'Supported assist guard: '+','.join(reasons))
    def path_clear(robot,target,rotation,allowed=()):
        s=_planning_copy(trial);s.held.pop(robot,None);a=s.arms[robot]
        origin=s.data.site_xpos[a.site].copy();q=s.data.qpos[a.qids].copy()
        for f in np.linspace(0,1,max(3,math.ceil(np.linalg.norm(target-origin)/.005))+1)[1:]:
            q=s.screen_ik(robot,origin+(target-origin)*f,rotation,seed=q)
            if q is None or not s.pose_clear(robot,q,allowed):return False
        return True
    trial.step=guarded_step
    try:
        point=np.asarray(plan['point_m']);tool=np.asarray(plan['tool']);approach=np.asarray(plan['approach_m'])
        trial.execute('assist-receiver-open',{receiver:('gripper',dict(mode='move',width_m=.072,speed_m_s=.05))})
        trial.move_free(receiver,approach,tool,'assist-approach',preferred_seed=plan['approach_q'],
                        continuation=(point,tool,(name,)),continuation_clearance=.003,route_joint_step_rad=.005)
        point+=trial.data.body(name).xpos-np.asarray(plan['source_center_m'])
        if not path_clear(receiver,point,tool,(name,)):return False
        trial.move(receiver,point,tool,'assist-contact',precise=True,speed_m_s=.01)
        trial.execute('assist-receiver-grasp',{receiver:('gripper',dict(mode='grasp',
            width_m=plan['expected_width_m'],force_n=30.,speed_m_s=.02,epsilon_inner_m=.004,epsilon_outer_m=.008))})
        trial.wait(.2)
        if stopped or not trial.finger_contacts(receiver,name) or receiver not in trial.monitor.bilateral(payload,positive_force=True):return False
        trial.held[receiver]=name
        start=trial.data.site_xpos[arm.site].copy();rotation=trial.data.site_xmat[arm.site].reshape(3,3).copy()
        if _robot_payload_contacts(trial,donor,name):
            direction=_single_finger_axis(trial,donor,name,supporting_robots=(receiver,))
            if direction is None:return False
            for distance in (.002,.005):
                target=start+direction*distance
                if not _supported_separation_clear(trial,donor,name,target,rotation):return False
                trial.move(donor,target,rotation,'assist-donor-departure',precise=True,speed_m_s=.005)
                if stopped:return False
                if not _robot_payload_contacts(trial,donor,name):break
        if _robot_payload_contacts(trial,donor,name):return False
        target=trial.data.site_xpos[arm.site].copy()+[0,0,.10]
        if not path_clear(donor,target,rotation):return False
        trial.move(donor,target,rotation,'assist-donor-withdraw',speed_m_s=.01)
        if stopped or not _table_supported_cylinder(trial,name):return False
        opened=trial.execute('assist-receiver-release',{receiver:('gripper',dict(mode='move',width_m=.08,speed_m_s=.05))},allow_failure=True)[receiver]
        known=(opened['status']=='completed' or (opened['status']=='failed' and
               opened.get('reason') in ('Requested opening not reached','Requested aperture exceeded')))
        if not known or ra.grasp_force is not None or not trial.monitor.released(payload):return False
        trial.held.pop(receiver,None)
        # A60mm exit is inside the actual61mm safely completed portion of the
        # canonical diagnostic witness;100mm is not a task requirement.
        target=trial.data.site_xpos[ra.site].copy()-tool[:,2]*.06
        if not path_clear(receiver,target,tool):return False
        trial.move(receiver,target,tool,'assist-receiver-withdraw',speed_m_s=.01)
        if stopped:return False
        window={obj:trial.data.body(obj).xpos.copy() for obj in others}
        evidence=_released_buffer_evidence(trial,donor,name,window)
        success=bool(evidence.get('allowed') and before_goals<=set(trial.monitor.result().get('assignments',{})))
        return success
    finally:
        trial.step=previous_step
        trial.event('reference_supported_receiver_assist',donor=donor,receiver=receiver,object=name,
            plan=plan,completed=success,guard_reasons=stopped,buffer_evidence=evidence,
            other_object_displacements_m={obj:float(np.linalg.norm(trial.data.body(obj).xpos-p)) for obj,p in others.items()},
            scope='Explicit L2-cylinder reference candidate; real target support/release and original goal scoring retained')


def solve(trial, *, assisted_release=False, forward_recovery=True, goal_ids=None,
          on_frontier=None, yield_on_deferral=False):
    """Run the reference; optionally hand a live blocked frontier to its owner.

    on_frontier(trial, frontier, resume) is an adapter seam, not a runtime.
    It receives the same live world and may call resume after a verified
    compensation. resume(retry_failed=True) explicitly requeues exhausted
    jobs without deleting attempt counts/history. Its caller owns waiting, recovery IDs/version provenance,
    assisted/llm_used reporting and final Trial closure. No physics checkpoint
    or per-arm rollback is created here.
    yield_on_deferral opts into one early owner handover after the first
    known-safe deferral, before the independent queue is drained. It requires
    the explicit callback and forward scheduler; ordinary runs are unchanged.
    """
    if yield_on_deferral and (on_frontier is None or not forward_recovery):
        raise ValueError('Early deferral handover requires a forward frontier handler')
    validation = validate_inputs(trial.task, trial.env)
    trial.event('transport_reference_contract', **validation)
    objects = {obj['id']: obj for obj in trial.env['objects']}
    original_pick, original_execute, original_place = trial.pick, trial.execute, trial.place
    original_stop = getattr(trial, 'stop_requested', None)
    old_forward=getattr(trial,'transport_forward_recovery',None)
    old_goals=getattr(trial,'transport_goal_ids',None)
    old_yield=getattr(trial,'transport_yield_on_deferral',None)
    trial.transport_forward_recovery=bool(forward_recovery)
    trial.transport_goal_ids=None if goal_ids is None else set(goal_ids)
    trial.transport_yield_on_deferral=bool(yield_on_deferral)
    trial.stop_requested = lambda: bool(trial.monitor.violations) or (original_stop is not None and original_stop())
    picking = {}

    def pick_impl(robot, name, **kwargs):
        if objects[name].get('shape') == 'cylinder' or trial.task['level'] > 1:
            kwargs['force'] = min(70., trial.cfg['allowed_grasp_force_n'][1])
        if trial.task['level'] > 1:
            rotation = trial.data.body(name).xmat.reshape(3, 3)
            if rotation[2, 2] < .8:
                # A high body grasp clears raised pins and leaves a second
                # gripper room to receive side-oriented plugs. This private
                # reference pose is still physically grasped and verified.
                kwargs['height'] = _pickup_height(trial.task, rotation, kwargs.get('height', .006), objects[name].get('shape', 'cube'))
                if objects[name].get('shape') == 'cylinder' and abs(rotation[2, 2]) < .8:
                    _pick_side_cylinder(trial, robot, name, kwargs['height'], kwargs['force'])
                else:
                    original_pick(robot, name, **kwargs)
                regrasp_held_plug(trial, robot, name)
        return original_pick(robot, name, **kwargs)

    def pick(robot, name, **kwargs):
        # Shared pick records held only after its first lift. Include the
        # pending payload once actual force grasp starts, so that lift uses
        # the same cylinder speed policy as all subsequent carrying moves.
        prior = picking.get(robot)
        picking[robot] = name
        try:
            return pick_impl(robot, name, **kwargs)
        finally:
            if prior is None:
                picking.pop(robot, None)
            else:
                picking[robot] = prior

    def execute(label, commands, allow_failure=False):
        carrying = {robot: name for robot, name in picking.items()
                    if trial.arms[robot].grasp_force is not None}
        carrying.update(trial.held)
        commands = cylinder_commands(commands, carrying, objects)
        if label.startswith(('regrasp-donor-transit', 'regrasp-send-clear')):
            for robot, (operation, parameters) in commands.items():
                if operation == 'move':
                    for key, maximum in (('joint_speed_rad_s', .15),
                                         ('speed_m_s', .025),
                                         ('angular_speed_rad_s', .25)):
                        if key in parameters:
                            parameters[key] = min(parameters[key], maximum)
        results = original_execute(label, commands, allow_failure=allow_failure)
        if label == 'place-relax':
            for robot in commands:
                name = trial.held.get(robot)
                if name is None:
                    continue
                arm, result = trial.arms[robot], results[robot]
                known = result['status'] == 'completed' or (result['status'] == 'failed' and result.get('reason') in (
                    'Requested opening not reached', 'Requested aperture exceeded'))
                if not known or arm.grasp_force is not None:
                    continue
                if trial.monitor.released(trial.monitor.geoms[name]):
                    continue
                retry = original_execute('reference-release-clear', {robot: ('gripper', dict(
                    mode='move', width_m=arm.grip_max, speed_m_s=.005))}, allow_failure=True)[robot]
                released = trial.monitor.released(trial.monitor.geoms[name])
                trial.event('reference_release_retry', robot=robot, object=name,
                            initial_result=result, retry_result=retry, physically_released=released)
                retry_known = retry['status']=='completed' or (retry['status']=='failed'
                    and retry.get('reason') in ('Requested opening not reached','Requested aperture exceeded'))
                if (not released and retry_known
                        and arm.grasp_force is None):
                    released = _supported_release_unload(trial, robot, name, objects, original_execute)
                    if not released:
                        released = _supported_release_separate(trial,robot,name,objects)
                    if not released and assisted_release:
                        released = _supported_release_assist(trial,robot,name,objects)
                if not retry_known or arm.grasp_force is not None or not released:
                    raise RuntimeError('Payload not physically released before withdrawal '+robot+'/'+name)
        return results

    def place(robot, position, object_rotation=None, precise=False, pose_provider=None):
        rotation = np.eye(3) if object_rotation is None else np.asarray(object_rotation)
        if not precise and pose_provider is None and rotation[2, 2] > .99:
            # Plain relay buffers have a real supporting plane. Seat the
            # body bottom, or the declared plug tips, before opening rather
            # than dropping the item from the shared buffer approach.
            point = np.asarray(position, dtype=float).copy()
            extension = trial.env['mechanism']['blade_size_m'][2] if trial.task['level'] > 1 else 0.
            point[2] = trial.task['table_height_m'] + trial.task['cube_size_m']/2 + extension
            trial.event('reference_supported_stage', robot=robot, object=trial.held[robot],
                        target_m=point.tolist(), scope='Actual supported tabletop seating before opening')
            return original_place(robot, point, object_rotation, precise=True)
        if trial.task['level'] <= 1 or not precise:
            return original_place(robot, position, object_rotation, precise=precise,
                                  pose_provider=pose_provider)
        boards = {goal['board'] for goal in trial.task['goals']}
        board = min(boards, key=lambda b: np.linalg.norm(
            trial.data.body(b).xpos[:2] - np.asarray(position)[:2]))
        name = trial.held[robot]
        trial.event('reference_socket_contact_phase', robot=robot, object=name,
                    board=board, scope='Real mating geometry in scratch; live physics and arm clearance unchanged')
        with socket_contact_clearance(trial, name, board):
            return original_place(robot, position, object_rotation, precise=precise,
                                  pose_provider=pose_provider)

    trial.pick, trial.execute, trial.place = pick, execute, place
    try:
        frontier=_baseline(trial)
        if on_frontier is not None and isinstance(frontier,dict) and frontier.get('needs_recovery'):
            if trial.monitor.violations or any(
                    getattr(arm,'job',None) is not None or getattr(arm,'uncertainty_reported',False)
                    for arm in trial.arms.values()):
                raise RuntimeError('External frontier handler cannot consume unresolved or prohibited execution')
            trial.event('reference_live_frontier_handoff',frontier=frontier,
                        handler=getattr(on_frontier,'__qualname__',type(on_frontier).__name__),
                        scope='Same live Trial; adapter must record recovery IDs, code versions and assisted mode')
            def resume(*,retry_failed=False):
                if retry_failed and getattr(trial,'transport_failed_jobs',None):
                    jobs=list(trial.transport_failed_jobs)
                    trial.transport_pending_jobs.extend(jobs)
                    trial.transport_failed_jobs.clear()
                    trial.event('reference_explicit_failed_job_retry',
                        goals=[g['goal_id'] for g,_,_ in jobs],
                        previous_attempt_counts=dict(trial.transport_job_attempts),
                        scope='Caller asserts verified new compensation; attempt history is retained')
                return _baseline(trial)
            return on_frontier(trial,frontier,resume)
        return frontier
    finally:
        trial.pick, trial.execute, trial.place = original_pick, original_execute, original_place
        trial.stop_requested = original_stop
        if old_forward is None:del trial.transport_forward_recovery
        else:trial.transport_forward_recovery=old_forward
        if old_goals is None:del trial.transport_goal_ids
        else:trial.transport_goal_ids=old_goals
        if old_yield is None:del trial.transport_yield_on_deferral
        else:trial.transport_yield_on_deferral=old_yield
