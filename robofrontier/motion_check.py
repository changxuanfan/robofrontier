"""Read-only, sampled inter-arm screening; not a trajectory planner or safety gate.

Only robot joints and controller state enter scratch data. No live object pose,
contact, scene evaluation, dynamics integration or automatic action submission.
"""
import copy
import hashlib
import itertools
import json
import math

import mujoco
import numpy as np

from .sim import RobotIK


CONTROL_DT = .02
JOINT_SPEED_LIMIT = .9
JOINT_ACCEL_LIMIT = 2.
TRACKING_LIMIT = .1


def robot_state(world):
    """Whitelist robot/controller values; never serialize the whole MjData."""
    result = {}
    for name, arm in world.arms.items():
        result[name] = {
            'joint_rad':world.data.qpos[arm.qids].tolist(),
            'joint_velocity_rad_s':world.data.qvel[arm.dofs].tolist(),
            'finger_rad_or_m':[float(world.data.joint(name+'_'+s).qpos[0])
                               for s in ('finger_joint1','finger_joint2')],
            'qref':arm.qref.tolist(), 'reference_velocity':arm.velocity.tolist(),
            'tcp_ref':None if arm.tcp_ref is None else arm.tcp_ref.tolist(),
            'integral':arm.cartesian_integral.tolist(),
            'goal':None if arm.goal is None else arm.goal.tolist(),
            'speed_m_s':arm.speed, 'gripper_reference':arm.gripper,
            'gripper_goal':arm.grip_goal, 'call_id':arm.job,
            'action_type':getattr(arm,'job_op',None),
            'action_started_sim_s':getattr(arm,'job_start',None),
            'arrival_stable_ticks':getattr(arm,'stable',None),
            'rotation_reference':arm.rotation.tolist(),
            'rotation_goal':None if arm.rotation_goal is None else arm.rotation_goal.tolist(),
            'joint_goal':None if arm.joint_goal is None else arm.joint_goal.tolist(),
            'angular_speed_rad_s':arm.angular_speed,'joint_speed_rad_s':arm.joint_speed,
            'gripper_speed_m_s':arm.grip_speed,'stopping':arm.stopping,
            'grasp_force_setpoint_n':arm.grasp_force,'grasp_width_m':arm.grasp_width,
            'grasp_epsilon_m':arm.grasp_epsilon,'grasp_confirmed':arm.grasp_confirmed,
        }
    return result


def state_id(world):
    return hashlib.sha256(json.dumps(robot_state(world),sort_keys=True,allow_nan=False).encode()).hexdigest()


class RobotGeometry:
    def __init__(self, model, arms):
        self.model = model
        self.bodies, self.geoms = {}, {}
        for name in arms:
            base = model.body(name+'_link0').id
            bodies = {base}
            for b in range(base+1, model.nbody):
                if int(model.body_parentid[b]) in bodies:
                    bodies.add(b)
            self.bodies[name] = sorted(bodies)
            self.geoms[name] = [g for g in range(model.ngeom)
                                if int(model.geom_bodyid[g]) in bodies
                                and (model.geom_contype[g] or model.geom_conaffinity[g])]
        if any(not g for g in self.geoms.values()):
            raise ValueError('Missing robot collision geometry')
        self.pairs = np.array([(g,h) for a,b in itertools.combinations(self.geoms,2)
                               for g,h in itertools.product(self.geoms[a],self.geoms[b])],dtype=int).reshape(-1,2)

    def label(self, g):
        return self.model.body(int(self.model.geom_bodyid[g])).name+':geom'+str(g)

    def describe(self, world, cfg):
        m = self.model
        robots = {}
        for name, arm in world.arms.items():
            bodies = []
            for b in self.bodies[name]:
                joints = []
                for j in range(int(m.body_jntadr[b]),int(m.body_jntadr[b]+m.body_jntnum[b])):
                    joints.append({'name':m.joint(j).name,'type':int(m.jnt_type[j]),
                                   'position_local_m':m.jnt_pos[j].tolist(),
                                   'axis_local':m.jnt_axis[j].tolist(),'range':m.jnt_range[j].tolist()})
                bodies.append({'name':m.body(b).name,'parent':m.body(int(m.body_parentid[b])).name,
                               'position_parent_m':m.body_pos[b].tolist(),
                               'quaternion_wxyz':m.body_quat[b].tolist(),'joints':joints})
            geoms = []
            for g in self.geoms[name]:
                mesh_id = int(m.geom_dataid[g])
                mesh_hash = None
                if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH):
                    start = int(m.mesh_vertadr[mesh_id]); count = int(m.mesh_vertnum[mesh_id])
                    mesh_hash = hashlib.sha256(m.mesh_vert[start:start+count].tobytes()).hexdigest()
                geoms.append({'label':self.label(g),'body':m.body(int(m.geom_bodyid[g])).name,
                              'type':int(m.geom_type[g]),'size':m.geom_size[g].tolist(),
                              'position_body_m':m.geom_pos[g].tolist(),
                              'quaternion_wxyz':m.geom_quat[g].tolist(),
                              'bounding_radius_m':float(m.geom_rbound[g]),'mesh_vertices_sha256':mesh_hash})
            robots[name] = {'bodies':bodies,'collision_geoms':geoms,
                            'ik_joint_limits_rad':arm.ik.limits.tolist(),
                            'tcp':{'body':m.body(int(m.site_bodyid[arm.site])).name,
                                   'position_body_m':m.site_pos[arm.site].tolist(),
                                   'home_reference_rotation_world':arm.home_rotation.tolist(),
                                   'orientation_control':'world-frame quaternion_wxyz; robot-only IK'}}
        result = {'robots':robots,'controller':{'tick_s':CONTROL_DT,
                    'joint_speed_limit_rad_s':JOINT_SPEED_LIMIT,'joint_acceleration_limit_rad_s2':JOINT_ACCEL_LIMIT,
                    'tracking_limit_rad':TRACKING_LIMIT,'tcp_speed_range_m_s':cfg['allowed_tcp_speed_m_s']},
                'motion_check_limits':cfg['motion_check'],
                'geometry_note':'Mesh bounds describe geometry; the checker uses internal convex collision meshes. Robot-only; no scene/object state.'}
        result['model_id'] = hashlib.sha256(json.dumps(result,sort_keys=True,allow_nan=False).encode()).hexdigest()
        return result

    def distance(self, data, probe):
        """All cross-robot collision geoms, independent of contact exclusions."""
        closest = {'distance_m':probe,'distance_capped':True,'pair':None}
        g,h = self.pairs.T
        if not np.isfinite(data.geom_xpos[np.r_[g,h]]).all() or not np.isfinite(data.geom_xmat[np.r_[g,h]]).all():
            raise ValueError('Non-finite robot geometry pose')
        lower = np.linalg.norm(data.geom_xpos[g]-data.geom_xpos[h],axis=1)-self.model.geom_rbound[g]-self.model.geom_rbound[h]
        for g,h in self.pairs[lower<probe]:
            d = float(mujoco.mj_geomDistance(self.model,data,int(g),int(h),probe,None))
            if not math.isfinite(d):
                raise ValueError('Non-finite geometry distance')
            if d < closest['distance_m']:
                actors = [n for n,gs in self.geoms.items() if g in gs or h in gs]
                closest = {'distance_m':d,'distance_capped':False,
                           'pair':{'actors':actors,'geoms':[self.label(g),self.label(h)]}}
        return closest


def prepare_check(world, parameters, cfg):
    """Capture on the owner thread, then yield one sample's work per pump.

    Perfect servo tracking follows cloned Arm.control/RobotIK references. It is
    intentionally not a dynamics rollout and cannot certify actual clearance.
    """
    if set(parameters)-{'motions','horizon_s','sample_dt_s'}:
        raise ValueError('check_motion accepts motions, horizon_s, sample_dt_s')
    limits = cfg['motion_check']
    horizon = float(parameters['horizon_s'])
    sample_dt = float(parameters.get('sample_dt_s',limits['default_sample_dt_s']))
    if not math.isfinite(horizon) or not 0 < horizon <= limits['max_horizon_s']:
        raise ValueError('Motion check horizon outside limits')
    if abs(round(horizon/CONTROL_DT)*CONTROL_DT-horizon)>1e-8:
        raise ValueError('Horizon must be a multiple of controller tick')
    ticks = round(sample_dt/CONTROL_DT) if math.isfinite(sample_dt) else 0
    if ticks < 1 or abs(ticks*CONTROL_DT-sample_dt)>1e-8 or sample_dt>limits['max_sample_dt_s']:
        raise ValueError('Sample interval must be a supported multiple of controller tick')
    motions = parameters['motions']
    if not isinstance(motions,list) or not motions or len(motions)>len(world.arms):
        raise ValueError('Supply one candidate per selected arm')
    geometry = RobotGeometry(world.model,world.arms)
    allowed = {mujoco.mjtGeom.mjGEOM_SPHERE,mujoco.mjtGeom.mjGEOM_CAPSULE,
               mujoco.mjtGeom.mjGEOM_ELLIPSOID,mujoco.mjtGeom.mjGEOM_CYLINDER,
               mujoco.mjtGeom.mjGEOM_BOX,mujoco.mjtGeom.mjGEOM_MESH}
    report = {'assessment':'unknown','source_sim_time_s':float(world.time),
              'source_model_id':geometry.describe(world,cfg)['model_id'],
              'source_state_id':state_id(world),
              'source_robot':{name:{k:v for k,v in state.items() if k in
                  ('joint_rad','joint_velocity_rad_s','finger_rad_or_m','goal','speed_m_s','call_id')}
                  for name,state in robot_state(world).items()},
              'motions':copy.deepcopy(motions),'horizon_s':horizon,'sample_dt_s':sample_dt,
              'nominal_start_after_s':{},
              'clearance_threshold_m':limits['clearance_m'],'samples_checked':0,
              'coverage':'Sampled nominal cross-robot collision geometry, including idle robots; perfect joint-servo tracking, fixed finger positions.',
              'not_checked':['between-sample motion','dynamics/tracking error','self collision',
                             'table/obstacles','payloads and object poses','future unsubmitted commands'],
              'advisory_only':True}
    # A fresh data object defaults non-robot coordinates; no live free joint or
    # contact state is copied. Only explicitly named robot coordinates enter.
    preview = copy.copy(world)
    preview.model = copy.copy(world.model)
    preview.data = mujoco.MjData(preview.model)
    preview.data.time = world.time
    preview.arms = {}
    for name,arm in world.arms.items():
        clone = copy.copy(arm)
        clone.model = preview.model
        clone.data = preview.data
        clone.ik = RobotIK(preview.model,name+'_')
        clone.grasp_settling=copy.deepcopy(arm.grasp_settling)
        for attr in ('qref','velocity','tcp_ref','cartesian_integral','goal','rotation','rotation_goal','joint_goal'):
            value = getattr(arm,attr)
            setattr(clone,attr,None if value is None else value.copy())
        preview.data.qpos[arm.qids] = world.data.qpos[arm.qids]
        preview.data.qvel[arm.dofs] = world.data.qvel[arm.dofs]
        for suffix in ('finger_joint1','finger_joint2'):
            preview.data.joint(name+'_'+suffix).qpos[:] = world.data.joint(name+'_'+suffix).qpos
        preview.arms[name] = clone
    mujoco.mj_kinematics(world.model,preview.data)
    scheduled = {}
    for motion in motions:
        if not isinstance(motion,dict) or set(motion)-{'actor','target_m','delta_m','speed_m_s','start_after_s','quaternion_wxyz','angular_speed_rad_s','joint_rad','joint_speed_rad_s'}:
            raise ValueError('Candidate is an actor, move parameters, optional start_after_s')
        name = motion['actor']
        if name not in preview.arms or name in scheduled:
            raise ValueError('Unknown/duplicate candidate arm')
        delay = float(motion.get('start_after_s',0))
        if not math.isfinite(delay) or not 0 <= delay < horizon:
            raise ValueError('Candidate start_after_s outside horizon')
        command = preview.validate_action(name,'move',{k:v for k,v in motion.items() if k not in ('actor','start_after_s')},cfg)
        scheduled[name] = (delay,command)
    reason = None
    if len(geometry.geoms)<2:
        reason = 'Need at least two robots for inter-arm screening'
    elif any(int(world.model.geom_type[g]) not in allowed for gs in geometry.geoms.values() for g in gs):
        reason = 'Unsupported collision geometry'
    elif int(world.model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD):
        reason = 'Native convex collision distance pipeline is disabled'
    elif any(a.job and a.job_op=='gripper' for a in world.arms.values()):
        reason = 'An active gripper changes geometry; fixed-finger prediction unavailable'

    def samples():
        if reason:
            yield {**report,'reason':reason}; return
        best = None
        steps = math.ceil(horizon/CONTROL_DT)
        for tick in range(steps+1):
            t = min(tick*CONTROL_DT,horizon)
            for name,(delay,command) in list(scheduled.items()):
                if t+1e-9 >= delay:
                    preview.start_action(name,'preview-'+name,command)
                    report['nominal_start_after_s'][name] = t
                    del scheduled[name]
            if tick % ticks == 0 or tick == steps:
                current = geometry.distance(preview.data,max(.1,limits['clearance_m']+.05))
                report['samples_checked'] += 1
                if best is None or current['distance_m']<best['distance_m']:
                    best = {**current,'time_after_snapshot_s':t}
                if current['distance_m']<limits['clearance_m']:
                    yield {**report,'assessment':'conflict','closest':best}; return
                if tick == steps:
                    unresolved = [n for n,a in preview.arms.items() if a.job is not None]
                    yield {**report,'assessment':'no_conflict_found','closest':best,
                           'unfinished_at_horizon':unresolved}; return
                yield None
            # Reuse the actual controller at its own 20 ms tick, but follow its
            # references exactly instead of integrating a physical scene.
            dt = min(CONTROL_DT,horizon-t)
            for arm in preview.arms.values():
                arm.control(dt)
                preview.data.qpos[arm.qids] = arm.qref
                preview.data.qvel[arm.dofs] = arm.velocity
            preview.data.time += dt
            mujoco.mj_kinematics(world.model,preview.data)
            failures = preview.poll_actions(cfg['action_timeout_sim_s'])
            if any(r['status']!='completed' for r in failures):
                yield {**report,'reason':'Nominal controller failed to reach a target','closest':best}; return
    return samples()
