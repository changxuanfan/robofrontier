"""One task definition feeds both the scene and public task instructions."""
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import hashlib

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TASK = ROOT / 'configs/tasks/three-arm-v1.json'
DEFAULT_HARNESS = ROOT / 'configs/harness.json'



ACTION_DEFAULTS = {
    'allowed_angular_speed_rad_s': [.05, 1.5], 'default_angular_speed_rad_s': .6,
    'allowed_gripper_speed_m_s': [.005, .10], 'default_gripper_speed_m_s': .08,
    'allowed_grasp_force_n': [30., 70.],
}


def execution_timing(config):
    """One timing contract for validation, READY and the physical executor."""
    return {'base_timeout_sim_s':config['action_timeout_sim_s'],
            'max_timeout_sim_s':config.get('max_action_timeout_sim_s',600.),
            'reference_time_factor':1.5,
            'scope':'base + factor * estimated reference duration; bounded execution budget, not a success prediction'}


def action_limits(config):
    return {key: config.get(key, value) for key, value in ACTION_DEFAULTS.items()}

def physical_control_fingerprint(source_hashes,config):
    """Compare shared execution, independently of policy model/topology."""
    names=('robot_control.py','sim.py','motion_check.py','task_config.py','pipeline.py')
    code={name:source_hashes['robofrontier/'+name] for name in names}
    value=dict(code=code,gripper_position_servo=config.get('gripper_position_servo','legacy'),action_limits=action_limits(config),timing=execution_timing(config),allowed_tcp_speed_m_s=config['allowed_tcp_speed_m_s'],default_tcp_speed_m_s=config['default_tcp_speed_m_s'])
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def panda_control_profile(config):
    """Declared current adapter subset; not a complete real-hardware driver."""
    from .robot_control import hand_position_profile
    return {'hand_position_servo':hand_position_profile(config.get('gripper_position_servo','legacy')),
            'supported_arm_modes':['cartesian_pose_target','joint_position_target'],
            'supported_hand_modes':['aperture_move','force_setpoint_grasp'],
            'arm_parameters':['target_m','delta_m','quaternion_wxyz','speed_m_s','angular_speed_rad_s','joint_rad','joint_speed_rad_s'],
            'hand_parameters':['state','width_m','speed_m_s','force_n','epsilon_inner_m','epsilon_outer_m'],
            'not_supported':['independent_joint_or_cartesian_velocity_mode','joint_torque_or_arm_wrench_control',
                'agent_configurable_impedance','gripper_homing','agent_tcp_or_load_model_calibration',
                'agent_collision_threshold_configuration','measured_contact_forces_or_external_wrench_feedback'],
            'limits_origin':'Frozen benchmark software limits within a declared Panda/Hand profile; not the complete hardware operating envelope.',
            'hardware_adapter_validated':False,'grasp_force_is_measured':False,
            'simulation_parameters_mutable_by_policy':False,
            'software_tools':'IK, motion screening and synchronized submission are software capabilities, not extra physical actuators.'}


def load_task(path=None):
    task = json.loads(Path(path or DEFAULT_TASK).read_text())
    if task.get('scene_asset'):
        if task.get('kind')=='frontier_suite':
            from .suite.tasks import validate
            return validate(task)
        if task.get('kind')=='spring_dial':
            from .spring_dial_task import validate_spring_task
            return validate_spring_task(task)
        if task.get('kind')=='beam_handoff':
            from .beam_task import validate_beam_task
            return validate_beam_task(task)
        from .scene_task import validate_scene_task
        return validate_scene_task(task)
    for key in ('table_height_m','cube_size_m','cube_mass_kg'):
        if not math.isfinite(float(task[key])) or task[key]<=0:raise ValueError('Invalid physical task parameter: '+key)
    if len(task['contact_friction'])!=3 or any(not math.isfinite(float(x)) or x<0 for x in task['contact_friction']):raise ValueError('Invalid friction')
    if not isinstance(task['noslip_iterations'],int) or task['noslip_iterations']<0:raise ValueError('Invalid NoSlip count')
    arms = task['arms']
    if not arms or any(not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', a) for a in arms):
        raise ValueError('Arm identifiers must be simple names')
    if {'Meta', 'Coordinator', 'Controller', 'research'} & set(arms):
        raise ValueError('Arm name conflicts with reserved role')
    if not all(math.isfinite(float(y)) for y in arms.values()):
        raise ValueError('Non-finite base position')
    if set(task['goals']) != set(arms):
        raise ValueError('Each arm must have goals')
    names = set()
    for obj in task['objects']:
        if obj['id'] in names or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', obj['id']):
            raise ValueError('Duplicate/invalid object id')
        names.add(obj['id'])
        if obj['region'] not in arms or obj['color'] not in ('gray','blue','red','green') or obj['pattern'] not in ('plain','dots','checker'):
            raise ValueError('Unsupported object attributes')
        check_position(obj['initial_position_m'])
    for goals in task['goals'].values():
        slots = set()
        for goal in goals:
            if goal['slot'] in slots:
                raise ValueError('Duplicate slot within arm')
            slots.add(goal['slot'])
            check_position(goal['position_m'])
            if not goal['requires'] or set(goal['requires']) - {'color','pattern','region'}:
                raise ValueError('Goal requires color/pattern/source region predicates')
    # Each goal needs a different available object. This checks inventory only,
    # not transport feasibility, geometry or a particular execution order.
    matched = {}
    goals = [g for values in task['goals'].values() for g in values]
    def assign(index, seen):
        for obj in task['objects']:
            name = obj['id']
            if name in seen or not all(obj.get(k)==v for k,v in goals[index]['requires'].items()):
                continue
            seen.add(name)
            if name not in matched or assign(matched[name], seen):
                matched[name] = index
                return True
        return False
    if not all(assign(i, set()) for i in range(len(goals))):
        raise ValueError('Goal inventory cannot be fulfilled by distinct objects')
    return task


def check_position(value):
    if len(value) != 3 or not all(math.isfinite(float(x)) for x in value):
        raise ValueError('Finite xyz required')


def public_task(task):
    if task.get('kind')=='frontier_suite':
        from .suite.tasks import public_task as project
        return project(task)
    if task.get('kind')=='spring_dial':
        from .spring_dial_task import public_spring_task
        return public_spring_task(task)
    if task.get('kind')=='beam_handoff':
        from .beam_task import public_beam_task
        return public_beam_task(task)
    result = deepcopy(task)
    result.pop('scene_asset', None)
    result.pop('scene_sha256', None)
    if not result['observation_contract']['initial_positions_public']:
        for obj in result['objects']:
            obj.pop('initial_position_m', None)
    return result


def load_harness(path=None):
    c = json.loads(Path(path or DEFAULT_HARNESS).read_text())
    if c.get('gripper_position_servo','legacy') not in ('legacy','pd1000-v1'):raise ValueError('Unknown hand position-servo profile')
    if c['manager'] != 'Meta' or c['coordinator'] != 'Coordinator':
        raise ValueError('Use explicit Meta / Coordinator role boundary')
    if c['topology'] not in ('hierarchical','peer','centralized','adaptive'):
        raise ValueError('Unknown topology')
    if c.get('engine','codex') not in ('codex','claude'):
        raise ValueError('Unknown policy engine')
    for key in ('poll_interval_s','integrity_check_interval_s','session_observation_ttl_s','manager_lease_s','max_batch_messages','max_batch_bytes','max_message_chars','min_check_interval_s','max_check_interval_s','action_timeout_sim_s','video_fps','max_unacknowledged_wakes'):
        if not math.isfinite(float(c[key])) or c[key] <= 0:
            raise ValueError('Invalid harness limit: ' + key)
    if c['min_check_interval_s'] > c['max_check_interval_s']:
        raise ValueError('Invalid check interval range')
    maximum=float(c.get('max_action_timeout_sim_s',600.))
    if not math.isfinite(maximum) or maximum<c['action_timeout_sim_s']:
        raise ValueError('Maximum action timeout must be finite and at least the base timeout')
    c['max_action_timeout_sim_s']=maximum
    setup_timeout=float(c.get('native_setup_timeout_s',120))
    if not math.isfinite(setup_timeout) or setup_timeout<=0:
        raise ValueError('Invalid native setup timeout')
    if 'native_setup_timeout_s' in c:c['native_setup_timeout_s']=setup_timeout
    screening = c['motion_check']
    for key in ('max_horizon_s','default_sample_dt_s','max_sample_dt_s','clearance_m'):
        if not math.isfinite(float(screening[key])) or screening[key]<=0:
            raise ValueError('Invalid motion check limit: '+key)
    if not .02 <= screening['default_sample_dt_s'] <= screening['max_sample_dt_s'] <= .2:
        raise ValueError('Invalid motion check sample range')
    if screening['max_horizon_s']>15:
        raise ValueError('Motion check computational horizon exceeds 15 s')
    for key in ('default_sample_dt_s','max_horizon_s'):
        if abs(round(screening[key]/.02)*.02-screening[key])>1e-8:
            raise ValueError('Motion check '+key+' must be a multiple of 20 ms')
    c.update(action_limits(c))
    for kind in ('angular_speed_rad_s','gripper_speed_m_s'):
        bounds=c['allowed_'+kind];default=c['default_'+kind]
        if len(bounds)!=2 or not all(math.isfinite(float(v)) and v>0 for v in bounds) or not bounds[0]<=default<=bounds[1]:
            raise ValueError('Invalid '+kind+' limits')
    force=c['allowed_grasp_force_n']
    if len(force)!=2 or not all(math.isfinite(float(v)) for v in force) or not 30<=force[0]<=force[1]<=70:
        raise ValueError('Grasp force range must stay within the configured Franka Hand 30..70 N profile')
    if c['allowed_gripper_speed_m_s'][1]>.10:
        raise ValueError('Total aperture reference speed exceeds the Franka Hand profile')
    return c
