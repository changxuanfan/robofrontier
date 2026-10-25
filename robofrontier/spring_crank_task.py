"""Policy task for fixed spring dials with physical red crank grips."""
from copy import deepcopy
import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
from .scene_task import ROOT,validate_scene_asset
from .spring_dial_task import CODE_VALUES,CODE_SYMBOLS,SYMBOL_ANGLES,HINTS


def rules():
    return [
        'All four dials start at symbol 0. Each hidden code entry is sampled independently and uniformly from symbols 1, 2, 3; repetitions are allowed and zero is never a code entry.',
        'Read each symbol on the stationary white dial from above: 0 north, 1 east (90 degrees clockwise), 2 south (180), 3 west (270). The displayed digit is the red crank rotor angle relative to its own white dial, not the robot wrist angle.',
        'Each red hand grip is physically connected to a passive spring-return rotor. Hold and move the grip to turn the pointer through real robot/gripper contact. The white dial is fixed to the table. Releasing the crank lets its rotor return toward zero. No dial actuator, angle-setting command, attachment, reset or physics change is available to policy.',
        'Feedback requires all four rotors near nonzero marks and slow/stable for the declared duration. The display reveals only the total correct count, never which dials match. Invalid/transient inputs display a dash; a valid zero score displays 0.',
        'Each newly entered stable configuration counts once, including intermediate configurations formed while adjusting. Continued holding and repeated observation do not submit another trial; leaving and re-entering can. There is no separate submit button.',
        'The first valid score 4 wins and latches visibly; the passive springs remain active. Post-success cleanup does not erase success or create further trials.',
        'A completed TCP or gripper command does not prove that the crank moved by the requested angle or stayed grasped. Verify the relative pointer position and current robot/gripper evidence.',
        'Nominal robot/dial labels show the initial layout, not exclusive ownership. Choose robot allocation, grasp pose, turn direction, observation views and communication within current role permissions.',
    ]


def hints():
    return list(HINTS)+[
        'The red cylindrical grip is part of the rotating crank. Its position changes around the fixed dial center as the pointer turns; keep contact while following that motion, then check the visible pointer for slip.',
        'Compare reachable grasp and wrist configurations with the crank motion needed for the next trial. Regrasping every dial is not inherently required; retain a useful grasp when it preserves access and does not obstruct another robot.',
        'Opening a hand after a turn can require a clear withdrawal path around the crank arm and fixed dial. Check physical release before assuming the spring-return rotor is free.',
    ]


def make_crank_task(directory):
    directory=Path(directory).resolve();env=json.loads((directory/'environment.json').read_text());p=json.loads((directory/'parameters.json').read_text());d=p['dial'];c=p['crank'];table=p['table']
    task=dict(kind='spring_dial',id=env['id']+'-policy-v1',status='Crank-handle apparatus; Oracle manipulation is separate from GPT performance',
        scene_asset=str(directory.relative_to(ROOT)),scene_sha256=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest(),
        arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},objects=[],dials=deepcopy(env['dials']),table_height_m=table['height_m'],
        mechanism=dict(apparatus='crank',hub_diameter_m=c['hub_diameter_m'],hub_height_m=c['hub_height_m'],
            crank_radius_m=c['radius_m'],arm_width_m=c['arm_width_m'],arm_thickness_m=c['arm_thickness_m'],
            grip_diameter_m=c['grip_diameter_m'],grip_height_m=c['grip_height_m'],assembly_mass_kg=c['assembly_mass_kg'],
            rotation_range_deg=list(d['range_rad'] and [-180,180]),return_mechanism='passive spring'),
        physics=deepcopy(p['physics']),goal='Discover the hidden four-symbol code and simultaneously hold the corresponding four crank settings until the display reads 4.',
        objective='Succeed while reducing valid code trials, robot actions and elapsed time. Evaluate information gained against crank-grasp, turn, release and coordination cost.',
        coordinate_frame='World +X right, +Y north, +Z up. Dial symbols are read on each stationary white dial from above; the crank rotates relative to that dial.',
        allowed_code_angles_deg=list(CODE_VALUES),allowed_code_symbols=list(CODE_SYMBOLS),symbol_to_angle_deg=dict(SYMBOL_ANGLES),candidate_codes=81,
        rules=rules(),planning_hints=hints(),feedback=deepcopy(p['feedback']),completion=dict(stable_duration_sim_s=p['feedback']['stable_duration_s'],max_sample_gap_sim_s=p['feedback']['max_sample_gap_s'],success_latched=True,complete_sampling_required=True),
        workspace_world_m=dict(lower=[-table['length_m']/2,-table['width_m']/2,table['height_m']+.025],upper=[table['length_m']/2,table['width_m']/2,1.30]),
        cameras=[c.get('name') for c in ET.parse(directory/'scene.xml').findall('worldbody/camera')],view_camera='global',recording_camera='board',
        camera_roles=dict(overview='global',game_feedback='board',dial_details={a:'dial_'+a for a in 'ABCD'}),
        observation_contract=dict(initial_positions_public=False,initial_geometry_public=True,policy=['RGB including the score and pointer','robot_proprioception','action_status'],live_object_truth_for_policy=False,
            fixture_geometry='Declared device/grip dimensions and fixed robot/camera calibration are public. Infer each dial position and live pointer angle from permitted RGB.',motion_screen='Optional nominal robot-only geometry; does not certify crank contact, force, slip or release.'),
        metrics=['success','valid_trials','robot_action_count','absolute_dial_travel_rad','wall_time','simulation_time','holding_departures','zero_entries','sampling_gaps'])
    return validate_crank_task(task)


def validate_crank_task(task):
    if task.get('kind')!='spring_dial' or task.get('rules')!=rules() or task.get('planning_hints')!=hints():raise ValueError('Crank task contract differs from its canonical definition')
    folder,env=validate_scene_asset(task);p=json.loads((folder/'parameters.json').read_text());c=p.get('crank');d=p['dial']
    if not c or c.get('id')!='spring-dial-crank-04' or len(task['arms'])!=4 or task['objects'] or task['dials']!=env['dials']:raise ValueError('Crank apparatus IDs differ from scene')
    if task['physics']!=p['physics'] or task['feedback']!=p['feedback'] or task['table_height_m']!=p['table']['height_m']:raise ValueError('Crank physics/feedback mismatch')
    expected=dict(apparatus='crank',hub_diameter_m=c['hub_diameter_m'],hub_height_m=c['hub_height_m'],crank_radius_m=c['radius_m'],arm_width_m=c['arm_width_m'],arm_thickness_m=c['arm_thickness_m'],grip_diameter_m=c['grip_diameter_m'],grip_height_m=c['grip_height_m'],assembly_mass_kg=c['assembly_mass_kg'],rotation_range_deg=[-180,180],return_mechanism='passive spring')
    if task['mechanism']!=expected:raise ValueError('Public crank dimensions mismatch the generated scene')
    if task['rules']!=rules() or task['planning_hints']!=hints():raise ValueError('Task instructions drifted from the canonical crank protocol')
    if task['observation_contract'].get('live_object_truth_for_policy') is not False:raise ValueError('Live dial state must remain private')
    if task['completion']!=dict(stable_duration_sim_s=task['feedback']['stable_duration_s'],max_sample_gap_sim_s=task['feedback']['max_sample_gap_s'],success_latched=True,complete_sampling_required=True):raise ValueError('Crank completion contract mismatch')
    if task['allowed_code_angles_deg']!=list(CODE_VALUES) or task['allowed_code_symbols']!=list(CODE_SYMBOLS) or task['symbol_to_angle_deg']!=SYMBOL_ANGLES or task['candidate_codes']!=81:raise ValueError('Crank code alphabet mismatch')
    root=ET.parse(folder/'scene.xml').getroot()
    for a in 'ABCD':
        body=root.find(f"worldbody/body[@name='dial_{a}']");joint=None if body is None else body.find('joint')
        if joint is None or joint.get('axis')!='0 0 -1' or not math.isclose(float(joint.get('stiffness')),d['spring_nm_per_rad']) or not math.isclose(float(joint.get('damping')),d['damping_nm_s_per_rad']):raise ValueError('Crank rotor must preserve the passive spring hinge')
        if body.find(f"geom[@name='dial_{a}_hand_grip']") is None or body.find(f"geom[@name='dial_{a}_crank_arm']") is None:raise ValueError('Red graspable crank is missing')
        if any(e.get('joint')==joint.get('name') for e in root.findall('actuator/*')):raise ValueError('Crank dial cannot have a policy actuator')
        if any(g.get('name','').startswith(f'dial_{a}_tooth_') for g in body.findall('geom')):raise ValueError('Gear teeth remain in crank scene')
    return task


def public_crank_task(task):
    keys=('kind','id','status','arms','objects','dials','table_height_m','mechanism','goal','objective',
          'coordinate_frame','allowed_code_angles_deg','allowed_code_symbols','symbol_to_angle_deg','candidate_codes','rules','planning_hints',
          'feedback','completion','workspace_world_m','cameras','view_camera','recording_camera','camera_roles','observation_contract','metrics')
    result={k:deepcopy(task[k]) for k in keys}
    if not task['observation_contract']['initial_positions_public']:
        for dial in result['dials']:dial.pop('center_m',None)
        result['observation_contract']['fixture_geometry']='Device/grip dimensions and robot/camera calibration are public; identify each fixed dial center and visible relative pointer angle from permitted RGB.'
    return result
