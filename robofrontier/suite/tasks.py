"""Frozen suite tasks and an explicit public projection; no live state tools."""
from copy import deepcopy
from collections import Counter
import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
from ..scene_task import ROOT,validate_scene_asset

PROCESS_CONTRACT=dict(collision_sampling='every physics step',inter_robot_penetration_limit_m=.0002,self_robot_penetration_limit_m=.001,
    expected_finger_to_finger_closure_excluded=True,relay_receiving_overlap_sim_s=.20,relay_support_gap_sim_s=.20,
    grasp_proximity_tolerance_m=.0002,relay_positive_contact_force_threshold_n=.02,relay_support_proxy='Opposed jaw proximity within 0.2 mm and at least one actual force-bearing finger contact per gripper; not an individual load-share measurement',
    driven_red_support_gap_sim_s=.05)

RULES={
 'transport':[
  'Deliver three requested source-identity items to each robot station; goal numbers do not prescribe execution order. Repeated colors have visible identity marks.',
  'Table-supported buffers are permitted. Each robot has one gripper; respect actual reach and shared-space occupancy.',
  'For socket versions both physical pegs must enter the requested pair of holes. The compliant board changes pose with load; inspect the actual board rather than treating its initial plane as permanent.'
 ],
 'mosaic':[
  'Assemble every pixel of the reference, including white cells, with one released stable cube per cell. Supply color does not assign permanent ownership.',
  'The reference is north-up in the common world frame, centered at the declared assembly origin, with 51 mm cell-center spacing. The three versions share horizontal centers.',
  'Pockets require physical seating. Printed and unmarked versions have the same target pattern. Previously correct cells can be disturbed by later contact.'
 ],
 'relay':[
  'Follow the declared route through all stations. At least two robots must physically grasp and support the wooden beam during airborne transport; gripping only its hanging weight or cable does not count as a beam grasp.',
  'Never drop below the required two beam grasps. Individual sending grippers may release after other real grasps provide sufficient support. Custody changes only after the receiving pair has held for 0.20 seconds together with at least one sender. Grasp points, poses, timing and Agent allocation are choices, not prescribed alternating steps.',
  'Keep the complete payload, including its attached weight and flexible suspension, clear of table and floor during transport. Initial pickup and stable final cradle landing are distinct phases.',
  'A drop is recorded as a failed attempt. Only the last established sending team may recover it; the receiver cannot pick it up to bypass an airborne handover. No reset or respawn.',
  'A closed loop requires the declared full circuit of actual handovers; remaining at the coincident start/finish is not completion.'
 ],
 'dial':[
  'Each hidden code digit is independently chosen from 1,2,3; zero is excluded. Digits may repeat. The code cannot be derived from the public scene seed.',
  'Relative to the printed white disk, 0/1/2/3 are zero, clockwise quarter-turn, half-turn and counterclockwise quarter-turn. Mechanical range is -180 to +180 degrees; either endpoint is digit 2.',
  'A count appears when every disk stays in the same nonzero digit band (within 4 degrees of its mark) for 0.25 simulated seconds. Small vibration inside that band is allowed; instantaneous angular velocity is diagnostic, not an extra display gate. The display reveals no individual matches; transient/invalid input is a dash. Every newly stable configuration counts, including intermediate configurations.',
  'For movable modules, another robot must establish a two-finger blue-handle hold and full-module clearance of at least 20 mm for 0.25 simulated seconds before red contact. Retain support while manipulating; stop and re-establish it after loss.',
  'A crank grip is rigid and eccentric. Its contact point travels around the axle; in-place wrist rotation alone does not follow that path. Frame orientation and gravity affect manipulation.',
  'Reach CORRECT equal to the number of disks. Success latches visibly, but physical support and springs remain active.'
 ]}

def rules_for(case):
    rules=list(RULES[case['family']])
    if case['family']=='dial' and case.get('scene_revision',1)>=3 and case['level']>1:rules.append('The rear parking rack is an initial fixture only. Lift the complete module clear of every external support before red operation; resting on the rack is not airborne manipulation.')
    return rules+['Use robot actuation and real physical contact. No attachments to hands, state writes, teleportation, simulator reset, material edits or hidden evaluator access.']

def make_task(directory):
    directory=Path(directory).resolve();env=json.loads((directory/'environment.json').read_text());case=env['suite_case'];family=case['family']
    if case.get('suite_version')==2:
        from .v2 import task_for
        return task_for(case,env,directory)
    centers=[s['base_position_m'] for s in env['stations']];xs=[p[0] for p in centers];ys=[p[1] for p in centers]
    for t in env['tables']:
        xs.extend((t['center_m'][0]-t['half_size_m'][0],t['center_m'][0]+t['half_size_m'][0]));ys.extend((t['center_m'][1]-t['half_size_m'][1],t['center_m'][1]+t['half_size_m'][1]))
    task=dict(kind='frontier_suite',id=case['id'],suite_version=1,family=family,level=case['level'],scale=deepcopy(case),
        scene_asset=str(directory.relative_to(ROOT)),scene_sha256=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest(),
        arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},objects=deepcopy(env['objects']),
        table_height_m=.4,cube_size_m=.04,physics=deepcopy(env['physics']),mechanism=deepcopy(env.get('mechanism',{})),
        goals=deepcopy(env.get('goals',[])),grid=deepcopy(env.get('grid')),route=deepcopy(env.get('route')),pairs=deepcopy(env.get('pairs')),dials=deepcopy(env.get('dials')),
        workspace_world_m=dict(lower=[min(xs)-.10,min(ys)-.10,.025 if family=='relay' else .39 if family=='mosaic' else .425],upper=[max(xs)+.10,max(ys)+.10,1.35]),
        coordinate_frame='World +X right/east, +Y north, +Z up. Device marks belong to the movable device frame.',
        rules=rules_for(case),
        objective='Complete the task correctly and efficiently; account for planning, observation, communication, recovery and useful holding as well as simultaneous motion.',
        completion=dict(stable_duration_sim_s=.6,max_sample_gap_sim_s=.03,xy_tolerance_m=.005,z_tolerance_m=.003,linear_speed_m_s=.01,angular_speed_rad_s=.1),
        cameras=[c.get('name') for c in ET.parse(directory/'scene.xml').findall('worldbody/camera')],view_camera='global',recording_camera='global',
        observation_contract=dict(initial_positions_public=False,initial_geometry_public=True,live_object_truth_for_policy=False,
            policy=['selected timestamped RGB','robot proprioception','action status','visible aggregate dial score'],
            motion_screen='Robot-only geometry. Does not certify apparatus contact, insertion, shared load or cable clearance.'),
        status='Suite development task; per-scene Oracle evidence is separate from policy performance.')
    task['process_contract']=deepcopy(PROCESS_CONTRACT)
    if family=='relay':task['completion']['stable_duration_sim_s']=1.
    if family=='dial':
        task['completion'].pop('angular_speed_rad_s')
        task['completion'].update(stable_duration_sim_s=.25,angle_tolerance_deg=4,diagnostic_angular_speed_rad_s=math.radians(5),mode='aggregate_count_latch',feedback_rule_version=2,stability='same nonzero detent band for the full dwell; in-band vibration allowed')
    if family=='mosaic' and case['level']==1:task['completion'].update(mode='pocket_seating',xy_tolerance_role='diagnostic only; full footprint containment is the placement criterion',containment_tolerance_m=.0003,floor_tolerance_m=.0008)
    return validate(task)

def validate(task):
    if task.get('suite_version')==2:
        from .v2 import validate_task
        return validate_task(task)
    folder,env=validate_scene_asset(task);case=env['suite_case']
    if task.get('kind')!='frontier_suite' or task.get('family')!=case['family'] or task.get('level')!=case['level']:raise ValueError('Suite identity mismatch')
    if task['objects']!=env['objects'] or task['mechanism']!=env.get('mechanism',{}):raise ValueError('Physical metadata mismatch')
    for key in ('goals','grid','route','pairs','dials'):
        if task.get(key)!=env.get(key,[] if key=='goals' else None):raise ValueError('Task geometry mismatch: '+key)
    if task['rules']!=rules_for(case):raise ValueError('Suite rules mismatch')
    if len(task['arms'])!=case['arms']:raise ValueError('Robot count mismatch')
    if case['family'] in ('transport','mosaic') and len(task['goals'])!=len(task['objects']):raise ValueError('Object/goal conservation failure')
    if case['family']=='transport':
        stock=Counter(o['identity'] for o in task['objects']);need=Counter(g['requires']['identity'] for g in task['goals'])
        if stock!=need:raise ValueError('Identity inventory mismatch')
    if case['family']=='mosaic' and (len(task['goals'])!=case['rows']*case['columns'] or task['grid']['pitch_m']!=.051):raise ValueError('Mosaic dimensions mismatch')
    if case['family']=='dial' and len(task['dials'])!=case['dials']:raise ValueError('Dial count mismatch')
    if case['family']=='relay' and len(task['pairs'])*2!=case['arms']:raise ValueError('Relay pair count mismatch')
    return task

def public_task(task):
    if task.get('suite_version')==2:
        from .v2 import public_task as project_v2
        return project_v2(task)
    keep=('kind','id','suite_version','family','level','arms','objects','coordinate_frame','rules','objective','completion','cameras','view_camera','observation_contract','mechanism')
    result={k:deepcopy(task[k]) for k in keep}
    result['process_contract']=deepcopy(task.get('process_contract',PROCESS_CONTRACT))
    for o in result['objects']:o.pop('initial_position_m',None)
    if task['family']=='transport':result['orders']=[{k:deepcopy(g[k]) for k in ('goal_id','robot','requires')} for g in task['goals']]
    if task['family']=='mosaic':
        inventory=Counter((o['source'],o['color'],o['shape']) for o in result.pop('objects'))
        result['initial_supplies']=[dict(stock_station=source,color=color,shape=shape,count=count) for (source,color,shape),count in sorted(inventory.items())]
        result['supply_scope']='Initial counts near each station only, not current inventory or exclusive robot ownership; locate and track items from observations.'
        result['target']={k:deepcopy(task['grid'][k]) for k in ('rows','columns','pitch_m','cube_side_m','pattern','pocket_opening_m','floor_z_m','rim_z_m')};result['target'].update(center_world_m=[0,0,.4],row1='north',column1='west',reference_plane='tabletop/rim, not cube-center height',reference_image='public/inputs/target-pattern.png')
    if task['family']=='dial':result['dials']=[{k:deepcopy(d[k]) for k in ('id','index','nominal_robots')} for d in task['dials']]
    if task['family']=='relay':result['stations']=[{'robots':p['robots'],'route_index':i} for i,p in enumerate(task['pairs'])];result['closed_loop']=task['scale']['layout']=='closed_loop'
    # No simulator solver/friction values or exact live apparatus states.
    return result
