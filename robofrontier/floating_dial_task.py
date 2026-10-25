"""Free-module apparatus adapter. Exact module geometry is private evaluation only."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .scene_task import ROOT, validate_scene_asset
from .spring_dial_task import HINTS, CODE_VALUES, CODE_SYMBOLS, SYMBOL_ANGLES, rules


CONTACT_SEQUENCE={'kind':'lift_before_knob_contact','airborne_duration_s':.25,'contact_tolerance_m':.0002}
PUBLIC_CRANK_FIELDS=('radius_m','arm_width_m','arm_thickness_m','grip_diameter_m','grip_height_m',
    'hub_diameter_m','hub_height_m','assembly_mass_kg','grip_center_radius_m','grip_center_above_hub_m','grip_mount')

def floating_rules(sequence=None, crank=False):
    r=rules()
    r[1]='Symbols belong to each movable white disk: 0 at its marked top, 1 clockwise quarter turn, 2 half turn, 3 counterclockwise quarter turn, viewed from the printed front. Angle is red rotor relative to its own white disk, not world yaw. Range -180 to +180 degrees; either endpoint is symbol 2.'
    r.insert(3,'Each blue handle and white disk form one free movable module. Lift and stabilize the complete module before turning its red knob. All four modules must simultaneously clear the tabletop by at least 20 mm, with no apparatus/table contact, for feedback to be eligible. Grounded configurations show a dash. No fixture attachment or automatic lifting/rotation tool is supplied.')
    if sequence is not None:
        r.insert(4, 'Before another robot touches, grasps or turns a red knob, a different robot must maintain a two-finger grip on the blue handle and the complete module must continuously clear the table by at least 20 mm for 0.25 simulated seconds. Valid feedback also requires all four blue-handle holds and airborne prerequisites to remain established. Verify lift and stable support with RGB, not jaw width alone. Before this prerequisite the partner may prepare a distant pose but must stay outside knob contact. Loss of support requires stopping rotation and re-establishing support before continuing; a new contact must satisfy the prerequisite again. The private contact audit uses a 0.2 mm numerical contact tolerance. Initiating knob contact prematurely makes the episode process-noncompliant, even if the code is later solved; no automatic reset or replay occurs. Post-win cleanup is excluded from this audit.')
    if crank:
        r=[s.replace('red knob','red crank').replace('knob contact','crank contact') for s in r]
        r.append('The red crank has an eccentric rigid grip. Turning it requires moving the grasp point around its axle while maintaining contact; the grip is not a free-spinning bearing. The passive spring restores toward zero, but gravity on the eccentric crank depends on disk orientation. Reorienting the supported disk is permitted.')
    return r


def module_mechanism(p):
    """Public device specifications, independent of the physical red interface."""
    m=p['module']
    mass=m['assembled_mass_kg'] if 'assembled_mass_kg' in m else sum(m[k] for k in ('stem_mass_kg','face_mass_kg','knob_mass_kg'))
    result=dict(apparatus='floating',face_diameter_m=2*m['face_radius_m'],face_thickness_m=m['face_thickness_m'],
        handle_length_m=m['stem_length_m'],handle_cross_section_m=m['stem_width_m'],handle_eccentric_offset_m=m['stem_offset_y_m'],
        rotation_range_deg=[-180,180],return_mechanism='passive spring',module_mass_kg=mass,minimum_airborne_clearance_m=.02)
    if 'crank' in p:
        result.update(red_interface='crank',crank={k:deepcopy(p['crank'][k]) for k in PUBLIC_CRANK_FIELDS})
    else:
        result.update(knob_core_diameter_m=2*m['knob_radius_m'],knob_overall_diameter_m=.059,knob_height_m=m['knob_height_m'])
    if p.get('contact_sequence') is not None:result['contact_sequence']=deepcopy(p['contact_sequence'])
    return result


def make_task(directory):
    directory=Path(directory).resolve();p=json.loads((directory/'parameters.json').read_text());env=json.loads((directory/'environment.json').read_text())
    m=p['module'];table=p['table'];sequence=p.get('contact_sequence')
    return validate(dict(kind='spring_dial',id=env['id']+'-policy-v1',status='Free apparatus; Oracle validation separate from policy performance',
        scene_asset=str(directory.relative_to(ROOT)),scene_sha256=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest(),
        arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},objects=[],dials=deepcopy(env['dials']),table_height_m=table['height_m'],
        mechanism=module_mechanism(p),
        physics=deepcopy(p['physics']),goal='Discover the code and simultaneously maintain four airborne modules at their correct relative dial settings until CORRECT reads 4.',
        objective='Succeed while reducing code trials, physical actions and elapsed time; include stabilization, observation and coordination costs.',
        coordinate_frame='World +X right, +Y north, +Z up. Dial symbols are in each movable disk frame, viewed from its printed front.',
        allowed_code_angles_deg=list(CODE_VALUES),allowed_code_symbols=list(CODE_SYMBOLS),symbol_to_angle_deg=dict(SYMBOL_ANGLES),candidate_codes=81,
        rules=floating_rules(sequence,'crank' in p),planning_hints=list(HINTS)+([
            'An off-axis grasp point travels on an arc. Plan its translation and orientation together, including intermediate poses, jaw clearance and withdrawal. In-place wrist rotation alone does not trace the crank grip path.'
        ] if 'crank' in p else [])+[
            'Grasping farther along a handle can increase the load moment. Choose contact point and bearing direction for gravity and partner forces. If the disk pitches, slips or remains grounded during a lift, reassess the grasp and support geometry rather than relying only on more vertical motion or more force. Reorienting the blue handle changes the whole disk frame, not the code digit.',
            'The support grasp and module orientation determine reaction to knob torque and the turning arm\'s approach and future joint range. Plan these together; recheck relative alignment and support after contact or slip.',
            'Holding the blue handle and turning the red knob can require different robots. Nominal pairs identify the initial layout, not fixed roles or a required agent split. One agent/delegate may operate several robots.',
            'Compare parallel progress with coupling: operations on independent modules may overlap, while changing a shared module\'s support changes the partner\'s reference. Use observed evidence rather than treating commanded wrist rotation as a measured dial change.'],
        feedback=deepcopy(p['feedback']),completion=dict(stable_duration_sim_s=p['feedback']['stable_duration_s'],max_sample_gap_sim_s=p['feedback']['max_sample_gap_s'],success_latched=True,complete_sampling_required=True),
        workspace_world_m=dict(lower=[table.get('center_x_m',0)-table['length_m']/2,-table['width_m']/2,table['height_m']+.025],upper=[table.get('center_x_m',0)+table['length_m']/2,table['width_m']/2,1.30]),
        cameras=[c.get('name') for c in ET.parse(directory/'scene.xml').findall('worldbody/camera')],view_camera='global',recording_camera='global',
        camera_roles=dict(overview='global',game_feedback='board',dial_details={a:['dial_'+a,'dial_'+a+'_front','dial_'+a+'_side','dial_'+a+'_high'] for a in 'ABCD'}),
        observation_contract=dict(initial_positions_public=False,initial_geometry_public=True,policy=['RGB including instrument display','robot_proprioception','action_status'],
            live_object_truth_for_policy=False,fixture_geometry='Device dimensions are public; localize movable modules and their axes from permitted RGB.',motion_screen='Optional robot-only CAD; excludes apparatus and contact dynamics.'),
        metrics=['success','valid_trials','robot_action_count','absolute_dial_travel_rad','wall_time','simulation_time','holding_departures','zero_entries','sampling_gaps','grounded_dial_travel_rad','premature_knob_contact_events']))


def validate(task):
    folder,env=validate_scene_asset(task);p=json.loads((folder/'parameters.json').read_text());m=task['mechanism'];sequence=p.get('contact_sequence')
    if sequence is not None and sequence!=CONTACT_SEQUENCE:raise ValueError('Unsupported contact sequence specification')
    if m.get('contact_sequence')!=sequence or task.get('kind')!='spring_dial' or task.get('rules')!=floating_rules(sequence,'crank' in p):raise ValueError('Floating apparatus rules differ from the adapter')
    if p.get('apparatus')!='floating' or len(task['arms'])!=8 or task['objects'] or task['dials']!=env['dials']:raise ValueError('Invalid free-module apparatus identities')
    if task['physics']!=p['physics'] or task['feedback']!=p['feedback']:raise ValueError('Apparatus physics/feedback mismatch')
    if m['minimum_airborne_clearance_m']!=p['airborne']['minimum_clearance_m'] or task['table_height_m']!=p['table']['height_m']:raise ValueError('Airborne specification mismatch')
    if m!=module_mechanism(p):raise ValueError('Public module dimensions mismatch')
    f=task['feedback']
    if set(f)!={'angle_tolerance_deg','max_speed_deg_s','stable_duration_s','max_sample_gap_s'} or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in f.values()) or f['angle_tolerance_deg']>=45:raise ValueError('Invalid feedback tolerances')
    if task['completion']!=dict(stable_duration_sim_s=f['stable_duration_s'],max_sample_gap_sim_s=f['max_sample_gap_s'],success_latched=True,complete_sampling_required=True):raise ValueError('Completion mismatch')
    root=ET.parse(folder/'scene.xml').getroot()
    for a in 'ABCD':
        body=root.find(f"worldbody/body[@name='dial_{a}_module']");joint=None if body is None else body.find(f"body/joint[@name='dial_{a}_hinge']")
        if body is None or body.find('freejoint') is None or joint is None:raise ValueError('Each module must remain a free body with a relative hinge')
        if joint.get('axis')!='0 0 -1' or [float(x) for x in joint.get('range').split()]!=p['module']['range_rad']:raise ValueError('Dial convention mismatch')
        if float(joint.get('stiffness'))!=p['module']['spring_nm_per_rad'] or float(joint.get('damping'))!=p['module']['damping_nm_s_per_rad']:raise ValueError('Passive spring mismatch')
        if any(e.get('joint')==joint.get('name') for e in root.findall('actuator/*')):raise ValueError('A passive dial cannot have an actuator')
    return task


class FloatingState:
    """Cache apparatus geometry for scoring; never expose it through policy tools."""
    def __init__(self,world):
        import numpy as np
        self.w=world;self.modules=[];model=world.model
        self.sequence=world.task['mechanism'].get('contact_sequence');self.ready_since=[None]*4
        self.contact_episodes=[];self.new_contact_events=[];self.previous_red_contacts=set()
        self.blue_geoms={};self.red_geoms={};self.robot_geoms={};self.finger_geoms={}
        self.table={i for i in range(model.ngeom) if (model.geom(i).name or '').startswith('table_')}
        self.table_top=world.task['table_height_m'];self.minimum=world.task['mechanism']['minimum_airborne_clearance_m']
        for a in 'ABCD':
            ids={model.body('dial_'+a+'_module').id,model.body('dial_'+a+'_rotor').id}
            geoms=[g for g in range(model.ngeom) if model.geom_bodyid[g] in ids and (model.geom_contype[g] or model.geom_conaffinity[g])]
            self.modules.append(geoms)
        for i,a in enumerate('ABCD'):
            self.blue_geoms[model.geom('dial_'+a+'_module_stem').id]=i
            rotor=model.body('dial_'+a+'_rotor').id
            self.red_geoms.update({g:i for g in self.modules[i] if model.geom_bodyid[g]==rotor})
        for g in range(model.ngeom):
            name=model.body(model.geom_bodyid[g]).name or '';owner=name.split('_')[0]
            if owner in world.task['arms']:
                self.robot_geoms[g]=owner
                if name in (owner+'_left_finger',owner+'_right_finger'):self.finger_geoms[g]=(owner,name)
        self.all_geoms=set(g for gs in self.modules for g in gs)
        self.last=None;self.grounded_travel=[0.]*4;self.snapshot={}

    def sample(self,angles, *, audit=True):
        import mujoco
        import numpy as np
        w=self.w;model=w.model;data=w.data;clear=[]
        for gs in self.modules:
            bottom=math.inf
            for g in gs:
                size=model.geom_size[g];r=data.geom_xmat[g].reshape(3,3)
                if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:extent=float(abs(r[2])@size)
                elif model.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER:extent=float(size[1]*abs(r[2,2])+size[0]*math.sqrt(max(0,1-r[2,2]**2)))
                else:raise ValueError('Unsupported apparatus collision geometry')
                bottom=min(bottom,float(data.geom_xpos[g,2])-extent)
            clear.append(bottom-self.table_top)
        contact=any(c.dist<=0 and ((c.geom1 in self.table and c.geom2 in self.all_geoms) or (c.geom2 in self.table and c.geom1 in self.all_geoms)) for c in data.contact)
        airborne=[x>=self.minimum for x in clear]
        if self.last is not None:
            for i,ok in enumerate(airborne):
                if not ok:self.grounded_travel[i]+=float(abs(angles[i]-self.last[i]))
        self.last=np.array(angles,copy=True)
        self.snapshot=dict(clearance_m=clear,airborne=airborne,table_contact=bool(contact))
        self.new_contact_events=[]
        if self.sequence is not None:
            blue=[{} for _ in range(4)];red=set();tolerance=self.sequence['contact_tolerance_m']
            for c in data.contact:
                if c.dist>tolerance:continue
                for robot,part in ((int(c.geom1),int(c.geom2)),(int(c.geom2),int(c.geom1))):
                    if robot in self.finger_geoms and part in self.blue_geoms:
                        owner,finger=self.finger_geoms[robot];blue[self.blue_geoms[part]].setdefault(owner,set()).add(finger)
                    if robot in self.robot_geoms and part in self.red_geoms:red.add((self.red_geoms[part],self.robot_geoms[robot]))
            supporters=[sorted(owner for owner,fingers in owners.items() if len(fingers)==2) for owners in blue]
            ready=[]
            for i in range(4):
                if airborne[i] and supporters[i]:
                    if self.ready_since[i] is None:self.ready_since[i]=float(data.time)
                else:self.ready_since[i]=None
                ready.append(self.ready_since[i] is not None and data.time-self.ready_since[i]>=self.sequence['airborne_duration_s']-1e-9)
            if audit:
                for i,owner in sorted(red-self.previous_red_contacts):
                    if not ready[i] or not any(s!=owner for s in supporters[i]):
                        event=dict(module='ABCD'[i],robot=owner,contact_sim_time_s=float(data.time),clearance_m=clear[i],supporters=supporters[i],airborne_ready=bool(ready[i]))
                        self.contact_episodes.append(event);self.new_contact_events.append(event)
            self.previous_red_contacts=red
            self.snapshot.update(supporting_robots=supporters,airborne_ready=ready,red_contact_robots=[sorted(owner for idx,owner in red if idx==i) for i in range(4)],
                module_pose_world=[dict(position_m=data.body('dial_'+a+'_module').xpos.tolist(),quaternion_wxyz=data.body('dial_'+a+'_module').xquat.tolist()) for a in 'ABCD'])
        return all(airborne) and not contact and (self.sequence is None or all(ready))
