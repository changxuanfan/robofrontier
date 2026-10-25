"""Spring-dial task adapter and private apparatus sampling, without a solver.

Only public_spring_task crosses into policy inputs. Dial angles, code and the
trial ledger stay private; the permitted game feedback is the rendered display.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import secrets
import time
import xml.etree.ElementTree as ET

from .scene_task import ROOT, validate_scene_asset

CODE_PATH = Path('private/inputs/spring-code.json')
CODE_VALUES = (90, 180, 270)
CODE_SYMBOLS = (1, 2, 3)
SYMBOL_ANGLES = {'0':0,'1':90,'2':180,'3':270}


def rules():
    return [
        'All four dials start at symbol 0. Each hidden code entry is sampled independently and uniformly from symbols 1, 2, 3; repetitions are allowed and zero is never a code entry.',
        'Top view: symbol 0 north, 1 east (90 degrees), 2 south (180 degrees), 3 west (270 degrees). Clockwise is positive; left 90 reaches symbol 3. Mechanical range is -180 to +180 degrees, not continuous winding; either endpoint represents symbol 2.',
        'Rotate and hold through real robot/gripper contact. The spring returns an unrestrained dial toward zero. Direct dial-state writes, attachments, simulator resets and physics-parameter changes are unavailable policy actions.',
        'Automatic feedback requires all four dials near nonzero marks and slow/stable for the declared duration. The display gives only the total correct count, not the identities of correct entries. Invalid/transient inputs display a dash; a valid zero score displays 0.',
        'Every newly entered stable configuration counts as a trial, including intermediate configurations formed while adjusting. Continued holding and repeated observation do not create extra trials; leaving and re-entering can. There is no separate submit button.',
        'The first valid score 4 wins. The visible 4 and private success record then latch; physical springs remain active. Post-success cleanup does not create more trials or erase success.',
        'Robot motion completion is not proof of dial alignment. Pose and established grasp holding persist across tool calls, waits and normal control handback until changed by an explicit command.',
        'Nominal robot/dial labels identify the initial arrangement, not an extra exclusive reservation. Use permitted ownership and actual reachability when choosing allocation.',
    ]


HINTS = [
    'Compare the information value of a proposed trial with its required angle changes, execution time and coordination cost. Separate confirmed deductions from hypotheses.',
    'You can retain some settings while changing others; a release/regrasp of every dial is not required for each trial.',
    'A grasp pose and current joint configuration affect subsequent rotation range. Interpret feedback against an observed configuration, accounting for slip, tracking error and stale views.',
    'When multiple roles operate one apparatus, coordinate the input configuration being tested so the aggregate feedback can be interpreted. Choose the communication and execution granularity yourself.',
]


def make_spring_task(directory):
    directory = Path(directory).resolve()
    env = json.loads((directory/'environment.json').read_text())
    params = json.loads((directory/'parameters.json').read_text())
    table = params['table']; dial = params['dial']
    task = dict(
        kind='spring_dial', id=env['id']+'-policy-v1',
        status='Policy adapter v1; engineering validated separately from model performance',
        scene_asset=str(directory.relative_to(ROOT)),
        scene_sha256=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest(),
        arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},
        objects=[], dials=deepcopy(env['dials']), table_height_m=table['height_m'],
        mechanism=dict(knob_core_diameter_m=2*dial['knob_core_radius_m'],
                       knob_overall_diameter_m=.054, knob_height_m=dial['knob_height_m'],
                       rotation_range_deg=[-180,180], return_mechanism='passive spring'),
        physics=deepcopy(params['physics']),
        goal='Discover the hidden code and simultaneously hold its four dial settings until the display reads 4.',
        objective='Succeed, and reduce valid code trials, unnecessary robot actions and total completion time. Compare information gain with physical execution and coordination cost; these metrics are reported separately.',
        coordinate_frame='World +X right, +Y north, +Z up; degree labels use the common north-up top view.',
        allowed_code_angles_deg=list(CODE_VALUES),allowed_code_symbols=list(CODE_SYMBOLS),
        symbol_to_angle_deg=dict(SYMBOL_ANGLES),candidate_codes=81,
        rules=rules(), planning_hints=list(HINTS), feedback=deepcopy(params['feedback']),
        completion=dict(stable_duration_sim_s=params['feedback']['stable_duration_s'],
                        max_sample_gap_sim_s=params['feedback']['max_sample_gap_s'],
                        success_latched=True, complete_sampling_required=True),
        workspace_world_m=dict(lower=[-table['length_m']/2,-table['width_m']/2,table['height_m']+.025],
                               upper=[table['length_m']/2,table['width_m']/2,1.30]),
        cameras=[c.get('name') for c in ET.parse(directory/'scene.xml').findall('worldbody/camera')],
        view_camera='global', recording_camera='board',
        camera_roles=dict(overview='global', game_feedback='board',
                          dial_details={a:'dial_'+a for a in 'ABCD'}),
        observation_contract=dict(initial_positions_public=False, initial_geometry_public=True,
            policy=['RGB including the instrument display','robot_proprioception','action_status'],
            live_object_truth_for_policy=False,
            fixture_geometry='Fixed fixture dimensions and initial axis locations are declared public; live dial angles and individual correctness are not.',
            motion_screen='Optional nominal robot-only geometry; does not certify contact manipulation or dial state.'),
        metrics=['success','valid_trials','robot_action_count','absolute_dial_travel_rad',
                 'wall_time','simulation_time','holding_departures','zero_entries','sampling_gaps'])
    return validate_spring_task(task)


def validate_spring_task(task):
    if task.get('kind') != 'spring_dial': raise ValueError('Expected spring_dial')
    if task.get('mechanism',{}).get('apparatus')=='floating':
        from .floating_dial_task import validate
        return validate(task)
    if task.get('mechanism',{}).get('apparatus')=='crank':
        from .spring_crank_task import validate_crank_task
        return validate_crank_task(task)
    folder, env = validate_scene_asset(task)
    params = json.loads((folder/'parameters.json').read_text())
    if list(task['arms']) != list('ABCD') or task['objects'] != [] or task['dials'] != env['dials']:
        raise ValueError('This apparatus requires the four generated dial/robot identities')
    if task['physics'] != params['physics'] or task['feedback'] != params['feedback']:
        raise ValueError('Physics or feedback differs from the generated apparatus')
    d=params['dial']
    expected=dict(knob_core_diameter_m=2*d['knob_core_radius_m'],knob_overall_diameter_m=.054,
                  knob_height_m=d['knob_height_m'],rotation_range_deg=[-180,180],return_mechanism='passive spring')
    if task['mechanism']!=expected or task['table_height_m']!=params['table']['height_m']:
        raise ValueError('Declared fixture specifications differ from the generated apparatus')
    f = task['feedback']; c = task['completion']
    if set(f) != {'angle_tolerance_deg','max_speed_deg_s','stable_duration_s','max_sample_gap_s'}:
        raise ValueError('Unknown feedback setting')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in f.values()):
        raise ValueError('Feedback settings must be finite and positive')
    if f['angle_tolerance_deg']>=45 or f['max_sample_gap_s']<task['physics']['timestep_s']:
        raise ValueError('Ambiguous angle marks or unsupported sample interval')
    if c != dict(stable_duration_sim_s=f['stable_duration_s'], max_sample_gap_sim_s=f['max_sample_gap_s'],
                 success_latched=True, complete_sampling_required=True):
        raise ValueError('Success/sampling contract differs from apparatus')
    if (task['allowed_code_angles_deg'] != list(CODE_VALUES) or task['candidate_codes'] != 81 or
        task['allowed_code_symbols']!=list(CODE_SYMBOLS) or task['symbol_to_angle_deg']!=SYMBOL_ANGLES):
        raise ValueError('Invalid code alphabet')
    if task['rules'] != rules() or task['planning_hints'] != HINTS:
        raise ValueError('Public rules/hints differ from canonical task')
    if task['observation_contract'].get('live_object_truth_for_policy') is not False:
        raise ValueError('Live dial truth is private')
    if any(task.get(k) not in task['cameras'] for k in ('view_camera','recording_camera')):
        raise ValueError('Unknown viewing/recording camera')
    expected_views=dict(overview='global',game_feedback='board',dial_details={a:'dial_'+a for a in 'ABCD'})
    if task['camera_roles']!=expected_views or not {'board','global',*expected_views['dial_details'].values()}<=set(task['cameras']):
        raise ValueError('Incomplete or inconsistent game camera catalog')
    root = ET.parse(folder/'scene.xml').getroot()
    for dial in task['dials']:
        body = root.find(f"worldbody/body[@name='dial_{dial['id']}']")
        joint = None if body is None else body.find('joint')
        if joint is None or joint.get('axis') != '0 0 -1' or float(joint.get('springref','0')) != 0:
            raise ValueError('Dial must have its declared passive axis and zero rest')
        for key, value in (('stiffness', params['dial']['spring_nm_per_rad']),
                           ('damping', params['dial']['damping_nm_s_per_rad'])):
            if not math.isclose(float(joint.get(key,'0')), value): raise ValueError('Dial '+key+' mismatch')
        if [float(v) for v in joint.get('range','').split()] != params['dial']['range_rad']:
            raise ValueError('Dial travel differs from the public convention')
        names={joint.get('name')}
        if any(a.get('joint') in names for a in root.findall('actuator/*')):
            raise ValueError('A passive dial cannot have a policy actuator')
    # Keyframe joint order is derived from XML, without constructing a simulator.
    offset=0; addresses={}
    for e in root.find('worldbody').iter():
        if e.tag not in ('joint','freejoint'): continue
        addresses[e.get('name')]=offset
        offset += {'free':7,'ball':4}.get('free' if e.tag=='freejoint' else e.get('type','hinge'),1)
    key=root.find("keyframe/key[@name='initial']")
    values=[] if key is None else [float(x) for x in key.get('qpos','').split()]
    if len(values)!=offset or any(abs(values[addresses['dial_'+a+'_hinge']])>1e-9 for a in 'ABCD'):
        raise ValueError('All dials must initialize at zero')
    return task


def public_spring_task(task):
    if task.get('mechanism',{}).get('apparatus')=='crank':
        from .spring_crank_task import public_crank_task
        return public_crank_task(task)
    keys=('kind','id','status','arms','objects','dials','table_height_m','mechanism','goal','objective',
          'coordinate_frame','allowed_code_angles_deg','allowed_code_symbols','symbol_to_angle_deg','candidate_codes','rules','planning_hints',
          'feedback','completion','workspace_world_m','cameras','view_camera','recording_camera',
          'camera_roles','observation_contract','metrics')
    result={k:deepcopy(task[k]) for k in keys}
    if not task['observation_contract']['initial_positions_public']:
        for dial in result['dials']:dial.pop('center_m',None)
        result['observation_contract']['fixture_geometry']='Device dimensions are public; localize the dial axes and movable frames from permitted RGB.'
    return result


def private_code(source=None):
    """Fresh independent symbols, or an explicit paired code. Normalize legacy engineering angles."""
    value={'code_symbols':[secrets.choice(CODE_SYMBOLS) for _ in range(4)]} if source is None else json.loads(Path(source).read_text())
    if not isinstance(value,dict) or set(value) not in ({'code_symbols'},{'code_degrees'}):
        raise ValueError('Private puzzle requires four code_symbols from 1, 2, 3')
    key=next(iter(value));entries=value[key];alphabet=CODE_SYMBOLS if key=='code_symbols' else CODE_VALUES
    if (not isinstance(entries,list) or len(entries)!=4 or
            any(type(x) is not int or x not in alphabet for x in entries)):
        raise ValueError('Private puzzle entries must be four valid nonzero integer symbols/angles')
    return {'code_symbols':entries if key=='code_symbols' else [x//90 for x in entries]}


class SpringDialSampler:
    """Private 2-ms apparatus update with sparse audit samples and exact trial edges."""
    def __init__(self, world):
        from .spring_dial import DialFeedback
        self.world=world; self.active=True; self.closed=False
        code=[90*x for x in private_code(Path(world.directory)/CODE_PATH)['code_symbols']]
        self.game=DialFeedback(world.task['feedback'],secret=code)
        self.qids=[world.model.jnt_qposadr[world.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
        self.dofs=[world.model.jnt_dofadr[world.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
        self.apparatus=None
        if world.task.get('mechanism',{}).get('apparatus')=='floating':
            from .floating_dial_task import FloatingState
            self.apparatus=FloatingState(world)
        self.started_sim=world.time; self.started_wall=time.monotonic()
        self.success_wall_elapsed=None
        self.last_angles=None; self.last_marks=None; self.last_sample=-math.inf; self.last_ready=None
        self.mark_since=[None]*4;self.mark_qualified=[False]*4;self.ever_nonzero=[False]*4
        self.last_visual=None;self.last_tick=None
        self.travel=[0.]*4; self.zero_entries=[0]*4; self.departures=[0]*4
        self.last_feedback=dict(ready=False,correct_count=None,won=False,valid_trials=0)
        self.stream=(Path(world.directory)/'private/outputs/spring-dial-process.jsonl').open('x')
        self.record('start', code_stored_privately=True)
        self.sample()

    def record(self, event, **values):
        self.stream.write(json.dumps(dict(event=event,sim_time_s=self.world.time,wall_time_s=time.time(),
                                         wall_elapsed_s=time.monotonic()-self.started_wall,**values),allow_nan=False)+'\n')
        self.stream.flush()

    def sample(self):
        if not self.active:return
        import numpy as np
        from .spring_dial import update_display
        w=self.world; angles=w.data.qpos[self.qids].copy(); speeds=w.data.qvel[self.dofs].copy()
        if self.last_angles is not None:
            self.travel=(np.array(self.travel)+abs(angles-self.last_angles)).tolist()
        self.last_angles=angles
        degrees=np.degrees(angles)%360; marks=(np.rint(degrees/90).astype(int)%4)*90
        near=abs((degrees-marks+180)%360-180)<=w.task['feedback']['angle_tolerance_deg']
        slow=abs(np.degrees(speeds))<=w.task['feedback']['max_speed_deg_s']
        states=[int(mark) if n and s else None for mark,n,s in zip(marks,near,slow)]
        gap=self.last_tick is not None and w.time-self.last_tick>w.task['feedback']['max_sample_gap_s']
        self.last_tick=w.time
        for i,new in enumerate(states):
            if abs((degrees[i]+180)%360-180)>w.task['feedback']['angle_tolerance_deg']:
                self.ever_nonzero[i]=True
            old=None if self.last_marks is None else self.last_marks[i]
            if old!=new or gap or self.last_marks is None:
                if self.mark_qualified[i] and old in CODE_VALUES and old!=new and not gap:
                    self.departures[i]+=1
                self.mark_since[i]=w.time if new is not None else None
                self.mark_qualified[i]=False
            if (new is not None and not self.mark_qualified[i] and
                    w.time-self.mark_since[i]>=w.task['feedback']['stable_duration_s']-1e-9):
                self.mark_qualified[i]=True
                if new==0 and self.ever_nonzero[i]:self.zero_entries[i]+=1
        self.last_marks=states
        gaps=self.game.sampling_gaps
        eligible=True if self.apparatus is None else self.apparatus.sample(angles,audit=not self.game.won)
        if self.apparatus is not None:
            for event in self.apparatus.new_contact_events:self.record('premature_knob_contact',**event)
        self.last_feedback=self.game.update(w.time,angles,speeds,eligible=eligible)
        if self.game.won and self.success_wall_elapsed is None:
            self.success_wall_elapsed=time.monotonic()-self.started_wall
        visual=tuple(self.last_feedback[k] for k in ('ready','correct_count','won'))
        if visual!=self.last_visual:
            update_display(w.model,self.last_feedback);self.last_visual=visual
        if self.game.sampling_gaps!=gaps:self.record('sampling_gap')
        if self.game.new_trial is not None:
            self.record('trial',**self.game.new_trial,actual_angle_rad=angles.tolist())
        if self.last_feedback['ready']!=self.last_ready:
            self.record('display_state',**self.last_feedback);self.last_ready=self.last_feedback['ready']
        if w.time-self.last_sample>=.1-1e-9:
            self.record('sample',angle_rad=angles.tolist(),angular_speed_rad_s=speeds.tolist(),
                        feedback=self.last_feedback,apparatus=None if self.apparatus is None else self.apparatus.snapshot)
            self.last_sample=w.time

    def result(self):
        complete=self.game.sampling_gaps==0
        compliant=self.apparatus is None or not self.apparatus.contact_episodes
        return dict(kind='spring_dial',all_goals_met=self.game.won and complete and compliant,
            task_success=self.game.won and complete and compliant, solved=self.game.won,process_compliant=compliant,
            evidence_complete=complete, run_valid=complete,
            invalid_reason=None if complete else 'incomplete_apparatus_sampling',
            first_success_sim_time_s=self.game.first_win_time_s,
            metrics=dict(valid_trials=self.game.queries,
                time_to_success_sim_s=None if self.game.first_win_time_s is None else self.game.first_win_time_s-self.started_sim,
                time_to_success_wall_s=self.success_wall_elapsed,
                absolute_dial_travel_rad=dict(zip('ABCD',self.travel)),
                zero_entries=dict(zip('ABCD',self.zero_entries)),
                holding_departures=dict(zip('ABCD',self.departures)),
                sampling_gaps=self.game.sampling_gaps,
                **({} if self.apparatus is None else {'grounded_dial_travel_rad':dict(zip('ABCD',self.apparatus.grounded_travel)),'premature_knob_contact_events':len(self.apparatus.contact_episodes)})),
            scope='Private apparatus scoring, not policy feedback. Holding departures leave individually stable nonzero settings; zero entries are stable returns, neither is a certified slip classifier. No collision-free or hardware-equivalence claim.')

    def finish(self):
        if self.active:
            self.record('end',result=self.result());self.active=False
        from .run_setup import write_json
        write_json(Path(self.world.directory)/'private/outputs/spring-dial-summary.json',self.result())

    def close(self):
        if self.closed:return
        try:self.finish()
        finally:self.stream.close();self.closed=True
