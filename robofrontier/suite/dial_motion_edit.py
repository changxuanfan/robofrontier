"""Dial-owned presentation classification, layered over immutable pose records.

Uses action intent and task-event evidence to protect progress. Large sustained
oscillation can be omitted without claiming that measured contact force is zero.
No physics, IK, commands, global factory scheduler or shared schema changes.
"""
from dataclasses import asdict, dataclass
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re

import mujoco
import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d

from ..motion_edit import (IdleSettings, build_edit, checksum, idle_mask,
    motion_features, ranges, EditedMotion)
from ..motion_recording import MotionReader


@dataclass(frozen=True)
class DialEditSettings:
    oscillation_window_s: float = 6.
    maximum_window_s: float = 30.
    minimum_cycles: int = 3
    spectral_fraction: float = .65
    event_padding_s: float = .5
    edge_padding_s: float = .5

    def __post_init__(self):
        if (not all(np.isfinite(x) and x > 0 for x in asdict(self).values())
                or type(self.minimum_cycles) is not int or self.minimum_cycles < 2
                or self.spectral_fraction > 1 or self.maximum_window_s < self.oscillation_window_s):
            raise ValueError('Invalid dial edit settings')


def stamp_mask(times, stamps, padding):
    mask = np.zeros(len(times),bool)
    for stamp in stamps:
        if not np.isfinite(stamp):
            raise ValueError('Nonfinite task event')
        a = np.searchsorted(times,stamp-padding)
        b = np.searchsorted(times,stamp+padding,side='right')
        mask[a:b] = True
    return mask


def periodic_without_drift(values, limit, hz, settings):
    """Require repeated motion and a stable centre, not merely small net change."""
    values = np.asarray(values,dtype=float)
    span = len(values)/hz
    if span < settings.oscillation_window_s-1/hz:
        return False
    for channel in values.T:
        amplitude = float(np.ptp(channel))
        if amplitude <= limit/np.sqrt(values.shape[1]):
            continue
        spectrum = np.abs(np.fft.rfft(channel-channel.mean()))**2
        spectrum[0] = 0
        peak = int(np.argmax(spectrum))
        if peak < settings.minimum_cycles:
            return False
        fraction = spectrum[max(1,peak-1):peak+2].sum()/max(spectrum.sum(),1e-30)
        if fraction < settings.spectral_fraction:
            return False
        # Fit the periodic component and drift jointly: a sine's slope alone
        # depends on its phase and is not a reliable slow-motion test.
        t = np.arange(len(channel))/hz
        omega = 2*np.pi*peak/span
        design = np.column_stack([np.ones(len(t)),t,np.sin(omega*t),np.cos(omega*t)])
        coefficients = np.linalg.lstsq(design,channel,rcond=None)[0]
        if abs(coefficients[1])*span > limit:
            return False
    return True


def classify(model, times, qpos, drives, event_times, *, evidence_complete,
             sample_hz=60., settings=DialEditSettings()):
    """Return a keep mask and independent, auditable presentation reasons."""
    times, qpos = np.asarray(times), np.asarray(qpos,dtype=float)
    basic = IdleSettings()
    keep, _ = idle_mask(model,times,qpos,basic,sample_hz)
    protected = stamp_mask(times,event_times,settings.event_padding_s)
    effective = np.zeros(len(times),bool)
    unknown = np.full(len(times),not evidence_complete,bool)
    for drive in drives:
        if drive['kind'] not in ('effective','hold','unknown') or drive['end'] < drive['start']:
            raise ValueError('Invalid action evidence interval')
        a = np.searchsorted(times,drive['start'])
        b = np.searchsorted(times,drive['end'],side='right')
        if drive['kind'] == 'effective':
            if 'progress_times' in drive:
                local = stamp_mask(times,drive['progress_times'],settings.event_padding_s)
                effective[a:b] |= local[a:b]
            else:effective[a:b] = True
        elif drive['kind'] == 'unknown':unknown[a:b] = True
    features = motion_features(model,qpos,basic)
    arm_groups = [g for g in features if re.fullmatch(r'.+_joint\d+',g[0]) and 'finger_joint' not in g[0]]
    if not arm_groups:
        raise ValueError('Dial classification requires robot joint evidence')
    window = max(3,round(settings.oscillation_window_s*sample_hz))
    width = window | 1
    arm_moving = np.zeros(len(times),bool)
    for _, values, limit in arm_groups:
        excursion = maximum_filter1d(values,width,axis=0,mode='nearest')-minimum_filter1d(values,width,axis=0,mode='nearest')
        arm_moving |= np.max(excursion,axis=1)>limit
    oscillation = np.zeros(len(times),bool)
    non_arm = [g for g in features if not (re.fullmatch(r'.+_joint\d+',g[0]) and 'finger_joint' not in g[0])]
    # Analyse fixed windows inside quiet-arm regions. A moving centre in any
    # appreciable object/finger channel prevents an oscillation classification.
    for start,stop in ranges(~(arm_moving|effective|unknown|protected)):
        maximum = min(stop-start,round(settings.maximum_window_s*sample_hz))
        widths = {min(maximum,window*factor) for factor in (1,2,4,8)}
        for extent in sorted(w for w in widths if w>=window):
            starts = list(range(start,max(start,stop-extent+1),max(1,extent//2)))
            starts.append(stop-extent)
            for a in sorted(set(starts)):
                b = a+extent
                if any(np.linalg.norm(np.ptp(v[a:b],axis=0))>limit for _,v,limit in arm_groups):
                    continue
                large = [(values[a:b],limit) for _,values,limit in non_arm
                    if np.linalg.norm(np.ptp(values[a:b],axis=0))>limit]
                if not large or not all(periodic_without_drift(v,limit,sample_hz,settings) for v,limit in large):
                    continue
                margin = round(settings.edge_padding_s*sample_hz)
                oscillation[a+margin:b-margin] = True
    reason = np.full(len(times),'active',dtype=object)
    reason[~keep] = 'bounded_micro_motion'
    reason[oscillation] = 'oscillating_without_progress'
    keep[oscillation] = False
    keep[protected|effective|unknown] = True
    reason[protected] = 'task_event_context'
    reason[effective] = 'effective_command'
    reason[unknown] = 'unknown_evidence'
    keep[0] = keep[-1] = True
    return keep, reason


def load_lines(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def event_evidence(events, process):
    names = {'rgba','observed_best_input','blind_best_guess','visible_win_reached',
        'blind_best_solved','validation_rejected','reference_blue_pressure_guard',
        'reference_blue_pressure_requalified','oracle_recovery_requested',
        'reference_live_repair_resumed','reference_live_repair_plan',
        'irreversible_process_violation','trial_end'}
    stamps = [e['sim_time_s'] for e in events if e['event'] in names]
    previous = None
    keys = ('all_goals_met','task_success','solved','valid_trials','display',
        'private_support_ready','input_ready','violations')
    for row in process:
        if 'result' not in row:continue
        state = {k:row['result'].get(k) for k in keys}
        if previous is None or state != previous:
            stamps.append(row['sim_time_s'])
        previous = state
    return sorted(set(stamps))


def command_evidence(model, times, qpos, events, attempt):
    """Resolve all ordinary/recovery commands; missing outcomes stay unknown."""
    intervals, stamps, open_actions, pending = [], [], {}, []
    last_commands = {}
    data = mujoco.MjData(model)

    def add(start,end,commands,status):
        index = min(len(times)-1,int(np.searchsorted(times,start)))
        final = min(len(times)-1,int(np.searchsorted(times,end)))
        data.qpos[:] = qpos[index]
        mujoco.mj_kinematics(model,data)
        kind = 'hold'
        progress_times = []
        for robot, (op, parameters) in commands.items():
            signature = json.dumps([op,{k:v for k,v in parameters.items()
                if k not in ('speed_m_s','speed','angular_speed_rad_s','joint_speed_rad_s')}],sort_keys=True)
            repeated = last_commands.get(robot)==signature
            last_commands[robot] = signature
            if op == 'move':
                joint_names = [f'{robot}_joint{i}' for i in range(1,8)]
                addresses = [int(model.joint(n).qposadr[0]) for n in joint_names]
                if 'joint_rad' in parameters:
                    changed = np.max(abs(np.asarray(parameters['joint_rad'])-qpos[index,addresses]))>1e-6
                elif 'delta_m' in parameters:
                    changed = np.linalg.norm(parameters['delta_m'])>1e-6
                elif 'target_m' in parameters:
                    site = model.site(robot+'_tcp').id
                    changed = np.linalg.norm(np.asarray(parameters['target_m'])-data.site_xpos[site])>1e-5
                else:
                    changed = False
                # Orientation targets are purposeful unless exactly repeated.
                changed |= 'quaternion_wxyz' in parameters and not repeated
                if changed and kind != 'unknown':kind = 'effective'
                channel_ids, resolution = addresses, 1e-4
            elif op == 'gripper':
                fingers = [int(model.joint(robot+'_finger_joint'+str(i)).qposadr[0]) for i in (1,2)]
                moved = abs(qpos[final,fingers].sum()-qpos[index,fingers].sum())>1e-5
                if (not repeated or moved) and kind != 'unknown':kind = 'effective'
                channel_ids, resolution = fingers, 1e-6
            else:
                kind = 'unknown'
                continue
            # Keep observed progress within an action, not its entire duration.
            # New distance records avoid counting endless repeated oscillations
            # as continued approach. Start/end/outcome contexts are kept below.
            values = qpos[index:final+1,channel_ids]
            distance = np.linalg.norm((values-values[-1])/resolution,axis=1)
            best = float('inf')
            for offset,error in enumerate(distance):
                if error < best-1:
                    progress_times.append(float(times[index+offset]))
                    best = error
        if status not in ('completed','failed'):kind = 'unknown'
        intervals.append(dict(start=float(start),end=float(end),kind=kind,progress_times=progress_times,
            execution_status=status))
        if kind != 'hold' or status != 'completed':stamps.extend([start,end])

    complete = True
    recovery = {e['request_id']:e for e in events if e['event']=='oracle_recovery_input'}
    for event in events:
        name = event['event']
        if name == 'action_start':
            label = event['label']
            if label in open_actions:complete = False
            open_actions[label] = event
        elif name in ('action_end','action_interrupted'):
            start = open_actions.pop(event['label'],None)
            if start is None:
                complete = False
                continue
            results = event.get('results',{})
            if name=='action_end' and set(results)!=set(start['commands']):complete = False
            status = 'unknown' if name=='action_interrupted' else 'completed' if all(r['status']=='completed' for r in results.values()) else 'failed'
            pending.append((start['sim_time_s'],event['sim_time_s'],start['commands'],status))
    for start in open_actions.values():
        intervals.append(dict(start=start['sim_time_s'],end=float(times[-1]),kind='unknown'))
    for ident,event in recovery.items():
        path = Path(attempt)/'recovery/results'/(ident+'.json')
        if not path.exists():
            intervals.append(dict(start=event['sim_time_s'],end=float(times[-1]),kind='unknown'))
            continue
        record = json.loads(path.read_text())
        request = record['request']
        fingerprint = hashlib.sha256(json.dumps(request,sort_keys=True).encode()).hexdigest()
        if fingerprint != event['input_sha256'] or fingerprint != record.get('input_sha256'):
            raise ValueError('Recovery request identity differs from its recorded event')
        sequence = request['sequence']
        end = record.get('finished_sim_s',float(times[-1]))
        if len(sequence) != 1:
            intervals.append(dict(start=event['sim_time_s'],end=end,kind='unknown'))
            continue
        step = sequence[0]
        pending.append((event['sim_time_s'],end,{request['robot']:[step['op'],step['parameters']]},record['status']))
    for args in sorted(pending,key=lambda x:(x[0],x[1])):
        add(*args)
    return intervals,stamps,complete


def process_covered(process, times):
    stamps = np.array([r['sim_time_s'] for r in process if 'result' in r],dtype=float)
    return bool(len(stamps) and np.isfinite(stamps).all() and np.all(np.diff(stamps)>=0)
        and stamps[0]<=times[0]+1.1 and stamps[-1]>=times[-1]-1.1
        and np.all(np.diff(stamps)<=1.1))


def build_dial_edit(recording, settings=DialEditSettings()):
    reader = MotionReader(recording)
    if not reader.meta['case_id'].startswith('dial-best5-'):
        raise ValueError('This classifier is restricted to BEST5 dial records')
    edit = build_edit(recording)
    pairs = list(reader.frames())
    times = np.array([t for t,_ in pairs])
    qpos = np.array([q for _,q in pairs],dtype=float)
    model = mujoco.MjModel.from_xml_path(str(reader.path/'source/scene.xml'))
    attempt = reader.path.parent
    event_path,process_path = reader.path/'events.jsonl',attempt/'process.jsonl'
    events = load_lines(event_path)
    process = load_lines(process_path) if process_path.exists() else []
    drives,command_stamps,complete = command_evidence(model,times,qpos,events,attempt)
    complete = complete and process_covered(process,times)
    semantic_times = event_evidence(events,process)
    semantic_times += [d['end'] for d in drives if d.get('execution_status','unknown')!='completed']
    event_times = semantic_times+command_stamps
    keep,reasons = classify(model,times,qpos,drives,event_times,
        evidence_complete=complete,sample_hz=reader.meta['sample_hz'],settings=settings)
    hz = float(reader.meta['sample_hz'])
    spans,retained = [],0
    for a,b in ranges(keep):
        spans.append(dict(first_frame=int(a),stop_frame=int(b),source_start_s=float(times[a]),
            source_last_s=float(times[b-1]),edited_start_s=retained/hz,edited_stop_s=(retained+b-a)/hz))
        retained += int(b-a)
    removed = [dict(first_frame=int(a),stop_frame=int(b),source_start_s=float(times[a]),
        source_last_s=float(times[b-1]),duration_s=(b-a)/hz,
        reasons=sorted(set(reasons[a:b]))) for a,b in ranges(~keep)]
    edit.update(retained_frames=retained,removed_frames=int((~keep).sum()),
        edited_duration_s=retained/hz,retained_spans=spans,removed_spans=removed)
    # Guarantee an actual rendered source sample at every semantic transition,
    # even when accelerated sampling would otherwise jump over the event.
    # Command boundaries protect source context but need not each add an output
    # frame. Otherwise many short commands would silently slow nominal 10x.
    protected_indices = [min(len(times)-1,int(np.searchsorted(times,t-1e-9))) for t in semantic_times]
    edit['appearance_frames'] = sorted(set(edit['appearance_frames'])|set(protected_indices))
    edit['dial_profile'] = dict(version='dial-progress-v1',settings=asdict(settings),
        classifier_source_sha256=checksum(__file__),
        source_evidence_complete=complete,
        measured_force_available=False,semantic_events_protected=len(set(semantic_times)),
        command_boundaries_protected=len(set(command_stamps)),
        process_sha256=checksum(process_path) if process_path.exists() else None,
        recovery_result_sha256={str(p.relative_to(attempt)):checksum(p) for p in
            sorted((attempt/'recovery/results').glob('*.json'))},
        removed_oscillation_frames=int(np.sum((~keep)&(reasons=='oscillating_without_progress'))),
        scope='Presentation omission from recorded intent/progress; not a zero-force measurement')
    return edit


class DialReplayTrack:
    """One independent, streaming dial cursor for a future shared compositor.

    There is no global renderer or shared simulation clock here. Seek/rewind is
    source-record playback, never a live physical reset. End holds the last pose;
    the future compositor decides whether to loop or mark a station complete.
    """
    def __init__(self, recording, edit, fps=30., speed=10.):
        self.view = EditedMotion(recording,edit)
        self.reader = self.view.reader
        if not self.reader.meta['case_id'].startswith('dial-best5-'):
            raise ValueError('Dial track requires a BEST5 source')
        self.indices = self.view.video_indices(fps,speed)
        self.fps = fps
        profile = self.view.edit.get('dial_profile')
        if profile:
            process = self.reader.path.parent/'process.jsonl'
            if profile['process_sha256'] is not None and checksum(process) != profile['process_sha256']:
                raise ValueError('Dial process evidence changed')
            for name,expected in profile['recovery_result_sha256'].items():
                relative = Path(name)
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError('Invalid recovery evidence path')
                if checksum(self.reader.path.parent/relative) != expected:
                    raise ValueError('Dial recovery evidence changed')
        self.reset()

    def reset(self):
        self.close()
        self._source_frames = self.reader.frames()
        self._frames = enumerate(self._source_frames)
        self._index = -1
        self._state = None
        self._events = iter(self.reader.events())
        self._event = next(self._events,None)
        self._rgba = {}
        self._reset_pending = True

    def close(self):
        for name in ('_source_frames','_events'):
            stream = getattr(self,name,None)
            if stream is not None:stream.close()

    def __enter__(self):return self

    def __exit__(self,*args):self.close()

    def at(self, video_frame):
        if type(video_frame) is not int or video_frame < 0:
            raise ValueError('Nonnegative integer output frame required')
        local = min(video_frame,len(self.indices)-1)
        target = int(self.indices[local])
        if target < self._index:self.reset()
        while self._index < target:
            self._index,self._state = next(self._frames)
        stamp,qpos = self._state
        while self._event is not None and self._event['sim_time_s'] <= stamp+1e-9:
            if self._event['event']=='rgba':self._rgba.update(self._event['geoms'])
            self._event = next(self._events,None)
        reset = self._reset_pending
        self._reset_pending = False
        return dict(case_id=self.reader.meta['case_id'],output_frame=video_frame,
            presentation_time_s=video_frame/self.fps,source_frame=target,source_time_s=stamp,
            qpos=qpos.copy(),appearance_changes=deepcopy(self._rgba),
            appearance_reset=reset,
            completed=video_frame>=len(self.indices))
