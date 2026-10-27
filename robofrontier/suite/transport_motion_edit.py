"""Source-native Transport motion edits, independent of video and background.

Family-owned format: keep the immutable motion stream and reference exact source
indices. No physics, force inference, IK, rendering or shared harness changes.
"""
from dataclasses import asdict,dataclass
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import maximum_filter1d,minimum_filter1d,uniform_filter1d

from ..motion_recording import MotionReader,digest
from ..motion_timeline import ranges,sample_timeline
from ..run_setup import write_json
from .transport_motion_signals import quaternion_path,signal_flags

SCHEMA='robofrontier.transport-source-edit.v1'

@dataclass(frozen=True)
class Settings:
    arm_rad:float=2e-4
    finger_m:float=2e-5
    object_m:float=2e-5
    object_rad:float=5e-4
    minimum_rattle_s:float=4.
    windows_s:tuple=(4.,8.,16.,32.,64.,128.,256.,512.)
    event_context_s:float=.3
    cut_context_s:float=.05


def raw_motion(x,dt,tolerances):
    x=np.asarray(x);result=np.zeros(len(x),bool)
    for seconds in (.25,2.):
        width=max(3,min(len(x),int(round(seconds/dt))|1))
        for axis,tol in enumerate(tolerances):
            span=maximum_filter1d(x[:,axis],width,mode='nearest')-minimum_filter1d(x[:,axis],width,mode='nearest')
            result|=span>tol
    return result


def stationary_oscillation(features,dt,tolerances,minimum_s=4.):
    """Find a stationary repeating envelope without an amplitude ceiling.

    A directed drift/continuous spin in any coordinate preserves the entity.
    Outliers, approach to the envelope and the first reversal remain visible.
    """
    x=np.asarray(features,dtype=float);n=len(x);tol=np.asarray(tolerances);empty=np.zeros(n,bool)
    if n*dt<minimum_s or n<12:return empty
    low,high=np.quantile(x,[.005,.995],axis=0);spread=high-low
    turn_index=None
    for axis in np.flatnonzero(spread>2*tol):
        y=uniform_filter1d(x[:,axis],max(3,int(round(.08/dt))|1),mode='nearest')
        delta=np.diff(y);nz=np.flatnonzero(abs(delta)>tol[axis]*.01)
        sign=np.sign(delta[nz]);turns=nz[1:][sign[1:]*sign[:-1]<0] if len(nz)>1 else np.array([],int)
        if len(turns)>=4:
            # Consecutive peak/trough midpoints cancel phase bias. First/last
            # window means alone mistake non-integer sine cycles for drift.
            centres=(y[turns[:-1]]+y[turns[1:]])/2
            clock=(turns[:-1]+turns[1:])*dt/2;clock-=clock.mean()
            slope=float(clock@(centres-centres.mean())/(clock@clock)) if clock@clock else 0.
            if abs(slope)*2>tol[axis]:return empty
            travel=float(np.abs(delta).sum())
            if travel>4*spread[axis] and turns[-1]-turns[0]>=n*.5:
                candidate=int(turns[0])+1
                turn_index=candidate if turn_index is None else min(turn_index,candidate)
        else:
            quarter=max(3,n//4)
            drift=abs(np.median(x[-quarter:,axis])-np.median(x[:quarter,axis]))/max(dt*(n-quarter),dt)*2
            if drift>tol[axis]:return empty
    if turn_index is None:return empty
    centre=(low+high)/2;radius=np.maximum(spread*.625,tol*3)
    mask=np.all(abs(x-centre)<=radius,axis=1)
    mask[:turn_index+max(2,int(round(.15/dt)))]=False
    return mask


def rattle_in_quiet(features,t,quiet,raw,tolerances,settings):
    result=np.zeros(len(t),bool);dt=float((t[-1]-t[0])/(len(t)-1))
    for start,stop in ranges(quiet):
        if (stop-start)*dt<settings.minimum_rattle_s or not raw[start:stop].any():continue
        for seconds in settings.windows_s:
            width=max(12,round(seconds/dt));stride=max(1,width//2)
            if width>stop-start:continue
            starts=list(range(start,stop-width+1,stride))
            if starts[-1]!=stop-width:starts.append(stop-width)
            for lo in starts:
                hi=lo+width
                if not raw[lo:hi].any():continue
                result[lo:hi]|=stationary_oscillation(features[lo:hi],dt,tolerances,settings.minimum_rattle_s)
    return result


def read_source(reader):
    n=reader.meta['frames'];q=np.empty((n,reader.meta['nq']),dtype='<f4');t=np.empty(n,dtype=float);offset=0
    for chunk in reader.meta['chunks']:
        path=reader.path/chunk['file']
        if digest(path)!=chunk['sha256']:raise ValueError('Motion chunk checksum mismatch')
        with np.load(path,allow_pickle=False)as z:
            count=chunk['frames'];states=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4')
            if states.shape!=(count,reader.meta['nq']):raise ValueError('Motion chunk shape mismatch')
            q[offset:offset+count]=states;t[offset:offset+count]=z['time_s'];offset+=count
    if offset!=n or not np.isfinite(q).all()or not np.isfinite(t).all()or np.any(np.diff(t)<=0):raise ValueError('Invalid source state/time stream')
    return t,q


def semantic_guards(trial,t,settings):
    keep=np.zeros(len(t),bool)
    def mark(stamp):
        lo=np.searchsorted(t,stamp-settings.event_context_s);hi=np.searchsorted(t,stamp+settings.event_context_s,side='right');keep[lo:hi]=True
    actions=json.loads((trial/'actions.json').read_text())
    for action in actions:
        mark(action['start_sim_s']);mark(action['end_sim_s'])
    previous=None
    with (trial/'process.jsonl').open()as stream:
        for line in stream:
            value=json.loads(line)
            if 'result'not in value:continue
            result=value['result'];state=json.dumps({k:result.get(k)for k in ('all_goals_met','task_success','matched_goals','assignments','violations')},sort_keys=True)
            if state!=previous:mark(value['sim_time_s']);previous=state
    for event in MotionReader(trial/'motion').events():mark(event['sim_time_s'])
    return keep


def classify(trial,reader,t,q,settings=Settings()):
    registry=json.loads((reader.path/'source/workstation.json').read_text());dt=1/reader.meta['sample_hz'];n=len(t)
    arm=np.zeros(n,bool);raw_all=np.zeros(n,bool);summaries=[];entities={}
    for joint in registry['joints']:
        address=joint['qpos_address'];kind=joint['type'];name=joint['id']
        robot=name.split('_')[0]
        if len(robot)==1 and robot.isalpha()and ('_joint'in name or '_finger_joint'in name):
            tol=settings.finger_m if '_finger_joint'in name else settings.arm_rad
            active,raw,_=signal_flags(q[:,address],dt,tol,.002 if '_finger_joint'in name else .01)
            arm|=active;raw_all|=raw
        else:entities.setdefault(joint['body'],[]).append(joint)
    keep=arm.copy();suppressed=np.zeros(n,bool)
    # Important: a command ID alone does not protect a stalled servo oscillation.
    # Real directed joint motion/opening and all command/result boundaries survive.
    for number,(name,joints)in enumerate(entities.items()):
        features=[];tolerances=[]
        for joint in joints:
            a=joint['qpos_address'];kind=joint['type']
            if kind==0:
                features.extend([q[:,a:a+3],quaternion_path(q[:,a+3:a+7])]);tolerances.extend([settings.object_m]*3+[settings.object_rad]*3)
            elif kind==1:
                features.append(quaternion_path(q[:,a:a+4]));tolerances.extend([settings.object_rad]*3)
            else:
                features.append(q[:,a:a+1]);tolerances.append(settings.object_rad if kind==3 else settings.object_m)
        x=np.concatenate(features,axis=1);raw=raw_motion(x,dt,tolerances);raw_all|=raw
        rattle=rattle_in_quiet(x,t,~arm,raw,tolerances,settings)
        active=raw&~rattle;keep|=active;suppressed|=raw&rattle
        summaries.append(dict(body=name,raw_frames=int(raw.sum()),repetitive_wait_frames=int((raw&rattle).sum())))
        if number%8==0:print('source-entities',number,'/',len(entities),flush=True)
    guards=semantic_guards(trial,t,settings);keep|=guards
    if settings.cut_context_s:
        keep=maximum_filter1d(keep,2*round(settings.cut_context_s/dt)+1,mode='constant')
    keep[0]=keep[-1]=True
    return keep,dict(robot_effective_frames=int(arm.sum()),protected_event_frames=int(guards.sum()),
        removed_repetitive_frames=int((suppressed&~keep).sum()),removed_static_frames=int((~raw_all&~keep).sum()),
        entities=summaries,force_interpretation='No measured force stream; effective motion and event/phase evidence only')


def build_edit(trial,output,settings=Settings()):
    trial=Path(trial).resolve();output=Path(output);reader=MotionReader(trial/'motion')
    if reader.meta['case_id'].split('-')[0]!='transport':raise ValueError('Transport recording required')
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);t,q=read_source(reader);keep,info=classify(trial,reader,t,q,settings)
    selected=sample_timeline(keep,reader.meta['sample_hz'],20,30)
    arrays=output/'selection.npz';np.savez_compressed(arrays,source_time_s=t,keep=keep,render_source_indices=selected)
    payload=dict(schema=SCHEMA,producer='transport',case_id=reader.meta['case_id'],source_recording=str(reader.path),
        source_stream_sha256=digest(reader.path/'stream.json'),source_scene_sha256=reader.meta['source_scene_sha256'],
        source_frames=len(t),source_hz=reader.meta['sample_hz'],selection_file=arrays.name,selection_sha256=digest(arrays),
        decision_resolution='Native original state indices; no rendered video input',settings=asdict(settings),
        algorithm_sha256=digest(__file__),signal_implementation_sha256=digest(Path(__file__).with_name('transport_motion_signals.py')),
        input_evidence_sha256={name:digest(trial/name)for name in ('actions.json','process.jsonl')},
        retained_frames=int(keep.sum()),removed_frames=int((~keep).sum()),output_frames=len(selected),
        output_fps=30,playback_speed=20,output_duration_s=len(selected)/30,
        keep_intervals=[dict(start_index=a,end_index_exclusive=b,start_source_s=float(t[a]),last_source_s=float(t[b-1]))for a,b in ranges(keep)],
        source_physics_modified=False,old_delivery_overwritten=False,shared_format_integration_pending=True,
        scope='Offline family edit only; source R13 remains immutable, R14 physical task pass is not inferred',analysis=info)
    write_json(output/'edit.json',payload);return payload


class TransportEdit:
    """Family adapter for the future shared multi-table player; no drawing."""
    def __init__(self,path):
        path=Path(path);self.meta=json.loads(path.read_text())
        if self.meta.get('schema')!=SCHEMA or self.meta.get('producer')!='transport':raise ValueError('Wrong Transport edit format')
        reader=MotionReader(self.meta['source_recording'])
        if self.meta['source_stream_sha256']!=digest(reader.path/'stream.json')or self.meta['source_scene_sha256']!=reader.meta['source_scene_sha256']:raise ValueError('Wrong source recording')
        if self.meta['source_frames']!=reader.meta['frames']or self.meta['source_hz']!=reader.meta['sample_hz']:raise ValueError('Wrong source frame/rate metadata')
        for name,checksum in self.meta['input_evidence_sha256'].items():
            if name not in ('actions.json','process.jsonl')or digest(reader.path.parent/name)!=checksum:raise ValueError('Transport event evidence changed')
        arrays=(path.parent/self.meta['selection_file']).resolve()
        if not arrays.is_relative_to(path.parent.resolve())or digest(arrays)!=self.meta['selection_sha256']:raise ValueError('Selection checksum/path mismatch')
        with np.load(arrays,allow_pickle=False)as z:self.times=z['source_time_s'];self.keep=z['keep']
        if len(self.times)!=reader.meta['frames']or len(self.keep)!=len(self.times):raise ValueError('Selection count mismatch')
        if self.keep.dtype!=np.bool_ or not self.keep[0]or not self.keep[-1]or not np.isfinite(self.times).all()or np.any(np.diff(self.times)<=0):raise ValueError('Invalid selection/source clock')
        self.indices=np.flatnonzero(self.keep)
    def at_video_time(self,seconds,speed=20.):
        if not np.isfinite(seconds)or seconds<0 or not np.isfinite(speed)or speed<=0:raise ValueError('Invalid display clock')
        position=int(seconds*speed*self.meta['source_hz']);ended=position>=len(self.indices)
        index=int(self.indices[min(position,len(self.indices)-1)])
        return dict(source_index=index,source_time_s=float(self.times[index]),ended=ended,
            effective_time_s=min(seconds*speed,len(self.indices)/self.meta['source_hz']))
