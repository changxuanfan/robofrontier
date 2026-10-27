"""Source-native presentation cuts, with motion and semantic event protection."""
import json
from pathlib import Path
import re

import numpy as np

from .motion_recording import MotionReader, digest
from .motion_timeline import sample_timeline
from .run_setup import write_json

SCHEMA = 'robofrontier.station-edit.v2'


def _rotation_path(quats):
    q=np.asarray(quats,dtype=float)
    q=q/np.maximum(np.linalg.norm(q,axis=-1,keepdims=True),1e-15)
    a,b=q[:-1],q[1:]
    w=a[...,0]*b[...,0]+np.sum(a[...,1:]*b[...,1:],axis=-1)
    v=a[...,0,None]*b[...,1:]-b[...,0,None]*a[...,1:]-np.cross(a[...,1:],b[...,1:])
    v=np.where((w<0)[...,None],-v,v);w=np.abs(w)
    norm=np.linalg.norm(v,axis=-1)
    angle=2*np.arctan2(norm,w)
    vectors=v*(angle/np.maximum(norm,1e-15))[...,None]
    excursion=2*np.arccos(np.clip(np.min(np.abs(np.sum(q*q[:1],axis=-1)),axis=0),0,1))
    return np.sum(angle,axis=0),np.linalg.norm(vectors.sum(axis=0),axis=-1),excursion


def _envelope_drift(features,tolerance,dt):
    """Peak/trough centres separate slow progress from a periodic excursion."""
    from scipy.ndimage import uniform_filter1d
    x=np.asarray(features)
    if x.ndim==1:x=x[:,None]
    for axis in range(x.shape[1]):
        y=uniform_filter1d(x[:,axis],3,mode='nearest')
        if np.ptp(y)<=2*tolerance:continue
        delta=np.diff(y);nz=np.flatnonzero(abs(delta)>tolerance*.01)
        sign=np.sign(delta[nz]);turns=nz[1:][sign[1:]*sign[:-1]<0]
        if len(turns)<4:continue
        centres=(y[turns[:-1]]+y[turns[1:]])/2
        clock=(turns[:-1]+turns[1:])*dt/2;clock-=clock.mean()
        slope=float(clock@(centres-centres.mean())/(clock@clock)) if clock@clock else 0.
        if abs(slope)*2>tolerance:return True
    return False


def classify_window(qpos,joints,dt=.1):
    """Keep directed motion; identify quiet/repeated motion using physical units."""
    q=np.asarray(qpos,dtype=float)
    if len(q)<3 or not np.isfinite(q).all():return 'keep'
    passive=[]
    for joint in joints:
        name=joint['id'];a=joint['qpos_address'];kind=joint['type']
        if re.fullmatch(r'[A-Z]_(?:joint[1-7]|finger_joint[12])',name):
            # Robot movement includes slow directed approaches and gripper work.
            v=q[:,a];span=np.ptp(v);path=np.abs(np.diff(v)).sum();net=abs(v[-1]-v[0])
            limit=.0002 if 'finger' in name else .001
            if span>limit or (net>1e-6 and net/max(path,1e-15)>.65):return 'keep'
        elif kind==0:
            pos=q[:,a:a+3];steps=np.linalg.norm(np.diff(pos,axis=0),axis=1)
            path=steps.sum();net=np.linalg.norm(pos[-1]-pos[0]);span=np.linalg.norm(np.ptp(pos,axis=0))
            if span>.0002 and _envelope_drift(pos,2e-5,dt):return 'keep'
            if net>1e-5 and net/max(path,1e-15)>.65:return 'keep'
            if span>.0002:passive.append(path/max(span,1e-15)>=4 and net/max(span,1e-15)<.25)
            angles,rotation_net,rotation_range=_rotation_path(q[:,None,a+3:a+7]);angles=float(angles[0]);rotation_net=float(rotation_net[0]);rotation_range=float(rotation_range[0])
            if rotation_range>.001:
                from .suite.transport_motion_signals import quaternion_path
                if _envelope_drift(quaternion_path(q[:,a+3:a+7]),5e-4,dt):return 'keep'
            if rotation_net>.001 and rotation_net/max(angles,1e-15)>.65:return 'keep'
            if rotation_range>.001:passive.append(angles/max(rotation_range,1e-15)>=4 and rotation_net/max(angles,1e-15)<.2)
        elif kind in (2,3):
            v=q[:,a];path=np.abs(np.diff(v)).sum();net=abs(v[-1]-v[0]);span=np.ptp(v)
            if _envelope_drift(v,2e-5 if kind==2 else 5e-4,dt):return 'keep'
            if net>1e-6 and net/max(path,1e-15)>.65:return 'keep'
            if span>(.0002 if kind==2 else .001):passive.append(path/max(span,1e-15)>=4 and net/max(span,1e-15)<.25)
        elif kind==1:
            angles,net,span=_rotation_path(q[:,None,a:a+4]);angles=float(angles[0]);net=float(net[0]);span=float(span[0])
            if span>.001:
                from .suite.transport_motion_signals import quaternion_path
                if _envelope_drift(quaternion_path(q[:,a:a+4]),5e-4,dt):return 'keep'
            if net>.001 and net/max(angles,1e-15)>.65:return 'keep'
            if span>.001:passive.append(angles/max(span,1e-15)>=4 and net/max(span,1e-15)<.2)
    return 'repeated_passive' if passive and all(passive) else 'keep' if passive else 'quiet'


def _base_keep(spec,reader,times):
    edit=json.loads(Path(spec['edit']).read_text());count=len(times)
    if digest(spec['edit'])!=spec['edit_sha256']:raise ValueError('Previous edit changed')
    keep=np.zeros(count,bool)
    if 'kept_intervals_source_s' in edit:
        for first,last in edit['kept_intervals_source_s']:keep|=(times>=first)&(times<last)
        cutoff=edit['original_completion_cutoff_s']
        end=int(np.searchsorted(times,cutoff+1e-9,side='right'))-1
    elif edit.get('schema')==SCHEMA:
        original=StationEdit(spec['edit'],reader);keep=original.keep.copy();end=original.meta['source_end_frame']
    elif edit.get('producer')=='transport':
        from .suite.transport_motion_edit import TransportEdit
        original=TransportEdit(spec['edit']);keep=original.keep.copy();end=count-1
    elif 'dial_profile' in edit:
        for span in edit['retained_spans']:keep[span['first_frame']:span['stop_frame']]=True
        end=count-1
    elif isinstance(edit.get('render_frames'),list):
        from .motion_edit import MotionEdit
        original=MotionEdit(spec['edit'],reader)
        for span in original.data['keep_source_ranges']:keep[span['first_source_frame']:span['last_source_frame']+1]=True
        end=count-1
    else:raise ValueError('Unknown original edit structure')
    keep[end+1:]=False;keep[[0,end]]=True
    return keep,end


def build(spec,output):
    """Stream raw chunks in bounded 32-second windows; never integrate physics."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    reader=MotionReader(spec['recording']);joints=json.loads((Path(spec['directory'])/'workstation.json').read_text())['joints']
    times=np.empty(reader.meta['frames'],dtype=float)
    candidates=[];buffer=[];offset=0;stride=max(1,round(reader.meta['sample_hz']/10))
    window=max(8,round(32*reader.meta['sample_hz']/stride))
    def inspect(rows):
        ids=np.array([r[0] for r in rows]);states=np.array([r[1] for r in rows])
        for seconds in (2,8,32):
            step=max(3,round(seconds*reader.meta['sample_hz']/stride))
            for start in range(0,len(rows)-step+1,step):
                label=classify_window(states[start:start+step],joints,stride/reader.meta['sample_hz'])
                if label=='quiet' or label=='repeated_passive' and seconds>=8:
                    candidates.append((int(ids[start]),int(ids[start+step-1])+1,label))
    for chunk in reader.meta['chunks']:
        file=reader.path/chunk['file']
        if digest(file)!=chunk['sha256']:raise ValueError('Raw motion chunk changed')
        with np.load(file,allow_pickle=False) as z:
            states=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4')
            count=len(states);times[offset:offset+count]=z['time_s']
            for local in np.flatnonzero((np.arange(count)+offset)%stride==0):
                buffer.append((offset+int(local),states[local].copy()))
                if len(buffer)>=window:inspect(buffer);buffer=[]
            offset+=count
    if buffer:inspect(buffer)
    if offset!=len(times):raise ValueError('Incomplete raw source')
    keep,end=_base_keep(spec,reader,times);before=keep.copy()
    for first,last,label in candidates:keep[first:last]=False
    guards=[]
    for event in reader.events():
        if event['event']=='rgba' or any(word in event['event'] for word in ('failed','drop','recover','initial','trial_end','violation')):
            guards.append(float(event['sim_time_s']))
    evidence={}
    for name in ('actions.json','process.jsonl'):
        path=reader.path.parent/name
        if not path.is_file():continue
        evidence[name]=digest(path)
        if name=='actions.json':
            for action in json.loads(path.read_text()):
                guards.extend(float(action[k]) for k in ('start_sim_s','end_sim_s') if k in action)
        else:
            previous=None
            for line in path.open():
                row=json.loads(line);result=row.get('result')
                if result is None:continue
                signature=json.dumps({k:result.get(k)for k in ('matched_goals','assignments','task_success','violations','all_goals_met','won','best')},sort_keys=True)
                if signature!=previous:guards.append(float(row['sim_time_s']));previous=signature
    for stamp in guards:
        first,last=np.searchsorted(times,[stamp-1,stamp+1],side='left');keep[first:min(last,end+1)]=True
    keep[end+1:]=False;keep[[0,end]]=True
    np.savez_compressed(output/'selection.npz',source_time_s=times,keep=keep)
    result=dict(schema=SCHEMA,case_id=spec['id'],source_scene_sha256=reader.meta['source_scene_sha256'],
        source_stream_sha256=digest(reader.path/'stream.json'),events_sha256=digest(reader.path/'events.jsonl'),
        source_frames=len(times),source_hz=reader.meta['sample_hz'],source_end_frame=end,
        previous_edit=dict(path=spec['edit'],sha256=spec['edit_sha256']),input_evidence_sha256=evidence,
        selection_file='selection.npz',selection_sha256=digest(output/'selection.npz'),
        retained_frames=int(keep.sum()),removed_frames=int((~keep[:end+1]).sum()),
        additional_removed_frames=int((before&~keep).sum()),restored_event_context_frames=int((keep&~before).sum()),
        candidate_windows=len(candidates),guard_events=len(guards),classification_hz=reader.meta['sample_hz']/stride,
        window_seconds=[2,8,32],physics_steps=0,raw_data_changed=False,
        evidence_scope='Presentation review at 10Hz plus exact source/event bindings; force is not measured. Original physics and outcomes remain authoritative.')
    write_json(output/'edit.json',result)
    return result


class StationEdit:
    def __init__(self,path,reader):
        path=Path(path);self.meta=json.loads(path.read_text())
        if self.meta['schema']!=SCHEMA:raise ValueError('Unknown station edit schema')
        if (self.meta['case_id']!=reader.meta['case_id'] or self.meta['source_scene_sha256']!=reader.meta['source_scene_sha256']
                or self.meta['source_stream_sha256']!=digest(reader.path/'stream.json')
                or self.meta['events_sha256']!=digest(reader.path/'events.jsonl')):raise ValueError('Station edit source differs')
        selection=path.parent/self.meta['selection_file']
        if selection.resolve().parent!=path.parent.resolve() or digest(selection)!=self.meta['selection_sha256']:raise ValueError('Station edit selection differs')
        for name,expected in self.meta['input_evidence_sha256'].items():
            if name not in ('actions.json','process.jsonl') or digest(reader.path.parent/name)!=expected:raise ValueError('Station semantic evidence differs')
        with np.load(selection,allow_pickle=False)as z:self.times=z['source_time_s'];self.keep=z['keep']
        end=self.meta['source_end_frame']
        if (self.keep.dtype!=np.bool_ or self.keep.shape!=(reader.meta['frames'],) or self.times.shape!=self.keep.shape
                or not np.isfinite(self.times).all() or np.any(np.diff(self.times)<=0)
                or not self.keep[0] or not self.keep[end] or self.keep[end+1:].any()):raise ValueError('Invalid source selection clock')

    def select(self,speed,fps):return sample_timeline(self.keep,self.meta['source_hz'],speed,fps)
