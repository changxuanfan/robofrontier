"""Read-only whole-scene motion detection for editing audited replay videos."""
from pathlib import Path
import json
import numpy as np
from robofrontier.motion_recording import MotionReader,digest

ALGORITHM_VERSION=1

class SceneMotionDetector:
    def __init__(self,joints,hz):
        angular=[];linear=[];quaternions=[];address=0
        for joint in joints:
            kind=joint['type']
            if kind==0:
                linear.extend(range(address,address+3));quaternions.append(list(range(address+3,address+7)));address+=7
            elif kind==1:
                quaternions.append(list(range(address,address+4)));address+=4
            elif kind==2:linear.append(address);address+=1
            elif kind==3:angular.append(address);address+=1
            else:raise ValueError('Unknown joint state type')
        self.nq=address;self.angular=np.asarray(angular,dtype=int);self.linear=np.asarray(linear,dtype=int);self.quaternions=np.asarray(quaternions,dtype=int).reshape(-1,4)
        self.lags=sorted({max(1,round(hz*.25)),max(1,round(hz*2))});self.history=max(self.lags)
        self.tail_q=np.empty((0,address),dtype=np.float32);self.tail_t=np.empty(0)
        self.thresholds=dict(angular_speed_rad_s=.001,linear_speed_m_s=.00005,window_angular_rad=.0001,window_linear_m=.00002,angular_noise_rad=.0000001,linear_noise_m=.0000001,window_s=[.25,2.])

    def above(self,a,b,linear,angular):
        count=len(a);result=np.zeros(count,dtype=bool)
        linear=np.broadcast_to(np.asarray(linear),(count,));angular=np.broadcast_to(np.asarray(angular),(count,))
        if len(self.linear):result|=np.any(abs(a[:,self.linear].astype(float)-b[:,self.linear])>linear[:,None],axis=1)
        if len(self.angular):result|=np.any(abs(a[:,self.angular].astype(float)-b[:,self.angular])>angular[:,None],axis=1)
        if len(self.quaternions):
            qa=a[:,self.quaternions].astype(float);qb=b[:,self.quaternions].astype(float)
            qa/=np.maximum(np.linalg.norm(qa,axis=2,keepdims=True),1e-15);qb/=np.maximum(np.linalg.norm(qb,axis=2,keepdims=True),1e-15)
            sign=np.where(np.sum(qa*qb,axis=2)<0,-1.,1.)
            chord=np.sum((qa-qb*sign[:,:,None])**2,axis=2)
            result|=np.any(chord>(2*np.sin(np.minimum(angular,np.pi)/4))[:,None]**2,axis=1)
        return result

    def process(self,times,states):
        times=np.asarray(times);states=np.asarray(states)
        if states.shape!=(len(times),self.nq) or not np.isfinite(states).all() or np.any(np.diff(times)<=0):raise ValueError('Invalid motion state block')
        if len(self.tail_t) and times[0]<=self.tail_t[-1]:raise ValueError('Non-monotonic motion blocks')
        q=np.concatenate((self.tail_q,states));t=np.r_[self.tail_t,times];indices=np.arange(len(self.tail_t),len(t));previous=np.maximum(indices-1,0);current=q[indices];dt=t[indices]-t[previous]
        fast=self.above(current,q[previous],dt*self.thresholds['linear_speed_m_s'],dt*self.thresholds['angular_speed_rad_s'])
        step=self.above(current,q[previous],self.thresholds['linear_noise_m'],self.thresholds['angular_noise_rad'])
        persistent=np.zeros(len(times),dtype=bool)
        for lag in self.lags:persistent|=self.above(current,q[np.maximum(indices-lag,0)],self.thresholds['window_linear_m'],self.thresholds['window_angular_rad'])
        active=fast|(step&persistent);active[dt==0]=False
        self.tail_q=q[-self.history:].copy();self.tail_t=t[-self.history:].copy()
        return active


def frame_intervals(mask):
    edges=np.diff(np.r_[False,np.asarray(mask,dtype=bool),False].astype(np.int8));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)-1
    return [[int(a),int(b)] for a,b in zip(starts,ends)]


def selected_video_frames(active,every,rendered_states):
    count=len(active);sample_indices=np.arange(0,count,every,dtype=np.int64)
    if not len(sample_indices) or sample_indices[-1]!=count-1:sample_indices=np.r_[sample_indices,count-1]
    if len(sample_indices)!=rendered_states:raise ValueError('Original encoded-frame/source-frame mapping differs')
    keep=np.zeros(rendered_states,dtype=bool);buckets=np.unique(np.flatnonzero(active)//every)
    for offset in (-1,0,1):keep[np.clip(buckets+offset,0,rendered_states-1)]=True
    keep[0]=True;keep[-1]=True
    if len(buckets) and not np.all(keep[buckets]):raise ValueError('A detected movement bucket was removed')
    return keep,sample_indices


def analyze(trial,video_manifest,output,progress=None):
    trial=Path(trial);reader=MotionReader(trial/'motion');registry=json.loads((reader.path/'source/workstation.json').read_text())
    if reader.meta['status']!='complete' or reader.meta['frames']!=video_manifest['source_frames']:raise ValueError('Require complete matching source recording')
    detector=SceneMotionDetector(registry['joints'],reader.meta['sample_hz'])
    if detector.nq!=reader.meta['nq']:raise ValueError('Joint state layout mismatch')
    count=reader.meta['frames'];times=np.empty(count);active=np.zeros(count,dtype=bool);offset=0;last=-np.inf
    for index,chunk in enumerate(reader.meta['chunks']):
        path=reader.path/chunk['file']
        if digest(path)!=chunk['sha256']:raise ValueError('Source motion checksum mismatch')
        with np.load(path,allow_pickle=False) as z:
            q=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4');t=z['time_s']
            if len(t)!=chunk['frames'] or not len(t) or t[0]<=last:raise ValueError('Source chunk shape/time mismatch')
            motion=detector.process(t,q);end=offset+len(t);times[offset:end]=t;active[offset:end]=motion;offset=end;last=float(t[-1])
        if progress is not None and index%100==0:progress(offset,count)
    if offset!=count:raise ValueError('Source frame count mismatch')
    appearance=0
    for event in reader.events():
        if event['event']=='rgba':active[min(count-1,int(np.searchsorted(times,event['sim_time_s'])))]=True;appearance+=1
    every=int(video_manifest['every']);keep,sample_indices=selected_video_frames(active,every,video_manifest['rendered_states'])
    intervals=frame_intervals(keep);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'motion-mask.npz',source_time_s=times,active=active,video_keep=keep,video_source_indices=sample_indices)
    mapping=[dict(first_frame=a,last_frame=b,source_start_s=float(times[sample_indices[a]]),source_end_s=float(times[sample_indices[b]])) for a,b in intervals]
    result=dict(kind='Whole-scene stationary-interval removal from existing recorded-state replay; no physics or trajectory change',algorithm_version=ALGORITHM_VERSION,source_trial=str(trial.resolve()),source_scene_sha256=reader.meta['source_scene_sha256'],source_frames=count,all_source_checksums_verified=True,thresholds=detector.thresholds,tracked_joint_counts=dict(angular=len(detector.angular),linear=len(detector.linear),quaternion=len(detector.quaternions)),appearance_changes_preserved=appearance,original_video_frames=len(keep),kept_video_frames=int(keep.sum()),removed_video_frames=int((~keep).sum()),removed_video_fraction=float((~keep).mean()),detected_source_motion_frames=int(active.sum()),all_detected_motion_buckets_preserved=True,initial_and_final_states_preserved=True,context='One encoded frame before/after each movement bucket, plus exact initial/final recorded states; microscopic numeric jitter ignored',fps=video_manifest['fps'],speedup=video_manifest['speedup'],source_intervals=mapping)
    (output/'motion-analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    terms=[f'between(n,{a},{b})' if a!=b else f'eq(n,{a})' for a,b in intervals]
    (output/'select-filter.txt').write_text("select='"+'+'.join(terms)+f"',setpts=N/({video_manifest['fps']}*TB)\n")
    return result
