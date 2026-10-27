"""Scene-independent edit decisions over immutable recorded motion.

Intervals are half-open source-state indices. Never splice motion into a new
physics recording: the jumps are video cuts, not executable robot commands.
"""
import json
from pathlib import Path
import numpy as np


def register_timeline(path,recording):
    """Associate an edit beside the trial, outside the original motion stream."""
    from .motion_recording import digest
    from .run_setup import write_json
    path=Path(path).resolve()
    write_json(Path(recording).parent/'motion-edit.json',dict(schema='robofrontier.motion-edit-reference.v1',timeline=str(path),timeline_sha256=digest(path)))


def resolve_timeline(recording,explicit=None):
    if explicit is not None:return Path(explicit)
    pointer=Path(recording).parent/'motion-edit.json'
    if not pointer.exists():return None
    from .motion_recording import digest
    data=json.loads(pointer.read_text());path=Path(data['timeline'])
    if data.get('schema')!='robofrontier.motion-edit-reference.v1' or digest(path)!=data['timeline_sha256']:raise ValueError('Default timeline reference checksum mismatch')
    return path


def ranges(mask):
    edges=np.diff(np.r_[False,mask,False].astype(np.int8))
    return list(zip(np.flatnonzero(edges==1).tolist(),np.flatnonzero(edges==-1).tolist()))


def lift_video_mask(source_indices, video_keep, source_count):
    """Lift the reviewed video's exact Voronoi state cells to the 60Hz clock.

    Each retained video cell conservatively retains every original state it
    represents. This reproduces reviewed cuts without another motion heuristic.
    """
    indices=np.asarray(source_indices,dtype=np.int64);keep=np.asarray(video_keep,dtype=bool)
    if len(indices)!=len(keep) or not len(indices) or indices[0]!=0 or indices[-1]!=source_count-1 or np.any(np.diff(indices)<=0):
        raise ValueError('Video/source frame mapping is incomplete or unordered')
    edges=np.r_[0,(indices[1:]+indices[:-1])//2+1,source_count]
    return np.repeat(keep,np.diff(edges))


def sample_timeline(keep, source_hz=60., speed=20., fps=30.):
    """Sample the concatenated retained clock, without resetting at each cut.

    Exact source states only; initial/final frames are always included. Rounding
    and the final-state insert contribute at most two output frames of duration.
    """
    rates=np.asarray([source_hz,speed,fps],dtype=float)
    if not np.isfinite(rates).all() or np.any(rates<=0):raise ValueError('Rates must be positive and finite')
    retained=np.flatnonzero(keep)
    if not len(retained):raise ValueError('Timeline retains no states')
    offsets=np.floor(np.arange(0,len(retained),speed*source_hz/fps)).astype(np.int64)
    selected=retained[np.unique(np.minimum(offsets,len(retained)-1))]
    return np.unique(np.r_[selected,retained[-1]])


def write_timeline(path, recording, source_times, source_indices, video_keep, evidence):
    """Small sidecar + source reference; original qpos chunks remain unchanged."""
    from .motion_recording import MotionReader,digest
    path=Path(path);reader=MotionReader(recording);times=np.asarray(source_times,dtype=np.float64)
    if len(times)!=reader.meta['frames'] or np.any(np.diff(times)<=0):raise ValueError('Source time count/order mismatch')
    keep=lift_video_mask(source_indices,video_keep,len(times))
    selected=sample_timeline(keep,reader.meta['sample_hz'])
    arrays=path.with_suffix('.npz');np.savez_compressed(arrays,source_time_s=times,keep=keep,render_source_indices=selected,render_source_time_s=times[selected],video_time_s=np.arange(len(selected))/30.)
    payload=dict(schema='robofrontier.motion-edit.v1',case_id=reader.meta['case_id'],recording=str(Path(recording).resolve()),source_stream_sha256=digest(Path(recording)/'stream.json'),source_scene_sha256=reader.meta['source_scene_sha256'],source_frames=len(times),source_hz=reader.meta['sample_hz'],arrays=arrays.name,arrays_sha256=digest(arrays),kept_source_frames=int(keep.sum()),removed_source_frames=int((~keep).sum()),keep_intervals=[dict(start_index=a,end_index_exclusive=b,start_sim_s=float(times[a]),last_sim_s=float(times[b-1]))for a,b in ranges(keep)],default_speed=20.,default_fps=30.,render_frames=len(selected),duration_s=len(selected)/30.,evidence=evidence,scope='Presentation cuts only; no resimulation, interpolation or modified physical recording',reuse='Apply identical source indices before either simple or factory rendering. Background pixels, materials, and rigid workstation placement do not alter the edit.')
    path.write_text(json.dumps(payload,indent=2)+'\n');return payload


class MotionTimeline:
    def __init__(self,path,reader):
        from .motion_recording import digest
        path=Path(path);self.meta=json.loads(path.read_text())
        if self.meta['schema']!='robofrontier.motion-edit.v1':raise ValueError('Unknown timeline schema')
        if self.meta['case_id']!=reader.meta['case_id'] or self.meta['source_scene_sha256']!=reader.meta['source_scene_sha256'] or self.meta['source_stream_sha256']!=digest(reader.path/'stream.json'):raise ValueError('Timeline belongs to another recording')
        arrays=(path.parent/self.meta['arrays']).resolve()
        if not arrays.is_relative_to(path.parent.resolve()) or digest(arrays)!=self.meta['arrays_sha256']:raise ValueError('Timeline arrays checksum/path mismatch')
        with np.load(arrays,allow_pickle=False)as z:self.times=z['source_time_s'];self.keep=z['keep']
        if len(self.times)!=reader.meta['frames'] or len(self.keep)!=len(self.times):raise ValueError('Timeline frame count differs from recording')
    def select(self,speed=20.,fps=30.):return sample_timeline(self.keep,self.meta['source_hz'],speed,fps)
