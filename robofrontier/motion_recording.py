"""Private bounded motion recording shared by Oracle and policy worlds.

float32 quantization is declared. XOR + ZIP compression is bit-exact thereafter;
no motion threshold, command reconstruction, or physics replay is used.
"""
import hashlib
import json
import os
import shutil
import time
from pathlib import Path

import numpy as np
import mujoco
from .run_setup import write_json


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_source(source,destination,cache=None):
    """Portable local paths; content-addressed cache avoids duplicate asset bytes."""
    source=Path(source).resolve();destination=Path(destination);destination.mkdir()
    cache=Path(cache) if cache else destination.parent/'asset-cache';cache.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((source/'workstation.json').read_text())
    files=['scene.xml','workstation.json','task.json','environment.json']+[a['path'] for a in manifest['assets']]
    for name in files:
        original=(source/name).resolve()
        if not original.is_relative_to(source):raise ValueError('Source escapes workstation')
        checksum=digest(original);cached=cache/checksum
        if not cached.exists():
            temporary=cache/(checksum+f'.{os.getpid()}.tmp');shutil.copyfile(original,temporary)
            if digest(temporary)!=checksum:raise ValueError('Source changed while freezing')
            os.replace(temporary,cached)
        elif digest(cached)!=checksum:raise ValueError('Corrupted source cache')
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True)
        try:os.link(cached,target)
        except OSError:shutil.copyfile(cached,target)
    return manifest


class MotionWriter:
    def __init__(self,path,source,model,data,hz=60,chunk_seconds=2,cache=None):
        if not 0<hz<=1/float(model.opt.timestep):raise ValueError('Sampling rate outside physics rate')
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=False)
        frozen=freeze_source(source,self.path/'source',cache)
        if model.nq!=sum({0:7,1:4,2:1,3:1}[j['type']] for j in frozen['joints']):raise ValueError('Model state layout mismatch')
        if [model.joint(i).name for i in range(model.njnt)]!=[j['id'] for j in frozen['joints']]:raise ValueError('Model joint identities mismatch')
        self.model=model;self.geom_ids=[g['id'] for g in frozen['geometries']];self.interval=1/hz;self.next_sample=float(data.time);self.last=-np.inf
        self.limit=max(1,round(hz*chunk_seconds));self.states=[];self.times=[];self.wall=[];self.closed=False
        self.previous=np.asarray(data.qpos,dtype='<f4').view('<u4').copy();self.colors=model.geom_rgba.copy()
        self.events=(self.path/'events.jsonl').open('x')
        self.meta=dict(schema='robofrontier.motion.v1',purpose='Private visual reconstruction; not a physics restart checkpoint',case_id=frozen['case']['id'],source_scene_sha256=frozen['scene_sha256'],
            nq=model.nq,dtype='float32',sample_hz=hz,physics_timestep_s=float(model.opt.timestep),compression='float32 bitwise XOR deltas + ZIP deflate; no thresholds',
            status='recording',chunks=[],frames=0,write_wall_s=0.,max_sample_gap_s=0.,initial_rgba=model.geom_rgba.tolist(),
            quantization='float64 to float32 is lossy; compressed float32 states reconstruct exactly',created_unix_s=time.time())
        self.meta['engine']=dict(name='MuJoCo',version=mujoco.__version__,plugin_instances=int(model.nplugin))
        self._index();self.capture(data,force=True)

    def _index(self):write_json(self.path/'stream.json',self.meta)

    def event(self,sim_time,kind,**fields):
        self.events.write(json.dumps(dict(sim_time_s=float(sim_time),event=kind,**fields),allow_nan=False)+'\n');self.events.flush()

    def capture(self,data,force=False):
        if self.closed:raise RuntimeError('Recorder already closed')
        # Monitor/device updates precede this hook. Appearance changes use the
        # same physics clock even when they fall between position samples.
        changed=np.flatnonzero(np.any(self.model.geom_rgba!=self.colors,axis=1))
        if len(changed):
            self.event(data.time,'rgba',geoms={self.geom_ids[int(g)]:self.model.geom_rgba[g].tolist() for g in changed})
            self.colors[changed]=self.model.geom_rgba[changed]
        now=float(data.time)
        if not np.isfinite(now):raise ValueError('Nonfinite recording time')
        if now<self.last-1e-10:raise ValueError('Simulation time moved backwards')
        if now<=self.last+1e-10 or (not force and now+1e-10<self.next_sample):return False
        state=np.asarray(data.qpos,dtype='<f4').copy()
        if not np.isfinite(state).all():raise ValueError('Nonfinite recording state')
        if np.isfinite(self.last):self.meta['max_sample_gap_s']=max(self.meta['max_sample_gap_s'],now-self.last)
        self.states.append(state);self.times.append(now);self.wall.append(time.monotonic());self.last=now
        while self.next_sample<=now+1e-10:self.next_sample+=self.interval
        if len(self.states)>=self.limit:self.flush()
        return True

    def flush(self):
        if not self.states:return
        started=time.monotonic();q=np.asarray(self.states,dtype='<f4');bits=q.view('<u4')
        previous=np.vstack((self.previous[None,:],bits[:-1]));delta=np.bitwise_xor(bits,previous)
        name=f'chunk-{len(self.meta["chunks"]):06d}.npz';temporary=self.path/(name+'.tmp')
        with temporary.open('xb') as file:
            np.savez_compressed(file,base=self.previous,xor=delta,time_s=np.asarray(self.times,dtype='<f8'),wall_monotonic_s=np.asarray(self.wall,dtype='<f8'))
            file.flush();os.fsync(file.fileno())
        target=self.path/name;os.replace(temporary,target)
        self.meta['chunks'].append(dict(file=name,sha256=digest(target),frames=len(q),start_sim_s=self.times[0],end_sim_s=self.times[-1],bytes=target.stat().st_size))
        self.meta['frames']+=len(q);self.meta['write_wall_s']+=time.monotonic()-started
        self.previous=bits[-1].copy();self.states.clear();self.times.clear();self.wall.clear();self._index()

    def close(self,data=None,status='complete'):
        if self.closed:return self.meta
        if data is not None:self.capture(data,force=True)
        self.flush();self.meta['status']=status;self.meta['closed_unix_s']=time.time();self._index();self.events.close();self.closed=True
        return self.meta


class MotionReader:
    def __init__(self,path):
        self.path=Path(path);self.meta=json.loads((self.path/'stream.json').read_text())
        if self.meta['schema']!='robofrontier.motion.v1':raise ValueError('Unsupported recording')
        source=json.loads((self.path/'source/workstation.json').read_text())
        if self.meta['source_scene_sha256']!=source['scene_sha256'] or self.meta['case_id']!=source['case']['id'] or digest(self.path/'source/scene.xml')!=source['scene_sha256']:
            raise ValueError('Recording source identity/checksum mismatch')

    def frames(self):
        last=-np.inf
        for chunk in self.meta['chunks']:
            path=self.path/chunk['file']
            if digest(path)!=chunk['sha256']:raise ValueError('Recording chunk checksum mismatch')
            with np.load(path,allow_pickle=False) as z:
                bits=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base'])
                q=bits.view('<f4');times=z['time_s']
                if q.shape!=(chunk['frames'],self.meta['nq']) or len(times)!=len(q):raise ValueError('Invalid recording chunk shape')
                for t,state in zip(times,q):
                    if not np.isfinite(t) or t<=last or not np.isfinite(state).all():raise ValueError('Invalid recording timeline/state')
                    last=float(t);yield last,state.copy()

    def events(self):
        with (self.path/'events.jsonl').open() as file:
            for line in file:
                try:yield json.loads(line)
                except json.JSONDecodeError:
                    if self.meta['status']=='complete':raise
                    if file.read().strip():raise ValueError('Corrupted event record before the interrupted tail')
                    break # Interrupted trailing write, never fabricate its event.

    def selected_frames(self,indices):
        """Decode selected original states without iterating over omitted frames.

        Each used chunk is checksum-verified, then reconstructed bit-exactly.
        This is an offline rendering optimization, not a new physics trajectory.
        """
        indices=np.asarray(indices,dtype=np.int64)
        if indices.ndim!=1 or not len(indices) or indices[0]<0 or indices[-1]>=self.meta['frames'] or np.any(np.diff(indices)<=0):raise ValueError('Selected source indices must be sorted, unique and in range')
        offset=0;cursor=0;last=-np.inf
        for chunk in self.meta['chunks']:
            end=offset+chunk['frames'];stop=int(np.searchsorted(indices,end))
            if stop>cursor:
                path=self.path/chunk['file']
                if digest(path)!=chunk['sha256']:raise ValueError('Recording chunk checksum mismatch')
                with np.load(path,allow_pickle=False)as z:
                    q=np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'],axis=0),z['base']).view('<f4');times=z['time_s']
                    if q.shape!=(chunk['frames'],self.meta['nq']) or len(times)!=len(q):raise ValueError('Invalid recording chunk shape')
                    for index in indices[cursor:stop]:
                        local=int(index)-offset;t=float(times[local]);state=q[local]
                        if not np.isfinite(t) or t<=last or not np.isfinite(state).all():raise ValueError('Invalid selected recording state')
                        last=t;yield int(index),t,state.copy()
            cursor=stop;offset=end
            if cursor==len(indices):break

    @classmethod
    def recover(cls,path):
        """Reindex finalized chunks after interruption; never use a .tmp tail."""
        path=Path(path);meta=json.loads((path/'stream.json').read_text());chunks=[];last=-np.inf
        for index,file in enumerate(sorted(path.glob('chunk-*.npz'))):
            if file.name!=f'chunk-{index:06d}.npz':raise ValueError('Recording contains a missing chunk')
            with np.load(file,allow_pickle=False) as z:
                times=z['time_s'];delta=z['xor']
                if not len(times) or delta.shape!=(len(times),meta['nq']) or np.any(np.diff(times)<=0) or times[0]<=last:raise ValueError('Invalid recovered chunk')
                last=float(times[-1]);chunks.append(dict(file=file.name,sha256=digest(file),frames=len(times),start_sim_s=float(times[0]),end_sim_s=last,bytes=file.stat().st_size))
        meta.update(chunks=chunks,frames=sum(c['frames'] for c in chunks),status='interrupted_recovered',recovered_unix_s=time.time())
        write_json(path/'stream.json',meta);return cls(path)
