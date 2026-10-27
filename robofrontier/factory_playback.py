"""Independent source clocks over one visual factory; no physical integration."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from .motion_recording import MotionReader, digest
from .motion_timeline import sample_timeline
from .workstation import Workstation
from .factory_workstations import FactoryPlayback


def current_scene():
    from .task_families import ROOT
    pointer=ROOT/'outputs/robo-factory/current.json'
    if pointer.exists():
        path=Path(json.loads(pointer.read_text())['scene'])
        return (ROOT/path).resolve()
    return ROOT/'outputs/robo-factory/scene/scene.xml'


class StationTrack:
    """One exact-state cursor, with producer-specific edit validation inside."""
    def __init__(self, spec, speed, fps=30, loop=True):
        self.spec, self.speed, self.fps, self.loop = spec, speed, fps, loop
        if not np.isfinite([speed, fps]).all() or min(speed, fps)<=0:
            raise ValueError('Positive finite presentation cadence required')
        self.edit_path = Path(spec['edit'])
        if digest(self.edit_path) != spec['edit_sha256']:
            raise ValueError('Released edit changed')
        self.edit = json.loads(self.edit_path.read_text())
        self.reader = MotionReader(spec['recording'])
        self.source = Workstation(spec['directory'])
        if (self.reader.meta['case_id'] != spec['id'] or self.edit['case_id'] != spec['id']
                or self.reader.meta['source_scene_sha256'] != self.source.manifest['scene_sha256']
                or self.edit['source_scene_sha256'] != self.source.manifest['scene_sha256']):
            raise ValueError('Station source/edit identity differs')
        self._poses = None
        if self.edit.get('schema')=='robofrontier.station-edit.v2':
            from .station_edit import StationEdit
            self.indices=StationEdit(self.edit_path,self.reader).select(speed,fps)
        elif 'derived_track' in spec:
            self._load_relay(Path(spec['derived_track']))
        elif self.edit.get('producer') == 'transport':
            from .suite.transport_motion_edit import TransportEdit
            edit = TransportEdit(self.edit_path)
            self.indices = sample_timeline(edit.keep, self.reader.meta['sample_hz'], speed, fps)
        elif 'dial_profile' in self.edit:
            from .suite.dial_motion_edit import DialReplayTrack
            with DialReplayTrack(spec['recording'], self.edit_path, fps, speed) as track:
                self.indices = track.indices.copy()
        else:
            from .replay_edit import ReplayEdit
            edit = ReplayEdit(self.edit_path, self.reader)
            if edit.exact and (fps != edit.fps or speed != edit.speed):
                raise ValueError('Exact reviewed frames require their declared cadence')
            self.indices = edit.select(speed, fps)
        if not len(self.indices):raise ValueError('Empty station track')
        self.frame_count = len(self.indices)
        self.initial_rgba = np.asarray(self.reader.meta['initial_rgba'])
        self._event_list = [e for e in self.reader.events() if e['event']=='rgba']
        self._global_frame = -1
        self._reset()

    def _load_relay(self, folder):
        if self.edit.get('schema') not in ('robofrontier.relay-motion-edit.v1','robofrontier.motion-edit.v1'):
            raise ValueError('Unsupported Relay derived track')
        for name, key in [('track.npz','track_sha256'), ('events.jsonl','events_sha256'),
                          ('initial-rgba.json','initial_rgba_sha256')]:
            if digest(folder/name) != self.edit[key]:raise ValueError('Relay track data changed: '+name)
        from .relay_motion_edit import video_rows
        with np.load(folder/'track.npz', allow_pickle=False) as z:
            rows, _ = video_rows(z['edited_time_s'], z['duration_s'], self.fps, self.speed)
            middle = rows[1:-1]
            self._poses = np.concatenate([z['context_qpos'][:1], z['qpos'][middle], z['context_qpos'][1:]])
            self._times = np.r_[z['context_source_time_s'][0], z['source_time_s'][middle], z['context_source_time_s'][1]]
            # Context rows bind the original initial and completion-prefix state.
            end = int(np.searchsorted([c['end_sim_s'] for c in self.reader.meta['chunks']], self._times[-1]))
            offset = sum(c['frames'] for c in self.reader.meta['chunks'][:end])
            chunk = self.reader.meta['chunks'][end]
            if digest(self.reader.path/chunk['file']) != chunk['sha256']:
                raise ValueError('Relay completion source chunk changed')
            with np.load(self.reader.path/chunk['file'], allow_pickle=False) as terminal:
                last = offset+int(np.searchsorted(terminal['time_s'], self._times[-1]))
            self.indices = np.r_[0, z['source_frame'][middle], last].astype(np.int64)

    def _reset(self):
        if hasattr(self, '_frames'):self._frames.close()
        self._frames = self.reader.selected_frames(np.unique(self.indices))
        self._source_index = -1
        self._sample = None
        self._local_frame = -1
        self._rgba = {}
        self._event_cursor = 0
        self._cycle = None

    def close(self):
        if hasattr(self, '_frames'):self._frames.close()

    def at(self, global_frame):
        if type(global_frame) is not int or global_frame<0:
            raise ValueError('Nonnegative integer output frame required')
        cycle, local = divmod(global_frame, self.frame_count) if self.loop else (0,min(global_frame,self.frame_count-1))
        reset = self._cycle != cycle or global_frame<self._global_frame
        if reset:self._reset()
        self._cycle, self._global_frame = cycle, global_frame
        wanted = int(self.indices[local])
        if self._poses is None:
            if local != self._local_frame+1:
                self._frames.close()
                self._frames = self.reader.selected_frames(np.unique(self.indices[local:]))
                self._source_index = -1
            while self._source_index < wanted:
                self._sample = next(self._frames)
                self._source_index = self._sample[0]
            index, stamp, qpos = self._sample
        else:
            index, stamp, qpos = wanted, float(self._times[local]), self._poses[local]
        self._local_frame = local
        while self._event_cursor<len(self._event_list) and self._event_list[self._event_cursor]['sim_time_s']<=stamp+1e-9:
            self._rgba.update(self._event_list[self._event_cursor]['geoms'])
            self._event_cursor+=1
        return dict(case_id=self.spec['id'], source_frame=index, source_time_s=stamp,
                    qpos=qpos.copy(), rgba=deepcopy(self._rgba), appearance_reset=reset,
                    cycle=cycle, local_frame=local, output_frame=global_frame,
                    completed=not self.loop and global_frame>=self.frame_count,
                    edit_sha256=self.spec['edit_sha256'])


class FactoryTimeline:
    def __init__(self, scene, release, families=None, loop=True, model=None, data=None):
        self.play = FactoryPlayback(scene, model, data)
        self.tracks = []
        self.fps = release['presentation']['fps']
        for name, family in release['families'].items():
            if families is not None and name not in families:continue
            for spec in family['oracle']:
                track = StationTrack(spec, family['speedup'], self.fps, loop)
                self.play.configure_workstation(spec['id'], track.source,
                    # This is the dedicated FK display model, never an executor.
                    # Apply once to its original R13 geometry; the composition
                    # already has this size, so copying it does not halve twice.
                    table_opening_scale=spec.get('table_opening_scale',1.))
                self.tracks.append(track)
        if not self.tracks:raise ValueError('Select at least one station')

    def at(self, frame):
        rows = []
        for track in self.tracks:
            sample = track.at(frame)
            if sample['appearance_reset']:
                self.play.appearance(track.spec['id'],
                    {g['id']:track.initial_rgba[g['source_index']] for g in track.source.manifest['geometries']})
            self.play.appearance(track.spec['id'], sample['rgba'])
            self.play.apply_arrays(track.spec['id'],track.source,sample['qpos'],
                                   sample['source_time_s'],commit=False)
            rows.append({k:v for k,v in sample.items() if k not in ('qpos','rgba')})
        self.play.commit_frame(frame/self.fps)
        return dict(output_frame=frame,presentation_time_s=frame/self.fps,stations=rows)

    def close(self):
        for track in self.tracks:track.close()
