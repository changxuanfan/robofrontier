"""Source-bound idle edits shared by every offline background and camera.

The original 60 Hz recording stays immutable. An edit is an index of retained
source frames, not a new physical episode or a restart checkpoint.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import re

import mujoco
import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d

from .motion_recording import MotionReader


def checksum(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def playback_speed(case_id, requested=None, other_default=1.):
    """Dial presentations default to 10x; other families keep caller defaults."""
    value = (10. if case_id.startswith('dial-') else other_default) if requested is None else requested
    if not np.isfinite(value) or value <= 0:
        raise ValueError('Positive finite playback speed required')
    return float(value)


@dataclass(frozen=True)
class IdleSettings:
    window_s: float = 1.0
    minimum_idle_s: float = 2.0
    edge_padding_s: float = .2
    feedback_padding_s: float = .25
    arm_rotation_deg: float = .3
    gripper_translation_m: float = .00025
    object_translation_m: float = .001
    object_rotation_deg: float = .75

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Idle settings must be finite and positive')


def ranges(mask):
    """Half-open intervals of true indices."""
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def motion_features(model, qpos, settings):
    """Physical-unit feature groups; quaternion signs cannot fabricate motion."""
    groups = []
    for joint in range(model.njnt):
        start, kind = int(model.jnt_qposadr[joint]), int(model.jnt_type[joint])
        name = model.joint(joint).name or f'joint-{joint}'
        arm = bool(re.fullmatch(r'.+_joint\d+', name))
        finger = 'finger_joint' in name
        if kind == mujoco.mjtJoint.mjJNT_FREE:
            groups.append((name + '/translation', qpos[:, start:start+3], settings.object_translation_m))
            rotation = qpos[:, start+3:start+7].copy()
        elif kind == mujoco.mjtJoint.mjJNT_BALL:
            rotation = qpos[:, start:start+4].copy()
        else:
            limit = (settings.gripper_translation_m if finger else settings.object_translation_m) if kind == mujoco.mjtJoint.mjJNT_SLIDE else np.deg2rad(settings.arm_rotation_deg if arm else settings.object_rotation_deg)
            groups.append((name, qpos[:, start:start+1], limit))
            continue
        norms = np.linalg.norm(rotation, axis=1)
        if np.any(norms < .5):
            raise ValueError('Invalid recorded quaternion')
        rotation /= norms[:, None]
        flips = np.r_[1., np.cumprod(np.where(np.sum(rotation[1:]*rotation[:-1], axis=1) < 0, -1., 1.))]
        rotation *= flips[:, None]
        # Quaternion chord: ||q1-q2|| = 2 sin(theta/4).
        groups.append((name + '/rotation', rotation, 2*np.sin(np.deg2rad(settings.object_rotation_deg)/4)))
    return groups


def idle_mask(model, times, qpos, settings=IdleSettings(), sample_hz=60.):
    """Remove only sustained, bounded micro-motion, retaining slow real drift.

    A one-second excursion window identifies candidates. A second whole-run
    excursion check prevents a slow real movement being mistaken for jitter.
    Translation/rotation bounds use the norm of the coordinate bounding box,
    conservatively preserving borderline motion.
    """
    times, qpos = np.asarray(times), np.asarray(qpos, dtype=float)
    if len(times) < 2 or qpos.shape != (len(times), model.nq) or not np.isfinite(qpos).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Ordered finite source states required')
    width = max(3, round(settings.window_s*sample_hz) | 1)
    features = motion_features(model, qpos, settings)
    moving = np.zeros(len(times), dtype=bool)
    for _, values, limit in features:
        excursion = maximum_filter1d(values, width, axis=0, mode='nearest') - minimum_filter1d(values, width, axis=0, mode='nearest')
        moving |= np.linalg.norm(excursion, axis=1) > limit
    keep = np.ones(len(times), dtype=bool)
    removed = []
    for start, stop in ranges(~moving):
        duration = times[stop-1]-times[start]+1/sample_hz
        if duration < settings.minimum_idle_s:
            continue
        # Long monotonic creeping is meaningful even if every local window is slow.
        if any(np.linalg.norm(np.ptp(values[start:stop], axis=0)) > limit for _, values, limit in features):
            continue
        first = int(np.searchsorted(times, times[start]+settings.edge_padding_s))
        last = int(np.searchsorted(times, times[stop-1]-settings.edge_padding_s, side='right'))
        if last > first:
            keep[first:last] = False
            removed.append(dict(first_frame=first, stop_frame=last, source_start_s=float(times[first]), source_stop_s=float(times[last-1]+1/sample_hz)))
    keep[0] = keep[-1] = True
    return keep, removed


def build_edit(recording, settings=IdleSettings()):
    reader = MotionReader(recording)
    pairs = list(reader.frames())
    times = np.array([p[0] for p in pairs])
    qpos = np.array([p[1] for p in pairs], dtype=float)
    model = mujoco.MjModel.from_xml_path(str(reader.path/'source/scene.xml'))
    hz = float(reader.meta['sample_hz'])
    keep, _ = idle_mask(model, times, qpos, settings, hz)
    appearance_frames = []
    for event in reader.events():
        if event['event'] != 'rgba':
            continue
        stamp = event['sim_time_s']
        start = np.searchsorted(times, stamp-settings.feedback_padding_s)
        stop = np.searchsorted(times, stamp+settings.feedback_padding_s, side='right')
        keep[start:stop] = True
        index = int(np.searchsorted(times, stamp-1e-9))
        if index < len(times):
            keep[index] = True
            appearance_frames.append(index)
    spans = []
    edited_frames = 0
    for first, stop in ranges(keep):
        count = int(stop-first)
        spans.append(dict(first_frame=int(first), stop_frame=int(stop),
            source_start_s=float(times[first]), source_last_s=float(times[stop-1]),
            edited_start_s=edited_frames/hz, edited_stop_s=(edited_frames+count)/hz))
        edited_frames += count
    removed = [dict(first_frame=int(a), stop_frame=int(b), source_start_s=float(times[a]),
        source_last_s=float(times[b-1]), duration_s=(b-a)/hz) for a, b in ranges(~keep)]
    return dict(schema='robofrontier.motion-edit.v1', recording=str(reader.path.resolve()),
        case_id=reader.meta['case_id'], source_scene_sha256=reader.meta['source_scene_sha256'],
        motion_index_sha256=checksum(reader.path/'stream.json'), events_sha256=checksum(reader.path/'events.jsonl'),
        source_frames=len(times), sample_hz=hz, settings=asdict(settings),
        retained_frames=edited_frames, removed_frames=int((~keep).sum()),
        source_duration_s=len(times)/hz, edited_duration_s=edited_frames/hz,
        retained_spans=spans, removed_spans=removed, appearance_frames=sorted(set(appearance_frames)),
        scope='Presentation time edit only. Original states/events unchanged; not physical completion-time evidence.')


class EditedMotion:
    """Validate an edit against its recording and expose one reusable timeline."""
    def __init__(self, recording, edit):
        self.reader = MotionReader(recording)
        self.edit = json.loads(Path(edit).read_text()) if not isinstance(edit, dict) else edit
        value = self.edit
        if (value.get('schema') != 'robofrontier.motion-edit.v1'
                or value['case_id'] != self.reader.meta['case_id']
                or value['source_scene_sha256'] != self.reader.meta['source_scene_sha256']
                or value['motion_index_sha256'] != checksum(self.reader.path/'stream.json')
                or value['events_sha256'] != checksum(self.reader.path/'events.jsonl')
                or value['source_frames'] != self.reader.meta['frames']
                or value['sample_hz'] != self.reader.meta['sample_hz']):
            raise ValueError('Edit source recording differs')
        last, count = 0, 0
        for span in value['retained_spans']:
            a, b = span['first_frame'], span['stop_frame']
            if type(a) is not int or type(b) is not int or not last <= a < b <= value['source_frames']:
                raise ValueError('Invalid edit intervals')
            count += b-a
            last = b
        if count != value['retained_frames'] or count == 0:
            raise ValueError('Invalid edit frame count')
        appearance=value['appearance_frames']
        if any(type(i) is not int or not any(s['first_frame'] <= i < s['stop_frame'] for s in value['retained_spans']) for i in appearance):
            raise ValueError('Appearance frame outside retained intervals')

    def video_indices(self, fps=30., speed=1.):
        if not np.isfinite([fps, speed]).all() or min(fps, speed) <= 0:
            raise ValueError('Positive finite playback rate required')
        if fps > speed*self.edit['sample_hz']:
            raise ValueError('Requested playback requires duplicated or interpolated frames')
        source = np.concatenate([np.arange(s['first_frame'], s['stop_frame']) for s in self.edit['retained_spans']])
        # Select original samples only; support arbitrary speeds without interpolation.
        positions = np.arange(0, len(source), speed*self.edit['sample_hz']/fps).astype(int)
        extra = [i for s in self.edit['retained_spans'] for i in (s['first_frame'], s['stop_frame']-1)]
        return np.unique(np.r_[source[positions], extra, self.edit['appearance_frames']]).astype(int)

    def frames(self):
        spans = iter(self.edit['retained_spans'])
        span = next(spans, None)
        edited_index = 0
        for index, (stamp, qpos) in enumerate(self.reader.frames()):
            while span is not None and index >= span['stop_frame']:
                span = next(spans, None)
            if span is None:
                break
            if index >= span['first_frame']:
                yield index, stamp, edited_index/self.edit['sample_hz'], qpos
                edited_index += 1
