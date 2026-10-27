"""Portable, source-bound edit decisions shared by every presentation scene.

A timeline references immutable recorded states. It is not a physics checkpoint
and never rewrites, smooths, or restarts the recorded motion.
"""
import json
from pathlib import Path

import numpy as np

from .motion_recording import digest

SCHEMA = 'robofrontier.motion-edit.v1'


def build_timeline(reader, source_times, video_source_indices, keep, fps=30, speedup=60):
    times = np.asarray(source_times, dtype=float)
    indices = np.asarray(video_source_indices, dtype=np.int64)
    keep = np.asarray(keep, dtype=bool)
    count = reader.meta['frames']
    if (times.shape != (count,) or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)):
        raise ValueError('Invalid original source timeline')
    if (not len(indices) or keep.shape != indices.shape
            or indices[0] != 0 or indices[-1] != count - 1
            or np.any(np.diff(indices) <= 0) or not keep[0] or not keep[-1]):
        raise ValueError('Invalid source frame selection or missing endpoints')
    if not np.isfinite(fps) or fps <= 0 or not np.isfinite(speedup) or speedup <= 0:
        raise ValueError('Invalid output cadence')
    selected = indices[keep]
    edges = np.diff(np.r_[False, keep, False].astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1) - 1
    ranges = []
    for a, b in zip(starts, ends):
        first = int(indices[a])
        last = int(indices[b + 1] - 1) if b + 1 < len(indices) else count - 1
        ranges.append(dict(first_source_frame=first, last_source_frame=last,
                           source_start_s=float(times[first]),
                           source_end_s=float(times[last])))
    return dict(
        schema=SCHEMA, case_id=reader.meta['case_id'],
        source_scene_sha256=reader.meta['source_scene_sha256'],
        motion_index_sha256=digest(reader.path / 'stream.json'),
        source_frames=count, source_sample_hz=reader.meta['sample_hz'],
        source_time_origin_s=float(times[0]), source_time_end_s=float(times[-1]),
        fps=float(fps), nominal_speedup=float(speedup),
        output_frames=len(selected), output_duration_s=len(selected) / fps,
        render_frames=[dict(output_frame=i, output_time_s=i / fps,
                            source_frame=int(frame), source_time_s=float(times[frame]))
                       for i, frame in enumerate(selected)],
        keep_source_ranges=ranges,
        range_semantics='Inclusive original-frame windows; boundaries follow the original video sampling grid, not subframe motion onset. Exact video matching uses render_frames.',
        policy='Background-independent fixed edit decisions derived from the reviewed simple-scene video. No per-background redetection.',
        physical_scope='Presentation only. Invisible jitter or occluded motion may be omitted. Full 60Hz recordings remain authoritative and unchanged.',
        initial_and_final_states_preserved=True,
        interpolation=False, physics_restarted=False)


class MotionEdit:
    def __init__(self, path, reader):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text())
        d = self.data
        if d.get('schema') != SCHEMA:
            raise ValueError('Unsupported motion edit schema')
        for key in ('case_id', 'source_scene_sha256', 'source_frames'):
            expected = reader.meta['frames'] if key == 'source_frames' else reader.meta[key]
            if d[key] != expected:
                raise ValueError('Motion edit source identity differs: ' + key)
        if d['motion_index_sha256'] != digest(reader.path / 'stream.json'):
            raise ValueError('Motion edit source index checksum differs')
        frames = d['render_frames']
        if not frames or len(frames) != d['output_frames']:
            raise ValueError('Invalid edit output count')
        if not np.isfinite(d['fps']) or d['fps'] <= 0:
            raise ValueError('Invalid edit output fps')
        previous = -1
        for i, frame in enumerate(frames):
            source = frame['source_frame']
            if (not isinstance(source, int) or not previous < source < d['source_frames']
                    or frame['output_frame'] != i
                    or not np.isfinite(frame['source_time_s'])
                    or not np.isfinite(frame['output_time_s'])
                    or abs(frame['output_time_s'] - i / d['fps']) > 1e-9):
                raise ValueError('Invalid ordered edit frame/time mapping')
            previous = source
        if frames[0]['source_frame'] != 0 or previous != d['source_frames'] - 1:
            raise ValueError('Motion edit must preserve exact source endpoints')
        ranges = d['keep_source_ranges']
        previous = -1
        for window in ranges:
            first, last = window['first_source_frame'], window['last_source_frame']
            if (not isinstance(first, int) or not isinstance(last, int)
                    or not previous < first <= last < d['source_frames']):
                raise ValueError('Invalid ordered retained source ranges')
            previous = last
        if not ranges or ranges[0]['first_source_frame'] != 0 or previous != d['source_frames'] - 1:
            raise ValueError('Retained source ranges must preserve endpoints')
        self.identity = (d['case_id'], d['source_scene_sha256'], d['motion_index_sha256'])

    def frames(self, reader):
        """Yield exact source poses in edited order; skipped events use source time."""
        identity = (reader.meta['case_id'], reader.meta['source_scene_sha256'],
                    digest(reader.path / 'stream.json'))
        if identity != self.identity:
            raise ValueError('Motion edit source identity differs')
        selected = iter(self.data['render_frames'])
        wanted = next(selected, None)
        emitted = 0
        for index, (stamp, state) in enumerate(reader.frames()):
            if wanted is None:
                raise ValueError('Source contains more frames than its edit metadata')
            if index != wanted['source_frame']:
                continue
            if abs(stamp - wanted['source_time_s']) > 1e-8:
                raise ValueError('Motion edit source timestamp differs')
            yield stamp, state
            emitted += 1
            wanted = next(selected, None)
        if wanted is not None or emitted != self.data['output_frames']:
            raise ValueError('Source ended before all edited frames were read')

    def kept_source_frames(self, reader):
        """A logical 60Hz cut of raw states, without copying or thinning qpos.

        Yields (edited_time_s, original_time_s, state). Original timestamps stay
        attached; the nominal-rate edited clock has no removed idle gaps.
        """
        identity = (reader.meta['case_id'], reader.meta['source_scene_sha256'],
                    digest(reader.path / 'stream.json'))
        if identity != self.identity:
            raise ValueError('Motion edit source identity differs')
        ranges = iter(self.data['keep_source_ranges'])
        window = next(ranges)
        emitted = 0
        last_index = -1
        for index, (stamp, state) in enumerate(reader.frames()):
            last_index = index
            while window is not None and index > window['last_source_frame']:
                window = next(ranges, None)
            if window is None:
                raise ValueError('Source contains more frames than retained ranges')
            if index >= window['first_source_frame']:
                yield emitted / reader.meta['sample_hz'], stamp, state
                emitted += 1
        if last_index != self.data['source_frames'] - 1:
            raise ValueError('Source ended before the retained final frame')


def resample_timeline(timeline, source_times, speedup=30, fps=30):
    """Sample the same retained source windows at a new presentation cadence."""
    from copy import deepcopy
    data = deepcopy(timeline)
    times = np.asarray(source_times, dtype=float)
    if (times.shape != (data['source_frames'],) or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)):
        raise ValueError('Invalid source times for edit resampling')
    if not np.isfinite(speedup) or speedup <= 0 or not np.isfinite(fps) or fps <= 0:
        raise ValueError('Invalid resampled cadence')
    stride = data['source_sample_hz'] * speedup / fps
    if stride < 1 or abs(stride - round(stride)) > 1e-9:
        raise ValueError('This exact-state sampler requires an integer source stride')
    stride = int(round(stride))
    selections = [np.arange(r['first_source_frame'], r['last_source_frame'] + 1,
                            stride, dtype=np.int64) for r in data['keep_source_ranges']]
    indices = np.unique(np.r_[np.concatenate(selections), 0, data['source_frames'] - 1])
    data.update(fps=float(fps), nominal_speedup=float(speedup),
                output_frames=len(indices), output_duration_s=len(indices) / fps,
                render_frames=[dict(output_frame=i, output_time_s=i/fps,
                                    source_frame=int(frame), source_time_s=float(times[frame]))
                               for i, frame in enumerate(indices)],
                nominal_source_stride=stride,
                resampling='Exact original states within unchanged retained source windows; no duplication, interpolation or physics')
    return data
