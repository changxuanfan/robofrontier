"""Detect visible scene changes while ignoring the replay's clock overlay.

This is a presentation edit. It does not change source trajectories or declare
subpixel physical motion to be absent.
"""
import subprocess

import numpy as np


class VisibleMotionDetector:
    """Compare to a persistent reference so slow visible changes accumulate."""

    def __init__(self, channel_delta=20, changed_pixels=4):
        self.channel_delta = channel_delta
        self.changed_pixels = changed_pixels
        self.reference = None

    def process(self, frame):
        frame = np.asarray(frame)
        if self.reference is None:
            self.reference = frame.copy()
            return False
        delta = np.abs(frame.astype(np.int16) - self.reference.astype(np.int16))
        changed = np.count_nonzero(np.max(delta, axis=2) > self.channel_delta)
        if changed < self.changed_pixels:
            return False
        self.reference = frame.copy()
        return True


def visual_mask(video, overlay_height=60, width=640, height=370):
    """Read the whole scene below its overlay, preserving its aspect ratio."""
    command = [
        'ffmpeg', '-v', 'error', '-threads', '1', '-filter_threads', '1',
        '-i', str(video), '-vf',
        f'crop=iw:ih-{overlay_height}:0:{overlay_height},scale={width}:{height}',
        '-pix_fmt', 'rgb24', '-threads', '1', '-f', 'rawvideo', '-',
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    detector = VisibleMotionDetector()
    size = width * height * 3
    active = []
    try:
        while True:
            raw = process.stdout.read(size)
            if not raw:
                break
            if len(raw) != size:
                raise ValueError('Incomplete decoded frame')
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
            active.append(detector.process(frame))
        stderr = process.stderr.read().decode()
        if process.wait():
            raise RuntimeError('Visual decoding failed: ' + stderr)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdout.close()
        process.stderr.close()
    return np.asarray(active, dtype=bool)


def visible_frame_selection(active):
    """Keep visible changes plus one frame on each side and exact endpoints."""
    active = np.asarray(active, dtype=bool)
    if not len(active):
        raise ValueError('Empty video')
    keep = active.copy()
    keep[:-1] |= active[1:]
    keep[1:] |= active[:-1]
    keep[0] = keep[-1] = True
    return keep


def longest_still_seconds(active, fps=30):
    edges = np.diff(np.r_[True, active, True].astype(np.int8))
    starts = np.flatnonzero(edges == -1)
    ends = np.flatnonzero(edges == 1)
    return float(max(ends - starts, default=0) / fps)


def refine_timeline(data, keep, times):
    """Compose visual selection with retained source windows without gaps."""
    selected=[frame for frame,chosen in zip(data['render_frames'],keep) if chosen]
    windows=data['keep_source_ranges'];pieces=[];j=0;stride=data['nominal_source_stride']
    for frame in selected:
        start=frame['source_frame']
        while start>windows[j]['last_source_frame']:j+=1
        if start<windows[j]['first_source_frame']:
            raise ValueError('Selected state lies in a removed source gap')
        end=min(start+stride-1,windows[j]['last_source_frame'])
        if pieces and start<=pieces[-1][1]+1:pieces[-1][1]=max(pieces[-1][1],end)
        else:pieces.append([start,end])
    result=dict(data)
    result.update(render_frames=[dict(frame,output_frame=i,output_time_s=i/data['fps']) for i,frame in enumerate(selected)],
        output_frames=len(selected),output_duration_s=len(selected)/data['fps'],
        keep_source_ranges=[dict(first_source_frame=a,last_source_frame=b,
            source_start_s=float(times[a]),source_end_s=float(times[b])) for a,b in pieces],
        visual_refinement=dict(overlay_excluded_px=80,analysis_width=640,analysis_height=360,channel_delta=20,changed_pixels=4),
        range_semantics='Inclusive source windows at the new stride60 grid, intersected with the previous reviewed windows. Original deleted gaps are never bridged.')
    return result
