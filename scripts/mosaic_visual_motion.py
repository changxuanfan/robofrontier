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
