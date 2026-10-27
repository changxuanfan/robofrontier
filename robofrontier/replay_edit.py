"""Adapt explicitly identified historical edit formats without rewriting them."""
import json
from pathlib import Path

import numpy as np

from .motion_recording import digest


class ReplayEdit:
    def __init__(self, path, reader):
        self.path = Path(path)
        data = json.loads(self.path.read_text())
        self.exact = isinstance(data.get('render_frames'), list)
        if self.exact:
            from .motion_edit import MotionEdit
            original = MotionEdit(path, reader)
            self.data = original.data
            self.meta = dict(data, arrays_sha256=digest(path))
            self.times = np.full(reader.meta['frames'], np.nan)
            self.indices = np.array([r['source_frame'] for r in data['render_frames']], dtype=np.int64)
            self.times[self.indices] = [r['source_time_s'] for r in data['render_frames']]
            self.speed = data['nominal_speedup']
            self.fps = data['fps']
        elif 'arrays' in data and 'source_stream_sha256' in data:
            from .motion_timeline import MotionTimeline
            self.original = MotionTimeline(path, reader)
            self.meta = self.original.meta
            self.times = self.original.times
        else:
            raise ValueError('Unrecognized edit producer/structure; schema name alone is insufficient')

    def select(self, speed=20., fps=30.):
        return self.indices.copy() if self.exact else self.original.select(speed, fps)
