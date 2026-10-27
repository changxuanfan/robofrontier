"""Background-independent cuts and source-time maps for recorded-state replay.

An edit is presentation data. Discontinuities at cuts must never be simulated or
interpolated, and the retained state rows remain original recorded states.
"""
import numpy as np
from .relay_video_motion import ranges


def keep_intervals(times, keep, hz):
    times = np.asarray(times, dtype=float)
    keep = np.asarray(keep, dtype=bool)
    if times.ndim != 1 or keep.shape != times.shape or not len(times):
        raise ValueError('Invalid edit mask')
    if np.any(np.diff(times) <= 0) or not np.isfinite(times).all() or hz <= 0:
        raise ValueError('Invalid edit timeline')
    # Each analysis decision owns a half-open time cell, not an interpolated
    # trajectory between the last moving state and the next moving state.
    edges = np.r_[times[0], (times[:-1] + times[1:]) / 2,
                  times[-1] + 1 / hz / 2]
    return np.asarray([(edges[lo], edges[hi + 1])
                       for lo, hi in ranges(keep)], dtype=float).reshape(-1, 2)


def select_source_rows(times, intervals):
    times = np.asarray(times, dtype=float)
    intervals = np.asarray(intervals, dtype=float).reshape(-1, 2)
    if not len(intervals):
        return np.zeros(len(times), dtype=bool)
    i = np.searchsorted(intervals[:, 0], times, side='right') - 1
    return (i >= 0) & (times < intervals[np.maximum(i, 0), 1])


def retained_timeline(times, intervals, hz):
    """Duration of each original 60 Hz row after removed waiting is excised."""
    times = np.asarray(times, dtype=float)
    intervals = np.asarray(intervals, dtype=float).reshape(-1, 2)
    segment = np.searchsorted(intervals[:, 0], times, side='right') - 1
    if not np.all(select_source_rows(times, intervals)):
        raise ValueError('Retained row outside the edit')
    end = np.minimum(times + 1 / hz, intervals[segment, 1])
    # The actual recorder samples on a 2ms simulation clock, so consecutive
    # stored sample intervals alternate slightly around 1/60 second.
    same = segment[:-1] == segment[1:]
    end[:-1] = np.where(same, times[1:], end[:-1])
    duration = end - times
    if np.any(duration <= 0):
        raise ValueError('Nonpositive retained time cell')
    edited_time = np.r_[0., np.cumsum(duration[:-1])]
    return edited_time, duration, segment


def video_rows(edited_time, duration, fps=30., speed=20.):
    if fps <= 0 or speed <= 0 or not len(edited_time):
        raise ValueError('Invalid playback configuration')
    total = float(edited_time[-1] + duration[-1])
    count = max(1, int(np.ceil(total * fps / speed)))
    clock = np.minimum((np.arange(count) + .5) * speed / fps,
                       np.nextafter(total, -np.inf))
    rows = np.searchsorted(edited_time, clock, side='right') - 1
    # Two explicit context frames preserve initial/final evidence regardless of
    # the 20x display sampling phase; they add only 2/fps seconds, no idle lead.
    return np.r_[-1, rows, -2].astype(np.int64), clock
