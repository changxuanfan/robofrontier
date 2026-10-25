"""Fail a run on numeric failure instead of continuing a reset world."""
import mujoco
import numpy as np

def require_valid_step(data,previous_time):
    bad=[]
    for name in ('mjWARN_BADQPOS','mjWARN_BADQVEL','mjWARN_BADQACC','mjWARN_BADCTRL'):
        kind=getattr(mujoco.mjtWarning,name)
        if data.warning[kind].number:bad.append(name)
    if bad or not np.isfinite(data.time) or data.time<=previous_time or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise RuntimeError('Numerical physics failure; abort without reset/replay: '+','.join(bad))
