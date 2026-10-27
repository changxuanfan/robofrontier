"""Exact privileged kinematic evidence from the existing physical owner.

This reads the running data only. Cached free-body poses match the published
robot/cargo view; the copy is serialized, never assigned to live physics.
"""
import mujoco


def native_kinematic_state(t):
    qpos=t.data.qpos.copy()
    for joint in range(t.model.njnt):
        if t.model.jnt_type[joint]!=mujoco.mjtJoint.mjJNT_FREE:continue
        address=int(t.model.jnt_qposadr[joint]);body=int(t.model.jnt_bodyid[joint])
        qpos[address:address+3]=t.data.xpos[body]
        qpos[address+3:address+7]=t.data.xquat[body]
    return dict(version=1,sim_time_s=float(t.data.time),qpos=qpos.tolist(),
        source='Read-only native double joints and cached free-body poses; no integration/state write')
