"""Exact privileged kinematic evidence from the existing physical owner.

This reads the running data only. Cached free-body poses match the published
robot/cargo view; the copy is serialized, never assigned to live physics.
"""
import json
from pathlib import Path
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


def install(reference):
    import robofrontier.suite.relay_live_recovery as recovery
    from robofrontier.run_setup import write_json
    cls=recovery.RelayLiveRecovery
    if hasattr(cls,'_before_native_state_publish'):return
    original=cls.publish;cls._before_native_state_publish=original
    def publish(self,error=None):
        original(self,error)
        path=self.directory/'observation.json'
        observation=json.loads(path.read_text())
        state=native_kinematic_state(self.trial)
        if abs(observation['sim_time_s']-state['sim_time_s'])>1e-8:
            raise RuntimeError('Native observation changed during synchronous publication')
        observation['native_kinematic_state']=state
        write_json(path,observation)
    cls.publish=publish
