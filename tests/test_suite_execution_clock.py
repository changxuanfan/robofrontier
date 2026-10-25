"""The engineering executor must use the policy world's physical tick order."""
import copy,tempfile,unittest
from pathlib import Path
import mujoco,numpy as np
from robofrontier.pipeline import World
from robofrontier.robot_control import PandaArm
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.oracle import Trial
from robofrontier.suite.scenes import catalog,case_directory

class SuiteExecutionClockTests(unittest.TestCase):
    def test_reference_and_policy_loops_match_on_identical_commands(self):
        scene=case_directory(next(c for c in catalog()['cases'] if c['id']=='transport-a04-d2'))
        with tempfile.TemporaryDirectory() as temp:
            t=Trial(scene,Path(temp)/'trial',frames=False);w=World.__new__(World)
            try:
                w.task=t.task;w.model=copy.copy(t.model);w.data=mujoco.MjData(w.model);mujoco.mj_resetDataKeyframe(w.model,w.data,w.model.key('initial').id);mujoco.mj_forward(w.model,w.data)
                w.arms={r:PandaArm(w.model,w.data,r) for r in t.arms};w.command_bounds=t.task['workspace_world_m'];w.beam_sampler=w.spring_sampler=None;w.suite_sampler=Monitor(w)
                q=t.arms['A'].qref.copy();q[0]+=.05
                for world in (t,w):
                    for robot,op,params in [('A','move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.3)),('B','gripper',dict(width_m=0.,speed_m_s=.04))]:
                        arm=world.arms[robot];command=arm.validate(op,params,t.cfg,np.asarray(t.task['workspace_world_m']['lower']),np.asarray(t.task['workspace_world_m']['upper']));arm.start(robot,command,0.)
                for _ in range(120):
                    w.step()
                    for _ in range(t.stride):t.step()
                    for world in (t,w):
                        for arm in world.arms.values():arm.poll(world.data.time,15.)
                    np.testing.assert_allclose(t.data.qpos,w.data.qpos,atol=1e-12,rtol=0);np.testing.assert_allclose(t.data.qvel,w.data.qvel,atol=1e-12,rtol=0)
                    np.testing.assert_allclose(t.data.ctrl,w.data.ctrl,atol=1e-12,rtol=0)
            finally:
                if getattr(w,'suite_sampler',None) is not None:w.close()
                t.close({'passed':False,'scope':'Executor clock equivalence only'})

if __name__=='__main__':unittest.main()
