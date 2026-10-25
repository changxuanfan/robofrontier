"""A formerly feasible vertical approach swept sideways through its target."""
import tempfile,unittest
from pathlib import Path
import mujoco,numpy as np
from robofrontier.suite.oracle import Trial
from robofrontier.suite.scenes import catalog,case_directory

class CartesianApproachTests(unittest.TestCase):
    def test_near_singular_vertical_approach_does_not_knock_stock_aside(self):
        case=next(c for c in catalog()['cases'] if c['id']=='mosaic-a08-7x21-d1')
        with tempfile.TemporaryDirectory() as temp:
            t=Trial(case_directory(case),Path(temp)/'trial')
            try:
                q=np.array([1.0294405963299438,-.38132483361565345,-.8978639987639979,-2.790256682303045,-.4963715556945826,2.4842319996276774,1.3586171456959137]);arm=t.arms['B']
                # Physical regression initial state, before the first step.
                t.data.qpos[arm.qids]=q;t.data.ctrl[arm.act]=q;arm.qref=q.copy()
                for suffix in ('finger_joint1','finger_joint2'):t.data.joint('B_'+suffix).qpos[0]=.026
                t.data.ctrl[arm.grip]=arm.gripper=arm.grip_goal=.052*255/.08
                mujoco.mj_forward(t.model,t.data);t.wait(.3)
                origin=t.data.body('B_red_3').xpos.copy();rotation=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
                t.execute('vertical-stock-approach',{'B':t.pose_command('B',origin+[0,0,.006],rotation,.10,.6)})
                self.assertLess(np.linalg.norm(t.data.body('B_red_3').xpos-origin),.002)
                positions=[]
                for state in t.frame_qpos:
                    scratch=mujoco.MjData(t.model);scratch.qpos[:]=state;mujoco.mj_kinematics(t.model,scratch);positions.append(scratch.site_xpos[arm.site,:2].copy())
                self.assertLess(np.max(np.linalg.norm(np.asarray(positions)-origin[:2],axis=1)),.004)
            finally:t.close({'passed':False,'scope':'Near-singularity regression only'})

if __name__=='__main__':unittest.main()
