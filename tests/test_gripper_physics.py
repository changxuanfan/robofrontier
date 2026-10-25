"""Real headless contact regression: an off-centre approach must not cross fingers."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robofrontier.pipeline import World
from robofrontier.task_config import load_task,load_harness
from robofrontier.robot_control import matrix_quat,rotation_vector


class GripperPhysicsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.cfg=load_harness()

    def tearDown(self):self.temp.cleanup()

    def world(self,cube_y):
        task=load_task();task.update(arms={'B':0.},goals={'B':[]},noslip_iterations=10)
        task['objects']=[{'id':'cube','region':'B','color':'green','pattern':'plain','initial_position_m':[.42,cube_y,.42]}]
        return World(Path(self.temp.name),task)

    def command(self,w,op,data):
        command=w.validate_action('B',op,data,self.cfg);w.start_action('B','test',command)
        for _ in range(int((command['timing']['timeout_sim_s']+4)/.02)):
            w.step();done=w.poll_actions(15)
            if done:return done[0]
        self.fail('Command did not terminate')

    def move(self,w,xyz):
        result=self.command(w,'move',{'target_m':xyz,'speed_m_s':.06})
        self.assertEqual(result['status'],'completed',result)

    def test_offcentre_contact_cannot_leave_fingers_crossed_after_open(self):
        w=self.world(.35)
        self.move(w,[.42,.30,.54])
        # An off-centre obstructed descent may stop safely instead of pushing
        # through the cube. This regression concerns crossed fingers, not a
        # requirement to complete an obstructed Cartesian target.
        obstructed=self.command(w,'move',{'target_m':[.42,.30,.42],'speed_m_s':.06})
        self.assertIn(obstructed['status'],('completed','failed'),obstructed)
        self.move(w,[.42,.30,.65])
        result=self.command(w,'gripper',{'state':'open'})
        for _ in range(100):w.step()
        self.assertEqual(result['status'],'completed',result)
        self.assertGreater(w.arms['B'].observe()['gripper_width_m'],.075)
        for name in ('finger_joint1','finger_joint2'):
            self.assertGreater(float(w.data.joint('B_'+name).qpos[0]),-.001)
        # The fix must retain the contact/grip settings requested for the demo.
        self.assertEqual(w.model.opt.noslip_iterations,10)
        self.assertAlmostEqual(w.model.actuator_gainprm[w.arms['B'].grip,0],.01568627451)
        self.assertEqual(w.model.actuator_forcerange[w.arms['B'].grip].tolist(),[-100.,100.])
        self.assertEqual(w.model.geom_friction[w.model.geom('cube_geom').id].tolist(),[1.,.005,.0001])

    def test_open_reports_failure_if_actuator_cannot_open(self):
        w=self.world(.35)
        self.assertEqual(self.command(w,'gripper',{'state':'close'})['status'],'completed')
        w.model.actuator_gainprm[w.arms['B'].grip,0]=0 # Inject a dead actuator.
        result=self.command(w,'gripper',{'state':'open'})
        self.assertEqual(result['status'],'failed',result)
        self.assertIn('open',result['reason'].lower())

    def test_normal_contact_grasp_lifts_and_releases_without_attachment(self):
        w=self.world(0.)
        self.move(w,[.42,0.,.52]);self.move(w,[.42,0.,.42])
        self.assertEqual(self.command(w,'gripper',{'state':'close'})['status'],'completed')
        self.move(w,[.42,0.,.56])
        self.assertGreater(w.data.joint('cube_free').qpos[2],.50)
        self.move(w,[.57,0.,.56]);self.move(w,[.57,0.,.42])
        self.assertEqual(self.command(w,'gripper',{'state':'open'})['status'],'completed')
        self.move(w,[.57,0.,.56])
        for _ in range(100):w.step()
        point=w.data.joint('cube_free').qpos[:3]
        self.assertLess(abs(point[0]-.57),.025)
        self.assertLess(abs(point[1]),.025)
        self.assertLess(abs(point[2]-.42),.012)

    def test_slow_aperture_and_rotation_complete_beyond_old_fixed_deadline(self):
        w=self.world(.35);a=w.arms['B']
        def execute(op,p,limit=90):
            command=w.validate_action('B',op,p,self.cfg);w.start_action('B','slow-'+op,command)
            began=w.time
            while w.time-began<limit:
                w.step();done=w.poll_actions(self.cfg['action_timeout_sim_s'])
                if done:return done[0],w.time-began
            self.fail('Slow valid action exceeded bounded check duration')
        result,elapsed=execute('gripper',{'width_m':0.,'speed_m_s':.005})
        self.assertEqual(result['status'],'completed',result);self.assertGreater(elapsed,15)
        self.assertLess(a.observe()['gripper_width_m'],.002)
        current=w.data.site_xmat[a.site].reshape(3,3).copy()
        target=np.array([[0,-1,0],[1,0,0],[0,0,1]])@current
        result,elapsed=execute('move',{'quaternion_wxyz':matrix_quat(target).tolist(),'angular_speed_rad_s':.05})
        self.assertEqual(result['status'],'completed',result);self.assertGreater(elapsed,15)
        self.assertLess(np.linalg.norm(rotation_vector(target,w.data.site_xmat[a.site].reshape(3,3))),.02)

    def test_timed_out_jaw_holds_reference_after_settled_failure(self):
        w=self.world(.35);a=w.arms['B'];command=w.validate_action('B','gripper',{'state':'close','speed_m_s':.005},self.cfg)
        command['timing']['timeout_sim_s']=.5 # Inject an early deadline to exercise failure recovery.
        w.start_action('B','expired',command);terminal=None
        for _ in range(200):
            w.step();done=w.poll_actions(15)
            if done:terminal=done[0];break
        self.assertIsNotNone(terminal);self.assertEqual(terminal['status'],'failed',terminal)
        reference=a.gripper;gap=a.observe()['gripper_width_m']
        self.assertGreater(gap,.07)
        for _ in range(100):w.step()
        self.assertEqual(a.gripper,reference);self.assertAlmostEqual(a.grip_goal,reference)
        self.assertLess(abs(a.observe()['gripper_width_m']-gap),.002)


if __name__=='__main__':unittest.main()
