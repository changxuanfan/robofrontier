import unittest
import numpy as np
from robofrontier.suite.mosaic_motion_review import GapFeatures,protected_intervals,gap_intervals
from robofrontier.suite.mosaic_policy_package import ROOT,read

JOINTS=[dict(id='A_joint1',type=3,qpos_address=0),
        dict(id='A_finger_joint1',type=2,qpos_address=1),
        dict(id='A_stock01_free',type=0,qpos_address=2)]


class MosaicMotionReviewTests(unittest.TestCase):
    def setUp(self):
        self.settings=read(ROOT/'configs/task_families/mosaic/motion-review-r13.json')
        self.times=np.arange(601)/60
        self.states=np.zeros((601,9));self.states[:,5]=1

    def classify(self,states):
        features=GapFeatures(JOINTS)
        for chunk in np.array_split(states,7):features.add(chunk)
        return features.result(self.settings)

    def test_stationary_and_31micrometer_finger_chatter_are_review_proposals(self):
        self.states[:,1]=(np.arange(601)%2)*.00003135
        result=self.classify(self.states)
        self.assertEqual(result['label'],'proposed_idle_cut')
        self.assertFalse(result['measured_zero_force_proven']);self.assertFalse(result['applied'])

    def test_large_repeated_object_oscillation_can_be_flagged_despite_visible_motion(self):
        self.states[:,2]=.02*np.sin(self.times*2*np.pi)
        self.assertEqual(self.classify(self.states)['label'],'review_repeated_large_object_oscillation')

    def test_very_slow_translation_is_retained(self):
        self.states[:,2]=self.times*.000001
        self.assertEqual(self.classify(self.states)['label'],'keep_slow_net_motion')

    def test_active_arm_and_gripper_motion_are_retained(self):
        for index in (0,1):
            q=self.states.copy();q[:,index]=.01*np.sin(self.times)
            self.assertEqual(self.classify(q)['label'],'keep_robot_or_gripper_motion')

    def test_repeated_idle_gripper_oscillation_is_reviewed_but_failed_open_stays_protected(self):
        self.states[:,1]=.01*np.sin(self.times*2*np.pi)
        self.assertEqual(self.classify(self.states)['label'],'review_repeated_robot_or_gripper_oscillation')
        actions=[dict(start_sim_s=1,end_sim_s=2,commands={'A':['gripper',{'mode':'grasp'}]},results={'A':{'status':'completed'}}),
                 dict(start_sim_s=5,end_sim_s=6,commands={'A':['gripper',{'mode':'move'}]},results={'A':{'status':'failed'}})]
        protected,_=protected_intervals(actions,[],[],20,1)
        self.assertTrue(any(a<=15<=b for a,b in protected))

    def test_single_out_and_back_is_not_assumed_meaningless_oscillation(self):
        self.states[:,2]=.02*np.sin(np.pi*self.times/self.times[-1])
        self.assertEqual(self.classify(self.states)['label'],'keep_nonrepeated_object_motion')

    def test_slow_rotation_and_antipodal_quaternion_noise(self):
        angle=self.times*.00001
        self.states[:,5]=np.cos(angle/2);self.states[:,8]=np.sin(angle/2)
        self.assertEqual(self.classify(self.states)['label'],'keep_slow_net_motion')
        self.states[:,5]=1;self.states[:,8]=0;self.states[::2,5]=-1
        self.assertEqual(self.classify(self.states)['label'],'proposed_idle_cut')

    def test_commanded_force_hold_is_protected_between_commands(self):
        actions=[dict(start_sim_s=1.,end_sim_s=2.,commands={'A':['gripper',{'mode':'grasp','force_n':50}]}),
                 dict(start_sim_s=9.,end_sim_s=10.,commands={'A':['gripper',{'mode':'move'}]},results={'A':{'status':'completed'}})]
        protected,_=protected_intervals(actions,[],[],15,1)
        self.assertTrue(any(a<=5<=b for a,b in protected))
        self.assertEqual(gap_intervals(0,15,protected,4),[(11.,15)])

    def test_progress_release_and_success_protect_their_intervals(self):
        progress=[dict(sim_time_s=4.,result={'matched_goals':1,'task_success':False}),
                  dict(sim_time_s=8.,result={'matched_goals':2,'task_success':True})]
        events=[dict(event='placed',sim_time_s=2.)]
        protected,_=protected_intervals([],events,progress,15,1)
        self.assertTrue(any(a<=2<=b for a,b in protected))
        self.assertTrue(any(a<=4<=b for a,b in protected))
        self.assertTrue(any(a<=14<=b for a,b in protected))
        self.assertEqual(gap_intervals(0,10,[(-10,-2),(12,20)],4),[(0,10)])


if __name__=='__main__':unittest.main()
