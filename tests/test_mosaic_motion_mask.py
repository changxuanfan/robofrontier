import unittest
import numpy as np
from scripts.mosaic_motion_mask import SceneMotionDetector,selected_video_frames,frame_intervals


class WholeSceneMotionTests(unittest.TestCase):
    def states(self,n=61):
        q=np.zeros((n,9),dtype=np.float32);q[:,5]=1
        return np.arange(n)/60,q

    def detector(self):return SceneMotionDetector([{'type':3},{'type':2},{'type':0}],60)

    def test_stationary_numeric_jitter_and_quaternion_sign_do_not_create_motion(self):
        t,q=self.states();q[:,0]=np.sin(t*20)*.0000001;q[:,2]=np.sin(t*30)*.00000005;q[::2,5]=-1
        self.assertFalse(self.detector().process(t,q).any())

    def test_gripper_or_cube_motion_is_kept_even_when_all_arm_joints_are_still(self):
        for index in (1,2):
            t,q=self.states();q[20:,index]=.003
            with self.subTest(index=index):self.assertTrue(self.detector().process(t,q)[20])

    def test_cube_rotation_alone_is_kept(self):
        t,q=self.states();q[20:,5]=np.cos(.1/2);q[20:,8]=np.sin(.1/2)
        self.assertTrue(self.detector().process(t,q)[20])

    def test_slow_persistent_motion_survives_chunk_boundaries(self):
        t,q=self.states(241);q[:,2]=t*.00002;detector=self.detector()
        result=np.r_[detector.process(t[:120],q[:120]),detector.process(t[120:],q[120:])]
        self.assertTrue(result[120:].all())

    def test_every_detected_motion_bucket_and_initial_final_states_survive_edit(self):
        active=np.zeros(1001,dtype=bool);active[400]=True;active[799]=True
        keep,source=selected_video_frames(active,100,11)
        self.assertTrue(keep[0] and keep[-1] and keep[4] and keep[7]);self.assertFalse(keep[1]);self.assertEqual(source[-1],1000)
        rebuilt=np.zeros(len(keep),dtype=bool)
        for a,b in frame_intervals(keep):rebuilt[a:b+1]=True
        np.testing.assert_array_equal(rebuilt,keep)


if __name__=='__main__':unittest.main()
