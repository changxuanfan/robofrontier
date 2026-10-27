import unittest
import numpy as np
from robofrontier.relay_video_motion import classify_motion,image_motion_evidence


class ReplayMotionTests(unittest.TestCase):
    def states(self):
        q=np.zeros((201,24),np.float32);q[:,6]=1;q[:,13]=1
        layout=dict(robot_hinges=[[14],[15]],robot_slides=[[16],[17]],primary_free=[0],primary_hinges=[],pendant_free=[7])
        return q,layout

    def test_entire_scene_static_is_removed(self):
        q,l=self.states();self.assertFalse(classify_motion(q,20,l)['keep'].any())

    def test_second_robot_moves_while_first_stops_is_retained(self):
        q,l=self.states();q[:,15]=np.arange(len(q))*.003
        self.assertTrue(classify_motion(q,20,l)['keep'][20:-20].all())

    def test_two_second_window_catches_slow_joint_motion(self):
        q,l=self.states();q[:,15]=np.arange(len(q))*.05*.001
        self.assertTrue(classify_motion(q,20,l)['keep'][30:-30].all())

    def test_tiny_numerical_jitter_is_not_motion(self):
        q,l=self.states();q[:,15]=np.sin(np.arange(len(q))*2.7)*1e-5
        self.assertFalse(classify_motion(q,20,l)['keep'].any())

    def test_only_pendant_swing_is_waiting(self):
        q,l=self.states();q[:,7]=np.sin(np.arange(len(q))*.1)*.02
        r=classify_motion(q,20,l);self.assertTrue(r['weight_only'].any());self.assertFalse(r['keep'].any())

    def test_pendant_swing_and_effective_robot_motion_are_kept(self):
        q,l=self.states();q[:,7]=np.sin(np.arange(len(q))*.1)*.02;q[:,14]=np.arange(len(q))*.003
        self.assertTrue(classify_motion(q,20,l)['keep'][20:-20].all())

    def test_main_object_fall_with_stationary_arms_is_kept(self):
        q,l=self.states();q[:,2]=-np.arange(len(q))*.003
        self.assertTrue(classify_motion(q,20,l)['keep'][20:-20].all())

    def test_independent_weight_fall_is_not_deleted_as_swing(self):
        q,l=self.states();q[:,9]=-np.arange(len(q))*.003
        self.assertTrue(classify_motion(q,20,l)['pendant_relocation'][30:-30].all())

    def test_slow_primary_rotation_is_kept(self):
        q,l=self.states();angle=np.arange(len(q))*.05*.001
        q[:,3]=np.cos(angle/2);q[:,6]=np.sin(angle/2)
        self.assertTrue(classify_motion(q,20,l)['primary'][30:-30].all())

    def test_stationary_tail_is_not_kept_by_long_window(self):
        q,l=self.states();q[:,14]=np.minimum(np.arange(len(q)),40)*.003
        self.assertFalse(classify_motion(q,20,l)['keep'][100:].any())

    def test_quaternion_sign_flip_is_not_rotation(self):
        q,l=self.states();q[::2,6]=-1
        self.assertFalse(classify_motion(q,20,l)['keep'].any())

    def test_image_weight_occlusion_does_not_fake_arm_motion(self):
        ids=np.full((20,30),-1);ids[5:15,5:15]=1;a=ids.copy();b=ids.copy();a[6:10,6:10]=2;b[10:14,10:14]=2
        rgb0=np.zeros((20,30,3),np.uint8);rgb1=rgb0.copy();rgb0[a==2]=255;rgb1[b==2]=255
        r=image_motion_evidence(rgb0,rgb1,a,b,{1},set(),{2});self.assertFalse(r['robot_effective']);self.assertGreater(r['pendant_changed_pixels'],0)

    def test_image_can_detect_real_arm_motion(self):
        a=np.full((24,40),-1);b=a.copy();a[5:15,5:15]=1;b[5:15,7:17]=1
        x=np.zeros((24,40,3),np.uint8);y=x.copy();x[a==1]=255;y[b==1]=255
        self.assertTrue(image_motion_evidence(x,y,a,b,{1},set(),{2})['robot_effective'])


if __name__=='__main__':unittest.main()
