import unittest
import numpy as np
from robofrontier.suite.transport_motion_signals import signal_flags,quaternion_path,changed_tiles

class IdleClassificationTests(unittest.TestCase):
    def setUp(self):self.dt=1/60;self.t=np.arange(0,80,self.dt)
    def flags(self,x,protect=None):return signal_flags(x,self.dt,2e-5,.015,protect)
    def test_numerical_noise_is_not_movement(self):
        noise=np.random.default_rng(2).normal(0,1e-7,len(self.t));active,_,_=self.flags(noise);self.assertFalse(active.any())
    def test_visible_repetitive_object_wobble_removed(self):
        active,raw,jitter=self.flags(.002*np.sin(2*np.pi*3*self.t));region=(self.t>20)&(self.t<60)
        self.assertGreater(raw[region].mean(),.99);self.assertLess(active[region].mean(),.02);self.assertGreater(jitter[region].mean(),.98)
    def test_slow_progress_is_kept(self):
        active,_,_=self.flags(self.t*.00005);self.assertGreater(active[120:-120].mean(),.95)
    def test_drift_under_visible_wobble_is_kept(self):
        active,_,_=self.flags(self.t*.00015+.001*np.sin(2*np.pi*3*self.t));self.assertGreater(active[1200:-1200].mean(),.85)
    def test_other_arm_activity_preserves_whole_scene(self):
        jitter=self.flags(.002*np.sin(2*np.pi*3*self.t))[0];arm=self.flags(self.t*.002)[0];combined=jitter|arm
        self.assertGreater(combined[120:-120].mean(),.99)
    def test_commanded_repeated_arm_action_is_not_deleted(self):
        signal=.002*np.sin(2*np.pi*3*self.t);active,_,_=signal_flags(signal,self.dt,2e-4,.01,np.ones(len(signal),bool));self.assertGreater(active.mean(),.99)
    def test_large_out_and_back_is_kept(self):
        active,_,_=self.flags(.05*np.sin(2*np.pi*self.t));self.assertGreater(active[120:-120].mean(),.99)
    def test_gripper_only_opening_is_kept(self):
        width=np.clip((self.t-10)*.01,0,.06);active,_,_=signal_flags(width,self.dt,2e-5,.002);self.assertGreater(active[(self.t>10.5)&(self.t<15.5)].mean(),.99)
    def test_quaternion_sign_is_not_rotation(self):
        q=np.zeros((len(self.t),4));q[:,0]=np.where(np.arange(len(q))%2,1,-1);self.assertLess(np.max(abs(quaternion_path(q))),1e-12)
    def test_continuous_spin_does_not_look_like_bounded_jitter(self):
        angle=self.t*2;q=np.c_[np.cos(angle/2),np.zeros((len(angle),2)),np.sin(angle/2)];path=quaternion_path(q)
        np.testing.assert_allclose(path[:,2],angle,atol=1e-9)
    def test_pixel_motion_in_jitter_region_does_not_override(self):
        a=np.zeros((200,320),np.uint8);b=a.copy();b[120:128,140:148]=200;mask=np.zeros_like(a,bool);mask[110:140,130:170]=True
        tiles,count=changed_tiles(b,a,a,mask);self.assertFalse(tiles.any());self.assertEqual(count,0)
    def test_unexplained_pixel_motion_is_detected(self):
        a=np.zeros((200,320),np.uint8);b=a.copy();b[120:128,140:148]=200;tiles,count=changed_tiles(b,a,a,np.zeros_like(a,bool));self.assertTrue(tiles.any());self.assertEqual(count,64)
    def test_timestamp_overlay_is_ignored(self):
        a=np.zeros((200,320),np.uint8);b=a.copy();b[20:40,10:100]=255;tiles,count=changed_tiles(b,a,a,np.zeros_like(a,bool));self.assertFalse(tiles.any())

class RecordedIdleRegressionTests(unittest.TestCase):
    def test_visible_cylinder_rolling_back_and_forth_is_idle(self):
        t=np.arange(0,120,1/60);x=.008*np.sin(2*np.pi*.3*t)
        active,_,_=signal_flags(x,1/60,2e-5,.06)
        self.assertLess(active[(t>35)&(t<85)].mean(),.03)
    def test_tiny_steady_creep_is_not_promoted_by_long_context(self):
        t=np.arange(0,120,1/60);active,_,_=signal_flags(t*.000002,1/60,2e-5,.06)
        self.assertFalse(active[120:-120].any())
    def test_bounded_wander_during_quiet_is_removed(self):
        from robofrontier.suite.transport_motion_signals import bounded_rattle
        t=np.arange(0,600,1/60);x=np.c_[.008*np.sin(2*np.pi*.4*t)+.001*np.sin(.02*t),np.zeros((len(t),5))]
        mask=bounded_rattle(x,1/60,[2e-5]*3+[5e-4]*3,[.06]*3+[np.pi]*3);self.assertGreater(mask.mean(),.99)
    def test_progressive_arrival_before_repeated_jitter_is_preserved(self):
        from robofrontier.suite.transport_motion_signals import bounded_rattle
        t=np.arange(0,120,1/60);moving=np.maximum(0,.01-.0025*t);y=np.where(t<4,moving,.002*np.sin(2*np.pi*(t-4)));x=np.c_[y,np.zeros((len(t),5))]
        mask=bounded_rattle(x,1/60,[2e-5]*3+[5e-4]*3,[.06]*3+[np.pi]*3)
        self.assertFalse(mask[t<4].any());self.assertGreater(mask[t>10].mean(),.9)
    def test_drift_in_another_coordinate_prevents_rattle_deletion(self):
        from robofrontier.suite.transport_motion_signals import bounded_rattle
        t=np.arange(0,120,1/60);x=np.c_[.008*np.sin(2*np.pi*.4*t),t*.0001,np.zeros((len(t),4))]
        self.assertFalse(bounded_rattle(x,1/60,[2e-5]*3+[5e-4]*3,[.06]*3+[np.pi]*3).any())

class CachePerformanceTests(unittest.TestCase):
    def test_npz_arrays_are_materialized_once_and_detached_from_file(self):
        import tempfile
        from pathlib import Path
        from robofrontier.suite.transport_motion_signals import load_state_cache
        with tempfile.TemporaryDirectory()as directory:
            path=Path(directory)/'cache.npz';np.savez_compressed(path,qpos=np.arange(20).reshape(5,4),keep=np.array([True,False]))
            cached=load_state_cache(path);path.unlink()
            self.assertIsInstance(cached,dict);np.testing.assert_array_equal(cached['qpos'][2],[8,9,10,11]);self.assertTrue(cached['keep'][0])

class ProjectionIntegrationTests(unittest.TestCase):
    def test_current_mujoco_light_api_and_jitter_projection(self):
        import tempfile
        from pathlib import Path
        from robofrontier.suite.transport_motion_signals import JitterRegions
        with tempfile.TemporaryDirectory()as directory:
            p=Path(directory);(p/'scene.xml').write_text('<mujoco><worldbody><light pos="0 -1 3" dir="0 0 -1"/><camera name="global" pos="0 -3 2" xyaxes="1 0 0 0 .5 .866"/><body name="osc" pos="0 0 1"><freejoint/><geom type="sphere" size=".03" mass=".1"/></body></worldbody></mujoco>')
            projector=JitterRegions(p,['body:osc'],320,200);q=np.array([0.,0.,1.,1.,0.,0.,0.])
            self.assertTrue(projector.mask(q,np.array([True])).any());self.assertFalse(projector.mask(q,np.array([False])).any())

if __name__=='__main__':unittest.main()
