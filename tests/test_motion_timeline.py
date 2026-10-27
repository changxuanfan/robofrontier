import unittest,tempfile
from pathlib import Path
import numpy as np
from robofrontier.motion_timeline import lift_video_mask,sample_timeline,register_timeline,resolve_timeline

class MotionTimelineTests(unittest.TestCase):
    def test_registered_default_is_resolved_and_checksum_protected(self):
        with tempfile.TemporaryDirectory()as td:
            root=Path(td);recording=root/'trial/motion';recording.mkdir(parents=True)
            self.assertIsNone(resolve_timeline(recording))
            timeline=root/'edit.json';timeline.write_text('{}')
            register_timeline(timeline,recording)
            self.assertEqual(resolve_timeline(recording),timeline.resolve())
            self.assertEqual(resolve_timeline(recording,root/'explicit.json'),root/'explicit.json')
            timeline.write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError,'checksum'):resolve_timeline(recording)
    def test_reviewed_cells_lift_exactly_to_source_clock(self):
        mask=lift_video_mask([0,4,8,9],[True,False,True,True],10)
        np.testing.assert_array_equal(mask,[1,1,1,0,0,0,0,1,1,1])
    def test_background_and_rigid_placement_do_not_enter_selection(self):
        mask=np.r_[np.ones(121,bool),np.zeros(800,bool),np.ones(120,bool)]
        selected=sample_timeline(mask)
        self.assertFalse(np.any((selected>=121)&(selected<921)))
        np.testing.assert_array_equal(selected,[0,40,80,120,960,1000,1040])
    def test_tiny_retained_segments_do_not_each_add_a_frame(self):
        mask=np.zeros(10000,bool);mask[::10]=True
        selected=sample_timeline(mask)
        self.assertEqual(len(selected),26)
        self.assertEqual(selected[-1],9990)
    def test_arbitrary_speed_and_frame_rate_have_bounded_duration_error(self):
        mask=np.ones(10001,bool)
        for speed,fps in [(20,30),(12,15),(7,24)]:
            selected=sample_timeline(mask,60,speed,fps)
            self.assertLessEqual(abs(len(selected)/fps-mask.sum()/60/speed),2/fps)
            self.assertEqual(selected[0],0);self.assertEqual(selected[-1],10000)
    def test_incomplete_source_mapping_rejected(self):
        with self.assertRaises(ValueError):lift_video_mask([0,4],[1,1],10)
    def test_empty_and_nonpositive_rates_rejected(self):
        with self.assertRaises(ValueError):sample_timeline(np.zeros(3,bool))
        with self.assertRaises(ValueError):sample_timeline([True],speed=0)
