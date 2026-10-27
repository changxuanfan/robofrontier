import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robofrontier.motion_edit import build_timeline, MotionEdit, resample_timeline


class SampleRecording:
    def __init__(self, path):
        self.path = path
        self.meta = dict(case_id='example', source_scene_sha256='frozen-scene',
                         frames=241, sample_hz=60)
        (path / 'stream.json').write_text(json.dumps(self.meta))

    def frames(self):
        for index in range(self.meta['frames']):
            yield index / 60, np.array([index], dtype=np.float32)


class MotionEditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.reader = SampleRecording(self.root)
        self.data = build_timeline(self.reader, np.arange(241) / 60,
                                   [0, 60, 120, 180, 240],
                                   [True, True, False, False, True])
        self.path = self.root / 'edit.json'
        self.write()

    def write(self):
        self.path.write_text(json.dumps(self.data))

    def test_exact_pose_and_source_time_survive_cut_with_separate_output_clock(self):
        edit = MotionEdit(self.path, self.reader)
        frames = list(edit.frames(self.reader))
        self.assertEqual([t for t, _ in frames], [0, 1, 4])
        self.assertEqual([int(q[0]) for _, q in frames], [0, 60, 240])
        self.assertEqual([x['output_time_s'] for x in edit.data['render_frames']],
                         [0, 1 / 30, 2 / 30])
        ranges = edit.data['keep_source_ranges']
        self.assertEqual([(r['first_source_frame'], r['last_source_frame']) for r in ranges],
                         [(0, 119), (240, 240)])

    def test_logical_cut_preserves_full_rate_states_and_original_clock(self):
        frames = list(MotionEdit(self.path, self.reader).kept_source_frames(self.reader))
        self.assertEqual(len(frames), 121)
        self.assertEqual([int(q[0]) for _, _, q in frames], list(range(120)) + [240])
        self.assertEqual(frames[-1][0], 2.)
        self.assertEqual(frames[-1][1], 4.)

    def test_truncated_full_rate_view_is_rejected(self):
        from itertools import islice
        original = self.reader.frames
        self.reader.frames = lambda: islice(original(), 240)
        with self.assertRaisesRegex(ValueError, 'retained final frame'):
            list(MotionEdit(self.path, self.reader).kept_source_frames(self.reader))

    def test_resampling_inserts_real_intermediate_states_without_filling_cuts(self):
        self.reader.meta['frames'] = 481
        (self.root / 'stream.json').write_text(json.dumps(self.reader.meta))
        times = np.arange(481) / 60
        old = build_timeline(self.reader, times, [0,120,240,360,480],
                             [True,True,False,False,True])
        new = resample_timeline(old, times, speedup=30, fps=30)
        self.assertEqual([f['source_frame'] for f in new['render_frames']], [0,60,120,180,480])
        self.assertEqual(new['keep_source_ranges'], old['keep_source_ranges'])
        self.assertEqual(new['nominal_source_stride'], 60)
        self.assertEqual(new['nominal_speedup'], 30)
        self.assertEqual(old['nominal_speedup'], 60)
        self.path.write_text(json.dumps(new))
        self.assertEqual([int(q[0]) for _,q in MotionEdit(self.path,self.reader).frames(self.reader)],
                         [0,60,120,180,480])

    def test_visual_refinement_never_bridges_previously_removed_source_gap(self):
        from scripts.mosaic_visual_motion import refine_timeline
        self.reader.meta['frames'] = 481
        times = np.arange(481) / 60
        old = build_timeline(self.reader,times,[0,120,240,360,480],
                             [True,True,False,False,True])
        sampled = resample_timeline(old,times,30,30)
        result = refine_timeline(sampled,[True,False,True,False,True],times)
        self.assertEqual([(r['first_source_frame'],r['last_source_frame'])
                          for r in result['keep_source_ranges']],
                         [(0,59),(120,179),(480,480)])
        self.assertEqual([f['source_frame'] for f in result['render_frames']],[0,120,480])

    def test_timeline_and_recording_are_portable(self):
        destination = self.root / 'moved'
        destination.mkdir()
        other = SampleRecording(destination)
        (destination / 'edit.json').write_text(self.path.read_text())
        self.assertEqual(len(list(MotionEdit(destination / 'edit.json', other).frames(other))), 3)

    def test_wrong_recording_is_rejected_even_when_scene_is_same(self):
        (self.root / 'stream.json').write_text(self.path.read_text())
        with self.assertRaisesRegex(ValueError, 'index checksum'):
            MotionEdit(self.path, self.reader)

    def test_reordered_frames_and_retimed_source_are_rejected(self):
        self.data['render_frames'][1]['source_frame'] = 0
        self.write()
        with self.assertRaisesRegex(ValueError, 'ordered edit'):
            MotionEdit(self.path, self.reader)
        self.data['render_frames'][1]['source_frame'] = 60
        self.data['render_frames'][1]['source_time_s'] = .75
        self.write()
        with self.assertRaisesRegex(ValueError, 'timestamp differs'):
            list(MotionEdit(self.path, self.reader).frames(self.reader))

    def test_missing_final_source_state_is_rejected(self):
        self.data['render_frames'][-1]['source_frame'] = 239
        self.write()
        with self.assertRaisesRegex(ValueError, 'endpoints'):
            MotionEdit(self.path, self.reader)


if __name__ == '__main__':
    unittest.main()
