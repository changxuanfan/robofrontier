import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robofrontier.motion_edit import build_timeline, MotionEdit


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
