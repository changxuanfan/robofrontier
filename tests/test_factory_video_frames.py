import json
from pathlib import Path
import tempfile
import unittest

from scripts.factory_video_frames import prune_frames


class FrameRetentionChecks(unittest.TestCase):
    def test_failed_video_keeps_every_frame(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'frame-000000.png'
            p.write_bytes(b'original')
            with self.assertRaises(ValueError):
                prune_frames(d, dict(animation=dict(full_decode_passed=False)))
            self.assertEqual(p.read_bytes(), b'original')

    def test_only_verified_sequence_is_pruned_and_three_stills_survive(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p / 'factory.mp4').write_bytes(b'validated-video')
            (p / 'important.png').write_bytes(b'not-a-frame')
            for i in range(5):
                (p / f'frame-{i:06d}.png').write_bytes(bytes([i]))
            r = dict(animation=dict(full_decode_passed=True,frames=5),views=[{} for _ in range(5)])
            result = prune_frames(p,r)
            self.assertEqual(result['removed_frames'],2)
            self.assertEqual(result['reclaimed_bytes'],2)
            self.assertEqual(sorted(x.name for x in (p/'stills').iterdir()),
                ['frame-000000.png','frame-000002.png','frame-000004.png'])
            self.assertFalse(list(p.glob('frame-*.png')))
            self.assertEqual((p/'important.png').read_bytes(),b'not-a-frame')
            self.assertEqual(json.loads((p/'report.json').read_text())['frame_storage']['status'],'complete')
