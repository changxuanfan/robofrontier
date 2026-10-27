import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from robofrontier.suite.dial_pose_timeline import EditedMotion, PRESENTATION


ROOT = Path(__file__).resolve().parents[1]


class DialEntrypointTests(unittest.TestCase):
    def test_presentation_finishes_once_instead_of_looping(self):
        self.assertEqual(PRESENTATION['speedup'],10)
        self.assertEqual(PRESENTATION['fps'],30)
        self.assertFalse(PRESENTATION['repeat'])
        self.assertEqual(PRESENTATION['stop_after'],'all_stations_complete')
        self.assertEqual(PRESENTATION['finished_station'],'hold_final_pose')

    def test_old_three_digit_case_is_rejected_before_creating_a_world(self):
        with tempfile.TemporaryDirectory() as td:
            output=Path(td)/'unused'
            result=subprocess.run([sys.executable,'-m','scripts.run_blind_dial_oracle',
                '--case','dial-n04-d1','--output',str(output),'--codebook',str(Path(td)/'not-read.json')],
                cwd=ROOT,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('invalid choice',result.stderr)
            self.assertFalse(list(Path(td).iterdir()))

    def test_retired_shoulder_strategy_is_rejected_before_creating_a_world(self):
        with tempfile.TemporaryDirectory() as td:
            result=subprocess.run([sys.executable,'-m','scripts.commission_best5_dials',
                '--case','dial-best5-n04-d1','--output',str(Path(td)/'unused'),
                '--blue-capture','shoulder'],cwd=ROOT,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('unrecognized arguments',result.stderr)
            self.assertFalse(list(Path(td).iterdir()))

    def test_published_source_schedule_survives_namespace_move(self):
        folder=ROOT/'outputs/robo-factory/dial-best5-edited-10x-20261027/dial-best5-n04-d2'
        if not (folder/'report.json').exists():self.skipTest('Published artifacts unavailable')
        report=json.loads((folder/'report.json').read_text())
        view=EditedMotion(json.loads(Path(report['edit_file']).read_text())['recording'],report['edit_file'])
        self.assertEqual(view.video_indices(30,10).tolist(),report['video_source_frame_indices'])

    def test_batch_rejects_old_protocol_before_allocating_output(self):
        from scripts.render_edited_dial_batch import run
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)
            selection=folder/'old.json'
            selection.write_text(json.dumps(dict(cases=[dict(case_id='dial-n04-d1')])))
            with self.assertRaisesRegex(ValueError,'R11 BEST5'):
                run(selection,folder/'unused-edits',folder/'unused-scene',folder/'unused-output')
            self.assertFalse((folder/'unused-output').exists())


if __name__=='__main__':unittest.main()
