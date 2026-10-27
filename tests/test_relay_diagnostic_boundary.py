"""Full-scene diagnostics must not mix snapshots across active repairs."""
import json
from pathlib import Path
import tempfile
import unittest

from scripts.diagnose_relay_descent import diagnostic_world


class DiagnosticBoundaryTests(unittest.TestCase):
    def test_queued_and_accepted_unresolved_moves_stop_before_scene_read(self):
        for folder in ('requests','accepted'):
            with self.subTest(folder=folder),tempfile.TemporaryDirectory() as directory:
                run=Path(directory);queue=run/'recovery'/folder;queue.mkdir(parents=True)
                path=queue/'original.json';payload=json.dumps({'id':'original','op':'carry'})
                path.write_text(payload)
                with self.assertRaisesRegex(RuntimeError,'Original owner repair unresolved'):
                    diagnostic_world(run)
                self.assertEqual(path.read_text(),payload)


if __name__=='__main__':unittest.main()
