"""Short actions must not indefinitely postpone the live image cadence."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase,main
from unittest.mock import patch

import numpy as np
from robofrontier.suite.oracle import Trial


class Renderer:
    def __init__(self):self.frames=0
    def update_scene(self,*args,**kwargs):pass
    def render(self):
        self.frames+=1
        return np.zeros((2,2,3),dtype=np.uint8)


class OracleMonitoringTests(TestCase):
    def test_frequent_action_events_do_not_starve_preview(self):
        t=Trial.__new__(Trial);t.live_preview=True;t.renderer=Renderer()
        t.data=SimpleNamespace(time=2.);t.started=0.;t.arms={};t.actions=[]
        with TemporaryDirectory() as directory:
            t.directory=Path(directory)
            for stamp in range(1,9):
                with patch('robofrontier.suite.oracle.time.monotonic',return_value=float(stamp)):
                    t._heartbeat('action_start')
            self.assertEqual(t.renderer.frames,2)
            self.assertEqual(t.last_preview,6.)
            self.assertEqual(t.last_heartbeat,8.)
            self.assertTrue((t.directory/'live.png').is_file())

    def test_scratch_copy_cannot_publish_hypothetical_state(self):
        from copy import copy
        t=Trial.__new__(Trial);t.data=SimpleNamespace(time=2.);t._live_data=t.data
        t.live_preview=True;t.renderer=Renderer();t.arms={};t.actions=[];t.started=0.
        with TemporaryDirectory() as directory:
            t.directory=Path(directory);probe=copy(t);probe.data=SimpleNamespace(time=99.)
            probe._heartbeat('planning')
            self.assertEqual(t.renderer.frames,0)
            self.assertFalse((t.directory/'status.json').exists())
        with self.assertRaisesRegex(RuntimeError,'Planning scratch'):probe.step()
        with self.assertRaisesRegex(RuntimeError,'Planning scratch'):probe.execute('forbidden',{})


if __name__=='__main__':main()
