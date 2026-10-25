import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robofrontier.suite.transport_recovery import EngineeringRecovery


class EngineeringRecoveryTests(unittest.TestCase):
    def test_foreign_or_state_edit_input_never_reaches_execution(self):
        for request in (dict(id='x',job='bad',scene_sha256='other'),
                        dict(id='x',job='bad',scene_sha256='correct',qpos=[0])):
            with self.subTest(request=request),TemporaryDirectory() as tmp:
                handler=EngineeringRecovery(tmp)
                handler.observation=Mock(return_value=dict(failed_jobs={},accepted_goals={}))
                (handler.path/'inbox/x.json').write_text(json.dumps(request))
                t=SimpleNamespace(time=0.,task={'scene_sha256':'correct'},event=Mock())
                runtime=SimpleNamespace(recover=Mock())
                self.assertFalse(handler.poll(t,runtime,[],[],force=True))
                runtime.recover.assert_not_called();self.assertTrue((handler.path/'rejected/x.json').exists())

    def test_frontier_polling_is_bounded_not_every_physics_step(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            handler.observation=Mock(return_value=dict(failed_jobs={},accepted_goals={}))
            t=SimpleNamespace(time=0.,task={'scene_sha256':'correct'})
            handler.poll(t,SimpleNamespace(),[],[],force=True)
            for _ in range(100):handler.poll(t,SimpleNamespace(),[],[])
            self.assertEqual(handler.observation.call_count,1)


if __name__=='__main__':unittest.main()
