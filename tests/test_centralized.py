"""Single policy context must retain the same cross-arm concurrency as workers."""
import json
from pathlib import Path
import tempfile
import time
import unittest

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService
from robofrontier.runtime_store import RuntimeStore
from robofrontier.task_config import load_harness


class CentralizedTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        cfg = load_harness(); cfg['topology'] = 'centralized'
        path = root/'config.json'; path.write_text(json.dumps(cfg))
        self.run = prepare_run(root/'run', harness_path=path)
        self.service = RuntimeService(self.run, FakeWorld, fake_capture)
        self.counter = 0

    def tearDown(self):
        if hasattr(self, 'service') and self.service.store:
            self.service.close()
        self.tmp.cleanup()

    def call(self, actor, op, data=None, call_id=None):
        self.counter += 1
        job = self.service.accept(actor, op, data or {}, call_id or 'c'+str(self.counter))
        self.service.pump()
        return job

    def start(self):
        self.call('Controller', 'ready')
        self.call('Meta', 'bind', {'actor':'Controller', 'thread_id':'thread-controller'})
        self.service.store.observe_session('Controller', 'idle', 'turn0', time.time(), 'codex.wait_threads')
        self.assertTrue(self.call('Meta', 'start')['result']['started'])

    def test_one_policy_context_and_explicit_robot_selection(self):
        self.assertEqual(set(self.service.store.agents()), {'Meta', 'Controller'})
        roles = json.loads((self.run/'public/inputs/roles.json').read_text())
        self.assertEqual(roles['roles']['Controller']['robots'], ['A','B','C'])
        ready = self.call('Controller', 'ready')['result']
        self.assertEqual(ready['controlled_robots'], ['A','B','C'])
        self.assertIn('error', self.call('Controller', 'move', {'robot':'A','target_m':[.4,0,.6]})['result'])
        self.assertEqual(self.service.world.executed, [])

    def test_direct_controller_starts_only_after_binding_without_ready_turn(self):
        cfg=dict(self.service.config, startup_protocol='direct_controller_v2')
        path=Path(self.tmp.name)/'direct-config.json';path.write_text(json.dumps(cfg))
        run=prepare_run(Path(self.tmp.name)/'direct-run',harness_path=path)
        service=RuntimeService(run,FakeWorld,fake_capture)
        def call(actor,op,data,ident):
            job=service.accept(actor,op,data,ident);service.pump();return job['result']
        try:
            self.assertIn('error',call('Meta','start',{},'before-bind'))
            self.assertIn('error',call('Controller','gripper',{'robot':'A','state':'close'},'before-start'))
            self.assertTrue(call('Meta','bind',{'actor':'Controller','thread_id':'new-thread'},'bind')['bound'])
            self.assertTrue(call('Meta','start',{},'start')['started'])
            self.assertEqual(service.store.get('lifecycle'),'running')
            self.assertFalse(service.store.agent('Controller')['ready'])
            self.assertEqual(service.store.agent('Controller')['work_state'],'waiting')
            self.assertEqual(call('Controller','gripper',{'robot':'A','state':'close','wait':False},'after-start')['execution_status'],'running')
            ready=call('Controller','ready',{},'ready-after-start')
            self.assertTrue(ready['ready'])
            self.assertTrue(ready['run_started'])
            self.assertEqual(service.store.get('lifecycle'),'running')
        finally:
            service.close()

    def test_parallel_robot_actions_idempotence_and_completion_images(self):
        self.start()
        a = self.call('Controller','gripper',{'robot':'A','state':'close','wait':False},'arm-A')
        b = self.call('Controller','gripper',{'robot':'B','state':'open','wait':False},'arm-B')
        self.assertEqual(a['result']['execution_status'], 'running')
        self.assertEqual(b['result']['execution_status'], 'running')
        self.assertEqual(set(self.service.world.running), {'A','B'})
        self.assertIs(self.call('Controller','gripper',{'robot':'A','state':'close','wait':False},'arm-A'), a)
        self.assertEqual(len(self.service.world.executed), 2)
        self.assertIn('busy', self.call('Controller','gripper',{'robot':'A','state':'open'})['result']['error'].lower())
        self.assertIn('error', self.call('Controller','gripper',{'robot':'missing','state':'open'})['result'])
        active = self.call('Controller','work')['result']['active_executions']
        self.assertEqual({x['arm'] for x in active.values()}, {'A','B'})
        self.service.world.complete('A'); self.service.pump()
        saved = self.call('Controller','job',{'call_id':'arm-A'})['result']
        self.assertEqual(saved['execution']['arm'], 'A')
        self.assertEqual(saved['execution']['status'], 'completed')
        self.assertEqual(saved['original_result']['observation']['camera_names'], ['A_front','A_side'])
        self.assertIn('error', self.call('Controller','finish')['result'])
        self.service.world.complete('B'); self.service.pump()
        self.assertTrue(self.call('Controller','finish')['result']['ended'])
        self.assertTrue((self.run/'private/outputs/evaluation.json').exists())

    def test_permissions_and_recovery_preserve_physical_robot(self):
        self.start()
        self.assertIn('error', self.call('Controller','bind',{'actor':'Controller','thread_id':'other'})['result'])
        self.assertIn('error', self.call('Meta','gripper',{'robot':'A','state':'close'})['result'])
        self.assertIn('error', self.call('Controller','delegate',{'scopes':[{'robots':['A'],'goal':'bounded work','done_when':'verified'}]})['result'])
        self.assertIn('error', self.call('Controller','return_task',{'outcome':'completed','result':'Single has no delegated scope.'})['result'])
        self.service.store.begin_call('accepted-B','Controller','move',{'robot':'B','target_m':[.4,0,.6]})
        self.service.close()
        store = RuntimeStore(self.run/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.actions()['accepted-B']['arm'], 'B')
            self.assertEqual(store.actions()['accepted-B']['status'], 'uncertain')
        finally:
            store.close()

    def test_selected_post_action_cameras_survive_async_completion_and_retry(self):
        self.start()
        for index, cameras in enumerate((['global','C_side'], [])):
            call_id = 'selected-'+str(index)
            data = {'robot':'B','state':'open','wait':False,'cameras':cameras}
            accepted = self.call('Controller','gripper',data,call_id)
            self.assertEqual(accepted['result']['execution_status'],'running')
            self.service.world.complete('B'); self.service.pump()
            saved = self.call('Controller','job',{'call_id':call_id})['result']['original_result']
            self.assertEqual(saved['observation']['camera_names'],cameras)
            self.assertNotIn('cameras',self.service.world.executed[-1][2]['data'])
            self.assertEqual(self.call('Controller','gripper',data,call_id)['result'],saved)
        count = len(self.service.world.executed)
        for cameras in (['not-a-camera'], ['global','global'], 'global'):
            self.assertIn('error',self.call('Controller','gripper',{'robot':'A','state':'close','cameras':cameras})['result'])
        self.assertEqual(len(self.service.world.executed),count)


if __name__ == '__main__':
    unittest.main()
