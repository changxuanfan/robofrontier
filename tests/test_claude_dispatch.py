import json
from pathlib import Path
import tempfile
import unittest

from robofrontier.claude.dispatch import ClaudeDispatcher
from robofrontier.runtime_store import RuntimeStore


class FakeHost:
    actor = 'Controller'
    session_id = 'test-session'
    state = 'idle'
    turn_id = 'initial-turn'
    sent = []
    def observe(self):
        return {'actor': self.actor, 'thread_id': self.session_id, 'status': self.state, 'turn_id': self.turn_id}
    def send(self, prompt):
        self.sent.append(prompt); self.state = 'active'
        return {'threadId': self.session_id}


class ClaudeDispatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = RuntimeStore(Path(self.tmp.name) / 'runtime.sqlite')
        cfg = json.loads((Path(__file__).resolve().parents[1] / 'configs/harnesses/policy.json').read_text())
        cfg.update(engine='claude', topology='centralized', startup_protocol='ready_barrier_v1')
        self.store.initialize('run', cfg, ['A'])
        self.store.update_agent('Controller', ready=True, thread_id='test-session')
        self.store.start()
        self.host = FakeHost(); self.host.state = 'idle'; self.host.sent = []

    def tearDown(self):
        self.store.close(); self.tmp.cleanup()

    def test_scheduler_wakes_idle_controller_without_supplying_task_strategy(self):
        dispatcher = ClaudeDispatcher(self.store, self.host)
        result = dispatcher.cycle()
        self.assertEqual(len(result['dispatched']), 1)
        self.assertEqual(len(self.host.sent), 1)
        self.assertIn('wake_id=', self.host.sent[0])
        self.assertEqual(self.host.state, 'active')
        self.assertEqual(dispatcher.cycle()['dispatched'], [])

    def test_unknown_session_pauses_instead_of_resending(self):
        self.host.state = 'unknown'
        result = ClaudeDispatcher(self.store, self.host).cycle()
        self.assertTrue(result['paused'])
        self.assertEqual(self.host.sent, [])
        self.assertTrue(self.store.get('dispatch_paused'))

    def test_uncertain_send_pauses_without_automatic_retry(self):
        def broken_send(prompt):
            self.host.sent.append(prompt)
            raise OSError('acknowledgement lost')
        self.host.send = broken_send
        dispatcher = ClaudeDispatcher(self.store, self.host)
        result = dispatcher.cycle()
        self.assertTrue(result['paused'])
        self.assertEqual(len(self.host.sent), 1)
        self.assertEqual(dispatcher.cycle()['dispatched'], [])
        self.assertEqual(len(self.host.sent), 1)
        self.assertIn('uncertain', [row['status'] for row in self.store.wake_rows()])

    def test_claude_delegate_binding_is_closed_until_verified(self):
        from robofrontier.runtime_cli import execute
        import time
        cfg = self.store.get('config'); cfg['topology'] = 'adaptive'; self.store.put('config', cfg)
        with self.assertRaisesRegex(ValueError, 'not yet qualified'):
            execute(self.store, 'delegate-bind', {}, 'test-owner', time.time())
        with self.assertRaisesRegex(ValueError, 'not yet qualified'):
            execute(self.store, 'delegate-snapshot-bind', {}, 'test-owner', time.time())
        self.assertEqual(execute(self.store, 'snapshot', {}, 'test-owner', time.time())['config']['engine'], 'claude')

    def test_unqualified_topology_and_wrong_identity_are_rejected(self):
        self.host.session_id = 'other'
        with self.assertRaises(ValueError): ClaudeDispatcher(self.store, self.host)
        self.host.session_id = 'test-session'
        cfg = self.store.get('config'); cfg['topology'] = 'adaptive'; self.store.put('config', cfg)
        with self.assertRaises(ValueError): ClaudeDispatcher(self.store, self.host)


if __name__ == '__main__': unittest.main()
