import unittest
from unittest.mock import patch

from robofrontier.claude.supervisor import CentralizedSupervisor


class Store:
    def __init__(self): self.data = {'lifecycle': 'prepared', 'dispatch_paused': False}; self.ready = False
    def get(self, key, default=None): return self.data.get(key, default)
    def agent(self, actor): return {'ready': self.ready}


class Host:
    def __init__(self): self.state = 'new'; self.stopped = False
    def observe(self): return {'status': self.state}
    def wait(self, timeout=None): return self.observe()


class Dispatcher:
    def __init__(self): self.calls = []
    def admin(self, op, data): self.calls.append((op, data)); return {'sealed': True}


class Policy:
    def __init__(self):
        self.store = Store(); self.host = Host(); self.dispatcher = None; self.binds = 0; self.cycles = 0
    def bind_and_bootstrap(self):
        self.binds += 1; self.dispatcher = Dispatcher(); self.host.state = 'active'
    def start_when_ready(self): self.store.data['lifecycle'] = 'running'
    def cycle(self): self.cycles += 1; return {'paused': False}
    def stop(self): self.host.stopped = True; self.host.state = 'unknown'


class SupervisorTests(unittest.TestCase):
    def supervisor(self, policy, sleep, **kwargs):
        self.now = 0
        def tick(seconds): self.now += seconds; sleep()
        return CentralizedSupervisor(policy, lambda: True, clock=lambda: self.now, sleep=tick,
                                     poll_s=1, startup_timeout_s=3, wall_limit_s=4, **kwargs)

    def test_ready_then_start_dispatch_wall_limit_and_seal(self):
        p = Policy()
        def tick(): p.host.state = 'idle'; p.store.ready = True
        supervisor = self.supervisor(p, tick)
        result = supervisor.run()
        self.assertEqual(result['reason'], 'wall_limit')
        self.assertTrue(result['started']); self.assertGreater(p.cycles, 0)
        self.assertTrue(p.host.stopped); self.assertEqual(p.dispatcher.calls[0][0], 'seal')
        with self.assertRaises(ValueError): supervisor.run()

    def test_startup_timeout_never_starts_or_dispatches(self):
        p = Policy(); result = self.supervisor(p, lambda: None).run()
        self.assertEqual(result['reason'], 'startup_timeout')
        self.assertFalse(result['started']); self.assertEqual(p.cycles, 0)
        self.assertTrue(p.host.stopped)

    def test_unknown_turn_stops_without_resending(self):
        p = Policy()
        result = self.supervisor(p, lambda: setattr(p.host, 'state', 'unknown')).run()
        self.assertEqual(result['reason'], 'session_unknown')
        self.assertEqual(p.binds, 1); self.assertEqual(p.cycles, 0)

    def test_unknown_budget_failure_preserves_bounded_session_evidence(self):
        p=Policy()
        p.host.observe=lambda: {'status':p.host.state,'budget_status':'uncertain',
                               'last_result_subtype':'error_max_budget_usd',
                               'last_turn_reported_cost_usd':.4755318}
        result=self.supervisor(p,lambda: setattr(p.host,'state','unknown')).run()
        self.assertEqual(result['reason'],'session_unknown')
        self.assertEqual(result['session_evidence'],{'budget_status':'uncertain',
            'last_result_subtype':'error_max_budget_usd','last_turn_reported_cost_usd':.4755318})
        self.assertTrue(p.host.stopped)

    def test_owner_loss_and_stop_before_binding_still_try_seal(self):
        p = Policy()
        supervisor = CentralizedSupervisor(p, lambda: False)
        with patch('robofrontier.run_seal.seal_run', return_value={'sealed': True}) as seal:
            result = supervisor.run()
        self.assertEqual(result['reason'], 'owner_unavailable')
        self.assertEqual(p.binds, 0); seal.assert_called_once()

    def test_seal_failure_is_reported_not_overwritten_as_success(self):
        p = Policy()
        def tick():
            p.host.state = 'unknown'
            def fail(*args): raise ValueError('unresolved physical action')
            p.dispatcher.admin = fail
        result = self.supervisor(p, tick).run()
        self.assertEqual(result['reason'], 'session_unknown')
        self.assertIsNone(result['seal']); self.assertEqual(result['seal_error_type'], 'ValueError')


if __name__ == '__main__': unittest.main()
