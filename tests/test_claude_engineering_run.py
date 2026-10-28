import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robofrontier.claude.engineering_run import run_engineering


class Store:
    def __init__(self, attached=False): self.attached = attached
    def get(self, key): return self.attached if key == 'execution_owner_attached' else None


class Policy:
    def __init__(self, *args, **kwargs): self.store = Store(); self.closed = False
    def close(self): self.closed = True


class Owner:
    pid = 1234
    def __init__(self, status): self.status = status
    def poll(self): return self.status


class EngineeringRunTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.run = Path(self.tmp.name) / 'run'
        for directory in ('private/inputs', 'private/outputs', 'private/state/connections'):
            (self.run / directory).mkdir(parents=True)
        (self.run / 'private/inputs/claude.json').write_text(json.dumps({
            'schema': 'robofrontier.claude-launch-draft.v1', 'qualified': False, 'execution_available': False,
            'profile': {'env_key': 'FAKE_KEY', 'model': 'fake', 'effort': 'high'}}))
        (self.run / 'private/inputs/manifest.json').write_text(json.dumps({
            'status': 'engineering draft; not qualified or runnable', 'engine': 'claude'}))
        (self.run / 'private/inputs/harness.json').write_text(json.dumps({'engine':'claude','topology':'centralized'}))
        (self.run / 'private/state/connections/Meta.json').write_text('{}')
        self.env = {'FAKE_KEY': 'secret', 'PATH': '/bin'}
        self.verify = patch('robofrontier.claude.engineering_run.verify_inputs', return_value=[])
        self.verify.start()

    def tearDown(self): self.verify.stop(); self.tmp.cleanup()

    def run_case(self, **kwargs):
        return run_engineering(self.run, self.env, turn_budget_usd=.5, total_budget_usd=2, wall_limit_s=30,
                               policy_factory=Policy, **kwargs)

    def test_owner_failure_consumes_claim_and_prevents_replay(self):
        observed = []
        def create(*args, **kwargs):
            observed.append(kwargs['env']); return Owner(1)
        result = self.run_case(owner_factory=create)
        self.assertEqual(result['status'], 'needs_review')
        self.assertEqual(result['reason'], 'launch_error')
        self.assertNotIn('FAKE_KEY', observed[0])
        status = (self.run / 'private/outputs/claude-engineering-status.json').read_text()
        with self.assertRaisesRegex(ValueError, 'never restart'):
            self.run_case(owner_factory=create)
        self.assertEqual(status, (self.run / 'private/outputs/claude-engineering-status.json').read_text())
        self.assertEqual(len(observed), 1)

    def test_stop_and_non_draft_fail_before_owner_or_claim(self):
        (self.run/'STOP').touch()
        with self.assertRaisesRegex(ValueError, 'stopped'):
            self.run_case(owner_factory=lambda *a, **k: self.fail('must not start'))
        self.assertFalse((self.run/'private/state/claude-engineering-claim.json').exists())
        (self.run/'STOP').unlink()
        path = self.run/'private/inputs/claude.json'; data = json.loads(path.read_text()); data['qualified'] = True
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'engineering drafts'):
            self.run_case(owner_factory=lambda *a, **k: self.fail('must not start'))

    def test_ready_owner_delegates_to_supervisor_and_records_seal(self):
        class Attached(Policy):
            def __init__(self, *a, **k): super().__init__(*a, **k); self.store.attached = True
        (self.run/'private/state/connections/Meta.json').write_text('{"url":"http://127.0.0.1:1234"}')
        class Supervisor:
            def __init__(self, policy, owner_alive, **kwargs): self.owner_alive = owner_alive
            def run(self):
                assert self.owner_alive()
                return {'reason': 'world_ended', 'seal': {'sealed': True}}
        with patch('robofrontier.claude.engineering_run.CentralizedSupervisor', Supervisor):
            result = run_engineering(self.run, self.env, turn_budget_usd=.5, total_budget_usd=2, wall_limit_s=30,
                                     policy_factory=Attached, owner_factory=lambda *a, **k: Owner(None))
        self.assertEqual(result['status'], 'needs_review')
        self.assertFalse(result['scored'])
        self.assertTrue(result['owner_alive'])  # live owner is recorded, never killed implicitly

    def test_adaptive_launch_creates_shared_run_budget_before_world(self):
        (self.run/'private/inputs/harness.json').write_text(json.dumps({'engine':'claude','topology':'adaptive'}))
        seen = []
        def adaptive(run, profile, env, ledger, **kwargs):
            seen.append((ledger.snapshot()['limit_usd'], kwargs))
            return Policy()
        result = run_engineering(self.run, self.env, turn_budget_usd=.5, total_budget_usd=2, wall_limit_s=30,
                                 adaptive_policy_factory=adaptive, owner_factory=lambda *a, **k: Owner(1))
        self.assertEqual(result['status'], 'needs_review')
        self.assertEqual(seen[0][0], '2')
        self.assertEqual(seen[0][1]['setup_budget_usd'], .5)
        self.assertTrue((self.run/'private/state/claude-budget.sqlite').exists())

    def test_requested_unreadable_recording_requires_review_even_after_seal(self):
        config = self.run/'private/inputs/harness.json'
        config.write_text(json.dumps({'engine':'claude','topology':'centralized','record_realtime_video':True}))
        (self.run/'private/state/connections/Meta.json').write_text('{"url":"http://127.0.0.1:1234"}')
        class Attached(Policy):
            def __init__(self,*a,**k): super().__init__(*a,**k); self.store.attached = True
        class ExitingOwner(Owner):
            def wait(self,timeout): self.status=0
        class Supervisor:
            def __init__(self,*a,**k): pass
            def run(self): return {'reason':'world_ended','seal':{'sealed':True,'video':{'readable':False}}}
        with patch('robofrontier.claude.engineering_run.CentralizedSupervisor', Supervisor):
            result=run_engineering(self.run,self.env,turn_budget_usd=.5,total_budget_usd=2,wall_limit_s=30,
                                   policy_factory=Attached,owner_factory=lambda *a,**k: ExitingOwner(None))
        self.assertTrue(result['video_required'])
        self.assertFalse(result['owner_alive'])
        self.assertEqual(result['status'],'needs_review')


if __name__ == '__main__': unittest.main()
