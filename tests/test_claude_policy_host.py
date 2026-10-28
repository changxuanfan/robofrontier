import json
from pathlib import Path
import tempfile
import unittest

from robofrontier.claude.policy_host import CentralizedPolicyHost
from robofrontier.runtime_store import RuntimeStore


class FakeHost:
    def __init__(self, run, actor, profile, base_env):
        self.actor = actor; self.session_id = 'test-session'; self.state = 'new'; self.turn = None; self.prompts = []
    def send(self, prompt):
        self.state = 'active'; self.turn = 'turn-1'; self.prompts.append(prompt)
        return {'threadId': self.session_id, 'turnId': self.turn}
    def observe(self): return {'actor': self.actor, 'thread_id': self.session_id, 'status': self.state, 'turn_id': self.turn}
    def stop(self): self.state = 'unknown'; return self.observe()


class PolicyHostTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.run = Path(self.tmp.name) / 'run'
        public = self.run / 'public/inputs'; public.mkdir(parents=True)
        (public/'task.json').write_text(json.dumps({'family_materials': []}))
        (public/'roles.json').write_text(json.dumps({'topology': 'centralized', 'roles': {'Controller': {'kind': 'controller'}}}))
        for name in ('claude-controller.md', 'policy-rules.md', 'skill/SKILL.md', 'skill/references/tools.md',
                     'skill/references/communication.md', 'skill/references/motion-check.md'):
            path=public/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text('public input')
        cfg=json.loads((Path(__file__).resolve().parents[1]/'configs/harnesses/policy.json').read_text())
        cfg.update(startup_protocol='ready_barrier_v1', engine='claude', topology='centralized', model='fake', effort='high')
        store=RuntimeStore(self.run/'private/state/runtime.sqlite'); store.initialize('run', cfg, ['A']); store.close()
        self.profile={'model':'fake', 'effort':'high', 'env_key':'FAKE'}
        self.calls=[]
        def client(connection, op, data, call_id, **kwargs):
            self.calls.append(op)
            if op == 'bind':
                store=RuntimeStore(self.run/'private/state/runtime.sqlite')
                store.update_agent(data['actor'], thread_id=data['thread_id']); store.close()
                return {'bound':True}
        self.client=client

    def tearDown(self): self.tmp.cleanup()

    def test_binding_once_and_start_requires_ready_and_idle(self):
        host=CentralizedPolicyHost(self.run, self.profile, {}, host_factory=FakeHost, client=self.client, verifier=lambda _: [])
        try:
            host.bind_and_bootstrap()
            self.assertEqual(self.calls, ['bind'])
            self.assertIn('public input', host.host.prompts[0])
            with self.assertRaises(ValueError): host.bind_and_bootstrap()
            with self.assertRaisesRegex(ValueError, 'READY'): host.start_when_ready()
            host.stop()
        finally: host.close()

    def test_unknown_binding_acknowledgement_is_not_retried(self):
        def lost(*args, **kwargs): raise OSError('lost acknowledgement')
        host=CentralizedPolicyHost(self.run, self.profile, {}, host_factory=FakeHost, client=lost, verifier=lambda _: [])
        try:
            with self.assertRaises(OSError): host.bind_and_bootstrap()
            with self.assertRaisesRegex(ValueError, 'Never repeat'): host.bind_and_bootstrap()
        finally: host.close()

    def test_draft_cannot_bypass_engine_topology_or_input_checks(self):
        for verifier in (lambda _: ['input:changed'],):
            with self.assertRaisesRegex(ValueError, 'Frozen inputs changed'):
                CentralizedPolicyHost(self.run, self.profile, {}, host_factory=FakeHost, verifier=verifier)
        with self.assertRaisesRegex(ValueError, 'Profile differs'):
            CentralizedPolicyHost(self.run, {**self.profile, 'model':'wrong'}, {}, host_factory=FakeHost, verifier=lambda _: [])


if __name__ == '__main__': unittest.main()
