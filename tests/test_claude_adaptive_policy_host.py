import json
from pathlib import Path
import tempfile
import unittest

from robofrontier.claude.adaptive_policy_host import AdaptivePolicyHost
from robofrontier.claude.budget import BudgetLedger
from robofrontier.runtime_store import RuntimeStore


class FakeHost:
    def __init__(self,run,actor,profile,env,*,max_budget_usd,budget_ledger):
        self.actor=actor; self.run=run; self.profile=profile; self.base_env=env; self.budget_ledger=budget_ledger
        self.session_id='parent-id'; self.state='new'; self.turn=None; self.prompts=[]
    def send(self,prompt): self.prompts.append(prompt); self.state='active'; self.turn='turn'; return {'threadId':self.session_id}
    def observe(self): return {'actor':self.actor,'thread_id':self.session_id,'status':self.state,'turn_id':self.turn}
    def stop(self): self.state='unknown'
    def wait(self,timeout=None): return self.observe()


class AdaptiveHostTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.run=Path(self.tmp.name)/'run'; public=self.run/'public/inputs'
        public.mkdir(parents=True)
        (public/'task.json').write_text(json.dumps({'family_materials':[]}))
        (public/'roles.json').write_text(json.dumps({'topology':'adaptive','roles':{'Controller':{'kind':'controller'}}}))
        for name in ('claude-controller.md','skill/SKILL.md','skill/references/tools.md','skill/references/communication.md',
                     'skill/references/motion-check.md','skill/references/adaptive.md'):
            p=public/name; p.parent.mkdir(parents=True,exist_ok=True); p.write_text('public '+name)
        self.profile={'model':'fake','effort':'high','env_key':'FAKE'}
        launch=self.run/'private/inputs/claude.json'; launch.parent.mkdir(parents=True)
        launch.write_text(json.dumps({'profile':self.profile}))
        cfg=json.loads((Path(__file__).resolve().parents[1]/'configs/harnesses/policy.json').read_text())
        cfg.update(engine='claude',topology='adaptive',model='fake',effort='high')
        store=RuntimeStore(self.run/'private/state/runtime.sqlite'); store.initialize('run',cfg,['A']); store.close()
        self.ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1); self.calls=[]
        def client(conn,op,data,call_id,**kwargs):
            self.calls.append(op); store=RuntimeStore(self.run/'private/state/runtime.sqlite')
            store.update_agent(data['actor'],thread_id=data['thread_id']); store.close(); return {'bound':True}
        self.client=client
    def tearDown(self): self.ledger.close(); self.tmp.cleanup()
    def make(self):
        return AdaptivePolicyHost(self.run,self.profile,{},self.ledger,turn_budget_usd=.2,setup_budget_usd=.1,
                                  host_factory=FakeHost,client=self.client,verifier=lambda _:[])
    def test_binding_readiness_and_multi_role_stop(self):
        host=self.make()
        try:
            host.bind_and_bootstrap(); self.assertIn('skill/references/adaptive.md',host.host.prompts[0])
            with self.assertRaisesRegex(ValueError,'Never repeat'): host.bind_and_bootstrap()
            with self.assertRaisesRegex(ValueError,'READY'): host.start_when_ready()
            host.store.update_agent('Controller',ready=True); host.host.state='idle'
            original=host.dispatcher.admin
            def admin(op,data=None):
                if op=='request-start': host.store.start(); return {'started':True}
                return original(op,data)
            host.dispatcher.admin=admin
            self.assertTrue(host.start_when_ready()['started'])
            child=FakeHost(self.run,'Delegate_test',self.profile,{},max_budget_usd=.2,budget_ledger=self.ledger)
            child.state='active'; host.setup.hosts['Delegate_test']=child
            self.assertEqual(host.stop()['roles'],2)
            self.assertEqual(child.state,'unknown'); self.assertEqual(host.host.state,'unknown')
        finally: host.close()
    def test_profile_and_input_mismatch_rejected(self):
        with self.assertRaisesRegex(ValueError,'Profile differs'):
            AdaptivePolicyHost(self.run,{**self.profile,'model':'other'},{},self.ledger,turn_budget_usd=.2,
                               setup_budget_usd=.1,host_factory=FakeHost,verifier=lambda _:[])
        with self.assertRaisesRegex(ValueError,'Frozen inputs changed'):
            AdaptivePolicyHost(self.run,self.profile,{},self.ledger,turn_budget_usd=.2,setup_budget_usd=.1,
                               host_factory=FakeHost,verifier=lambda _:['changed'])


if __name__=='__main__': unittest.main()
