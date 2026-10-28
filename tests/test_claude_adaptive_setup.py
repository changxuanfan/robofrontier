"""Trusted setup integration against the real delegation store, no provider calls."""
import json
from pathlib import Path
import threading
import unittest

import test_claude_fork_setup as fixtures
from robofrontier.claude.adaptive_setup import AdaptiveForkSetup
from robofrontier.claude.budget import BudgetLedger
from robofrontier.claude.fork_setup import create_fork
from robofrontier.claude.session_host import ClaudeSessionHost


class AdaptiveSetupTests(unittest.TestCase):
    runner = fixtures.ForkSetupTests.runner
    def setUp(self):
        fixtures.ForkSetupTests.setUp(self)
        # The fixture claims ownership itself; restore to the state before the host claims.
        batch=self.d.batch('batch'); batch['claimed_by']=None; self.d.save(batch)
        self.parent.actor='Controller'
        self.parent.profile={'model':'fake','effort':'high','env_key':'FAKE'}
        self.ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1)
        self.parent.budget_ledger=self.ledger
        roles=self.run/'public/inputs/roles.json'; roles.parent.mkdir(parents=True)
        roles.write_text(json.dumps({'roles':{'Controller':{'kind':'controller'}}}))
        conn=self.run/'private/state/connections'/(self.actor+'.json'); conn.parent.mkdir(parents=True)
        conn.write_text(json.dumps({'schema_version':3,'run_id':'run','actor':self.actor,'token':'fake'}))
        launch=self.run/'private/inputs/claude.json'; launch.parent.mkdir(parents=True)
        launch.write_text(json.dumps({'profile':self.parent.profile}))
        self.child_calls=[]
    def tearDown(self): self.ledger.close(); fixtures.ForkSetupTests.tearDown(self)
    def child_runner(self,*args,**kwargs):
        self.child_calls.append(kwargs)
        return {'process':'exited','exit_code':0,'summary':{'session_id':kwargs['resume'],
                'turn_status':'ended','malformed_lines':0,'total_cost_usd':.02}}
    def fork(self,*args,**kwargs): return create_fork(*args,runner=self.runner,**kwargs)
    def child(self,*args,**kwargs):
        return ClaudeSessionHost.adopt_verified_fork(*args,runner=self.child_runner,_verifier=lambda _:[],**kwargs)
    def coordinator(self, **kwargs):
        return AdaptiveForkSetup(self.store,self.parent,self.ledger,'owner',setup_budget_usd=.1,
                                 child_budget_usd=.2,fork_factory=kwargs.get('fork',self.fork),child_factory=self.child)

    def test_create_bind_activate_and_resume_bound_child(self):
        host=self.coordinator(); result=host.establish('batch')
        self.assertEqual(result['status'],'active')
        self.assertEqual(result['actors'],[self.actor])
        child=self.d.batch('batch')['children'][0]
        self.assertTrue(child['verified']); self.assertEqual(child['start_status'],'sent')
        self.assertEqual(self.store.agent(self.actor)['delegation_state'],'active')
        self.assertEqual(self.store.action_robot(self.actor,'A'),'A')
        with self.assertRaises(ValueError): self.store.action_robot('Controller','A')
        self.assertEqual(host.hosts[self.actor].wait(3)['status'],'idle')
        self.assertEqual(self.child_calls[0]['resume'],child['thread_id'])
        self.assertEqual(self.ledger.snapshot()['reported_spend_usd'],'0.03')
        with self.assertRaises(ValueError): host.establish('batch')

    def test_uncertain_fork_keeps_reserved_robot_and_known_child_id(self):
        def lost(*args,**kwargs):
            def bad_runner(*a,**kw):
                result=self.runner(*a,**kw); result['summary']['total_cost_usd']=None; return result
            return create_fork(*args,runner=bad_runner,**kwargs)
        result=self.coordinator(fork=lost).establish('batch')
        self.assertEqual(result['status'],'needs_review')
        self.assertTrue(self.store.get('dispatch_paused'))
        batch=self.d.batch('batch')
        self.assertEqual(batch['status'],'needs_review')
        self.assertIsNotNone(batch['children'][0]['thread_id'])
        self.assertEqual(self.d.owners()['A'],self.actor)
        with self.assertRaises(ValueError): self.store.action_robot('Controller','A')
        with self.assertRaises(ValueError): self.store.action_robot(self.actor,'A')
        self.assertEqual(self.ledger.snapshot()['status'],'uncertain')
        self.assertEqual(self.child_calls,[])

    def test_stop_interrupts_fork_and_never_starts_child(self):
        entered=threading.Event(); results=[]
        def fork(*args,**kwargs):
            def waiting(*a,**kw):
                entered.set(); self.assertTrue(kw['cancel_event'].wait(3))
                return {'process':'cancelled','exit_code':-9,'summary':{'turn_status':'unknown'}}
            return create_fork(*args,runner=waiting,**kwargs)
        setup=self.coordinator(fork=fork)
        worker=threading.Thread(target=lambda:results.append(setup.establish('batch')))
        worker.start(); self.assertTrue(entered.wait(3)); setup.request_stop(); worker.join(3)
        self.assertFalse(worker.is_alive()); self.assertEqual(results[0]['status'],'needs_review')
        self.assertTrue(self.store.get('dispatch_paused')); self.assertEqual(self.child_calls,[])
        self.assertEqual(self.ledger.snapshot()['status'],'uncertain')


if __name__=='__main__': unittest.main()
