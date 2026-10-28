import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from robofrontier.claude.adaptive_dispatch import AdaptiveDispatcher
from robofrontier.delegation import Delegations
from robofrontier.runtime_store import RuntimeStore


class Host:
    def __init__(self,actor,session): self.actor=actor; self.session_id=session; self.state='idle'; self.sent=[]
    def observe(self): return {'actor':self.actor,'thread_id':self.session_id,'status':self.state,'turn_id':'turn-'+self.actor}
    def send(self,prompt):
        self.sent.append(prompt); self.state='active'; return {'threadId':self.session_id}


class Setup:
    def __init__(self,store,parent): self.store=store; self.parent=parent; self.hosts={}; self.owner='adaptive-test'
    def establish(self,batch): raise AssertionError('No unexpected fork')


class AdaptiveDispatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.store=RuntimeStore(Path(self.tmp.name)/'runtime.sqlite')
        cfg=json.loads((Path(__file__).resolve().parents[1]/'configs/harnesses/policy.json').read_text())
        cfg.update(engine='claude',topology='adaptive')
        self.store.initialize('run',cfg,['A']); self.store.update_agent('Controller',thread_id='parent',ready=True)
        self.store.start(); self.d=Delegations(self.store)
        batch=self.d.create('Controller','batch',[{'robots':['A'],'goal':'g','done_when':'d'}])
        self.child=batch['children'][0]['actor']; self.d.claim('batch','adaptive-test')
        self.d.bind('batch',self.child,'child',{'source_thread_id':'parent','source_sha256':'x'},'adaptive-test')
        self.d.activate('batch','adaptive-test'); self.d.started('batch',self.child,'sent','adaptive-test')
        self.store.update_agent(self.child,work_state='waiting',next_check=time.time()-1,work_revision=1)
        self.setup=Setup(self.store,Host('Controller','parent')); self.setup.hosts[self.child]=Host(self.child,'child')
    def tearDown(self): self.store.close(); self.tmp.cleanup()

    def test_wake_is_delivered_to_matching_child_and_parent(self):
        dispatcher=AdaptiveDispatcher(self.store,self.setup)
        result=dispatcher.cycle()
        self.assertFalse(result['paused'])
        self.assertEqual(len(self.setup.hosts[self.child].sent),1)
        self.assertEqual(len(self.setup.parent.sent),1)
        self.assertEqual(len(result['dispatched']),2)
        self.assertEqual(self.setup.hosts[self.child].state,'active')
        self.assertEqual(dispatcher.cycle()['dispatched'],[])

    def test_wrong_or_missing_child_host_pauses_without_sending(self):
        self.setup.hosts[self.child].session_id='different'
        result=AdaptiveDispatcher(self.store,self.setup).cycle()
        self.assertTrue(result['paused']); self.assertTrue(self.store.get('dispatch_paused'))
        self.assertEqual(self.setup.hosts[self.child].sent,[])

    def test_lost_wake_ack_is_uncertain_and_never_retried(self):
        host=self.setup.hosts[self.child]
        def fail(prompt): host.sent.append(prompt); raise OSError('lost ack')
        host.send=fail
        dispatcher=AdaptiveDispatcher(self.store,self.setup)
        result=dispatcher.cycle()
        self.assertTrue(result['paused']); self.assertEqual(len(host.sent),1)
        self.assertIn('uncertain',[w['status'] for w in self.store.wake_rows()])
        self.assertEqual(dispatcher.cycle()['dispatched'],[])
        self.assertEqual(len(host.sent),1)

    def test_claimed_fork_is_not_recreated_on_supervisor_change(self):
        self.d.return_task(self.child,'completed','report')
        batch=self.d.create('Controller','second',[{'robots':['A'],'goal':'next','done_when':'d'}])
        self.d.claim('second','previous-owner')
        dispatcher=AdaptiveDispatcher(self.store,self.setup)
        result=dispatcher.cycle()
        self.assertTrue(result['paused'])
        self.assertEqual(self.d.batch('second')['status'],'needs_review')
        self.assertEqual(self.d.owners()['A'],batch['children'][0]['actor'])

    def test_slow_fork_setup_does_not_block_dispatch_cycle(self):
        self.d.return_task(self.child,'completed','report')
        self.d.create('Controller','second',[{'robots':['A'],'goal':'next','done_when':'d'}])
        entered=threading.Event(); release=threading.Event()
        def slow(batch):
            entered.set(); release.wait(3); return {'status':'needs_review'}
        self.setup.establish=slow
        dispatcher=AdaptiveDispatcher(self.store,self.setup)
        result=dispatcher.cycle(); self.assertEqual(result['setup'],'pending')
        self.assertTrue(entered.wait(3))
        self.assertEqual(dispatcher.cycle()['setup'],'pending')
        release.set(); self.assertTrue(dispatcher.wait_setup(3))
        self.assertTrue(dispatcher.cycle()['paused'])


if __name__=='__main__': unittest.main()
