"""Bounded delegation uses the real service seam with a synthetic executor."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService
from robofrontier.task_config import load_harness
from robofrontier.delegation import Delegations


class DelegationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        cfg = load_harness(); cfg['topology'] = 'adaptive'
        path = root/'config.json'; path.write_text(json.dumps(cfg))
        self.run = prepare_run(root/'run', harness_path=path)
        self.service = RuntimeService(self.run, FakeWorld, fake_capture)
        self.n = 0
        self.call('Controller', 'ready')
        self.call('Meta', 'bind', {'actor':'Controller', 'thread_id':'parent'})
        self.service.store.observe_session('Controller','idle','ready',time.time(),'codex.wait_threads')
        assert self.call('Meta','start')['result']['started']
        self.d = Delegations(self.service.store)

    def tearDown(self):
        if hasattr(self, 'service') and self.service.store:
            self.service.close()
        self.tmp.cleanup()

    def call(self, actor, op, data=None, call_id=None):
        self.n += 1
        job = self.service.accept(actor,op,data or {},call_id or 'c'+str(self.n))
        self.service.pump()
        return job

    def delegate(self, robots=('A',), call_id='batch'):
        return self.call('Controller','delegate',{'scopes':[
            {'robots':[r],'goal':'test outcome','done_when':'verified evidence'} for r in robots]},call_id)

    def activate(self, batch='batch'):
        self.d.claim(batch,'test-manager')
        value = self.d.batch(batch)
        for i, child in enumerate(value['children']):
            self.d.bind(batch,child['actor'],'child-'+str(i),{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'same'},'test-manager')
        self.d.activate(batch,'test-manager')
        for child in value['children']:
            self.d.started(batch,child['actor'],'sent','test-manager')
        self.service.pump()
        return value['children']

    def test_async_batch_dedup_and_other_robot_can_work(self):
        job = self.delegate(('A','B'))
        self.assertIsNone(job['result'])
        self.assertIs(self.delegate(('A','B')),job)
        self.assertEqual(len(self.d.batch('batch')['children']),2)
        self.assertIn('error',self.call('Controller','gripper',{'robot':'A','state':'open'})['result'])
        c = self.call('Controller','gripper',{'robot':'C','state':'open','wait':False})
        self.assertEqual(c['result']['execution_status'],'running')
        children=self.activate()
        self.assertEqual(job['result']['delegation']['status'],'active')
        self.assertEqual(set(self.service.tokens)&{x['actor'] for x in children},{x['actor'] for x in children})
        for child in children:
            result=self.call(child['actor'],'gripper',{'robot':child['robots'][0],'state':'open','wait':False})
            self.assertEqual(result['result']['execution_status'],'running')
        self.assertEqual(set(self.service.world.running),{'A','B','C'})

    def test_first_return_releases_only_own_robots_and_wakes_parent(self):
        self.delegate(('A','B')); a,b=self.activate()
        wait=self.call('Controller','wait_events',{'timeout_s':30})
        result=self.call(a['actor'],'return_task',{'outcome':'completed','result':'RGB evidence reference'})
        self.assertTrue(result['result']['returned'])
        self.assertTrue(wait['done'].is_set())
        self.assertEqual(self.service.store.action_robot('Controller','A'),'A')
        self.assertIn('error',self.call('Controller','gripper',{'robot':'B','state':'open'})['result'])
        self.assertIn('error',self.call(a['actor'],'gripper',{'robot':'A','state':'open'})['result'])
        self.assertIn('error',self.call(b['actor'],'finish')['result'])
        self.assertIn('error',self.call('Controller','finish')['result'])
        self.assertEqual(self.service.store.agent(b['actor'])['delegation_state'],'active')

    def test_no_handoff_of_running_or_accepted_actions(self):
        self.call('Controller','gripper',{'robot':'A','state':'open','wait':False})
        self.assertIn('error',self.delegate()['result'])
        self.service.world.complete('A');self.service.pump()
        pending=self.service.accept('Controller','gripper',{'robot':'A','state':'open'},'queued')
        with self.assertRaises(ValueError):
            self.d.create('Controller','direct',[{'robots':['A'],'goal':'x','done_when':'y'}])
        self.service.pump();self.assertIn('A',self.service.world.running)
        self.service.world.complete('A');self.service.pump()
        self.delegate(call_id='new'); child=self.activate('new')[0]
        self.call(child['actor'],'gripper',{'robot':'A','state':'close','wait':False})
        self.assertIn('error',self.call(child['actor'],'return_task',{'outcome':'completed','result':'not done yet'})['result'])

    def test_same_cutoff_and_uncertain_native_creation_fail_closed(self):
        self.delegate(('A','B')); self.d.claim('batch','manager')
        a,b=self.d.batch('batch')['children']
        self.d.bind('batch',a['actor'],'child-a',{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'one'},'manager')
        with self.assertRaises(ValueError):
            self.d.bind('batch',b['actor'],'child-b',{'source_thread_id':'parent','source_ordinal':43,'source_sha256':'two'},'manager')
        self.d.uncertain('batch','fork result unknown','manager');self.service.pump()
        self.assertEqual(self.d.batch('batch')['status'],'needs_review')
        self.assertIn('error',self.service.requests['batch']['result'])
        with self.assertRaises(ValueError):self.d.claim('batch','another-manager')
        self.assertIn('error',self.call(a['actor'],'gripper',{'robot':'A','state':'open'})['result'])

    def test_scope_permissions_overlap_nested_and_routing(self):
        bad=self.call('Controller','delegate',{'scopes':[
            {'robots':['A'],'goal':'x','done_when':'y'}, {'robots':['A'],'goal':'z','done_when':'q'}]})
        self.assertIn('error',bad['result'])
        self.delegate(('A','B'));a,b=self.activate()
        self.assertIn('error',self.call(a['actor'],'delegate',{'scopes':[]})['result'])
        self.assertIn('error',self.call(a['actor'],'gripper',{'robot':'C','state':'open'})['result'])
        self.assertIn('error',self.call(a['actor'],'send',{'to':[b['actor']],'text':'x'})['result'])
        self.assertIn('message_id',self.call(a['actor'],'send',{'to':['Controller'],'text':'early enabling evidence'})['result'])

    def test_return_before_native_start_ack_preserves_finished_state(self):
        self.delegate();self.d.claim('batch','manager')
        a=self.d.batch('batch')['children'][0]
        self.d.bind('batch',a['actor'],'child',{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'same'},'manager')
        self.d.activate('batch','manager')
        self.call(a['actor'],'return_task',{'outcome':'completed','result':'fast child evidence'})
        self.d.started('batch',a['actor'],'sent','manager');self.service.pump()
        self.assertEqual(self.service.store.agent(a['actor'])['work_state'],'available')
        self.assertEqual(self.d.batch('batch')['status'],'finished')
        self.assertEqual(self.service.requests['batch']['result']['delegation']['status'],'finished')

    def test_queued_child_command_prevents_early_return_and_retry_is_idempotent(self):
        self.delegate();a=self.activate()[0]['actor']
        self.service.accept(a,'gripper',{'robot':'A','state':'open'},'old-child-command')
        with self.assertRaises(ValueError):self.d.return_task(a,'completed','premature')
        self.service.pump();self.service.world.complete('A');self.service.pump()
        data={'outcome':'completed','result':'terminal evidence'}
        one=self.call(a,'return_task',data,'return-once')
        self.assertIs(self.call(a,'return_task',data,'return-once'),one)
        self.assertEqual(len([e for e in self.service.store.events() if e['kind']=='DELEGATION_RETURNED']),1)

    def test_full_reports_are_files_and_parallel_notifications_stay_small(self):
        self.delegate(('A','B','C'));children=self.activate()
        report='视觉分析与证据。'*1500
        for child in children:
            data={'outcome':'completed','result':report}
            reply=self.call(child['actor'],'return_task',data,'return-'+child['actor'])['result']
            self.assertTrue(reply['returned'])
            ref=Path(reply['delegation']['result_ref'])
            self.assertEqual(ref.parent,self.run/'public/results')
            saved=json.loads(ref.read_text())
            self.assertEqual(saved['result'],report)
            self.assertEqual(saved['actor'],child['actor'])
            self.assertEqual(saved['scope']['goal'],'test outcome')
            self.assertNotIn(report,json.dumps(reply,ensure_ascii=False))
            before=ref.read_bytes()
            self.call(child['actor'],'return_task',data,'return-'+child['actor'])
            self.assertEqual(ref.read_bytes(),before)
            failed=self.call(child['actor'],'return_task',{'outcome':'completed','result':'overwrite attempt'})['result']
            self.assertIn('error',failed);self.assertEqual(ref.read_bytes(),before)
        delivery=self.service.store.delivery('Controller')
        self.assertEqual(len(delivery['system_events']),3)
        self.assertLess(len(json.dumps(delivery,ensure_ascii=False).encode()),self.service.config['max_batch_bytes'])
        self.assertNotIn(report,json.dumps(self.d.batches(),ensure_ascii=False))
        self.assertNotIn(report,json.dumps(self.call('Controller','work')['result'],ensure_ascii=False))

    def test_report_write_failure_keeps_ownership_and_sends_no_completion(self):
        self.delegate();child=self.activate()[0]['actor']
        with patch('robofrontier.run_setup.write_json',side_effect=OSError('synthetic disk failure')):
            reply=self.call(child,'return_task',{'outcome':'completed','result':'evidence'})['result']
        self.assertIn('error',reply)
        self.assertEqual(self.d.owners()['A'],child)
        self.assertEqual(self.service.store.pending('Controller')['signals'],0)
        self.assertEqual(self.service.store.agent(child)['delegation_state'],'active')

    def test_terminal_return_resolves_lost_initial_ack_without_refork(self):
        self.delegate();self.d.claim('batch','manager');child=self.d.batch('batch')['children'][0]
        self.d.bind('batch',child['actor'],'child',{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'same'},'manager')
        self.d.activate('batch','manager')
        self.call(child['actor'],'return_task',{'outcome':'completed','result':'terminal evidence'})
        self.d.uncertain('batch','native input acknowledgement lost','manager');self.service.pump()
        self.assertEqual(self.service.requests['batch']['result']['delegation']['status'],'finished')
        self.assertFalse(self.service.store.get('dispatch_paused',False))
        self.assertEqual(self.service.store.agent(child['actor'])['work_state'],'available')

    def test_finished_child_lost_ack_does_not_block_starting_other_child(self):
        self.delegate(('A','B'));self.d.claim('batch','manager')
        a,b=self.d.batch('batch')['children']
        for i,c in enumerate((a,b)):
            self.d.bind('batch',c['actor'],'child-'+str(i),{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'same'},'manager')
        self.d.activate('batch','manager')
        self.call(a['actor'],'return_task',{'outcome':'completed','result':'A terminal'})
        self.d.started('batch',a['actor'],'uncertain','manager')
        self.d.started('batch',b['actor'],'sent','manager');self.service.pump()
        self.assertEqual(self.service.requests['batch']['result']['delegation']['status'],'active')
        self.assertFalse(self.service.store.get('dispatch_paused',False))

    def test_known_native_id_survives_provenance_failure_without_authorization(self):
        from robofrontier.runtime_cli import execute
        self.delegate();self.d.claim('batch','manager');a=self.d.batch('batch')['children'][0]['actor']
        tid='11111111-1111-4111-8111-111111111111'
        with patch('robofrontier.runtime_cli.fork_provenance',side_effect=ValueError('lineage unavailable')):
            with self.assertRaises(ValueError):
                execute(self.service.store,'delegate-bind',{'id':'batch','actor':a,'thread_id':tid},'manager',time.time())
        self.assertEqual(self.service.store.agent(a)['thread_id'],tid)
        self.assertIn(tid,json.dumps(self.service.store.events()))
        with self.assertRaises(ValueError):self.d.activate('batch','manager')
        self.assertIn('error',self.call(a,'gripper',{'robot':'A','state':'open'})['result'])

    def test_snapshot_lineage_and_children_require_same_snapshot_boundary(self):
        from robofrontier.runtime_cli import execute
        self.delegate(('A','B'));self.d.claim('batch','manager')
        children=self.d.batch('batch')['children']
        snapshot='22222222-2222-4222-8222-222222222222'
        anchor={'source_thread_id':'parent','source_ordinal':13,'source_sha256':'controller-cut'}
        with patch('robofrontier.runtime_cli.fork_provenance',return_value=anchor):
            execute(self.service.store,'delegate-snapshot-bind',{'id':'batch','thread_id':snapshot},'manager',time.time())
        self.assertEqual(self.d.batch('batch')['snapshot_provenance'],anchor)
        for i, child in enumerate(children):
            proof={'source_thread_id':snapshot,'source_ordinal':17,'source_sha256':'snapshot-cut'}
            with patch('robofrontier.runtime_cli.fork_provenance',return_value=proof) as read:
                execute(self.service.store,'delegate-bind',{'id':'batch','actor':child['actor'],'thread_id':'child-'+str(i)},'manager',time.time())
                read.assert_called_once_with(snapshot,'child-'+str(i))
        self.d.activate('batch','manager')
        self.assertEqual(self.d.batch('batch')['status'],'active')

    def test_snapshot_id_persists_on_failed_lineage_and_cannot_activate(self):
        from robofrontier.runtime_cli import execute
        self.delegate();self.d.claim('batch','manager')
        snapshot='22222222-2222-4222-8222-222222222222'
        with patch('robofrontier.runtime_cli.fork_provenance',side_effect=ValueError('lineage unavailable')):
            with self.assertRaisesRegex(ValueError,'lineage unavailable'):
                execute(self.service.store,'delegate-snapshot-bind',{'id':'batch','thread_id':snapshot},'manager',time.time())
        self.assertEqual(self.d.batch('batch')['snapshot_thread_id'],snapshot)
        self.assertNotIn('snapshot_provenance',self.d.batch('batch'))
        with self.assertRaisesRegex(ValueError,'Snapshot is not verified'):
            self.d.bind('batch',self.d.batch('batch')['children'][0]['actor'],'child',
                        {'source_thread_id':snapshot,'source_ordinal':17,'source_sha256':'x'},'manager')
        with self.assertRaises(ValueError):self.d.activate('batch','manager')

    def test_snapshot_rejects_wrong_parent_or_different_child_boundary(self):
        self.delegate(('A','B'));self.d.claim('batch','manager')
        a,b=self.d.batch('batch')['children'];snapshot='anchor'
        self.d.snapshot_created('batch',snapshot,'manager')
        with self.assertRaisesRegex(ValueError,'wrong source'):
            self.d.snapshot_bind('batch',snapshot,{'source_thread_id':'other','source_ordinal':13,'source_sha256':'x'},'manager')
        self.d.snapshot_bind('batch',snapshot,{'source_thread_id':'parent','source_ordinal':13,'source_sha256':'x'},'manager')
        self.d.bind('batch',a['actor'],'child-a',{'source_thread_id':snapshot,'source_ordinal':17,'source_sha256':'a'},'manager')
        with self.assertRaisesRegex(ValueError,'exact source history boundary'):
            self.d.bind('batch',b['actor'],'child-b',{'source_thread_id':snapshot,'source_ordinal':18,'source_sha256':'b'},'manager')
        with self.assertRaises(ValueError):self.d.activate('batch','manager')

    def test_setup_deadline_returns_unknown_without_releasing_or_reforking(self):
        self.delegate();b=self.d.batch('batch');b['setup_deadline']=time.time()-1;self.d.save(b)
        self.service.pump()
        self.assertIn('error',self.service.requests['batch']['result'])
        self.assertEqual(self.d.batch('batch')['status'],'needs_review')
        self.assertTrue(self.service.store.get('dispatch_paused'))
        self.assertNotEqual(self.d.owners()['A'],'Controller')
        self.assertIn('error',self.call('Controller','gripper',{'robot':'C','state':'open'})['result'])

    def test_finished_delegates_do_not_accept_new_business_work(self):
        self.delegate();child=self.activate()[0]['actor']
        self.call(child,'return_task',{'outcome':'completed','result':'evidence'})
        self.assertIn('error',self.call('Controller','send',{'to':[child],'text':'new work'})['result'])

    def test_missing_native_status_invalidates_previously_fresh_idle_claim(self):
        from robofrontier.runtime_cli import execute
        from robofrontier.scheduling import Scheduler
        s=self.service.store;now=time.time()
        command=Scheduler(s).plan('manager',now)[0]
        execute(s,'session-unavailable',{'actor':'Controller','thread_id':'parent','reason':'no authoritative poll'},'manager',now)
        self.assertEqual(s.agent('Controller')['session_status'],'unknown')
        self.assertIsNone(Scheduler(s).claim(command['wake_id'],'manager',now))

    def test_unconfirmed_input_pauses_without_releasing_active_scope(self):
        self.delegate();self.d.claim('batch','manager');c=self.d.batch('batch')['children'][0]
        self.d.bind('batch',c['actor'],'child',{'source_thread_id':'parent','source_ordinal':42,'source_sha256':'same'},'manager')
        self.d.activate('batch','manager');self.d.started('batch',c['actor'],'uncertain','manager')
        self.service.pump()
        self.assertEqual(self.d.batch('batch')['status'],'needs_review')
        self.assertEqual(self.d.owners()['A'],c['actor'])
        self.assertTrue(self.service.requests['batch']['result']['run_paused'])
        self.assertIn('error',self.call(c['actor'],'gripper',{'robot':'A','state':'open'})['result'])
        self.assertEqual(self.service.world.executed,[])


if __name__ == '__main__':unittest.main()
