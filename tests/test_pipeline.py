"""Runtime integration tests with fake executor; never constructs MuJoCo World."""
import json
from pathlib import Path
import tempfile
import time
import unittest

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService
from robofrontier.runtime_store import RuntimeStore

class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)/'run'
        prepare_run(self.path)
        self.service=RuntimeService(self.path,FakeWorld,fake_capture)
        self.counter=0
    def tearDown(self):
        if self.service.store:self.service.close()
        self.temp.cleanup()
    def call(self,actor,op,data=None,call_id=None,ack=None):
        self.counter+=1
        job=self.service.accept(actor,op,data or {},call_id or 'call-'+str(self.counter),ack)
        self.service.pump()
        return job
    def start(self):
        for actor in ('Coordinator','A','B','C'):
            self.call(actor,'ready')
            self.call('Meta','bind',{'actor':actor,'thread_id':'thread-'+actor})
            self.service.store.observe_session(actor,'idle','turn-'+actor,time.time(),'codex.wait_threads')
        self.assertTrue(self.call('Meta','start')['result']['started'])
    def test_no_action_before_start_and_no_meta_business_or_motion(self):
        self.assertIn('error',self.call('A','move',{'target_m':[1,2,3]})['result'])
        self.assertIn('error',self.call('Meta','send',{'to':['A'],'text':'x'})['result'])
        self.assertIn('error',self.call('Coordinator','gripper',{'state':'close'})['result'])
        self.assertEqual(self.service.world.executed,[])

    def test_worker_cannot_select_another_robot_and_admin_pause_releases_wait(self):
        self.start()
        result=self.call('A','gripper',{'robot':'B','state':'close'})['result']
        self.assertIn('error',result);self.assertEqual(self.service.world.executed,[])
        wait=self.call('B','wait_events',{'timeout_s':600})
        self.assertFalse(wait['done'].is_set())
        self.service.store.put('dispatch_paused',True)
        self.service.store.put('physics_state','paused_by_user')
        self.service.pump()
        self.assertEqual(wait['result']['wait_status'],'paused')
        self.assertTrue(wait['result']['run_paused'])
        self.assertIn('paused',self.call('A','gripper',{'state':'close'})['result']['error'])

    def test_active_idle_yield_requires_self_selected_review_deadline(self):
        self.assertTrue(self.call('A','yield',{'state':'waiting','note':'READY gate'})['result']['saved'])
        self.start()
        for actor in ['A','Coordinator']:
            result=self.call(actor,'yield',{'state':'available','note':'Waiting on team progress'})['result']
            self.assertIn('after_s',result.get('error',''))
            result=self.call(actor,'yield',{'state':'waiting','note':'Review uncertain progress','after_s':23})['result']
            self.assertTrue(result['saved']);self.assertIsNotNone(result['work']['next_check'])
        self.service.store.put('dispatch_paused',True)
        result=self.call('A','yield',{'state':'waiting','note':'Engineering pause'})['result']
        self.assertTrue(result['saved'])

    def test_standing_worker_remains_available_for_more_work_after_local_completion(self):
        self.start()
        self.call('A','gripper',{'state':'open','wait':False},'first-local-work')
        self.service.world.complete('A');self.service.pump()
        saved=self.call('A','yield',{'state':'available','note':'Local work complete; available for team work.','after_s':23})['result']
        self.assertTrue(saved['saved'])
        notice=self.call('Coordinator','inbox')['result']['system_events']
        self.assertTrue(any(e['event']=='worker_self_reported_available' and e['worker']=='A' for e in notice))
        message=self.call('Coordinator','send',{'to':['A'],'text':'New team work within your robot scope.'})['result']
        boundary=self.call('A','work',ack=saved.get('delivery_id'))['result']
        self.assertIn(message['message_id'],[m['message_id'] for m in boundary['messages']])
        self.assertEqual(boundary['work']['kind'],'worker')
        self.assertEqual(self.service.store.get('lifecycle'),'running')
        self.assertIn('error',self.call('A','return_task',{'outcome':'completed','result':'Not a temporary delegate.'})['result'])
        next_work=self.call('A','gripper',{'state':'close','wait':False},'next-team-work')['result']
        self.assertEqual(next_work['execution_status'],'running')

    def test_ready_delivers_the_adapter_summary_to_every_policy_role(self):
        from unittest.mock import Mock
        facts={'adapter':'synthetic','robots':['A','B','C'],'capabilities':'test fixture'}
        self.service.world.robot_facts=Mock(return_value=facts)
        self.service.capture=Mock(side_effect=AssertionError('READY must not take a fresh observation'))
        for actor in ('Coordinator','A','B','C'):
            result=self.call(actor,'ready')['result']
            self.assertTrue(result['ready']);self.assertEqual(result['robot_facts'],facts)
        self.assertEqual(self.service.world.robot_facts.call_count,4)
        self.assertEqual(self.service.store.actions(),{})

    def test_unavailable_robot_summary_cannot_register_a_false_ready(self):
        def missing(cfg):raise ValueError('Unsupported robot facts')
        self.service.world.robot_facts=missing
        result=self.call('Coordinator','ready')['result']
        self.assertIn('error',result)
        self.assertFalse(self.service.store.agent('Coordinator')['ready'])
    def test_read_only_screening_can_compute_while_mail_and_actions_progress(self):
        self.start()
        self.service.world.robot_model=lambda cfg:{'model_id':'fake-model','robots':{},'scope':'fake robot geometry'}
        self.service.world.motion_state_id=lambda:'new-state'
        def samples(data,cfg):
            yield None
            yield None
            yield {'assessment':'no_conflict_found','source_state_id':'old-state','advisory_only':True}
        self.service.world.check_motion=samples
        self.assertIn('robot_model',self.call('Coordinator','robot_model')['result'])
        checking=self.call('Coordinator','check_motion',{},'preview')
        self.assertFalse(checking['done'].is_set())
        self.assertIs(checking,self.service.accept('Coordinator','check_motion',{},'preview'))
        self.assertIn('error',self.call('Meta','check_motion',{})['result'])
        self.service.accept('B','send',{'to':['Coordinator'],'text':'screening notification'},'screen-mail')
        action=self.call('A','gripper',{'state':'open','wait':False})
        self.assertEqual(action['result']['execution_status'],'running')
        self.assertTrue(checking['done'].is_set())
        self.assertTrue(checking['result']['motion_check']['live_state_changed'])
        self.assertEqual(checking['result']['messages'][0]['text'],'screening notification')
        self.assertEqual(len(self.service.world.executed),1)
        self.assertEqual(self.service.store.call('preview')['status'],'completed')

    def test_screening_pause_and_unknown_do_not_submit_actions(self):
        self.start()
        def samples(data,cfg):
            yield None
            raise ValueError('missing geometry')
        self.service.world.check_motion=samples
        check=self.call('A','check_motion',{})
        self.service.pump()
        self.assertEqual(check['result']['motion_check']['assessment'],'unknown')
        check=self.call('B','check_motion',{})
        self.service.store.put('physics_state','paused_by_user');self.service.pump()
        self.assertEqual(check['result']['motion_check']['assessment'],'unknown')
        self.assertEqual(self.service.world.executed,[])

    def test_screening_retries_contention_and_owner_close_keep_read_only_calls_separate(self):
        self.start()
        def samples(data,cfg):
            for _ in range(8):yield None
            yield {'assessment':'unknown','advisory_only':True}
        self.service.world.check_motion=samples
        pending=self.call('A','check_motion',{},'read-A')
        self.assertIn('already computing',self.call('B','check_motion',{},'read-B')['result']['error'])
        self.assertFalse(pending['done'].is_set())
        self.assertEqual(self.service.store.actions(),{})
        self.service.close()
        self.assertTrue(pending['done'].is_set())
        self.assertEqual(pending['result']['motion_check']['assessment'],'unknown')
    def test_tree_routing_and_batch_receipt(self):
        self.assertIn('error',self.call('A','send',{'to':['B'],'text':'x'})['result'])
        for text in ('one','two'):self.call('Coordinator','send',{'to':['A'],'text':text})
        a=self.call('A','inbox')['result'];b=self.call('A','inbox')['result']
        self.assertEqual([m['text'] for m in a['messages']],['one','two'])
        self.assertEqual(a['delivery_id'],b['delivery_id'])
        self.assertEqual(self.service.store.pending('A')['messages'],2)
        self.call('A','work',ack=a['delivery_id'])
        self.assertEqual(self.service.store.pending('A')['messages'],0)
        with self.assertRaisesRegex(ValueError,'Unknown delivery'):
            self.call('B','work',ack=a['delivery_id'])
    def test_ready_gate_is_actual_idle_observation(self):
        for a in ('Coordinator','A','B','C'):self.call(a,'ready')
        self.assertIn('error',self.call('Meta','start')['result'])
        self.assertFalse(self.service.started)
    def test_parallel_arms_same_arm_exclusion_and_idempotent_retry(self):
        self.start()
        first=self.call('A','gripper',{'state':'close'},'motion-A')
        second=self.call('B','gripper',{'state':'close'},'motion-B')
        self.assertFalse(first['done'].is_set());self.assertFalse(second['done'].is_set())
        retry=self.call('A','gripper',{'state':'close'},'motion-A')
        self.assertIs(first,retry)
        self.assertEqual(len(self.service.world.executed),2)
        self.assertIn('error',self.call('A','gripper',{'state':'open'})['result'])
        with self.assertRaises(ValueError):self.call('A','move',{'target_m':[0,0,1]},'motion-A')
        self.service.world.complete('A');self.service.pump()
        self.assertEqual(first['result']['execution']['status'],'completed')
        self.assertEqual(self.call('A','gripper',{'state':'close'},'motion-A')['result'],first['result'])
    def test_moves_can_start_without_any_screening_call_or_result(self):
        self.start()
        def forbidden(*args):
            raise AssertionError('Move must not implicitly invoke screening')
        self.service.world.check_motion=forbidden
        for actor in ('A','B'):
            result=self.call(actor,'move',{'delta_m':[.01,0,0],'wait':False})['result']
            self.assertEqual(result['execution_status'],'running')
        self.assertEqual(len(self.service.world.executed),2)
    def test_restart_unknown_execution_does_not_replay(self):
        self.start();self.call('A','gripper',{'state':'close'},'uncertain-A')
        self.service.close()
        with self.assertRaisesRegex(ValueError,'Physical restart is unsupported'):
            RuntimeService(self.path,FakeWorld,fake_capture)
        store=RuntimeStore(self.path/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.actions()['uncertain-A']['status'],'uncertain')
            self.assertEqual(store.get('lifecycle'),'interrupted')
        finally:store.close()
    def test_abrupt_owner_loss_marks_uncertain_without_calling_world_factory(self):
        self.start()
        self.call('A','gripper',{'state':'close'},'crashed-action')
        # Simulate process loss: release OS resources without owner-close bookkeeping.
        self.service.store.close();self.service.store=None
        self.service.world_lock.close()
        worlds=[]
        def factory(*args):worlds.append(args);return FakeWorld(*args)
        with self.assertRaisesRegex(ValueError,'Physical restart is unsupported'):
            RuntimeService(self.path,factory,fake_capture)
        self.assertEqual(worlds,[])
        store=RuntimeStore(self.path/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.actions()['crashed-action']['status'],'uncertain')
            self.assertEqual(store.get('lifecycle'),'interrupted')
            self.assertTrue(store.get('dispatch_paused'))
        finally:store.close()
    def test_notes_deadline_and_mail_survive_without_recreating_a_world(self):
        self.start()
        self.call('A','yield',{'state':'waiting','note':'await assigned dependency','after_s':50})
        self.call('Coordinator','send',{'to':['A'],'text':'new fact'})
        deadline=self.service.store.agent('A')['next_check']
        self.service.close()
        worlds=[]
        def factory(*args):worlds.append(args);return FakeWorld(*args)
        with self.assertRaisesRegex(ValueError,'Physical restart is unsupported'):
            RuntimeService(self.path,factory,fake_capture)
        self.assertEqual(worlds,[])
        store=RuntimeStore(self.path/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.agent('A')['note'],'await assigned dependency')
            self.assertEqual(store.agent('A')['next_check'],deadline)
            self.assertEqual(store.pending('A')['messages'],1)
            self.assertEqual(store.get('lifecycle'),'interrupted')
        finally:store.close()

    def test_idle_ticks_do_not_create_checkpoints_or_rewrite_audit_exports(self):
        self.start()
        for _ in range(5):self.service.periodic()
        output=self.path/'private/outputs'
        self.assertFalse((output/'events.jsonl').exists())
        self.assertFalse((output/'runtime.json').exists())
        self.assertFalse(list(self.path.rglob('*.npz')))
        self.assertFalse((self.path/'private/checkpoints').exists())
        self.call('Coordinator','finish',{'note':'fake finish'})
        self.assertTrue((output/'events.jsonl').is_file())
        self.assertTrue((output/'runtime.json').is_file())

    def test_periodic_integrity_check_is_throttled(self):
        from unittest.mock import patch
        self.start();last=self.service.last_integrity_check
        interval=self.service.config['integrity_check_interval_s']
        with patch('robofrontier.runtime_service.verify_inputs',return_value=[]) as verify:
            with patch('robofrontier.runtime_service.time.monotonic',return_value=last+interval-1):
                self.service.periodic()
            verify.assert_not_called()
            with patch('robofrontier.runtime_service.time.monotonic',return_value=last+interval):
                self.service.periodic();self.service.periodic()
            self.assertEqual(verify.call_count,1)

    def test_finish_checks_integrity_and_never_reports_tampered_run_as_valid(self):
        self.start()
        (self.path/'public/inputs/task.json').write_text('{}')
        result=self.call('Coordinator','finish',{'note':'fake finish'})['result']
        self.assertIn('error',result)
        self.assertFalse(self.service.store.get('integrity_valid'))
        self.assertEqual(self.service.store.get('lifecycle'),'recovery_required')
        self.assertFalse((self.path/'private/outputs/evaluation.json').exists())

    def test_corrupt_manifest_is_an_integrity_failure_not_a_successful_finish(self):
        self.start()
        (self.path/'private/inputs/manifest.json').write_text('{')
        result=self.call('Coordinator','finish',{})['result']
        self.assertIn('error',result)
        self.assertFalse(self.service.store.get('integrity_valid'))
        self.assertEqual(self.service.store.get('lifecycle'),'recovery_required')

    def test_a_second_owner_cannot_replace_the_shared_world(self):
        with self.assertRaisesRegex(ValueError,'Another process owns'):
            RuntimeService(self.path,FakeWorld,fake_capture)
        self.assertEqual(self.service.store.get('lifecycle'),'prepared')
        self.start()

    def test_inbox_race_on_yield_is_preserved(self):
        self.start()
        self.call('Coordinator','send',{'to':['A'],'text':'arrived just before yield'})
        result=self.call('A','yield',{'state':'waiting','note':'wait'})['result']
        self.assertEqual(result['messages'][0]['text'],'arrived just before yield')
        self.assertEqual(self.service.store.pending('A')['messages'],1)
    def test_result_is_released_only_after_outer_transaction_commit(self):
        import threading
        service=self.service
        class ReceiptEvent(threading.Event):
            def set(event):
                self.assertFalse(service.store.db.in_transaction)
                super().set()
        self.call('Coordinator','send',{'to':['A'],'text':'prior message'})
        job=service.accept('A','send',{'to':['Coordinator'],'text':'reply'},'atomic-reply')
        job['done']=ReceiptEvent();service.pump()
        self.assertTrue(job['done'].is_set())
    def test_executor_fault_is_uncertain_and_cannot_be_replayed(self):
        self.start()
        def fail(*args):raise RuntimeError('fake acknowledgement lost')
        self.service.world.start_action=fail
        result=self.call('A','gripper',{'state':'close'},'unknown')['result']
        self.assertEqual(result['execution_status'],'uncertain')
        self.assertEqual(self.service.store.actions()['unknown']['status'],'uncertain')
        self.assertEqual(self.service.store.get('lifecycle'),'recovery_required')
        self.assertIn('error',self.call('A','gripper',{'state':'open'})['result'])
    def test_finish_closes_team_but_not_a_success_claim(self):
        self.start()
        self.assertTrue(self.call('Coordinator','finish',{'note':'test only'})['result']['ended'])
        self.assertIn('error',self.call('A','move',{'target_m':[1,2,3]})['result'])
        evaluation=json.loads((self.path/'private/outputs/evaluation.json').read_text())
        self.assertIn('fake',evaluation['scope'])

    def test_coordinator_gets_mail_at_tool_return_inside_active_turn(self):
        self.start()
        self.call('A','send',{'to':['Coordinator'],'text':'new opportunity'})
        result=self.call('Coordinator','team')['result']
        self.assertEqual(result['messages'][0]['text'],'new opportunity')
        self.call('Coordinator','work',ack=result['delivery_id'])
        self.assertEqual(self.service.store.pending('Coordinator')['messages'],0)

    def test_team_separates_old_note_from_fresh_execution_without_new_checkpoint(self):
        from unittest.mock import patch
        self.start()
        with patch('robofrontier.runtime_store.time.time',return_value=100):
            self.call('B','checkpoint',{'state':'waiting','note':'waiting for authority'})
        with patch('robofrontier.runtime_store.time.time',return_value=200):
            self.call('B','gripper',{'state':'close','wait':False},'accepted-new-work')
        team=self.call('Coordinator','team')['result']
        worker=team['workers']['B']
        self.assertEqual(worker['work']['note'],'waiting for authority')
        self.assertEqual(worker['work']['saved_at'],100)
        self.assertEqual(worker['execution']['action_id'],'accepted-new-work')
        self.assertEqual(worker['execution']['status'],'running')
        self.assertEqual(worker['execution']['started_at'],200)
        self.assertNotIn('thread_id',worker)
        self.service.world.complete('B');self.service.pump()
        worker=self.call('Coordinator','team')['result']['workers']['B']
        self.assertEqual(worker['execution']['status'],'completed')
        self.assertGreater(worker['execution']['finished_at'],worker['execution']['started_at'])
        self.assertEqual(worker['work']['saved_at'],100)

    def test_agent_chosen_wait_can_span_transport_slices_and_pause_returns_immediately(self):
        from unittest.mock import patch
        self.start()
        with patch('robofrontier.runtime_service.time.monotonic',return_value=0):
            wait=self.call('Coordinator','wait_events',{'timeout_s':120},'long-wait')
        self.assertFalse(wait['done'].is_set())
        with patch('robofrontier.runtime_service.time.monotonic',return_value=31):
            self.service.pump()
        self.assertFalse(wait['done'].is_set())
        self.service.store.put('lifecycle','paused')
        self.service.pump()
        self.assertEqual(wait['result']['wait_status'],'paused')
        self.assertFalse(wait['result']['run_ended'])

    def test_wait_retry_keeps_original_deadline(self):
        from unittest.mock import patch
        self.start()
        with patch('robofrontier.runtime_service.time.monotonic',return_value=0):
            first=self.call('A','wait_events',{'timeout_s':120},'wait-id')
        with patch('robofrontier.runtime_service.time.monotonic',return_value=80):
            self.assertIs(self.call('A','wait_events',{'timeout_s':120},'wait-id'),first)
            self.assertFalse(first['done'].is_set())
        with patch('robofrontier.runtime_service.time.monotonic',return_value=120):
            self.service.pump()
        self.assertEqual(first['result']['wait_status'],'timeout')

    def test_wait_registration_and_delivery_do_not_lose_a_message(self):
        self.start()
        waiting=self.call('A','wait_events',{'timeout_s':30})
        self.assertFalse(waiting['done'].is_set())
        self.assertFalse(self.service.store.db.in_transaction)
        self.call('Coordinator','send',{'to':['B'],'text':'unrelated'})
        self.assertFalse(waiting['done'].is_set())
        for text in ('first','second'):
            self.service.accept('Coordinator','send',{'to':['A'],'text':text},'mail-'+text)
        self.service.pump()
        self.assertTrue(waiting['done'].is_set())
        self.assertEqual([m['text'] for m in waiting['result']['messages']],['first','second'])
        # A message before registration also returns immediately.
        again=self.call('A','wait_events',{'timeout_s':30},ack=waiting['result']['delivery_id'])
        self.assertFalse(again['done'].is_set())
        self.call('Coordinator','send',{'to':['A'],'text':'third'})
        self.assertEqual(again['result']['messages'][0]['text'],'third')

    def test_wait_timeout_rewait_and_start_gate(self):
        from unittest.mock import patch
        self.assertIn('error',self.call('A','wait_events',{})['result'])
        self.start()
        with patch('robofrontier.runtime_service.time.monotonic',return_value=10):
            waiting=self.call('A','wait_events',{'timeout_s':5},'waiting')
        with patch('robofrontier.runtime_service.time.monotonic',return_value=16):
            self.service.pump()
        self.assertEqual(waiting['result']['wait_status'],'timeout')
        self.assertEqual(self.call('A','wait_events',{'timeout_s':0})['result']['wait_status'],'timeout')
        again=self.call('A','wait_events',{'timeout_s':30})
        self.assertFalse(again['done'].is_set())
        self.call('Coordinator','finish',{})
        self.assertEqual(again['result']['wait_status'],'ended')

    def test_async_motion_can_communicate_wait_and_retry_without_duplicate_execution(self):
        self.start()
        job=self.call('A','gripper',{'state':'close','wait':False},'async-A')
        self.assertTrue(job['done'].is_set())
        self.assertEqual(job['result']['execution_status'],'running')
        self.assertEqual(self.service.store.call('async-A')['status'],'running')
        self.assertNotIn('error',self.call('A','send',{'to':['Coordinator'],'text':'planning while moving'})['result'])
        self.assertIn('error',self.call('A','gripper',{'state':'open','wait':False})['result'])
        waiting=self.call('A','wait_events',{'call_ids':['async-A'],'timeout_s':30})
        self.assertFalse(waiting['done'].is_set())
        self.service.world.complete('A');self.service.pump()
        self.assertEqual(waiting['result']['executions']['async-A']['status'],'completed')
        self.assertEqual(waiting['result']['system_events'][0]['event'],'action_finished')
        retry=self.call('A','gripper',{'state':'close','wait':False},'async-A')
        self.assertEqual(retry['result']['execution']['status'],'completed')
        self.assertEqual(len(self.service.world.executed),1)
        self.assertIn('error',self.call('B','wait_events',{'call_ids':['async-A']})['result'])

    def test_late_report_is_audited_but_cannot_reopen_finished_world(self):
        self.start();self.call('Coordinator','finish',{'note':'fake'})
        report=self.call('C','send',{'to':['Coordinator'],'text':'late completion report'})['result']
        self.assertNotIn('error',report);self.assertTrue(report['run_ended'])
        self.assertIn('error',self.call('C','gripper',{'state':'close'})['result'])
        events=json.loads((self.path/'private/outputs/runtime.json').read_text())['events']
        self.assertTrue(any(e['kind']=='LATE_MESSAGE_AFTER_FINISH' for e in events))

    def test_async_owner_loss_stays_uncertain_even_after_accepted_reply(self):
        self.start()
        self.call('A','gripper',{'state':'close','wait':False},'async-crash')
        self.service.close()
        store=RuntimeStore(self.path/'private/state/runtime.sqlite')
        try:
            self.assertEqual(store.actions()['async-crash']['status'],'uncertain')
            self.assertEqual(store.call('async-crash')['status'],'uncertain')
        finally:store.close()

    def test_peer_wait_is_addressed_and_has_no_coordinator(self):
        from robofrontier.task_config import load_harness
        cfg=load_harness();cfg['topology']='peer'
        config=self.path.parent/'peer.json';config.write_text(json.dumps(cfg))
        run=prepare_run(self.path.parent/'peer',harness_path=config)
        service=RuntimeService(run,FakeWorld,fake_capture)
        def call(actor,op,data,key):
            job=service.accept(actor,op,data,key);service.pump();return job
        try:
            for actor in ('A','B','C'):
                call(actor,'ready',{},'ready-'+actor)
                call('Meta','bind',{'actor':actor,'thread_id':'peer-'+actor},'bind-'+actor)
                service.store.observe_session(actor,'idle','turn-'+actor,time.time(),'codex.wait_threads')
            call('Meta','start',{},'start')
            self.assertNotIn('Coordinator',service.roles)
            a=call('A','wait_events',{'timeout_s':30},'wait-A')
            b=call('B','wait_events',{'timeout_s':30},'wait-B')
            call('C','send',{'to':['B'],'text':'cooperation request'},'peer-message')
            self.assertFalse(a['done'].is_set())
            self.assertTrue(b['done'].is_set())
            self.assertEqual(b['result']['messages'][0]['from'],'C')
        finally:service.close()

if __name__=='__main__':unittest.main()
