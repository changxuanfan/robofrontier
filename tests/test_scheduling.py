from pathlib import Path
import tempfile
import unittest
from robofrontier.runtime_store import RuntimeStore
from robofrontier.task_config import load_harness
from robofrontier.scheduling import Scheduler, FakeSessionAdapter, dispatch_once

class SchedulingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'runtime.sqlite'
        self.s=RuntimeStore(self.path);self.s.initialize('fake',load_harness(),['A','B','C'])
        for actor in ('Coordinator','A','B','C'):
            self.s.update_agent(actor,ready=True,thread_id='fake-'+actor)
        self.s.start()
        for actor in ('Coordinator','A','B','C'):
            self.s.checkpoint_work(actor,'waiting','waiting for a relevant event',now=100)
            self.s.observe_session(actor,'idle','t0',100,'codex.wait_threads')
        self.planner=Scheduler(self.s)
    def tearDown(self):self.s.close();self.tmp.cleanup()
    def commands(self,now=100,owner='owner'):
        return self.planner.plan(owner,now)
    def test_addressed_only_batched_and_not_on_message_semantics(self):
        self.s.send('Coordinator',['A'],'C is mentioned but not addressed','1')
        self.s.send('Coordinator',['A'],'another message','2')
        commands=self.commands()
        self.assertEqual([x['actor'] for x in commands],['A'])
        self.assertEqual(len(self.commands()),1)
        self.assertEqual(len(self.s.wake_rows()),1)
        self.assertNotIn('C is mentioned',commands[0]['prompt'])
    def test_busy_receiver_keeps_mail_then_wakes_at_idle(self):
        self.s.observe_session('A','active','t0',100,'codex.wait_threads')
        self.s.send('Coordinator',['A'],'work','1')
        self.assertEqual(self.commands(),[])
        self.s.observe_session('A','idle','t0',101,'codex.wait_threads')
        self.assertEqual(self.commands(101)[0]['actor'],'A')
    def start_motion(self,wait=False):
        data={'delta_m':[.01,0,0],'wait':wait}
        self.s.begin_call('move-A','A','move',data)
        self.s.start_action('move-A','A','move',data,0)
        self.s.checkpoint_work('A','waiting','motion in progress, waiting for a relevant event',now=100)
    def test_idle_async_worker_can_receive_mail_before_motion_finishes(self):
        self.start_motion()
        self.s.send('Coordinator',['A'],'new relevant constraint','1')
        self.s.observe_session('A','active','t0',100,'codex.wait_threads')
        self.assertEqual(self.commands(),[])
        self.s.observe_session('A','idle','t0',101,'codex.wait_threads')
        wake=self.commands(101)[0]
        self.assertEqual(wake['actor'],'A')
        self.assertEqual(self.commands(101)[0]['wake_id'],wake['wake_id'])
        self.assertIsNotNone(self.planner.claim(wake['wake_id'],'owner',101))
        self.s.record_dispatch(wake['wake_id'],'sent',{})
        self.assertEqual(self.commands(101),[])
        self.assertEqual(self.s.actions()['move-A']['status'],'running')
        self.assertEqual(len(self.s.actions()),1)
    def test_async_work_waits_for_agent_selected_deadline_or_continuation(self):
        self.start_motion()
        self.assertEqual(self.commands(),[])
        self.s.checkpoint_work('A','waiting','check chosen by agent',after_s=5,now=100)
        self.assertEqual(self.commands(104),[])
        wake=self.commands(105)[0]
        self.planner.claim(wake['wake_id'],'owner',105)
        self.s.record_dispatch(wake['wake_id'],'sent',{})
        self.s.observe_session('A','active','t1',105,'codex.wait_threads')
        self.s.observe_session('A','idle','t1',106,'codex.wait_threads')
        self.assertEqual(self.commands(106),[])
        self.s.checkpoint_work('A','continue','continue local planning',now=106)
        self.assertEqual(self.commands(106)[0]['actor'],'A')
    def test_blocking_or_uncertain_motion_does_not_create_a_continuation(self):
        self.start_motion(wait=True)
        self.s.send('Coordinator',['A'],'pending update','1')
        self.assertEqual(self.commands(),[])
        self.s.uncertain_action('move-A','injected executor uncertainty')
        self.assertEqual(self.commands(),[])
        self.assertTrue(self.planner.execution_blocks_wake('A'))
    def test_claim_rechecks_motion_state_after_wake_was_planned(self):
        self.s.send('Coordinator',['A'],'update','1')
        wake=self.commands()[0]
        self.start_motion(wait=True)
        self.assertIsNone(self.planner.claim(wake['wake_id'],'owner',100))
        self.assertEqual(self.s.wake_rows()[0]['status'],'cancelled')
    def test_message_race_after_round_end_is_not_lost(self):
        self.assertEqual(self.commands(),[])
        self.s.send('Coordinator',['A'],'arrived after idle check','1')
        self.assertEqual(self.commands()[0]['actor'],'A')
    def test_deadline_is_agent_selected_and_fires_once_per_saved_revision(self):
        self.s.checkpoint_work('A','waiting','expect event',after_s=10,now=100)
        self.s.observe_session('A','idle','t0',109,'codex.wait_threads')
        self.assertEqual(self.commands(109),[])
        self.s.observe_session('A','idle','t0',110,'codex.wait_threads')
        wake=self.commands(110)[0];self.planner.claim(wake['wake_id'],'owner',110)
        self.s.record_dispatch(wake['wake_id'],'sent',{})
        self.s.observe_session('A','active','t1',111,'codex.wait_threads')
        self.s.observe_session('A','idle','t1',112,'codex.wait_threads')
        self.assertEqual(self.commands(112),[])
        self.s.checkpoint_work('A','waiting','new event expectation',after_s=5,now=120)
        self.s.observe_session('A','idle','t1',125,'codex.wait_threads')
        self.assertEqual(self.commands(125)[0]['actor'],'A')
    def test_pending_work_continues_without_inventing_a_robot_action(self):
        self.s.checkpoint_work('A','continue','finish the saved assignment',now=100)
        wake=self.commands()[0]
        self.assertEqual(wake['actor'],'A')
        self.assertEqual(self.s.actions(),{})
    def test_availability_is_declared_not_inferred_from_idle_or_tool_end(self):
        self.assertEqual(self.s.pending('Coordinator')['signals'],0)
        self.s.checkpoint_work('A','available','local work done, can cooperate',now=100)
        self.s.checkpoint_work('A','available','unchanged',now=101)
        self.assertEqual(self.s.pending('Coordinator')['signals'],1)
        signal=self.s.delivery('Coordinator')['system_events'][0]
        self.assertFalse(signal['verified_task_completion'])
    def test_recheck_cancels_pending_wake_if_receiver_became_active(self):
        self.s.send('Coordinator',['A'],'work','1');wake=self.commands()[0]
        self.s.observe_session('A','active','t1',101,'codex.wait_threads')
        self.assertIsNone(self.planner.claim(wake['wake_id'],'owner',101))
        self.assertEqual(self.s.wake_rows()[0]['status'],'cancelled')
    def test_cancelled_stale_claim_can_be_replanned_after_fresh_same_turn(self):
        self.s.send('Coordinator',['A'],'work','1');wake=self.commands()[0]
        self.assertIsNone(self.planner.claim(wake['wake_id'],'owner',107))
        self.s.observe_session('A','idle','t0',108,'codex.wait_threads')
        replacement=self.commands(108)[0]
        self.assertNotEqual(replacement['wake_id'],wake['wake_id'])
        self.assertIsNotNone(self.planner.claim(replacement['wake_id'],'owner',108))
    def test_uncertain_send_is_not_retried_on_old_idle_snapshot(self):
        self.s.send('Coordinator',['A'],'work','1');wake=self.commands()[0]
        self.planner.claim(wake['wake_id'],'owner',100)
        self.s.record_dispatch(wake['wake_id'],'uncertain',{'error':'lost response'})
        self.assertEqual(self.commands(),[])
        self.s.close();self.s=RuntimeStore(self.path);self.planner=Scheduler(self.s)
        self.assertEqual(self.commands(),[])
        self.s.observe_session('A','active','t1',101,'codex.wait_threads')
        self.assertEqual(self.s.wake_rows()[0]['status'],'observed')
    def test_stale_or_future_session_status_cannot_authorize_send(self):
        self.s.send('Coordinator',['A'],'work','1')
        self.assertEqual(self.commands(110),[])
        self.s.observe_session('A','idle','t0',120,'codex.wait_threads')
        self.assertEqual(self.commands(110),[])
    def test_one_manager_lease_and_no_duplicate_dispatch(self):
        self.s.send('Coordinator',['A'],'work','1');wake=self.commands()[0]
        with self.assertRaises(ValueError):self.commands(101,'other')
        self.planner.claim(wake['wake_id'],'owner',100)
        with self.assertRaises(ValueError):self.planner.claim(wake['wake_id'],'owner',100)
    def test_no_progress_has_a_bounded_recovery_state(self):
        self.s.send('Coordinator',['A'],'work','1')
        for n in range(3):
            self.s.observe_session('A','idle','t'+str(n),100+n,'codex.wait_threads')
            wake=self.commands(100+n)[0];self.planner.claim(wake['wake_id'],'owner',100+n)
            self.s.record_dispatch(wake['wake_id'],'sent',{})
            self.s.observe_session('A','active','t'+str(n+1),100+n+.1,'codex.wait_threads')
        self.s.observe_session('A','idle','t3',104,'codex.wait_threads')
        self.assertEqual(self.commands(104),[])
        self.assertTrue(self.s.agent('A')['recovery_required'])
    def test_fake_adapter_complete_dispatch_contract(self):
        observations={a:{'status':'idle','turn_id':'t0'} for a in ('Coordinator','A','B','C')}
        adapter=FakeSessionAdapter(observations)
        self.s.send('Coordinator',['B'],'work','1')
        self.assertEqual(len(dispatch_once(self.s,adapter,'owner',100)),1)
        self.assertEqual(adapter.sent[0]['actor'],'B')
        self.assertEqual(dispatch_once(self.s,adapter,'owner',101),[])
    def test_meta_dispatch_snapshot_does_not_need_business_notes(self):
        from robofrontier.runtime_cli import execute
        self.s.update_agent('A',note='private task planning text '*600)
        snapshot=execute(self.s,'snapshot',{},'owner',100)
        self.assertNotIn('note',snapshot['agents']['A'])
        self.assertIn('private task planning text',self.s.agent('A')['note'])
        self.assertEqual(self.commands(),[])
    def test_ended_or_paused_run_never_wakes(self):
        self.s.send('Coordinator',['A'],'work','1')
        self.s.put('dispatch_paused',True);self.assertEqual(self.commands(),[])
        self.s.put('dispatch_paused',False);self.s.put('lifecycle','ended')
        self.assertEqual(self.commands(),[])

if __name__=='__main__':unittest.main()
