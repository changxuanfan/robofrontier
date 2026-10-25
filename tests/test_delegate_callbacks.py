"""Actual runtime seam: returns, missed handoffs, recovery and first-result flow.

Synthetic executor/platform observations; no model sessions or robot physics.
"""
import time
import unittest

import test_delegation as fixture
from robofrontier.scheduling import Scheduler


class DelegateCallbackTests(unittest.TestCase):
    def setUp(self):
        self.f=fixture.DelegationTests();self.f.setUp()
        self.s=self.f.service.store;self.d=self.f.d;self.call=self.f.call
    def tearDown(self):self.f.tearDown()
    def start_children(self):
        self.f.delegate(('A','B'));self.f.activate();return self.d.batch('batch')['children']
    def observe(self,child,status='idle',turn='child-turn',now=None):
        self.s.observe_session(child['actor'],status,turn,now or time.time(),'codex.wait_threads')

    def test_return_is_delivered_with_parent_tool_result_while_other_child_runs(self):
        a,b=self.start_children()
        self.call(b['actor'],'gripper',{'robot':'B','state':'open','wait':False})
        parent=self.call('Controller','gripper',{'robot':'C','state':'open'},'parent-blocking')
        self.assertFalse(parent['done'].is_set())
        self.call(a['actor'],'return_task',{'outcome':'completed','result':'bounded outcome evidence'})
        self.f.service.world.complete('C');self.f.service.pump()
        notices=parent['result']['system_events']
        self.assertEqual([n['event'] for n in notices],['delegation_returned'])
        self.assertIn('B',self.f.service.world.running)
        self.assertEqual(self.d.owners()['A'],'Controller')

    def test_idle_without_return_notifies_parent_once_and_retains_control(self):
        a,b=self.start_children();waiting=self.call('Controller','wait_events',{'timeout_s':30})
        self.observe(a);self.f.service.pump()
        self.assertTrue(waiting['done'].is_set())
        n=waiting['result']['system_events'][0]
        self.assertEqual(n['event'],'delegate_stopped_without_return')
        self.assertEqual(n['actor'],a['actor']);self.assertEqual(n['thread_id'],a['thread_id'])
        self.assertEqual(n['execution_state'],'idle');self.assertEqual(self.d.owners()['A'],a['actor'])
        self.observe(a);self.observe(a,now=time.time()+1)
        self.assertEqual(len([e for e in self.s.events() if e['kind']=='DELEGATE_STOPPED_WITHOUT_RETURN']),1)
        self.assertEqual(self.s.pending(b['actor'])['signals'],0)

    def test_native_idle_never_returns_running_actions_or_claims_success(self):
        a,b=self.start_children()
        self.call(a['actor'],'gripper',{'robot':'A','state':'open','wait':False},'still-running')
        self.observe(a)
        n=self.s.delivery('Controller')['system_events'][0]
        self.assertEqual(n['execution_state'],'unresolved');self.assertEqual(n['unresolved_count'],1)
        self.assertEqual(self.d.owners()['A'],a['actor'])
        self.assertIn('error',self.call(a['actor'],'return_task',{'outcome':'failed','result':'not settled'})['result'])

    def test_fault_cleanup_returns_control_then_new_fork_can_use_same_robot(self):
        a,b=self.start_children();self.observe(a)
        self.call('Controller','send',{'to':[a['actor']],'text':'Resolve calls and hand back with your current report.'})
        self.call(a['actor'],'return_task',{'outcome':'failed','result':'Unable to complete scope; no pending action; holding state described.'})
        self.observe(a,turn='cleanup-turn')
        self.assertEqual(len([e for e in self.s.events() if e['kind']=='DELEGATE_STOPPED_WITHOUT_RETURN']),1)
        self.f.delegate(('A',),call_id='next-batch');self.d.claim('next-batch','test-manager')
        new=self.d.batch('next-batch')['children'][0]
        self.d.bind('next-batch',new['actor'],'new-child',{'source_thread_id':'parent','source_ordinal':99,'source_sha256':'new-cutoff'},'test-manager')
        self.d.activate('next-batch','test-manager');self.d.started('next-batch',new['actor'],'sent','test-manager');self.f.service.pump()
        self.assertNotEqual(new['actor'],a['actor']);self.assertEqual(self.d.owners()['A'],new['actor'])
        self.assertIn('error',self.call(a['actor'],'gripper',{'robot':'A','state':'open'})['result'])
        self.assertEqual(self.call(new['actor'],'gripper',{'robot':'A','state':'open','wait':False})['result']['execution_status'],'running')

    def test_returned_child_native_end_does_not_create_missing_report_alarm(self):
        a,b=self.start_children()
        self.call(a['actor'],'return_task',{'outcome':'completed','result':'report saved'})
        self.observe(a)
        self.assertFalse(any(e['kind']=='DELEGATE_STOPPED_WITHOUT_RETURN' for e in self.s.events()))

    def test_cooperative_pause_uses_delegate_boundary_and_own_stop_without_handback(self):
        a,b=self.start_children()
        self.call(a['actor'],'gripper',{'robot':'A','state':'open','wait':False},'pause-action-A')
        self.call(b['actor'],'gripper',{'robot':'B','state':'open','wait':False},'sibling-action-B')
        message=self.call('Controller','send',{'to':[a['actor']],'text':'Pause this scope and report the settled execution and payload state.'})['result']
        self.assertEqual(set(self.f.service.world.running),{'A','B'})
        self.assertIn('error',self.call('Controller','stop',{'robot':'A','call_id':'pause-action-A'})['result'])
        boundary=self.call(a['actor'],'work')['result']
        self.assertIn(message['message_id'],[m['message_id'] for m in boundary['messages']])
        requests=[]
        # Synthetic stop acknowledgement is deliberately separate from terminal execution.
        self.f.service.world.request_stop=lambda robot,call_id:requests.append((robot,call_id))
        stopping=self.call(a['actor'],'stop',{'robot':'A','call_id':'pause-action-A'})['result']
        self.assertTrue(stopping['stop_requested']);self.assertEqual(requests,[('A','pause-action-A')])
        self.assertEqual(self.s.actions()['pause-action-A']['status'],'running')
        self.f.service.world.complete('A',status='failed');self.f.service.pump()
        self.assertEqual(self.call(a['actor'],'job',{'call_id':'pause-action-A'})['result']['execution']['status'],'failed')
        confirmation=self.call(a['actor'],'send',{'to':['Controller'],'text':'Current action settled; scope paused, not returned. Payload state requires observation.'})['result']
        parent=self.call('Controller','inbox')['result']
        self.assertIn(confirmation['message_id'],[m['message_id'] for m in parent['messages']])
        self.assertEqual(self.d.owners()['A'],a['actor'])
        self.assertEqual(self.s.agent(a['actor'])['delegation_state'],'active')
        self.assertIn('B',self.f.service.world.running)
        self.assertFalse(any(e['kind']=='DELEGATION_RETURNED' for e in self.s.events()))
        returned=self.call(a['actor'],'return_task',{'outcome':'cancelled','result':'Partial work handed back for reallocation; action settled, payload not verified.'})['result']
        self.assertTrue(returned['returned']);self.assertEqual(self.d.owners()['A'],'Controller')

    def test_idle_parent_is_woken_without_waiting_for_other_child(self):
        a,b=self.start_children();now=time.time()
        self.s.checkpoint_work('Controller','waiting','Await relevant results',after_s=100,now=now)
        self.s.observe_session('Controller','idle','parent-review',now,'codex.wait_threads')
        self.observe(b,status='active',turn='b-active',now=now)
        self.observe(a,now=now)
        wakes=Scheduler(self.s).plan('manager',now)
        self.assertEqual([w['actor'] for w in wakes],['Controller'])

    def test_old_mail_and_timer_do_not_silently_restart_stopped_delegate(self):
        a,b=self.start_children();now=time.time()
        self.s.send('Controller',[a['actor']],'earlier scope message','old')
        self.s.checkpoint_work(a['actor'],'working','Old running state, not an explicit wait',after_s=1,now=now-2)
        self.observe(a,now=now)
        self.s.checkpoint_work('Controller','waiting','parent will decide',after_s=100,now=now)
        self.s.observe_session('Controller','idle','parent-review',now,'codex.wait_threads')
        wakes=Scheduler(self.s).plan('manager',now)
        self.assertNotIn(a['actor'],[w['actor'] for w in wakes])
        self.s.send('Controller',[a['actor']],'Please complete the missing handoff.','new')
        wakes=Scheduler(self.s).plan('manager',now)
        self.assertIn(a['actor'],[w['actor'] for w in wakes])

    def test_failed_native_turn_resumes_on_new_parent_request_with_terminal_evidence(self):
        from robofrontier.runtime_cli import execute
        a,b=self.start_children();now=time.time()
        self.s.send('Controller',[a['actor']],'Old instruction before failure','old-before-failure')
        observed={'actor':a['actor'],'thread_id':a['thread_id'],'status':'error',
                  'turn_id':'failed-turn','turn_status':'failed','observed_at':now,'source':'codex.wait_threads'}
        execute(self.s,'observe-sessions',{'observations':[observed]},'manager',now)
        self.assertNotIn(a['actor'],[w['actor'] for w in Scheduler(self.s).plan('manager',now)])
        self.s.send('Controller',[a['actor']],'Resolve the failed turn and hand back known partial work.','explicit-recovery')
        scheduler=Scheduler(self.s)
        command=next((w for w in scheduler.plan('manager',now) if w['actor']==a['actor']),None)
        self.assertIsNotNone(command,'Fresh parent recovery request must be deliverable after a confirmed failed native turn')
        self.assertIsNotNone(scheduler.claim(command['wake_id'],'manager',now))
        self.assertEqual(self.d.owners()['A'],a['actor'])
        self.assertEqual(self.s.agent(a['actor'])['delegation_state'],'active')
        self.s.record_dispatch(command['wake_id'],'sent',{'threadId':a['thread_id']})
        observed.update(turn_id='failed-recovery-turn',observed_at=now+1)
        execute(self.s,'observe-sessions',{'observations':[observed]},'manager',now+1)
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',now+1)],
                         'A recovery turn failing again must not silently repeat the same parent request')

    def test_error_recovery_requires_failed_turn_and_rechecks_before_send(self):
        from robofrontier.runtime_cli import execute
        a,b=self.start_children();now=time.time()
        def observed(status):
            execute(self.s,'observe-sessions',{'observations':[{'actor':a['actor'],'thread_id':a['thread_id'],
                'status':'error','turn_id':'failed-turn','turn_status':status,'observed_at':now,
                'source':'codex.wait_threads'}]},'manager',now)
        observed(None)
        self.s.send('Controller',[a['actor']],'Please resolve and return.','recovery')
        scheduler=Scheduler(self.s)
        for status in (None,'inProgress','completed'):
            observed(status)
            self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',now)])
        observed('failed')
        command=next((w for w in scheduler.plan('manager',now) if w['actor']==a['actor']),None)
        self.assertIsNotNone(command)
        self.observe(a,status='active',turn='new-active',now=now)
        self.assertIsNone(scheduler.claim(command['wake_id'],'manager',now))

    def test_failed_native_recovery_blocks_unresolved_physical_calls(self):
        from robofrontier.runtime_cli import execute
        a,b=self.start_children();now=time.time()
        execute(self.s,'observe-sessions',{'observations':[{'actor':a['actor'],'thread_id':a['thread_id'],
            'status':'error','turn_id':'failed-turn','turn_status':'failed','observed_at':now,
            'source':'codex.wait_threads'}]},'manager',now)
        self.s.send('Controller',[a['actor']],'Resolve calls and return partial work.','recovery')
        params={'robot':'A','delta_m':[.01,0,0],'wait':False}
        self.s.begin_call('accepted-before-error',a['actor'],'move',params)
        scheduler=Scheduler(self.s)
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',now)])
        self.s.start_action('accepted-before-error',a['actor'],'move',params,0)
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',now)])
        self.s.uncertain_action('accepted-before-error','Synthetic acknowledgement loss')
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',now)])

    def test_parent_can_inspect_own_delegate_without_loading_whole_trace(self):
        a,b=self.start_children()
        self.s.checkpoint_work(a['actor'],'working','Timestamped progress and uncertainty')
        self.observe(a,status='active')
        summary=self.call('Controller','work')['result']['delegate_status'][a['actor']]
        self.assertNotIn('note',summary['work']);self.assertEqual(summary['thread_id'],a['thread_id'])
        detail=self.call('Controller','work',{'delegate_id':a['actor']})['result']['delegate_status'][a['actor']]
        self.assertEqual(detail['work']['note'],'Timestamped progress and uncertainty')
        self.assertIn('error',self.call(b['actor'],'work',{'delegate_id':a['actor']})['result'])
        self.assertIn('error',self.call('Controller','work',{'delegate_id':'Meta'})['result'])

    def test_error_notifies_parent_and_notloaded_does_not_claim_a_turn_ended(self):
        a,b=self.start_children();self.observe(a,status='notLoaded')
        self.assertEqual(self.s.pending('Controller')['signals'],0)
        self.observe(a,status='error');self.observe(a,status='idle')
        notices=self.s.delivery('Controller')['system_events']
        self.assertEqual(len(notices),1);self.assertEqual(notices[0]['native_status'],'error')
        self.assertEqual(self.d.owners()['A'],a['actor'])

    def test_session_observation_health_is_persistent_and_distinct_from_delivery(self):
        from robofrontier.runtime_cli import execute
        now=time.time()
        execute(self.s,'observe-sessions',{'cycle':7,'observations':[{'actor':'Controller','thread_id':'parent',
            'status':'idle','turn_id':'review','observed_at':now,'source':'codex.wait_threads'}]},'manager',now)
        h=self.s.get('dispatcher_health');self.assertEqual(h['cycle'],7)
        self.assertEqual(h['last_successful_observation_at'],now);self.assertNotIn('last_delivery_at',h)
        execute(self.s,'session-unavailable',{'actor':'Controller','thread_id':'parent','reason':'Test adapter failure'},'manager',now+1)
        h=self.s.get('dispatcher_health');self.assertEqual(h['last_successful_observation_at'],now)
        self.assertEqual(h['last_error']['reason'],'Test adapter failure')
        self.assertEqual(self.s.agent('Controller')['session_status'],'unknown')
        self.assertEqual(self.f.service.snapshot()['dispatcher_health'],h)

    def test_pending_wake_is_rechecked_when_missing_handoff_is_discovered(self):
        a,b=self.start_children();now=time.time()
        self.s.observe_session(a['actor'],'idle','old-turn',now,'codex.wait_threads')
        self.s.send('Controller',[a['actor']],'Please report and hand back.','cleanup-1')
        command=next(w for w in Scheduler(self.s).plan('manager',now) if w['actor']==a['actor'])
        # Simulate discovery of a stopped turn after the wake was planned.
        self.s.update_agent(a['actor'],stop_notice_message_seq=self.s.pending(a['actor'])['message_seq'])
        self.assertIsNone(Scheduler(self.s).claim(command['wake_id'],'manager',now))

    def test_saved_wait_deadline_resumes_active_scope_without_parent_resend(self):
        a,b=self.start_children();now=time.time()
        self.s.send('Controller',[a['actor']],'Earlier scope information','old-mail-before-wait')
        saved=self.call(a['actor'],'yield',{'state':'waiting','note':'Review this existing scope after the selected wait.','after_s':5})['result']
        self.assertTrue(saved['saved']);deadline=saved['work']['next_check']
        self.observe(a,now=now)
        notice=next(e['details'] for e in self.s.events() if e['kind']=='DELEGATE_STOPPED_WITHOUT_RETURN')
        self.assertEqual(notice['work_state'],'waiting');self.assertEqual(notice['next_check'],deadline)
        scheduler=Scheduler(self.s)
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',deadline-1)])
        self.observe(a,now=deadline+1)
        command=next((w for w in scheduler.plan('manager',deadline+1) if w['actor']==a['actor']),None)
        self.assertIsNotNone(command,'A saved explicit review deadline must not be silently disabled for an active scope')
        self.assertEqual(command['timer_revision'],self.s.agent(a['actor'])['work_revision'])
        self.assertEqual(self.d.owners()['A'],a['actor'])
        scheduler.claim(command['wake_id'],'manager',deadline+1)
        self.s.record_dispatch(command['wake_id'],'sent',{'threadId':a['thread_id']})
        self.observe(a,status='active',turn='self-review',now=deadline+2)
        self.observe(a,turn='self-review',now=deadline+3)
        self.assertNotIn(a['actor'],[w['actor'] for w in scheduler.plan('manager',deadline+4)],
                         'One saved deadline is not an automatic periodic loop')

    def test_return_before_pending_wake_claim_cancels_native_continuation(self):
        a,b=self.start_children();now=time.time()
        self.s.checkpoint_work(a['actor'],'waiting','Review pending scope',after_s=1,now=now-2)
        # A late tool result can return ownership after the wake was planned.
        self.s.update_agent(a['actor'],session_status='idle',session_turn='pending-return',session_observed_at=now)
        scheduler=Scheduler(self.s)
        command=next(w for w in scheduler.plan('manager',now) if w['actor']==a['actor'])
        self.call(a['actor'],'return_task',{'outcome':'cancelled','result':'Partial handback; no unresolved action.'})
        self.assertIsNone(scheduler.claim(command['wake_id'],'manager',now))
        self.assertEqual(self.d.owners()['A'],'Controller')
