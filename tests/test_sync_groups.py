"""Signed current-command synchronization; FakeWorld only, no physics/models."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from fake_pipeline import FakeWorld,fake_capture
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService
from robofrontier.delegation import Delegations
from robofrontier.scheduling import Scheduler
from robofrontier.task_config import load_harness


class SyncTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.services=[];self.n=0
    def tearDown(self):
        for s in self.services:s.close()
        self.tmp.cleanup()
    def service(self,mode='peer'):
        root=Path(self.tmp.name)/str(len(self.services));root.mkdir();cfg=load_harness();cfg['topology']=mode
        path=root/'cfg.json';path.write_text(json.dumps(cfg));run=prepare_run(root/'run',harness_path=path)
        s=RuntimeService(run,FakeWorld,fake_capture);self.services.append(s)
        for actor,value in s.store.agents().items():
            if value['kind']=='manager':continue
            self.call(s,actor,'ready');self.call(s,'Meta','bind',{'actor':actor,'thread_id':actor})
            s.store.observe_session(actor,'idle','ready',time.time(),'codex.wait_threads')
        self.call(s,'Meta','start');s.stops=[]
        def stop(robot,ident):s.stops.append(robot);s.world.complete(robot,'failed')
        s.world.request_stop=stop;return s
    def call(self,s,actor,op,data=None,key=None):
        self.n+=1;j=s.accept(actor,op,data or {},key or 'c'+str(self.n));s.pump();return j
    def part(self,robot,ident='g',roster=('A','B'),timeout=60,wait=False):
        return dict(sync_group=dict(id=ident,robots=list(roster),timeout_s=timeout),wait=wait,cameras=[],
                    actions=[dict(robot=robot,op='move',parameters=dict(delta_m=[.01,0,0]))])
    def delegate_b(self,s):
        self.call(s,'Controller','delegate',{'scopes':[dict(robots=['B'],goal='bounded outcome',done_when='verified handback')]},'fork-b')
        d=Delegations(s.store);d.claim('fork-b','manager');child=d.batch('fork-b')['children'][0]
        d.bind('fork-b',child['actor'],'delegate-b',dict(source_thread_id='Controller',source_ordinal=1,source_sha256='lineage'),'manager')
        d.activate('fork-b','manager');d.started('fork-b',child['actor'],'sent','manager');s.pump();return child['actor']
    def test_all_four_modes_use_same_executor_with_same_cycle_start(self):
        for mode in ('peer','hierarchical','adaptive','centralized'):
            with self.subTest(mode=mode):
                s=self.service(mode);times=[];original=s.world.start_action
                def start(*args):times.append(s.world.time);original(*args)
                s.world.start_action=start
                if mode=='centralized':
                    data=self.part('A');data['actions']+=self.part('B')['actions']
                    s.world.time=5;parts=[self.call(s,'Controller','action_batch',data)]
                else:
                    b=self.delegate_b(s) if mode=='adaptive' else 'B';a='Controller' if mode=='adaptive' else 'A'
                    first=self.call(s,a,'action_batch',self.part('A'));self.assertEqual(first['result']['execution_status'],'pending')
                    self.assertEqual(s.world.executed,[]);s.world.time=5
                    parts=[first,self.call(s,b,'action_batch',self.part('B'))]
                self.assertEqual(times,[5,5]);self.assertEqual(set(s.world.running),{'A','B'})
                s.world.complete('A');s.world.complete('B');s.pump()
                for part in parts:self.assertEqual(part['result']['execution_status'],'completed')
    def test_no_cross_robot_authority_or_admin_control_and_no_command_disclosure(self):
        s=self.service('hierarchical');j=self.call(s,'A','action_batch',self.part('A'))
        for actor in ('Meta','Coordinator','B'):
            self.assertIn('error',self.call(s,actor,'action_batch',self.part('A'))['result'])
        self.assertEqual(s.world.executed,[])
        self.call(s,'B','action_batch',self.part('B'))
        r=self.call(s,'A','job',{'call_id':j['id']})['result']
        foreign=[v for v in r['executions'].values() if v['arm']=='B']
        self.assertEqual(len(foreign),1);self.assertNotIn('parameters',foreign[0])
        self.assertNotIn('error',self.call(s,'Coordinator','send',{'to':['A'],'text':'review'})['result'])
        self.assertIn('error',self.call(s,'A','send',{'to':['B'],'text':'forbidden tree message'})['result'])
    def test_pending_reserves_only_submitted_robot_and_not_other_work(self):
        s=self.service();j=self.call(s,'A','action_batch',self.part('A'))
        self.assertFalse(j['result']['execution_started']);self.assertFalse(Scheduler(s.store).execution_blocks_wake('A'))
        self.assertIn('error',self.call(s,'A','move',{'delta_m':[0,0,.01]})['result'])
        self.assertNotIn('error',self.call(s,'C','move',{'delta_m':[0,0,.01],'wait':False})['result'])
        self.assertEqual(set(s.world.running),{'C'})
    def test_contract_change_duplicate_and_late_join_do_not_renew_or_start(self):
        s=self.service();data=self.part('A');j=self.call(s,'A','action_batch',data,'a-part')
        deadline=s.store.get('action_groups')['g']['deadline_at']
        self.call(s,'A','action_batch',data,'a-part');self.assertEqual(s.store.get('action_groups')['g']['deadline_at'],deadline)
        self.assertIn('error',self.call(s,'A','action_batch',data)['result'])
        self.assertIn('error',self.call(s,'B','action_batch',self.part('B',timeout=30))['result'])
        with patch('time.time',return_value=deadline+1):s.pump()
        self.assertEqual(j['result']['execution_status'],'failed');self.assertFalse(j['result']['execution_started'])
        self.assertEqual(s.world.executed,[])
        self.assertIn('error',self.call(s,'B','action_batch',self.part('B'))['result'])
    def test_owner_can_withdraw_via_parent_or_member_and_group_never_starts(self):
        for member in (False,True):
            s=self.service();j=self.call(s,'A','action_batch',self.part('A'))
            ident=j['result']['action_ids'][0] if member else j['id']
            self.assertTrue(self.call(s,'A','stop',{'call_id':ident})['result']['stop_requested'])
            self.assertEqual(j['result']['execution_status'],'failed');self.assertEqual(s.world.executed,[])
            self.assertIn('error',self.call(s,'B','action_batch',self.part('B'))['result'])
    def test_revalidate_entire_group_at_commit_without_any_partial_start(self):
        s=self.service();j=self.call(s,'A','action_batch',self.part('A'));original=s.world.validate_action;counts={}
        def validate(robot,*args):
            counts[robot]=counts.get(robot,0)+1
            if robot=='A':raise ValueError('Changed pose no longer admissible')
            return original(robot,*args)
        with patch.object(s.world,'validate_action',side_effect=validate):
            k=self.call(s,'B','action_batch',self.part('B'))
        self.assertEqual(s.world.executed,[])
        self.assertEqual(j['result']['execution_status'],'failed');self.assertEqual(k['result']['execution_status'],'failed')
        self.assertTrue(all(a['status']=='failed' for a in s.store.actions().values()))
    def test_pause_cancels_only_unstarted_group_and_waiters_receive_terminal(self):
        s=self.service();j=self.call(s,'A','action_batch',self.part('A',wait=True))
        self.assertFalse(j['done'].is_set());w=self.call(s,'A','wait_events',{'call_ids':[j['id']],'timeout_s':60})
        s.store.put('dispatch_paused',True);s.pump()
        self.assertTrue(j['done'].is_set());self.assertTrue(w['done'].is_set());self.assertEqual(s.world.executed,[])
        self.assertEqual(j['result']['execution_status'],'failed')
    def test_member_failure_brakes_agreed_group_not_unrelated_robot(self):
        s=self.service();a=self.call(s,'A','action_batch',self.part('A'));b=self.call(s,'B','action_batch',self.part('B'))
        self.call(s,'C','move',{'delta_m':[0,0,.01],'wait':False})
        s.world.complete('A','failed');s.pump();s.pump()
        self.assertEqual(s.stops,['B']);self.assertIn('C',s.world.running)
        for actor,j in [('A',a),('B',b)]:
            r=self.call(s,actor,'job',{'call_id':j['id']})['result'];self.assertEqual(r['execution_status'],'failed')
    def test_pending_delegate_cannot_return_or_be_reassigned(self):
        s=self.service('adaptive');b=self.delegate_b(s)
        self.call(s,b,'action_batch',self.part('B'))
        self.assertIn('error',self.call(s,b,'return_task',{'outcome':'cancelled','result':'not yet'})['result'])
        self.assertIn('error',self.call(s,'Controller','delegate',{'scopes':[dict(robots=['B'],goal='x',done_when='y')]})['result'])
    def test_owner_loss_before_start_is_known_cancel_not_unknown_replay(self):
        s=self.service();j=self.call(s,'A','action_batch',self.part('A'))
        self.assertEqual(s.store.recover_execution(),0)
        r=self.call(s,'A','job',{'call_id':j['id']})['result']
        self.assertEqual(r['execution_status'],'failed');self.assertFalse(r['execution_started']);self.assertEqual(s.world.executed,[])
    def test_single_owner_must_submit_all_its_roster_members_once(self):
        s=self.service('centralized')
        self.assertIn('error',self.call(s,'Controller','action_batch',self.part('A'))['result'])
        self.assertEqual(s.store.actions(),{})

    def test_partial_start_unknown_has_no_automatic_replay(self):
        s=self.service();a=self.call(s,'A','action_batch',self.part('A'),'a-part');original=s.world.start_action
        def start(robot,*args):
            if robot=='B':raise RuntimeError('lost drive acknowledgement')
            return original(robot,*args)
        with patch.object(s.world,'start_action',side_effect=start):b=self.call(s,'B','action_batch',self.part('B'),'b-part')
        self.assertEqual(a['result']['execution_status'],'uncertain');self.assertEqual(b['result']['execution_status'],'uncertain')
        self.assertEqual(s.store.get('lifecycle'),'recovery_required')
        self.call(s,'A','action_batch',self.part('A'),'a-part');self.assertEqual(len(s.world.executed),1)

    def test_completed_own_member_cannot_hand_back_unresolved_signed_operation(self):
        s=self.service('adaptive');b=self.delegate_b(s)
        self.call(s,'Controller','action_batch',self.part('A'));self.call(s,b,'action_batch',self.part('B'))
        s.world.complete('B');s.pump()
        result=self.call(s,b,'return_task',{'outcome':'completed','result':'own move settled'})['result']
        self.assertIn('error',result)
        s.world.complete('A');s.pump()
        self.assertTrue(self.call(s,b,'return_task',{'outcome':'completed','result':'group settled'})['result']['returned'])

    def test_shutdown_cancels_unstarted_group_and_bad_data_cannot_crash_owner(self):
        s=self.service();a=self.call(s,'A','action_batch',self.part('A'))
        for data in (None,[],1,'bad'):
            with self.assertRaises(ValueError):s.accept('A','action_batch',data,'bad-type')
        self.assertTrue(self.call(s,'Meta','shutdown')['result']['shutdown'])
        self.assertEqual(a['result']['execution_status'],'failed');self.assertEqual(s.world.executed,[])

    def test_report_counts_actual_motion_not_cancelled_reservations(self):
        from scripts.export_run_report import export
        s=self.service();a=self.call(s,'A','action_batch',self.part('A'))
        self.call(s,'A','stop',{'call_id':a['id']})
        self.call(s,'C','move',{'delta_m':[0,0,.01],'wait':False});s.world.complete('C');s.pump()
        self.call(s,'C','finish')
        for actor in s.policy_roles:s.store.update_agent(actor,thread_id=None)
        report=export(s.directory)
        self.assertEqual(report['async_actions'],1);self.assertEqual(report['unstarted_members'],1)
        self.assertEqual(report['executed_actions_by_arm'],{'C':1})

if __name__=='__main__':unittest.main()
