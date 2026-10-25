"""Same-cycle execution invariants through the public runtime interface."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService
from robofrontier.runtime_store import RuntimeStore
from robofrontier.task_config import load_harness
from robofrontier.scheduling import Scheduler


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name);self.run=root/'run';self.n=0
        cfg=load_harness();cfg['topology']='adaptive';p=root/'harness.json';p.write_text(json.dumps(cfg))
        prepare_run(self.run,harness_path=p);self.s=RuntimeService(self.run,FakeWorld,fake_capture)
        self.call('Controller','ready');self.call('Meta','bind',{'actor':'Controller','thread_id':'controller'})
        self.s.store.observe_session('Controller','idle','t1',time.time(),'codex.wait_threads');self.call('Meta','start')
        self.stops=[]
        def stop(robot,ident):
            self.stops.append((robot,ident));self.s.world.complete(robot,'failed')
        self.s.world.request_stop=stop
        self.data={'actions':[dict(robot=a,op='move',parameters={'delta_m':[.01,0,0]}) for a in ('A','B')],'wait':False}
    def tearDown(self):self.s.close();self.tmp.cleanup()
    def call(self,actor,op,data=None,key=None):
        self.n+=1;j=self.s.accept(actor,op,data or {},key or 'c'+str(self.n));self.s.pump();return j
    def test_starts_in_one_physics_cycle_nonblocking_query_and_no_replay(self):
        times=[];original=self.s.world.start_action
        def start(*args):times.append(self.s.world.time);return original(*args)
        with patch.object(self.s.world,'start_action',side_effect=start):
            j=self.call('Controller','action_batch',self.data,'group')
        self.assertEqual(times,[0.,0.]);ids=j['result']['action_ids']
        self.assertEqual(len(ids),2);self.assertEqual(j['result']['execution_status'],'running')
        self.call('Controller','action_batch',self.data,'group');self.assertEqual(len(self.s.world.executed),2)
        self.s.world.complete('A');self.s.pump()
        q=self.call('Controller','job',{'call_id':'group'})['result'];self.assertEqual(q['execution_status'],'running')
        self.assertEqual(q['executions'][ids[0]]['status'],'completed')
        self.s.world.complete('B');self.s.pump()
        self.assertEqual(j['result']['execution_status'],'completed')
        self.call('Controller','action_batch',self.data,'group');self.assertEqual(len(self.s.world.executed),2)
    def test_all_validate_before_start_and_duplicate_robot_rejected(self):
        bad=json.loads(json.dumps(self.data));bad['actions'][1]={'robot':'B','op':'gripper','parameters':{'state':'wrong'}}
        self.assertIn('error',self.call('Controller','action_batch',bad)['result'])
        bad['actions'][1]=bad['actions'][0]
        self.assertIn('error',self.call('Controller','action_batch',bad)['result'])
        self.assertEqual(self.s.world.executed,[]);self.assertEqual(self.s.store.actions(),{})
    def test_authority_pause_and_busy_reject_entire_group(self):
        self.assertIn('error',self.call('Meta','action_batch',self.data)['result'])
        self.s.store.put('dispatch_paused',True)
        self.assertIn('error',self.call('Controller','action_batch',self.data)['result'])
        self.assertEqual(self.s.store.actions(),{});self.assertEqual(self.s.world.executed,[])
        self.s.store.put('dispatch_paused',False)
        self.call('Controller','move',{'robot':'B','delta_m':[0,0,.01],'wait':False})
        self.assertIn('error',self.call('Controller','action_batch',self.data)['result'])
        self.assertEqual([r[0] for r in self.s.world.executed],['B'])
    def test_one_failure_stops_group_motion_but_not_another_robot(self):
        self.call('Controller','move',{'robot':'C','delta_m':[0,0,.01],'wait':False})
        j=self.call('Controller','action_batch',self.data)
        self.s.world.complete('A','failed');self.s.pump();self.s.pump()
        self.assertEqual([r[0] for r in self.stops],['B']);self.assertIn('C',self.s.world.running)
        self.assertEqual(j['result']['execution_status'],'failed')
    def test_wait_true_and_wait_events_on_batch_and_terminal_observation(self):
        data={**self.data,'wait':True,'cameras':['global']};j=self.call('Controller','action_batch',data,'blocking')
        self.assertFalse(j['done'].is_set())
        wait=self.call('Controller','wait_events',{'call_ids':['blocking'],'timeout_s':30})
        self.assertFalse(wait['done'].is_set())
        self.s.world.complete('A');self.s.pump();self.assertTrue(wait['done'].is_set());self.assertFalse(j['done'].is_set())
        self.s.world.complete('B');self.s.pump();self.assertTrue(j['done'].is_set())
        self.assertEqual(j['result']['observation']['camera_names'],['global'])
    def test_partial_start_is_unknown_not_rolled_back_or_retried(self):
        original=self.s.world.start_action
        def start(robot,*args):
            if robot=='B':raise RuntimeError('drive acknowledgement lost')
            return original(robot,*args)
        with patch.object(self.s.world,'start_action',side_effect=start):
            j=self.call('Controller','action_batch',self.data,'bad-start')
        self.assertEqual(j['result']['execution_status'],'uncertain')
        self.assertEqual(self.s.store.get('lifecycle'),'recovery_required')
        self.assertEqual(self.s.store.actions()[j['result']['action_ids'][1]]['status'],'uncertain')
        self.assertEqual(self.s.store.actions()[j['result']['action_ids'][0]]['status'],'failed')
        self.call('Controller','action_batch',self.data,'bad-start');self.assertEqual(len(self.s.world.executed),1)
    def test_owner_death_before_reservation_marks_members_unknown(self):
        self.s.store.begin_call('accepted-group','Controller','action_batch',self.data)
        count=self.s.store.recover_execution();self.assertEqual(count,2)
        self.assertTrue(all(a['status']=='uncertain' for a in self.s.store.actions().values()))
        self.assertEqual(self.s.store.call('accepted-group')['status'],'uncertain')
    def test_group_failure_preserves_other_gripper_operation(self):
        data=json.loads(json.dumps(self.data));data['actions'][1]={'robot':'B','op':'gripper','parameters':{'mode':'grasp','width_m':.045,'force_n':50}}
        self.call('Controller','action_batch',data);self.s.world.complete('A','failed');self.s.pump()
        self.assertEqual(self.stops,[]);self.assertIn('B',self.s.world.running)
        self.s.world.complete('B');self.s.pump()

    def test_nonblocking_batch_does_not_block_saved_continuation(self):
        self.call('Controller','action_batch',self.data)
        self.assertFalse(Scheduler(self.s.store).execution_blocks_wake('Controller'))

    def test_blocking_and_unknown_batch_still_block_continuation(self):
        j=self.call('Controller','action_batch',{**self.data,'wait':True})
        self.assertTrue(Scheduler(self.s.store).execution_blocks_wake('Controller'))
        ident=next(iter(self.s.store.actions()))
        self.s.store.uncertain_action(ident,'lost acknowledgement')
        self.assertTrue(Scheduler(self.s.store).execution_blocks_wake('Controller'))

    def test_terminal_failed_query_uses_physical_outcome_not_rpc_status(self):
        self.call('Controller','action_batch',self.data,'failed-group')
        self.s.world.complete('A','failed');self.s.pump();self.s.pump()
        result=self.call('Controller','job',{'call_id':'failed-group'})['result']
        self.assertEqual(result['execution_status'],'failed')
        self.assertEqual(result['execution']['status'],'failed')
        self.assertEqual(result['call_status'],'completed')

    def test_rejected_batch_query_is_failed_without_unknown_physical_work(self):
        self.call('Controller','action_batch',{'actions':[]},'invalid-group')
        result=self.call('Controller','job',{'call_id':'invalid-group'})['result']
        self.assertEqual(result['execution_status'],'failed')
        self.assertFalse(result['execution_started'])
        self.assertEqual(result['executions'],{})

if __name__=='__main__':unittest.main()
