import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from robofrontier.suite.relay_live_recovery import RelayLiveRecovery

class RelayLiveRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.t=SimpleNamespace(directory=Path(self.tmp.name),time=1.,
            arms={a:SimpleNamespace(job=None) for a in ['G','H']},event=Mock(),close=Mock(),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3))),
            monitor=SimpleNamespace(violations=[],result=lambda:{'task_success':False},
                bilateral=lambda *a,**k:{'G','H'},relay_v2=SimpleNamespace(states={'p':{'rod_geoms':{1}}})))
        self.recovery=RelayLiveRecovery(self.t,{})
        self.request=dict(id='repair-1',world_id=self.recovery.world_id,op='carry',payload='p',robots=['G','H'],target_m=[1.3,0,1.2],review='finite reviewed move',assisted=True)
    def tearDown(self):self.tmp.cleanup()
    def test_known_rejection_retains_world_and_failed_request_history(self):
        with patch('robofrontier.suite.relay_live_recovery.ref.carry',side_effect=RuntimeError('guard rejection')):
            result=self.recovery.process(self.request)
        self.assertEqual(result['status'],'failed');self.assertIn('guard rejection',result['error'])
        self.t.close.assert_not_called();self.assertTrue(self.recovery.assisted)
        self.assertTrue((self.recovery.directory/'accepted/repair-1.json').exists())
    def test_duplicate_id_returns_feedback_without_repeating_motion(self):
        with patch('robofrontier.suite.relay_live_recovery.ref.carry') as carry:
            first=self.recovery.process(self.request);second=self.recovery.process(self.request)
        self.assertEqual(first,second);self.assertEqual(carry.call_count,1)
        with self.assertRaises(ValueError):self.recovery.process(dict(self.request,target_m=[1.5,0,1.2]))
    def test_unknown_original_execution_cannot_be_bypassed_by_new_id(self):
        self.t.arms['G'].job='original-unresolved'
        with self.assertRaisesRegex(ValueError,'Original execution unresolved'):self.recovery.process(self.request)
        self.t.close.assert_not_called()
    def test_state_edits_and_wrong_world_are_rejected(self):
        for request in [dict(self.request,op='reset'),dict(self.request,world_id='old-world')]:
            with self.assertRaises(ValueError):self.recovery.process(request)
        self.t.close.assert_not_called()

    def test_retained_handover_preserves_the_other_holder_and_selected_successor(self):
        self.t.data.body=lambda _:SimpleNamespace(xpos=np.array([0.,0.,1.2]),xmat=np.eye(3))
        self.t.arms['I']=SimpleNamespace(job=None);self.t.wait=Mock()
        continuation=dict(future_region=[0.,0.,1.2],future_waypoints=[[[.2,0.,1.2],0.]])
        request=dict(self.request,op='handover',retained_side=0,continuation=continuation)
        with patch('robofrontier.suite.relay_live_recovery.ref.choose_team',return_value=['G','I']) as choose,patch('robofrontier.suite.relay_live_recovery.ref.grasp'),patch('robofrontier.suite.relay_live_recovery.ref.release') as release:
            result=self.recovery.process(request)
        self.assertEqual(result['status'],'completed')
        self.assertEqual(choose.call_args.kwargs['retained'],{0:'G'})
        self.assertEqual(choose.call_args.kwargs['future_waypoints'],continuation['future_waypoints'])
        release.assert_called_once_with(self.t,'p',['H'])

    def test_unbounded_or_unknown_continuation_is_rejected_before_motion(self):
        for continuation in [{'execute':'code'},{'future_region':[0,0,1],'future_waypoints':[[[0,0,1],0]]*5}]:
            with self.assertRaises(ValueError):self.recovery.process(dict(self.request,op='handover',continuation=continuation))

    def test_supported_open_uses_payload_then_robot_contact_arguments(self):
        self.t.execute=Mock();self.t.held={'G':'p','H':'p'}
        self.t.monitor.relay_v2.states['p']['all_geoms']={1}
        self.t.monitor.relay_v2.environment_contact=lambda _:True
        with patch('robofrontier.suite.relay_live_recovery.ref.arm_payload_contact',return_value=False) as contact:
            result=self.recovery.process(dict(self.request,op='open'))
        self.assertEqual(result['status'],'completed');self.assertEqual(self.t.held,{})
        self.assertEqual([call.args[1:] for call in contact.call_args_list],[('p','G'),('p','H')])

    def test_single_empty_robot_joint_request_is_typed_and_not_replayed(self):
        request=dict(self.request,op='move_empty',robots=['G'],joint_rad=[0.]*7)
        with patch('robofrontier.suite.relay_live_recovery.ref.move_empty_robot') as move:
            first=self.recovery.process(request);second=self.recovery.process(request)
        self.assertEqual(first['status'],'completed');self.assertEqual(first,second)
        move.assert_called_once_with(self.t,'p','G',[0.]*7)
        with self.assertRaisesRegex(ValueError,'Joint target applies only'):
            self.recovery.process(dict(self.request,id='wrong-op',joint_rad=[0.]*7))

    def test_user_accepted_goal_stack_can_open_without_direct_table_contact(self):
        self.t.execute=Mock();self.t.held={'G':'p','H':'p'}
        self.t.monitor.relay_v2.states['p']['all_geoms']={1}
        self.t.monitor.relay_v2.environment_contact=lambda _:False
        with patch('robofrontier.suite.relay_live_recovery.ref.user_goal_open_supported',return_value=True,create=True),patch('robofrontier.suite.relay_live_recovery.ref.arm_payload_contact',return_value=True):
            result=self.recovery.process(dict(self.request,op='open'))
        self.assertEqual(result['status'],'completed')
        self.assertEqual(self.t.execute.call_args.args[1]['G'][1]['width_m'],.08)
        self.assertEqual(self.t.held,{'G':'p','H':'p'})

    def test_supported_open_cannot_release_differently_labelled_cargo(self):
        self.t.execute=Mock();self.t.held={'G':'other','H':'p'}
        self.t.monitor.relay_v2.states['p']['all_geoms']={1}
        self.t.monitor.relay_v2.environment_contact=lambda _:True
        result=self.recovery.process(dict(self.request,op='open'))
        self.assertEqual(result['status'],'failed');self.t.execute.assert_not_called()

    def test_cooperative_roll_is_a_normal_carry_orientation_not_a_pose_write(self):
        request=dict(self.request,target_quaternion_wxyz=[np.sqrt(.5),0,np.sqrt(.5),0])
        with patch('robofrontier.suite.relay_live_recovery.ref.carry') as carry:
            result=self.recovery.process(request)
        self.assertEqual(result['status'],'completed')
        np.testing.assert_allclose(carry.call_args.kwargs['target_rotation'],[[0,0,1],[0,1,0],[-1,0,0]],atol=1e-12)
        self.t.close.assert_not_called()

    def test_roll_orientation_rejects_nonunit_or_conflicting_input_without_motion(self):
        requests=[dict(self.request,target_quaternion_wxyz=[2,0,0,0]),
                  dict(self.request,target_quaternion_wxyz=[1,0,0]),
                  dict(self.request,target_quaternion_wxyz=[1,0,0,0],retain_attitude=True)]
        with patch('robofrontier.suite.relay_live_recovery.ref.carry') as carry:
            for index,request in enumerate(requests):
                result=self.recovery.process(dict(request,id='bad-roll-'+str(index)))
                self.assertEqual(result['status'],'failed')
        carry.assert_not_called()

    def test_canonical_owner_boundary_does_not_invent_a_recoverable_failure(self):
        self.t.monitor.result=lambda:{'task_success':True}
        self.t.wait=Mock();self.t.check_stop=Mock()
        with patch.object(self.recovery,'publish'):
            self.assertEqual(self.recovery.wait_for_repair(),'complete')
        self.assertNotIn('recoverable_failures',self.recovery.report)
        self.assertEqual(self.t.event.call_args.args[0],'relay_owner_boundary')

    def test_partial_original_custody_recovery_preserves_surviving_grasp(self):
        self.t.monitor.bilateral=lambda *a,**k:{'H'}
        self.t.monitor.relay_v2.states['p'].update(dropped=True,custody={'G','H'})
        self.t.arms['H'].site=0
        self.t.data=SimpleNamespace(site_xpos=np.array([[0.,-.48,1.2]]),
            body=lambda _:SimpleNamespace(xpos=np.array([0.,0.,1.2]),xmat=np.eye(3)))
        with patch('robofrontier.suite.relay_live_recovery.ref.choose_team',return_value=['H','G']) as choose,patch('robofrontier.suite.relay_live_recovery.ref.grasp') as grasp:
            result=self.recovery.process(dict(self.request,op='grasp'))
        self.assertEqual(result['status'],'completed')
        self.assertEqual(choose.call_args.kwargs['retained'],{0:'H'})
        self.assertEqual(choose.call_args.kwargs['exclude'],{'H'})
        self.assertEqual(choose.call_args.kwargs['departing'],())
        grasp.assert_called_once_with(self.t,'p',['H','G'],.44)

    def test_healthy_pair_is_not_reopened_by_recovery_grasp(self):
        self.t.monitor.relay_v2.states['p'].update(dropped=False,custody={'G','H'})
        with patch('robofrontier.suite.relay_live_recovery.ref.grasp') as grasp:
            result=self.recovery.process(dict(self.request,op='grasp'))
        self.assertEqual(result['status'],'failed')
        grasp.assert_not_called()

    def test_physics_catchup_services_new_owner_request_before_finishing_wall_debt(self):
        self.t.check_stop=Mock();self.t.wait=Mock();self.t.time=0.
        self.recovery.publish=Mock();state={'steps':0,'done':False,'clock_calls':0}
        self.t.monitor.result=lambda:{'task_success':state['done']}
        def clock():
            state['clock_calls']+=1
            return 0. if state['clock_calls']<=2 else 1.
        def step():
            state['steps']+=1;self.t.time+=.002
            if state['steps']==1:
                (self.recovery.directory/'requests/new-owner.json').write_text(json.dumps({'id':'new-owner'}))
            if state['steps']>100:raise AssertionError('Owner request starved inside physics catchup')
        def process(request):
            self.assertEqual(request['id'],'new-owner');state['done']=True
            return {'status':'completed'}
        self.t.step=step
        with patch.object(self.recovery,'process',side_effect=process),patch('robofrontier.suite.relay_live_recovery.time.monotonic',side_effect=clock),patch('robofrontier.suite.relay_live_recovery.time.sleep'):
            self.assertEqual(self.recovery.wait_for_repair(RuntimeError('retained world')),'complete')
        self.assertLessEqual(state['steps'],25)

if __name__=='__main__':unittest.main()
