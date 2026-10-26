import copy
import unittest
from scripts.supervise_mosaic_dropped_payloads import select,select_retract,vertical_clearance_plan,full_original_goal_set


class DroppedPayloadAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.s=dict(phase='review',world_id='world',review_id=3,review_history=[dict(error='RuntimeError: Payload contact missing before loaded transit H_stock17')])
        self.o=dict(world_id='world',review_id=3,evaluation=dict(violations=[],assignments={}),job=dict(goal_id='g',object='H_stock17'),held={'H':'H_stock17'},
            robots={'H':dict(grasp_force_setpoint_n=50,gripper_width_m=0,tcp_m=[1.06,0,1.0]),'B':dict(grasp_force_setpoint_n=None)},
            objects={'H_stock17':dict(position_m=[.84,1.21,.83],rotation=[[0,0,-1],[-1,0,0],[0,1,0]])},last_actions=[dict(results={'H':dict(status='completed')})])
        self.task=dict(table_height_m=.8,cube_size_m=.06)

    def test_accepts_the_observed_separated_empty_hand_and_flat_table_cube(self):
        self.assertEqual(select(self.s,self.o,self.task),('H','H_stock17','g'))

    def test_rejects_stale_identity_new_errors_and_prior_process_violations(self):
        for change in ('identity','error','violation'):
            s=copy.deepcopy(self.s);o=copy.deepcopy(self.o)
            if change=='identity':o['review_id']=2
            elif change=='error':s['review_history'][0]['error']='RuntimeError: Current grip has no screened release'
            else:o['evaluation']['violations']=[dict(kind='inter_robot_contact')]
            with self.assertRaises(ValueError):select(s,o,self.task)

    def test_rejects_a_still_held_width_or_nearby_payload(self):
        for change in ('width','distance','other_force'):
            o=copy.deepcopy(self.o)
            if change=='width':o['robots']['H']['gripper_width_m']=.06
            elif change=='distance':o['objects']['H_stock17']['position_m']=[1.06,0,1.0]
            else:o['robots']['B']['grasp_force_setpoint_n']=30
            with self.assertRaises(ValueError):select(self.s,o,self.task)

    def test_rejects_off_table_tilted_completed_or_unknown_native_state(self):
        for change in ('height','rotation','completed','feedback','held'):
            o=copy.deepcopy(self.o)
            if change=='height':o['objects']['H_stock17']['position_m'][2]=.7
            elif change=='rotation':o['objects']['H_stock17']['rotation']=[[1,0,0],[0,.7,-.7],[0,.7,.7]]
            elif change=='completed':o['evaluation']['assignments']['other']='H_stock17'
            elif change=='feedback':o['last_actions'][0]['results']['H']['status']='uncertain'
            else:o['held']={}
            with self.assertRaises(ValueError):select(self.s,o,self.task)


    def test_measured_retract_requires_a_raised_close_supported_grip(self):
        s=copy.deepcopy(self.s);o=copy.deepcopy(self.o)
        s['review_history'][0]['error']='RuntimeError: Transfer retract infeasible with measured payload H_stock17'
        o['objects']['H_stock17']['position_m']=[.84,1.21,.8615]
        o['robots']['H'].update(gripper_width_m=.0598,tcp_m=[.84,1.21,.8877])
        self.assertEqual(select_retract(s,o,self.task),('H','H_stock17','g'))
        for change in ('width','height','distance','feedback','other_force'):
            bad=copy.deepcopy(o)
            if change=='width':bad['robots']['H']['gripper_width_m']=0
            elif change=='height':bad['objects']['H_stock17']['position_m'][2]=.83
            elif change=='distance':bad['robots']['H']['tcp_m']=[0,0,1]
            elif change=='feedback':bad['last_actions'][0]['results']['H']['status']='uncertain'
            else:bad['robots']['B']['grasp_force_setpoint_n']=50
            with self.assertRaises(ValueError):select_retract(s,bad,self.task)

    def test_only_the_full_original_goal_set_bypasses_the_empty_remaining_cache(self):
        o=copy.deepcopy(self.o);o['held']={};o['robots']['H']['grasp_force_setpoint_n']=None
        task=dict(goals=[dict(goal_id='g'),dict(goal_id='g2')])
        o['evaluation'].update(task_success=True,assignments={'g':'x'})
        self.assertFalse(full_original_goal_set(o,task))
        o['evaluation']['assignments']['g2']='y';self.assertTrue(full_original_goal_set(o,task))
        o['held']={'H':'x'};self.assertFalse(full_original_goal_set(o,task))

    def test_vertical_plan_keeps_observed_state_and_rejects_any_blocked_segment(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from pathlib import Path
        import numpy as np
        arm=SimpleNamespace(site=0,qids=np.arange(7),ik=SimpleNamespace(limits=np.tile([-1.,1.],(7,1))))
        data=SimpleNamespace(site_xpos=np.array([[0.,0.,1.]]),site_xmat=np.eye(3).reshape(1,9),qpos=np.zeros(7))
        t=SimpleNamespace(arms={'H':arm},data=data)
        t.screen_ik=lambda robot,point,rotation,seed:np.full(7,(point[2]-1)*.1)
        t.pose_clear=lambda robot,q,allowed:allowed==('H_stock17',)
        observed=dict(sim_time_s=4.,robots={'H':dict(tcp_quaternion_wxyz=[1,0,0,0])})
        with patch('scripts.supervise_mosaic_dropped_payloads.observed_trial',return_value=(t,observed,b'observation')):
            result=vertical_clearance_plan(Path('.'),'H','H_stock17')
            np.testing.assert_allclose(result['target_m'],[0,0,1.1]);np.testing.assert_array_equal(data.qpos,np.zeros(7))
            t.pose_clear=lambda robot,q,allowed:False
            with self.assertRaisesRegex(ValueError,'no screened route'):vertical_clearance_plan(Path('.'),'H','H_stock17')


if __name__=='__main__':unittest.main()
