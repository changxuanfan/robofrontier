import copy
import unittest
from scripts.supervise_mosaic_dropped_payloads import select


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


if __name__=='__main__':unittest.main()
