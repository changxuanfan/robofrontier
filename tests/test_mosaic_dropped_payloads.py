import copy
import unittest
from scripts.supervise_mosaic_dropped_payloads import select,select_retract,vertical_clearance_plan,full_original_goal_set,select_settled_drop,select_high_receiver,select_near_goal,plan_near_goal_pick


class DroppedPayloadAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.s=dict(phase='review',world_id='world',review_id=3,review_history=[dict(error='RuntimeError: Payload contact missing before loaded transit H_stock17')])
        self.o=dict(world_id='world',review_id=3,evaluation=dict(violations=[],assignments={}),job=dict(goal_id='g',object='H_stock17'),held={'H':'H_stock17'},
            robots={'H':dict(grasp_force_setpoint_n=50,gripper_width_m=0,tcp_m=[1.06,0,1.0]),'B':dict(grasp_force_setpoint_n=None)},
            objects={'H_stock17':dict(position_m=[.84,1.21,.83],rotation=[[0,0,-1],[-1,0,0],[0,1,0]])},last_actions=[dict(results={'H':dict(status='completed')})])
        self.task=dict(table_height_m=.8,cube_size_m=.06)

    def test_accepts_the_observed_separated_empty_hand_and_flat_table_cube(self):
        self.assertEqual(select(self.s,self.o,self.task),('H','H_stock17','g'))

    def test_a_previous_drift_error_can_recover_only_after_actual_empty_hand_table_drop(self):
        s=copy.deepcopy(self.s);s['review_history'][0]['error']='RuntimeError: Payload still drifting during bounded re-observation H_stock17'
        self.assertEqual(select(s,self.o,self.task),('H','H_stock17','g'))
        held=copy.deepcopy(self.o);held['robots']['H']['gripper_width_m']=.06
        with self.assertRaises(ValueError):select(s,held,self.task)

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


    def test_already_qualified_pocket_drop_is_kept_without_regrasping(self):
        o=copy.deepcopy(self.o);o['objects']['H_stock17']['position_m'][2]=.81
        o['evaluation']['assignments']={'actual_slot':'H_stock17'}
        self.assertEqual(select_settled_drop(self.s,o,self.task),('H','H_stock17','actual_slot'))
        o['robots']['H']['gripper_width_m']=.06
        with self.assertRaises(ValueError):select_settled_drop(self.s,o,self.task)
        o['robots']['H']['gripper_width_m']=0;o['evaluation']['assignments']={}
        with self.assertRaises(ValueError):select_settled_drop(self.s,o,self.task)

    def test_high_receiver_requires_uncommitted_flat_table_and_final_job_actor(self):
        s=copy.deepcopy(self.s);o=copy.deepcopy(self.o);task=dict(self.task,goals=[dict(goal_id='g',position_m=[-.85,-.04,.83])])
        s['review_history'][0]['error']='RuntimeError: No transfer pickup with usable continuation A/H_stock17'
        o['held']={};o['robots']={'A':dict(grasp_force_setpoint_n=None)};o['job']['path']=['C','A']
        o['objects']['H_stock17']['position_m']=[-.348,-1.078,.83]
        self.assertEqual(select_high_receiver(s,o,task),('A','H_stock17','g'))
        for change in ('not_final','force','near_goal','not_flat','stacked'):
            bad=copy.deepcopy(o)
            if change=='not_final':bad['job']['path']=['C','A','B']
            elif change=='force':bad['robots']['A']['grasp_force_setpoint_n']=50
            elif change=='near_goal':bad['objects']['H_stock17']['position_m']=[-.85,-.04,.83]
            elif change=='not_flat':bad['objects']['H_stock17']['rotation']=[[1,0,0],[0,.7,-.7],[0,.7,.7]]
            else:bad['objects']['H_stock17']['position_m'][2]=.89
            with self.assertRaises(ValueError):select_high_receiver(s,bad,task)


class NearGoalAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.s=dict(phase='review',world_id='world',review_id=12,review_history=[dict(
            error='RuntimeError: No pickup pose B/B_stock02',protected_assignments={'old':'A_stock05'})])
        self.o=dict(world_id='world',review_id=12,held={},job=dict(object='B_stock02',goal_id='g',path=['B']),
            evaluation=dict(violations=[],assignments={'old':'A_stock05'}),robots={'B':dict(grasp_force_setpoint_n=None)},
            objects={'B_stock02':dict(position_m=[.0327,-.2825,.82998],rotation=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])},
            last_actions=[dict(results={'B':dict(status='completed')})])
        self.task=dict(level=3,table_height_m=.8,cube_size_m=.06,objects=[dict(id='B_stock02',color='cream')],
            goals=[dict(goal_id='g',position_m=[.0405,-.2835,.83],requires=dict(color='cream'))])

    def test_accepts_seated_unmatched_small_error_without_changing_evidence(self):
        before=copy.deepcopy((self.s,self.o,self.task))
        self.assertEqual(select_near_goal(self.s,self.o,self.task),('B','B_stock02','g'))
        self.assertEqual((self.s,self.o,self.task),before)

    def test_rejects_incomplete_feedback_force_or_lost_protected_pair_before_planning(self):
        for change in ('failed','unknown','no_feedback','force','held','protected','matched','path','identity','violation'):
            s=copy.deepcopy(self.s);o=copy.deepcopy(self.o)
            if change in ('failed','unknown'):o['last_actions'][0]['results']['B']['status']=change
            elif change=='no_feedback':o['last_actions']=[]
            elif change=='force':o['robots']['B']['grasp_force_setpoint_n']=50
            elif change=='held':o['held']={'B':'B_stock02'}
            elif change=='protected':o['evaluation']['assignments']={}
            elif change=='matched':o['evaluation']['assignments']['g']='B_stock02'
            elif change=='path':o['job']['path']=['A','B']
            elif change=='identity':s['review_id']=11
            else:o['evaluation']['violations']=[dict(kind='contact')]
            with self.subTest(change=change),self.assertRaises(ValueError):select_near_goal(s,o,self.task)

    def test_rejects_other_recovery_geometries_and_wrong_color(self):
        for change in ('far','already_precise','stacked','tilted','other_up_face','pocket','color'):
            o=copy.deepcopy(self.o);task=copy.deepcopy(self.task)
            if change=='far':o['objects']['B_stock02']['position_m'][0]=.020
            elif change=='already_precise':o['objects']['B_stock02']['position_m'][0]=.0405
            elif change=='stacked':o['objects']['B_stock02']['position_m'][2]=.89
            elif change=='tilted':o['objects']['B_stock02']['rotation']=[[1,0,0],[0,.98,-.2],[0,.2,.98]]
            elif change=='other_up_face':o['objects']['B_stock02']['rotation']=[[0,0,-1],[0,1,0],[1,0,0]]
            elif change=='pocket':task['level']=1
            else:task['objects'][0]['color']='navy'
            with self.subTest(change=change),self.assertRaises(ValueError):select_near_goal(self.s,o,task)

    def test_candidate_exhaustion_emits_only_engineering_proposals_and_never_hides_unexpected_errors(self):
        from pathlib import Path
        from unittest.mock import patch
        from scripts.supervise_mosaic_dropped_payloads import near_goal_pick_options
        root=Path('/trial');called=[]
        def blocked(a):
            called.append(a);raise ValueError('No finite screened repair pickup candidate; no command emitted')
        with patch('scripts.supervise_mosaic_dropped_payloads.plan',side_effect=blocked):
            with self.assertRaisesRegex(ValueError,'no physical request emitted'):
                plan_near_goal_pick(root,self.o,self.task,'B','B_stock02','g','prefix')
        self.assertTrue(called)
        self.assertTrue(all(a.request.parent==root/'review/engineering' for a in called))
        for option in near_goal_pick_options(self.o,self.task,'B_stock02','g'):
            self.assertLessEqual(sum(x*x for x in option['contact_offset_m']),.010**2)
        with patch('scripts.supervise_mosaic_dropped_payloads.plan',side_effect=ValueError('Retained process violation')):
            with self.assertRaisesRegex(ValueError,'Retained process violation'):
                plan_near_goal_pick(root,self.o,self.task,'B','B_stock02','g','prefix')

    def test_physical_failure_never_tries_a_second_candidate_or_resume(self):
        import json,os,subprocess,tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        from scripts.supervise_mosaic_dropped_payloads import run
        task=dict(self.task,family='mosaic',arms={r:{} for r in 'ABCDEFGH'})
        option=dict(contact_height=.035,contact_offset_m=(0.,0.,0.),centering_contact_m=0.)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'review/requests').mkdir(parents=True)
            for filename,value in [('process.json',dict(git_commit='frozen123',pid=os.getpid())),('task.json',task),
                    ('runner-status.json',self.s),('review/observation.json',self.o)]:
                (root/filename).write_text(json.dumps(value))
            with patch('scripts.supervise_mosaic_dropped_payloads.plan_near_goal_pick',return_value=option) as planning, \
                    patch('scripts.supervise_mosaic_dropped_payloads.subprocess.run',side_effect=subprocess.CalledProcessError(1,['physical-helper'])) as physical:
                with self.assertRaises(subprocess.CalledProcessError):run(SimpleNamespace(trial=root,expected_source_commit='frozen'))
                self.assertEqual(planning.call_count,1);self.assertEqual(physical.call_count,1)
                self.assertEqual(list((root/'review/requests').iterdir()),[])


if __name__=='__main__':unittest.main()
