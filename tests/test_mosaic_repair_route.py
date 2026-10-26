import unittest
from scripts.execute_mosaic_repair_route import choose


class RepairRouteTests(unittest.TestCase):
    def test_small_full_orientation_correction_preserves_rotation_and_endpoint(self):
        import numpy as np
        from scripts.execute_mosaic_flat_pose_correction import rotation_step
        from robofrontier.robot_control import rotation_vector
        old=rotation_step(np.array([.1,-.07,.03]));target=rotation_step(np.array([-.04,.08,.12]))
        delta=rotation_vector(target,old)
        np.testing.assert_allclose(rotation_step(delta)@old,target,atol=1e-12)
        np.testing.assert_allclose(rotation_step(delta/2).T@rotation_step(delta/2),np.eye(3),atol=1e-12)

    def test_review_completion_uses_normal_staging_before_final_original_goal(self):
        from scripts.execute_mosaic_remaining_review import legs
        job=dict(path=['A','D'],goal='g',object='cube');point=[-.8,0,.83]
        self.assertEqual(legs(job,{frozenset(('A','D')):point},{'goal_id':'g'}),[('A','stage',point),('D','goal','g')])
        with self.assertRaisesRegex(ValueError,'normal port'):legs(job,{}, {'goal_id':'g'})

    def test_pose_liftoff_excepts_only_original_horizontal_support(self):
        from scripts.execute_mosaic_flat_pose_correction import original_support_contact
        self.assertTrue(original_support_contact('table_0',-.00002,[0,0,1]))
        self.assertTrue(original_support_contact('pocket_bottom',-.00002,[0,0,-1]))
        self.assertFalse(original_support_contact('pocket_bottom',0,[1,0,0]))
        self.assertFalse(original_support_contact('pocket_wall',0,[0,0,1]))
        self.assertFalse(original_support_contact('pocket_bottom',-.001,[0,0,1]))

    def test_flat_pose_selection_requires_free_grippers_and_the_original_named_job(self):
        from scripts.supervise_mosaic_normal_routes import select_flat_pose
        s=dict(phase='review',world_id='w',review_id=1,review_history=[dict(error='RuntimeError: No pickup pose F/E_stock20')])
        o=dict(world_id='w',review_id=1,evaluation=dict(violations=[]),job=dict(goal_id='g',object='E_stock20'),held={},
            robots={'F':dict(grasp_force_setpoint_n=None)},last_actions=[dict(results={'F':dict(status='completed')})])
        self.assertEqual(select_flat_pose(s,o),('g','E_stock20','F'))
        o['robots']['F']['grasp_force_setpoint_n']=50
        with self.assertRaisesRegex(ValueError,'free grippers'):select_flat_pose(s,o)

    def test_known_route_supervisor_requires_actual_custody_and_completed_feedback(self):
        from scripts.supervise_mosaic_normal_routes import select
        s=dict(phase='review',world_id='w',review_id=1,review_history=[dict(error='RuntimeError: No reachable loaded mosaic hover C/C_stock27')])
        o=dict(world_id='w',review_id=1,evaluation=dict(violations=[]),job=dict(goal_id='g',object='C_stock27'),
            held={'C':'C_stock27'},last_actions=[dict(results={'C':dict(status='completed')})])
        self.assertEqual(select(s,o),('g','C_stock27'))
        o['held']={}
        with self.assertRaisesRegex(ValueError,'custody'):select(s,o)
        o['held']={'C':'C_stock27'};o['last_actions'][0]['results']['C']['status']='uncertain'
        with self.assertRaisesRegex(ValueError,'feedback'):select(s,o)

    def test_supervisor_does_not_generalize_to_a_new_failure_class(self):
        from scripts.supervise_mosaic_normal_routes import select
        s=dict(phase='review',world_id='w',review_id=1,review_history=[dict(error='RuntimeError: Payload not retained')])
        o=dict(world_id='w',review_id=1,evaluation=dict(violations=[]))
        with self.assertRaisesRegex(ValueError,'New error class'):select(s,o)

    def rows(self):
        return [dict(robots=['C','A'],position_m=[0,0,0],angled_grasp={'C':15}),
            dict(robots=['C','B'],position_m=[1,0,0],angled_grasp=None),
            dict(robots=['B','A'],position_m=[2,0,0],angled_grasp=None)]
    def test_use_a_reachable_alternative_receiver_instead_of_an_incompatible_shortcut(self):
        self.assertEqual(choose(self.rows(),'C',['A','B'],['A','B','C'])[0],['C','B'])
    def test_use_intermediate_robot_when_only_the_original_receiver_reaches_the_goal(self):
        self.assertEqual(choose(self.rows(),'C',['A'],['A','B','C'])[0],['C','B','A'])
    def test_occupied_port_is_not_silently_reused(self):
        with self.assertRaises(ValueError):choose(self.rows(),'C',['A'],['A','B','C'],[frozenset(('B','C'))])


if __name__=='__main__':unittest.main()
