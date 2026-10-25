"""Suite contracts and feedback logic, not physical task success claims."""
import json,unittest
import mujoco
from itertools import product
import numpy as np
from robofrontier.suite.scenes import catalog,case_directory
from robofrontier.suite.tasks import make_task,public_task
from robofrontier.suite.evaluate import CountFeedback,SupportQualification
from robofrontier.suite.dial_oracle import Decoder
from robofrontier.suite.dial_oracle import Manipulator
from types import SimpleNamespace

class SuiteContractTests(unittest.TestCase):
    def test_36_configs_and_public_projection_do_not_export_initial_object_poses(self):
        cases=catalog()['cases'];self.assertEqual(len(cases),36)
        for case in cases:
            with self.subTest(case=case['id']):
                task=make_task(case_directory(case));public=public_task(task)
                self.assertEqual(len(task['arms']),case['arms'])
                self.assertNotIn('goals',public);self.assertNotIn('route',public)
                self.assertNotIn('scene_asset',public);self.assertNotIn('physics',public)
                for obj in public.get('objects',[]):self.assertNotIn('initial_position_m',obj)
                if case['family']=='mosaic':
                    self.assertNotIn('objects',public);self.assertEqual(sum(s['count'] for s in public['initial_supplies']),len(task['objects']))
                self.assertNotIn('private-code',json.dumps(public))

    def test_feedback_scales_and_zero_invalidation(self):
        for n in (4,6,8):
            game=CountFeedback([1]*n)
            for i in range(20):game.update(i*.02,np.full(n,np.pi),np.zeros(n),True)
            self.assertEqual(game.display,0);self.assertEqual(game.queries,1)
            game.update(.4,np.zeros(n),np.zeros(n),True);self.assertIsNone(game.display)
            for i in range(21,42):game.update(i*.02,np.full(n,np.pi/2+.02),np.zeros(n),True)
            self.assertEqual(game.display,n);self.assertTrue(game.won)
            game.update(.84,np.zeros(n),np.zeros(n),False);self.assertEqual(game.display,n)

    def test_display_accepts_in_band_vibration_but_restarts_after_leaving_the_band(self):
        game=CountFeedback([2]*4)
        for i in range(20):
            t=i*.02;angles=np.full(4,np.pi/2+np.radians(1)*np.sin(2*np.pi*20*t));speeds=np.full(4,np.radians(1)*2*np.pi*20*np.cos(2*np.pi*20*t))
            game.update(t,angles,speeds,True)
        self.assertEqual(game.display,0)
        game.update(.4,np.full(4,np.pi/2+np.radians(4.1)),np.zeros(4),True);self.assertIsNone(game.display)
        for i in range(21,30):game.update(i*.02,np.full(4,np.pi/2),np.zeros(4),True)
        self.assertIsNone(game.display)

    def test_decoder_uses_only_aggregate_counts(self):
        codes=list(product((1,2,3),repeat=4))+[(1,)*6,(3,)*8,(1,2,3,1,3,2),(3,1,2,2,1,3,2,1)]
        for code in codes:
            d=Decoder(len(code));guess=[0]*len(code)
            for _ in range(24):
                guess=d.choose(guess);count=sum(x==y for x,y in zip(guess,code));d.update(guess,count)
                if count==len(code):break
            self.assertEqual(tuple(guess),code)

    def test_physical_reference_decoder_does_not_read_hidden_input_history(self):
        class VisibleCount:
            display=0
            @property
            def trials(self):raise AssertionError('Unobserved evaluator history is not decoder input')
        pose=SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3).ravel())
        world=SimpleNamespace(task={'dials':[{'id':f'D{i:02d}'} for i in range(4)]},data=SimpleNamespace(body=lambda _:pose),monitor=SimpleNamespace(game=VisibleCount()))
        operator=Manipulator(world);operator.update_decoder([1]*4)
        self.assertEqual(len(operator.decoder.candidates),16)

    def test_support_short_gap_does_not_amplify_into_a_new_full_dwell(self):
        q=SupportQualification()
        for i in range(15):q.update(i*.02,True)
        self.assertFalse(q.update(.30,False));self.assertTrue(q.update(.32,True))
        for i in range(17,22):q.update(i*.02,False)
        self.assertFalse(q.update(.44,True))
        for i in range(23,37):ready=q.update(i*.02,True)
        self.assertTrue(ready)

    def test_support_cannot_be_qualified_across_an_unsampled_interval(self):
        q=SupportQualification()
        q.update(0.,True);self.assertFalse(q.update(1.,True))

    def test_grid_and_unmarked_mosaics_have_matching_initial_dynamics(self):
        for size in ('a04-7x7','a06-7x14','a08-7x21'):
            scenes=[case_directory(next(c for c in catalog()['cases'] if c['id']==f'mosaic-{size}-d{level}')) for level in (2,3)]
            a,b=[mujoco.MjModel.from_xml_path(str(scene/'scene.xml')) for scene in scenes]
            for field in ('body_pos','body_quat','body_mass','body_inertia','body_ipos','body_iquat','jnt_type','jnt_pos','jnt_axis','jnt_range','dof_damping','dof_armature','actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange','actuator_trnid','tendon_stiffness','tendon_damping','eq_type','eq_obj1id','eq_obj2id','eq_data','key_qpos'):
                np.testing.assert_allclose(getattr(a,field),getattr(b,field),atol=1e-12,err_msg=f'{size}/{field}')
            masks=[(m.geom_contype!=0)|(m.geom_conaffinity!=0) for m in (a,b)]
            for field in ('geom_type','geom_bodyid','geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity','geom_solref','geom_solimp','geom_condim','geom_friction','geom_priority','geom_solmix','geom_margin','geom_gap'):
                np.testing.assert_allclose(getattr(a,field)[masks[0]],getattr(b,field)[masks[1]],atol=1e-12,err_msg=f'{size}/{field}')

if __name__=='__main__':unittest.main()
