import json
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import mujoco
import numpy as np
from robofrontier.suite.dial_feedback import DialFeedback,alphabet,paint_display,DIGIT_BRIGHT_RGB,DIGIT_DIM_RGB
from robofrontier.suite.evaluate import CountFeedback,Monitor

ROOT=Path(__file__).resolve().parents[1]


def hold(game,symbols,start=0.,eligible=True):
    for t in np.arange(start,start+.28,.02):
        result=game.update(float(t),np.radians(np.asarray(symbols)*60),np.zeros(len(symbols)),eligible)
    return result


class BestFiveFeedbackTests(unittest.TestCase):
    def test_all_digits_zero_to_eight_render_dim_and_ready_without_losing_segments(self):
        folder=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n08-d1'
        if not folder.exists():self.skipTest('Generate the new scene first')
        m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
        # Explicit reference masks, independent of the renderer's lookup table.
        expected=['abcdef','bc','abdeg','abcdg','bcfg','acdfg','acdefg','abc','abcdefg']
        for score,mask in enumerate(expected):
            for ready in (False,True):
                with self.subTest(score=score,ready=ready):
                    game=SimpleNamespace(display=score,ready=ready,indicator='ready' if ready else 'invalid')
                    paint_display(m,game)
                    for k,segment in enumerate('abcdefg'):
                        rgba=m.geom_rgba[m.geom(f'score_0_{k}').id]
                        self.assertEqual(rgba[3],float(segment in mask))
                        np.testing.assert_allclose(rgba[:3],DIGIT_BRIGHT_RGB if ready else DIGIT_DIM_RGB)
                    self.assertEqual(game.display,score)

    def test_record_brightness_tracks_readiness_and_replay_records_it(self):
        folder=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
        if not folder.exists():self.skipTest('Generate the new scene first')
        from robofrontier.motion_recording import MotionWriter,MotionReader
        m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m)
        game=DialFeedback([1]*4)
        with tempfile.TemporaryDirectory() as tmp:
            hold(game,[1,1,1,2]);paint_display(m,game)
            bright=m.geom_rgba.copy()
            writer=MotionWriter(Path(tmp)/'motion',folder,m,d)
            hold(game,[0]*4,.28);paint_display(m,game)
            self.assertEqual(game.display,3);self.assertFalse(game.ready)
            gid=m.geom('score_0_0').id
            np.testing.assert_allclose(m.geom_rgba[gid,:3],DIGIT_DIM_RGB)
            self.assertTrue(np.array_equal(m.geom_rgba[:,3],bright[:,3]))
            d.time=.54;writer.capture(d)
            game.update(.56,np.radians([120]*4),np.zeros(4),True);paint_display(m,game)
            self.assertEqual(game.indicator,'qualifying')
            np.testing.assert_allclose(m.geom_rgba[gid,:3],DIGIT_DIM_RGB)
            d.time=.56;writer.capture(d)
            hold(game,[2]*4,.58);paint_display(m,game)
            self.assertEqual(game.display,3);self.assertTrue(game.ready)
            np.testing.assert_allclose(m.geom_rgba[gid,:3],DIGIT_BRIGHT_RGB)
            d.time=.84;writer.close(d)
            events=list(MotionReader(Path(tmp)/'motion').events())
            colors=[e['geoms']['score_0_0'] for e in events if e['event']=='rgba' and 'score_0_0' in e['geoms']]
            np.testing.assert_allclose(colors,[[*DIGIT_DIM_RGB,1],[*DIGIT_BRIGHT_RGB,1]])

    def test_five_symbol_inputs_are_private_and_feedback_source_is_fingerprinted(self):
        task=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1/task.json'
        if not task.exists():self.skipTest('Generate the new scene first')
        from robofrontier.run_setup import prepare_run,verify_inputs
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);code=p/'code.json';code.write_text(json.dumps({'code_symbols':[5]*4}))
            run=prepare_run(p/'run',task,private_puzzle_path=code)
            self.assertEqual(json.loads((run/'private/inputs/suite-code.json').read_text())['code_symbols'],[5]*4)
            self.assertNotIn('code_symbols',(run/'public/inputs/task.json').read_text())
            self.assertEqual(verify_inputs(run),[])
            manifest=json.loads((run/'private/inputs/manifest.json').read_text())
            self.assertIn('robofrontier/suite/dial_feedback.py',manifest['environment_logic_provenance']['sources'])
            code.write_text(json.dumps({'code_symbols':[6]*4}))
            with self.assertRaises(ValueError):prepare_run(p/'bad',task,private_puzzle_path=code)
            self.assertFalse((p/'bad').exists())

    def test_initial_zero_and_off_mark_never_blank_the_record(self):
        game=DialFeedback([1]*4)
        self.assertEqual(game.display,0)
        hold(game,[0]*4);self.assertFalse(game.ready);self.assertEqual(game.queries,0)
        for t in np.arange(.28,.56,.02):game.update(float(t),np.radians([65]*4),np.zeros(4),True)
        self.assertEqual(game.display,0);self.assertFalse(game.input_valid);self.assertEqual(game.queries,0)

    def test_ready_is_independent_of_password_correctness(self):
        game=DialFeedback([1]*4)
        game.update(0.,np.radians([120]*4),np.zeros(4),True)
        self.assertEqual(game.indicator,'qualifying')
        hold(game,[2]*4,.02)
        self.assertEqual(game.display,0);self.assertTrue(game.ready);self.assertFalse(game.won)
        self.assertEqual(game.queries,1);self.assertEqual(game.indicator,'ready')

    def test_record_survives_lower_and_invalid_inputs(self):
        game=DialFeedback([1]*4)
        hold(game,[1,1,1,2]);self.assertEqual(game.display,3)
        hold(game,[2]*4,.28);self.assertEqual(game.display,3);self.assertTrue(game.ready)
        hold(game,[0]*4,.56);self.assertEqual(game.display,3);self.assertFalse(game.ready)
        self.assertEqual(game.queries,2);self.assertNotIn('correct_count',game.trials[-1])

    def test_record_does_not_accumulate_across_different_combinations(self):
        game=DialFeedback([1]*4)
        hold(game,[1,1,2,2]);hold(game,[2,2,1,1],.28)
        self.assertEqual(game.best,2);self.assertFalse(game.won)

    def test_hold_and_tolerance_are_required(self):
        game=DialFeedback([1]*4)
        hold(game,[1]*4,eligible=False);self.assertEqual(game.best,0);self.assertFalse(game.ready)
        for t in np.arange(.28,.56,.02):game.update(float(t),np.radians([64.01]*4),np.zeros(4),True)
        self.assertEqual(game.best,0)
        for t in np.arange(.56,.84,.02):game.update(float(t),np.radians([63.99]*4),np.zeros(4),True)
        self.assertEqual(game.best,4);self.assertTrue(game.ready)

    def test_signed_angles_and_five_symbol_validation(self):
        game=DialFeedback([1,2,3,4,5])
        for t in np.arange(0,.28,.02):game.update(float(t),np.radians([60,120,-180,-120,-60]),np.zeros(5),True)
        self.assertTrue(game.won);self.assertEqual(game.display,5)
        self.assertEqual(alphabet({'mechanism':{'feedback':{'symbol_count':5}}}),(1,2,3,4,5))
        self.assertEqual(alphabet({'mechanism':{}}),(1,2,3))

    def test_new_engine_current_mode_matches_legacy_normal_samples(self):
        old=CountFeedback([1,2,3,1]);new=DialFeedback([1,2,3,1],symbol_count=3,mode='current')
        for i in range(80):
            angles=np.radians(([90]*4 if i<20 else [0]*4 if i<40 else [90,180,270,90]))
            self.assertEqual(old.update(i*.02,angles,np.zeros(4),True),new.update(i*.02,angles,np.zeros(4),True))
        self.assertEqual(old.queries,new.queries)

    def test_compiled_world_cannot_qualify_angles_without_robot_grasps(self):
        folder=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
        if not folder.exists():self.skipTest('Generate the new scene first')
        from robofrontier.robot_control import PandaArm
        task=json.loads((folder/'task.json').read_text());m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id)
        arms={name:PandaArm(m,d,name) for name in task['arms']}
        world=SimpleNamespace(model=m,data=d,task=task,arms=arms);mon=Monitor(world,secret=[1]*4)
        for a in task['dials']:d.joint(a['id']+'_hinge').qpos[0]=math.pi/3
        mujoco.mj_forward(m,d)
        for t in np.arange(0,.28,.02):d.time=float(t);mon.sample_dial(.02)
        self.assertEqual(mon.game.display,0);self.assertFalse(mon.game.ready)
        self.assertEqual(mon.result_cache['private_red_hold_ready'],[False]*4)
        np.testing.assert_allclose(m.geom_rgba[m.geom('score_ready_light').id],[.47,.51,.53,1])


if __name__=='__main__':unittest.main()
