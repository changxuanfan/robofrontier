import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import mujoco
from robofrontier.suite.dial_best_worker import RecordDecoder
from robofrontier.suite.dial_best_oracle import BestDecoder,BestManipulator,WORKER
from robofrontier.suite.dial_hold_oracle import Channel
from robofrontier.suite.dial_oracle import Manipulator
from scripts.build_best5_dials import verify_panel_support

ROOT=Path(__file__).resolve().parents[1]


class BestOracleTests(unittest.TestCase):
    def test_unlimited_run_keeps_progressing_but_explicit_stops_still_apply(self):
        from scripts.run_blind_dial_oracle import make_stop_check
        from types import SimpleNamespace
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'attempt';trial=SimpleNamespace(time=100000.,actions=range(100000))
            signal=[False]
            with patch('scripts.run_blind_dial_oracle.time.monotonic',return_value=100000.):
                unlimited=make_stop_check(trial,signal,output,0)
                self.assertFalse(unlimited())
                self.assertTrue(make_stop_check(trial,signal,output,0,wall_s=1)())
                self.assertTrue(make_stop_check(trial,signal,output,0,sim_s=1)())
                self.assertTrue(make_stop_check(trial,signal,output,0,actions=1)())
                signal[0]=True;self.assertTrue(unlimited());signal[0]=False
                (Path(tmp)/'STOP').touch();self.assertTrue(unlimited())

    def test_final_win_is_rechecked_after_cleanup(self):
        from scripts.run_blind_dial_oracle import final_success
        good=dict(passed=True,evaluation=dict(task_success=True,violations=[]),unresolved_robot_actions=[])
        self.assertTrue(final_success(good))
        self.assertFalse(final_success({**good,'evaluation':dict(task_success=False,violations=['late contact'])}))
        self.assertFalse(final_success({**good,'unresolved_robot_actions':['A']}))
        self.assertFalse(final_success({**good,'passed':False}))
        self.assertFalse(final_success({**good,'error':'interrupted'}))

    def test_worker_response_deadline_is_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);worker=p/'silent.py';worker.write_text('import sys,time\nsys.stdin.readline()\nsys.stdout.write("{");sys.stdout.flush()\ntime.sleep(10)\n')
            c=Channel('solver',[],p/'io.jsonl',worker=worker,timeout_s=.1)
            try:
                with self.assertRaises(TimeoutError):c.call(dict(op='choose',current=[0]*4))
                self.assertIsNotNone(c.process.poll())
            finally:
                with self.assertRaises(RuntimeError):c.close()

    def test_unchanged_angle_does_not_hide_a_lost_grip(self):
        from types import SimpleNamespace
        m=object.__new__(BestManipulator);m.dials=[{'id':'D01','index':0}]
        m.grips={'D01':{'robot':'A'}};m.hold_recoveries={};calls=[]
        m.t=SimpleNamespace(arms={'A':SimpleNamespace(grasp_force=30)},monitor=SimpleNamespace(bilateral=lambda _:set()),event=lambda *a,**kw:None)
        m.red_geoms=lambda _:[];m.prepare_symbols=lambda entries,symbols:calls.append([a['id'] for a in entries])
        m.maintain_holds([1]);self.assertEqual(calls,[['D01']])
        m.maintain_holds([1])
        with self.assertRaisesRegex(RuntimeError,'recovery budget'):m.maintain_holds([1])

    def test_live_monitor_uses_remote_evaluator_without_generating_parent_secret(self):
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.dial_best_oracle import BestMonitor
        scene=ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);code=p/'private.json';code.write_text(json.dumps({'code_symbols':[1]*4}))
            t=Trial(scene,p/'run',frames=False,monitor_factory=lambda w:BestMonitor(w,code))
            try:
                t.wait(.04)
                self.assertEqual(t.monitor.result()['feedback_mode'],'best_so_far')
                self.assertFalse((t.directory/'private-code.json').exists())
            finally:t.close(dict(scope='unit integration',passed=False))

    def test_intermediate_record_is_not_attributed_to_commanded_target(self):
        d=RecordDecoder(4,5)
        d.observe(.3,[[1,1,2,2]],2)
        d.observe(.6,[[2,2,2,2]],2)
        self.assertIn((1,1,1,1),d.candidates)
        self.assertTrue(all(sum(a==b for a,b in zip(c,[2]*4))<=2 for c in d.candidates))
        with self.assertRaises(ValueError):d.observe(.8,[[3]*4],1)

    def test_ambiguous_record_preserves_either_explanation_and_unknown_attribution(self):
        d=RecordDecoder(4,5)
        d.observe(.3,[[1,1,1,2],[1,1,2,1]],3)
        self.assertIn((1,1,1,3),d.candidates)
        count=len(d.candidates)
        d.observe(.4,[],4)
        self.assertEqual(count,len(d.candidates));self.assertIsNone(d.incumbent)

    def test_five_symbol_search_solves_synthetic_codes_with_best_only(self):
        for secret in ((1,2,3,4),(5,5,2,1),(3,3,3,3)):
            d=RecordDecoder(4,5);current=[0]*4;record=0
            for step in range(48):
                guess=d.choose(current)['guess']
                record=max(record,sum(a==b for a,b in zip(guess,secret)))
                d.observe(float(step),[guess],record)
                self.assertIn(secret,d.candidates)
                current=guess
                if record==4:break
            self.assertEqual(record,4)

    def test_solver_is_answer_independent_and_rejects_current_score_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);a=BestDecoder(4,p)
            try:
                first=a.choose([0]*4)
                a.observe(.3,[first],0)
                self.assertEqual(len(a.candidates),4**4)
                self.assertTrue(all(x!=y for x,y in zip(first,a.choose(first))))
            finally:a.close()
            (p/'b').mkdir();b=BestDecoder(4,p/'b')
            try:self.assertEqual(first,b.choose([0]*4))
            finally:b.close()
            self.assertNotIn('secret',(p/'solver-io.jsonl').read_text())

    def test_isolated_evaluator_holds_history_and_does_not_return_current_score(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);code=p/'code.json';code.write_text(json.dumps({'code_symbols':[1]*4}))
            c=Channel('evaluator',[code,ROOT/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1/task.json',ROOT/'robofrontier/suite/dial_feedback.py'],p/'eval.jsonl',worker=WORKER)
            try:
                for i in range(15):r=c.call(dict(op='sample',t=i*.02,angles=np.radians([60,60,60,120]).tolist(),eligible=True))
                self.assertEqual(set(r),{'display','won','queries','since','ready','input_valid','indicator'})
                self.assertEqual(r['display'],3);self.assertTrue(r['ready'])
                for i in range(15,31):r=c.call(dict(op='sample',t=i*.02,angles=np.radians([120]*4).tolist(),eligible=True))
                self.assertEqual(r['display'],3);self.assertTrue(r['ready'])
                r=c.call(dict(op='sample',t=.62,angles=[0.]*4,eligible=False))
                self.assertEqual(r['display'],3);self.assertFalse(r['ready'])
            finally:c.close()

    def test_new_and_legacy_angles_keep_their_own_meaning(self):
        from types import SimpleNamespace
        m=object.__new__(Manipulator)
        m.t=SimpleNamespace(task={'mechanism':{'feedback':{'step_deg':60}}})
        actual=[[round(math.degrees(a),1) for a in m.target_angles(s)] for s in range(1,6)]
        self.assertEqual(actual,[[60.],[120.],[179.5,-179.5],[-120.],[-60.]])
        m.t.task={};self.assertEqual([round(math.degrees(a),1) for a in m.target_angles(2)],[179.5,-179.5])

    def test_panel_layout_has_actual_table_support_and_expected_sizes(self):
        for n in (4,6,8):
            for level in (1,2,3):
                folder=ROOT/f'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n{n:02}-d{level}'
                m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
                verify_panel_support(m,d)
                scale=1.5 if n>=6 and level>1 else 1.
                np.testing.assert_allclose(m.geom('score_base').size,[.22*scale,.17*scale,.018*scale])
                if n==8 and level==1:np.testing.assert_allclose(d.geom('score_base').xpos[:2],[0.,0.])
                task=json.loads((folder/'public-task.json').read_text())
                self.assertNotIn('initial_supplies',task);self.assertIn('Unlock',task['objective'])


if __name__=='__main__':unittest.main()
