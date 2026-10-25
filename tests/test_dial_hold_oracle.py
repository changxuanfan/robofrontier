import json
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import numpy as np
from robofrontier.suite.dial_hold_oracle import BlindDecoder, Channel, HoldingManipulator


class BlindHoldTests(unittest.TestCase):
    def test_held_target_does_not_require_a_post_turn_exit(self):
        def screen(robot,point,rotation,seed=None):
            return None if point[2]>.04 else np.zeros(7)
        trial=SimpleNamespace(task={'level':3},arms={'A':SimpleNamespace(qref=np.zeros(7))},
            home_q={'A':np.zeros(7)},model=SimpleNamespace(joint=lambda _:SimpleNamespace(id=0)),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3)),
                joint=lambda _:SimpleNamespace(qpos=[0.]),xanchor=np.zeros((1,3)),xaxis=np.array([[0.,0.,1.]])),
            screen_ik=screen,pose_clear=lambda *args:True,
            move_free=lambda *args,**kwargs:(_ for _ in ()).throw(RuntimeError('Plan admitted before actuation')))
        m=object.__new__(HoldingManipulator);m.t=trial;m.grips={};m.wait_support=lambda _:None
        m.red_pose=lambda *args:(np.zeros(3),np.eye(3))
        m.require_grasp_exit=True
        with self.assertRaisesRegex(RuntimeError,'No complete red-grasp arc'):
            m.grasp([dict(id='D01',index=0,nominal_robots=['A','B'])],[2])
        m.require_grasp_exit=False
        with self.assertRaisesRegex(RuntimeError,'Plan admitted before actuation'):
            m.grasp([dict(id='D01',index=0,nominal_robots=['A','B'])],[2])

    def test_solver_first_guess_cannot_depend_on_an_answer(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=BlindDecoder(4,tmp)
            try:first=a.choose([0]*4)
            finally:a.close()
            Path(tmp,'second').mkdir();b=BlindDecoder(4,Path(tmp)/'second')
            try:
                self.assertEqual(first,b.choose([0]*4))
                self.assertTrue(set(first)<={1,2,3})
                b.update(first,0);self.assertEqual(len(b.candidates),16)
                self.assertTrue(all(x!=y for x,y in zip(first,b.choose(first))))
            finally:b.close()

    def test_evaluator_does_not_return_a_secret_and_invalidates_an_unheld_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);book=p/'book.json';book.write_text(json.dumps({'cases':[{'case_id':'unit','symbols':[1,2,3,1]}]}))
            c=Channel('evaluator',[book,'unit'],p/'io.jsonl')
            try:
                for t in np.arange(0,.30,.02):r=c.call(dict(op='sample',t=float(t),angles=[math.pi/2]*4,eligible=True))
                self.assertEqual(set(r),{'display','won','queries','since'});self.assertEqual(r['display'],2)
                r=c.call(dict(op='sample',t=.30,angles=[math.pi/2]*4,eligible=False))
                self.assertIsNone(r['display']);self.assertFalse(r['won'])
            finally:c.close()

    def test_normal_held_update_changes_target_without_release_or_regrasp(self):
        events=[];m=object.__new__(HoldingManipulator)
        arm=SimpleNamespace(grasp_force=30.,site=0,qref=np.zeros(7))
        m.t=SimpleNamespace(task={'level':1},arms={'A':arm},
            monitor=SimpleNamespace(bilateral=lambda _:{'A'}),
            model=SimpleNamespace(joint=lambda _:SimpleNamespace(id=0)),
            data=SimpleNamespace(joint=lambda _:SimpleNamespace(qpos=[math.pi/2]),
                xanchor=np.zeros((1,3)),xaxis=np.array([[0.,0.,1.]]),
                site_xpos=np.array([[.05,0.,0.]]),site_xmat=np.eye(3).reshape(1,9)),
            screen_ik=lambda *args,**kwargs:np.zeros(7),event=lambda kind,**kw:events.append(kind))
        m.grips={'D01':dict(robot='A',target_angle_rad=math.pi/2)};m.red_geoms=lambda _:[]
        m.release=lambda _:self.fail('Normal update released the dial')
        m.grasp=lambda *args:self.fail('Normal update unnecessarily regrasped')
        m.prepare_symbols([dict(id='D01',index=0)],[3])
        self.assertAlmostEqual(m.grips['D01']['target_angle_rad'],-math.pi/2)
        self.assertEqual(events,['continuous_grasp_target'])


if __name__=='__main__':unittest.main()
