"""Non-physical checks: dimensions and visible-count-only reasoning."""
from pathlib import Path
from types import SimpleNamespace
import json
import mujoco
import numpy as np
import unittest
from unittest.mock import patch
from robofrontier.suite.dial_oracle_v2 import Decoder, grip_width, Manipulator, BaselineManipulator

ROOT = Path(__file__).resolve().parents[1]

class DialOracleV2Tests(unittest.TestCase):
    def test_frozen_r10_rotor_diameters(self):
        for level, expected in [(1,.054),(2,.054),(3,.0275)]:
            directory = ROOT / f'configs/environments/spring_dial/frontier-v2-r10-dial-n04-d{level}'
            task = json.loads((directory/'task.json').read_text())
            model = mujoco.MjModel.from_xml_path(str(directory/'scene.xml'))
            trial = SimpleNamespace(task=task, model=model)
            self.assertAlmostEqual(grip_width(trial,task['dials'][0]),expected)
            self.assertEqual(task['table_height_m'],.8)

    def test_count_decoder_keeps_all_consistent_candidates(self):
        decoder=Decoder(4);guess=[1,2,3,1];decoder.update(guess,2)
        self.assertGreater(len(decoder.candidates),1)
        self.assertTrue(np.all(np.sum(decoder.candidates==guess,axis=1)==2))
        before=decoder.candidates.copy();decoder.update(guess,2)
        self.assertTrue(np.array_equal(before,decoder.candidates))

    def test_count_decoder_finishes_without_answer_access(self):
        decoder=Decoder(4);current=[0]*4
        def visible_count(guess):
            return sum(a==b for a,b in zip(guess,[3,1,2,3]))
        for _ in range(24):
            current=decoder.choose(current);count=visible_count(current)
            if count==4:break
            decoder.update(current,count)
        else:self.fail('Count-only solver exceeded trial bound')

    def test_crank_release_rejects_unknown_action_or_force_mode_before_motion(self):
        for width,force,bilateral,status in [(.08,30.,set(),'completed'),(.08,None,set(),'uncertain'),(.08,None,set(),'failed')]:
            with self.subTest(width=width,force=force,bilateral=bilateral,status=status):
                calls=[]
                arm=SimpleNamespace(grasp_force=force,grip_max=.08,observe=lambda:dict(gripper_width_m=width))
                def execute(label, commands, allow_failure=False):
                    calls.append(label)
                    return {'A':dict(status=status,reason='Unexpected driver fault')}
                trial=SimpleNamespace(task={'level':3},arms={'A':arm},execute=execute,
                    data=SimpleNamespace(joint=lambda _:SimpleNamespace(qpos=[0.])),
                    monitor=SimpleNamespace(bilateral=lambda _:bilateral))
                m=object.__new__(Manipulator);m.t=trial;m.grips={'D01':{'robot':'A'}}
                m.wait_support=lambda _:None;m.red_geoms=lambda _:[]
                with self.assertRaisesRegex(RuntimeError,'action unresolved'):
                    m.release([{'id':'D01','index':0}])
                self.assertEqual(calls,['crank-open-maximum'])

    def test_non_crank_release_keeps_baseline(self):
        m=object.__new__(Manipulator);m.t=SimpleNamespace(task={'level':2})
        entries=[{'id':'D01','index':0}]
        with patch.object(BaselineManipulator,'release',return_value='baseline') as release:
            self.assertEqual(m.release(entries),'baseline')
            release.assert_called_once_with(entries)

    def test_loaded_crank_does_not_drive_return_before_uncertain_open_result(self):
        calls=[]
        def execute(label, commands, allow_failure=False):
            calls.append(label)
            return {'A':dict(status='uncertain',reason='Transport outcome unknown')}
        trial=SimpleNamespace(task={'level':3},
            arms={'A':SimpleNamespace(grasp_force=None,grip_max=.08)},
            data=SimpleNamespace(joint=lambda _:SimpleNamespace(qpos=[np.pi/2])),
            execute=execute)
        manipulator=object.__new__(Manipulator)
        manipulator.t=trial;manipulator.grips={'D01':{'robot':'A'}}
        manipulator.wait_support=lambda _:None
        with self.assertRaisesRegex(RuntimeError,'action unresolved'):
            manipulator.release([{'id':'D01','index':0}])
        self.assertEqual(calls,['crank-open-maximum'])

    def test_clear_exit_requires_actual_separation(self):
        arm=SimpleNamespace(job=None,grasp_force=None)
        trial=SimpleNamespace(arms={'A':arm},monitor=SimpleNamespace(released=lambda _:False))
        m=object.__new__(Manipulator);m.t=trial;m.grips={'D01':{'robot':'A'}}
        m.red_geoms=lambda _:[]
        with self.assertRaisesRegex(RuntimeError,'real separation'):
            m.clear_released_crank({'id':'D01'},np.zeros(3),np.eye(3))

    def test_clear_exit_never_backtracks_toward_rotor(self):
        calls=[]
        arm=SimpleNamespace(job=None,grasp_force=None,grip_max=.08,site=0,
            qids=np.array([0]),observe=lambda:dict(gripper_width_m=.08))
        trial=SimpleNamespace(arms={'A':arm},monitor=SimpleNamespace(released=lambda _:True),
            data=SimpleNamespace(site_xpos=np.array([[0.,0.,-.061]]),qpos=np.zeros(1)),
            pose_clear=lambda *_:False,pose_command=lambda *args,**kwargs:calls.append('command'))
        m=object.__new__(Manipulator);m.t=trial;m.grips={'D01':{'robot':'A'}};m.red_geoms=lambda _:[]
        with self.assertRaisesRegex(RuntimeError,'No reachable clear'):
            m.clear_released_crank({'id':'D01'},np.zeros(3),np.eye(3))
        self.assertEqual(calls,[])

    def test_capture_planning_uses_the_same_pose_override_as_contact(self):
        points=[]
        def screen(robot,point,rotation,seed=None):
            points.append(np.asarray(point).copy());return np.zeros(7)
        def stop_after_plan(*args,**kwargs):raise RuntimeError('planned sentinel')
        trial=SimpleNamespace(task={'level':3},
            arms={'A':SimpleNamespace(qref=np.zeros(7))},home_q={'A':np.zeros(7)},
            model=SimpleNamespace(joint=lambda _:SimpleNamespace(id=0)),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3)),
                joint=lambda _:SimpleNamespace(qpos=[0.]),xanchor=np.zeros((1,3)),xaxis=np.array([[0.,0.,1.]])),
            screen_ik=screen,pose_clear=lambda *args:True,move_free=stop_after_plan)
        m=object.__new__(Manipulator);m.t=trial;m.grips={};m.wait_support=lambda _:None
        m.red_pose=lambda *args:(np.array([0.,0.,.007]),np.eye(3))
        with self.assertRaisesRegex(RuntimeError,'planned sentinel'):
            m.grasp([{'id':'D01','index':0,'nominal_robots':['A','B']}],[1])
        self.assertAlmostEqual(points[1][2],.007)

    def test_wide_open_clearance_includes_the_future_finger_position(self):
        model=SimpleNamespace(ngeom=2,geom_contype=np.ones(2),geom_conaffinity=np.ones(2),
            geom_bodyid=np.array([0,1]),geom_aabb=np.array([[0,0,0,.01,.01,.01],[0,0,0,.001,.001,.001]]),
            body=lambda i:SimpleNamespace(name=('rotor','A_left_finger')[i]),
            joint=lambda _:SimpleNamespace(id=0))
        data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3)),
            geom_xmat=np.tile(np.eye(3).ravel(),(2,1)),geom_xpos=np.array([[0.,0.,0.],[0.,0.,.02]]),
            xaxis=np.array([[0.,0.,-1.]]),joint=lambda _:SimpleNamespace(qpos=[.015]))
        m=object.__new__(Manipulator);m.grips={'D01':{'robot':'A'}};m.red_geoms=lambda _:[0]
        m.t=SimpleNamespace(model=model,data=data,arms={'A':SimpleNamespace(grip_max=.08,grip_min=0.)})
        # The current hand clears, but widening would enter the rotor's axial band.
        self.assertFalse(m.wide_open_sweep_is_axially_clear({'id':'D01'}))
        data.xaxis[0,2]=1.
        self.assertTrue(m.wide_open_sweep_is_axially_clear({'id':'D01'}))

    def test_release_retries_known_terminal_recontact_from_current_state(self):
        events=[]
        m=object.__new__(Manipulator);m.grips={'D01':{'robot':'A'}}
        m.t=SimpleNamespace(task={'level':3},arms={'A':SimpleNamespace(job=None)},
            monitor=SimpleNamespace(violations=[],dial_ready=[True]),
            event=lambda name,**kwargs:events.append(name))
        with patch.object(m,'_release_cranks_once',side_effect=[
            RuntimeError('Crank clear requires terminal opening and real separation A'),'done']) as once:
            self.assertEqual(m.release([{'id':'D01','index':0}]),'done')
            self.assertEqual(once.call_count,2)
        self.assertEqual(events,['reference_forward_release_retry'])

    def test_release_does_not_retry_an_unresolved_actor(self):
        m=object.__new__(Manipulator);m.grips={'D01':{'robot':'A'}}
        m.t=SimpleNamespace(task={'level':3},arms={'A':SimpleNamespace(job='unresolved')},
            monitor=SimpleNamespace(violations=[],dial_ready=[True]))
        with patch.object(m,'_release_cranks_once',side_effect=
            RuntimeError('Crank clear requires terminal opening and real separation A')) as once:
            with self.assertRaisesRegex(RuntimeError,'real separation'):
                m.release([{'id':'D01','index':0}])
            self.assertEqual(once.call_count,1)

    def test_grasp_retries_do_not_cycle_back_to_rejected_twist(self):
        selected=[];latest=[None]
        def red_pose(dial,twist):latest[0]=twist;return np.zeros(3),np.eye(3)
        def execute(label,commands,allow_failure=False):
            if label=='red-grasp':selected.append(latest[0])
            return {'A':dict(status='failed',reason='Grasp stopped outside expected object-width tolerance')}
        trial=SimpleNamespace(task={'level':3},
            arms={'A':SimpleNamespace(qref=np.zeros(7),site=0)},home_q={'A':np.zeros(7)},
            model=SimpleNamespace(joint=lambda _:SimpleNamespace(id=0),
                geom=lambda _:SimpleNamespace(size=np.array([.01375,.022]),id=0)),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3)),
                joint=lambda _:SimpleNamespace(qpos=[0.]),xanchor=np.zeros((1,3)),xaxis=np.array([[0.,0.,1.]]),
                site_xpos=np.zeros((1,3)),site_xmat=np.eye(3).reshape(1,9)),
            screen_ik=lambda *args,**kwargs:np.zeros(7),pose_clear=lambda *args:True,
            move_free=lambda *args,**kwargs:None,execute=execute,wait=lambda _:None,
            monitor=SimpleNamespace(violations=[],bilateral=lambda _:set()))
        trial.event=lambda *args,**kwargs:None
        m=object.__new__(Manipulator);m.t=trial;m.grips={};m.red_pose=red_pose;m.wait_support=lambda _:None
        m.release=lambda _:m.grips.clear()
        with self.assertRaisesRegex(RuntimeError,'Bilateral red regrasp exhausted'):
            m.grasp([{'id':'D01','index':0,'nominal_robots':['A','B']}],[1])
        self.assertEqual(selected,[0,180,45])
