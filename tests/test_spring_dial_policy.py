"""Task boundaries and actual protocol routes with a synthetic dial apparatus."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import numpy as np

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.delegation import Delegations
from robofrontier.run_setup import prepare_run, verify_inputs
from robofrontier.runtime_service import RuntimeService
from robofrontier.spring_dial import DIRECTORY, update_display
from robofrontier.spring_dial_task import (CODE_PATH, SpringDialSampler, make_spring_task,
                                         private_code, public_spring_task, validate_spring_task)
from robofrontier.task_config import load_harness


class SyntheticDialWorld(FakeWorld):
    """Exercises the real sampler on synthetic measurements, without physics."""
    def __init__(self, directory, task):
        super().__init__(directory, task)
        names=['score_base']+[f'score_segment_{i}' for i in range(7)]+['score_ready','score_win']
        self.model=SimpleNamespace(jnt_qposadr=np.arange(4),jnt_dofadr=np.arange(4),
            joint=lambda name:SimpleNamespace(id='ABCD'.index(name.split('_')[1])),
            geom=lambda name:SimpleNamespace(id=names.index(name)),geom_rgba=np.ones((len(names),4)))
        self.data=SimpleNamespace(qpos=np.zeros(4),qvel=np.zeros(4))
        self.sampler=None
    def start_trial(self):self.sampler=SpringDialSampler(self)
    def feed(self,degrees,duration=.30):
        self.data.qpos[:]=np.radians(degrees)
        for _ in range(round(duration/.002)):
            self.time+=.002;self.sampler.sample()
    def evaluate(self):return self.sampler.result() if self.sampler else {'task_success':False}
    def evaluation_evidence(self):return {'result':self.evaluate(),'private_canary':'PRIVATE_SAMPLING_RECORD'}
    def end_trial(self):self.sampler.finish()
    def close(self):
        if self.sampler:self.sampler.close()


class SpringPolicyTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.task=make_spring_task(DIRECTORY)
        self.task_path=self.root/'task.json';self.task_path.write_text(json.dumps(self.task))
        self.code=self.root/'paired.json';self.code.write_text(json.dumps({'code_degrees':[90]*4}))
    def tearDown(self):self.tmp.cleanup()

    def prepare(self,mode,name=None):
        cfg=load_harness();cfg['topology']=mode
        path=self.root/(mode+'-harness.json');path.write_text(json.dumps(cfg))
        return prepare_run(self.root/(name or mode),self.task_path,path,self.code)

    def test_public_projection_excludes_undeclared_fields_and_private_checksums(self):
        t=copy.deepcopy(self.task);t['private_canary']={'code_degrees':[90]*4};t['last_angles']=[1,2,3,4]
        p=public_spring_task(t)
        for key in ('private_canary','last_angles','scene_asset','scene_sha256','physics'):
            self.assertNotIn(key,p)
        self.assertFalse(p['observation_contract']['live_object_truth_for_policy'])
        self.assertEqual(p['camera_roles']['game_feedback'],'board')
        t['observation_contract']['initial_positions_public']=False
        self.assertTrue(all('center_m' not in d for d in public_spring_task(t)['dials']))

    def test_rules_and_feedback_cannot_silently_drift(self):
        for change in ('rules','feedback','completion'):
            t=copy.deepcopy(self.task)
            if change=='rules':t[change]=[]
            elif change=='feedback':t[change]['angle_tolerance_deg']=44
            else:t[change]['success_latched']=False
            with self.assertRaises(ValueError):validate_spring_task(t)

    def test_private_code_input_is_validated_before_creating_a_run(self):
        for code in ([0,90,90,90],[True,90,90,90],[90]*3,[90.,90,90,90]):
            self.code.write_text(json.dumps({'code_degrees':code}))
            with self.assertRaises(ValueError):self.prepare('adaptive')
            self.assertFalse((self.root/'adaptive').exists())
        self.assertEqual(set(private_code()),{'code_symbols'})

    def test_all_modes_freeze_same_code_and_scene_without_public_leakage(self):
        for mode in ('centralized','adaptive','hierarchical','peer'):
            with self.subTest(mode=mode):
                run=self.prepare(mode)
                self.assertEqual(verify_inputs(run),[])
                self.assertEqual(json.loads((run/CODE_PATH).read_text()),{'code_symbols':[1]*4})
                self.assertEqual((run/CODE_PATH).stat().st_mode&0o777,0o600)
                public=''.join(p.read_text(errors='replace') for p in (run/'public/inputs').rglob('*')
                               if p.is_file() and p.suffix in ('.json','.md','.js'))
                self.assertNotIn('code_degrees',public)
                digest=hashlib.sha256((run/CODE_PATH).read_bytes()).hexdigest()
                self.assertNotIn(digest,public)
                self.assertTrue((run/'public/inputs/preview-board.png').is_file())
                self.assertFalse((run/'private/outputs/spring-dial-process.jsonl').exists())
                (run/CODE_PATH).write_text(json.dumps({'code_degrees':[180]*4}))
                self.assertIn('input:'+str(CODE_PATH),verify_inputs(run))

    def test_symbol_mapping_and_fresh_private_generation(self):
        from unittest.mock import patch
        with patch('robofrontier.spring_dial_task.secrets.choice',side_effect=[1,3,2,1]) as choice:
            self.assertEqual(private_code(),{'code_symbols':[1,3,2,1]})
            self.assertEqual(choice.call_count,4)
        self.assertEqual(self.task['symbol_to_angle_deg'],{'0':0,'1':90,'2':180,'3':270})
        self.code.write_text(json.dumps({'code_symbols':[3,2,1,3]}))
        self.assertEqual(private_code(self.code),{'code_symbols':[3,2,1,3]})
        self.code.write_text(json.dumps({'code_symbols':[0,2,1,3]}))
        with self.assertRaises(ValueError):private_code(self.code)

    def test_four_mode_runtime_observation_and_finish_routes(self):
        for mode in ('centralized','adaptive','hierarchical','peer'):
            with self.subTest(mode=mode):
                run=self.prepare(mode);svc=RuntimeService(run,SyntheticDialWorld,fake_capture)
                count=0
                def call(actor,op,data=None):
                    nonlocal count
                    count+=1;j=svc.accept(actor,op,data or {},f'c{count}');svc.pump();return j['result']
                try:
                    for actor in svc.policy_roles:
                        self.assertTrue(call(actor,'ready')['ready'])
                        call('Meta','bind',dict(actor=actor,thread_id='native-'+actor))
                        svc.store.observe_session(actor,'idle','ready',time.time(),'codex.wait_threads')
                    self.assertTrue(call('Meta','start')['started'])
                    authority='Controller' if mode in ('centralized','adaptive') else 'Coordinator' if mode=='hierarchical' else 'A'
                    svc.world.feed([180]*4)
                    self.assertEqual(svc.world.sampler.game.queries,1)
                    observation=call(authority,'observe',{'cameras':['board']})
                    for secret in ('code_degrees','input_degrees','actual_angle_rad','PRIVATE_SAMPLING_RECORD'):
                        self.assertNotIn(secret,json.dumps(observation))
                    svc.world.feed([90]*4)
                    self.assertTrue(svc.world.evaluate()['task_success'])
                    svc.world.feed([0]*4)
                    self.assertTrue(svc.world.evaluate()['task_success'])
                    # Latching changes the visual display, not the measured angles.
                    self.assertTrue(np.all(svc.world.data.qpos==0))
                    expected={2,3,6,7} # glyph 4 = segments b,c,f,g (+ base offset)
                    actual={i for i in range(1,8) if svc.world.model.geom_rgba[i,3]>0}
                    self.assertEqual(actual,expected)
                    result=call(authority,'finish',{'note':'Synthetic evidence only'})
                    self.assertTrue(result['ended']);self.assertNotIn('PRIVATE_SAMPLING_RECORD',json.dumps(result))
                    saved=json.loads((run/'private/outputs/evaluation.json').read_text())
                    self.assertTrue(saved['task_success']);self.assertEqual(saved['metrics']['valid_trials'],2)
                    svc.world.feed([180]*4)
                    self.assertEqual(svc.world.evaluate()['metrics'],saved['metrics'])
                finally:svc.close()

    def test_sampling_gap_is_incomplete_evidence_not_an_extra_attempt(self):
        run=self.prepare('centralized');world=SyntheticDialWorld(run,self.task);world.start_trial()
        try:
            world.feed([180]*4)
            world.time+=2.;world.sampler.sample();world.feed([180]*4)
            self.assertEqual(world.evaluate()['metrics']['valid_trials'],1)
            world.feed([90]*4)
            self.assertTrue(world.evaluate()['solved'])
            self.assertFalse(world.evaluate()['task_success'])
            self.assertFalse(world.evaluate()['evidence_complete'])
        finally:world.close()

    def test_hold_departures_require_a_stable_setting_not_a_transient_crossing(self):
        run=self.prepare('centralized');world=SyntheticDialWorld(run,self.task);world.start_trial()
        try:
            world.feed([0]*4)
            self.assertEqual(world.evaluate()['metrics']['zero_entries'],dict.fromkeys('ABCD',0))
            world.feed([180]*4,.10);world.feed([135]*4,.10)
            self.assertEqual(world.evaluate()['metrics']['holding_departures'],dict.fromkeys('ABCD',0))
            world.feed([180]*4);world.feed([135]*4,.10)
            self.assertEqual(world.evaluate()['metrics']['holding_departures'],dict.fromkeys('ABCD',1))
            world.feed([0]*4)
            self.assertEqual(world.evaluate()['metrics']['zero_entries'],dict.fromkeys('ABCD',1))
        finally:world.close()

    def test_owner_loss_remains_an_interrupted_run_even_if_apparatus_was_solved(self):
        run=self.prepare('centralized');svc=RuntimeService(run,SyntheticDialWorld,fake_capture)
        # The synthetic world directly supplies solved measurements for this
        # interruption contract; this is not a policy performance test.
        svc.world.start_trial();svc.world.feed([90]*4)
        svc.close()
        saved=json.loads((run/'private/outputs/interruption-evaluation.json').read_text())
        self.assertFalse(saved['task_success']);self.assertTrue(saved['evaluation']['solved'])
        with self.assertRaises(ValueError):RuntimeService(run,SyntheticDialWorld,fake_capture)

    def test_adaptive_return_keeps_current_measurements_and_closes_scope(self):
        run=self.prepare('adaptive');svc=RuntimeService(run,SyntheticDialWorld,fake_capture);n=0
        def call(actor,op,data=None,key=None):
            nonlocal n
            n+=1;j=svc.accept(actor,op,data or {},key or 'c'+str(n));svc.pump();return j
        try:
            call('Controller','ready');call('Meta','bind',{'actor':'Controller','thread_id':'parent'})
            svc.store.observe_session('Controller','idle','ready',time.time(),'codex.wait_threads')
            call('Meta','start')
            pending=call('Controller','delegate',{'scopes':[{'robots':['A','B'],'goal':'bounded local adjustment','done_when':'verified setting; return with holding intact'}]},'fork')
            d=Delegations(svc.store);d.claim('fork','synthetic-manager');child=d.batch('fork')['children'][0]
            d.bind('fork',child['actor'],'child-native',{'source_thread_id':'parent','source_ordinal':1,'source_sha256':'cutoff'},'synthetic-manager')
            d.activate('fork','synthetic-manager');d.started('fork',child['actor'],'sent','synthetic-manager');svc.pump()
            self.assertEqual(pending['result']['delegation']['status'],'active')
            svc.world.data.qpos[:]=[.1,.2,.3,.4];before=svc.world.data.qpos.copy()
            r=call(child['actor'],'return_task',{'outcome':'completed','result':'Synthetic setting report; holding retained'})['result']
            self.assertTrue(r['returned']);np.testing.assert_array_equal(before,svc.world.data.qpos)
            self.assertEqual(d.owners()['A'],'Controller');self.assertEqual(d.active(),[])
            self.assertIn('error',call(child['actor'],'gripper',{'robot':'A','state':'open'})['result'])
        finally:svc.close()


if __name__=='__main__':unittest.main()
