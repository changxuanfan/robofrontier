import unittest
import os
from pathlib import Path
import numpy as np
from robofrontier.suite.transport_motion_edit import stationary_oscillation,rattle_in_quiet,raw_motion,Settings,TransportEdit

DATA_ROOT=Path(os.environ.get('TRANSPORT_DATA_ROOT',Path(__file__).resolve().parents[1]))
BATCH=DATA_ROOT/'runs/oracle/transport-continuation-20261026'
EDITS=DATA_ROOT/'outputs/oracle/transport-continuation-20261026/transport-source-edits-v2'

class SourceRattleTests(unittest.TestCase):
    def setUp(self):self.t=np.arange(0,32,1/60);self.dt=1/60
    def test_visible_large_non_integer_cycle_rattle_is_wait(self):
        x=np.c_[.2*np.sin(2*np.pi*.31*self.t),.4*np.sin(2*np.pi*.47*self.t)]
        removed=stationary_oscillation(x,self.dt,[2e-5,5e-4])
        self.assertGreater(removed.mean(),.9)
    def test_directed_drift_under_large_rattle_is_preserved(self):
        x=(.2*np.sin(2*np.pi*.31*self.t)+self.t*.0001)[:,None]
        self.assertFalse(stationary_oscillation(x,self.dt,[2e-5]).any())
    def test_continuous_unwrapped_rotation_is_preserved(self):
        x=np.c_[.2*np.sin(2*np.pi*.31*self.t),self.t*2]
        self.assertFalse(stationary_oscillation(x,self.dt,[2e-5,5e-4]).any())
    def test_one_out_and_back_is_not_repeated_wait(self):
        x=np.sin(np.pi*self.t/self.t[-1])[:,None]
        self.assertFalse(stationary_oscillation(x,self.dt,[2e-5]).any())
    def test_short_sequence_is_preserved(self):
        x=np.sin(self.t[:120]*30)[:,None]
        self.assertFalse(stationary_oscillation(x,self.dt,[2e-5]).any())
    def test_other_robot_movement_protects_entire_workstation(self):
        x=(.2*np.sin(2*np.pi*.31*self.t))[:,None];raw=raw_motion(x,self.dt,[2e-5]);quiet=np.zeros(len(x),bool)
        self.assertFalse(rattle_in_quiet(x,self.t,quiet,raw,[2e-5],Settings()).any())
    def test_release_and_fall_are_preserved_before_bounded_rattle(self):
        x=.2*np.sin(2*np.pi*.5*self.t);x[:30]=np.linspace(1.,x[30],30)
        removed=rattle_in_quiet(x[:,None],self.t,np.ones(len(x),bool),np.ones(len(x),bool),[2e-5],Settings())
        self.assertFalse(removed[:30].any());self.assertGreater(removed[self.t>16].mean(),.8)
    def test_passive_hinge_and_translation_use_same_interval_rule(self):
        x=np.c_[.001*np.sin(self.t*5),1.2*np.sin(self.t*5)]
        self.assertGreater(stationary_oscillation(x,self.dt,[2e-5,5e-4]).mean(),.9)
    def test_two_tables_have_independent_source_clocks_and_end_hold(self):
        def edit(indices):
            obj=TransportEdit.__new__(TransportEdit);obj.indices=np.asarray(indices);obj.times=np.arange(30,dtype=float);obj.meta=dict(source_hz=1.)
            return obj
        a=edit([0,1,10,11,29]);b=edit([0,1,2,3,29])
        self.assertEqual(a.at_video_time(2.,1.)['source_index'],10)
        self.assertEqual(b.at_video_time(2.,1.)['source_index'],2)
        self.assertEqual(a.at_video_time(99.,1.)['source_index'],29)
        self.assertTrue(a.at_video_time(99.,1.)['ended'])
        with self.assertRaises(ValueError):a.at_video_time(-1.)

class RecordedSlowRockingTests(unittest.TestCase):
    def test_slow_visible_h3_roll_needs_long_context_not_higher_amplitude_threshold(self):
        from pathlib import Path
        from robofrontier.suite.transport_motion_signals import quaternion_path
        path=BATCH/'engineering/transport-pre-gpt-20261027/h3-rattle-regression.npz'
        if not path.exists():self.skipTest('Private source regression sample is not materialized')
        with np.load(path,allow_pickle=False)as z:q=z['qpos']
        x=np.c_[q[:,:3],quaternion_path(q[:,3:])];tol=[2e-5]*3+[5e-4]*3
        self.assertGreater(stationary_oscillation(x,1/60,tol).mean(),.8)
        settings=Settings();self.assertGreaterEqual(max(settings.windows_s),256.)

class ActualSourceAdapterTests(unittest.TestCase):
    def test_all_nine_source_native_adapters_hold_exact_terminal_frame(self):
        from pathlib import Path
        import json
        paths=sorted(EDITS.glob('transport-*/edit.json'))
        if len(paths)!=9:self.skipTest('Private source edits are not materialized')
        for path in paths:
            with self.subTest(case=path.parent.name):
                edit=TransportEdit(path);self.assertEqual(edit.at_video_time(0)['source_index'],0)
                final=edit.at_video_time(1e8);self.assertTrue(final['ended']);self.assertEqual(final['source_index'],edit.meta['source_frames']-1)
    def test_rate_metadata_tamper_is_rejected(self):
        import json,tempfile
        from pathlib import Path
        source=EDITS/'transport-a04-d1/edit.json'
        if not source.exists():self.skipTest('Private source edit is not materialized')
        with tempfile.TemporaryDirectory()as td:
            meta=json.loads(source.read_text());meta['source_hz']=61.;path=Path(td)/'edit.json';path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'frame/rate'):TransportEdit(path)
