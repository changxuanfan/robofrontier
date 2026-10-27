from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.motion_recording import MotionWriter
from robofrontier.run_setup import write_json
from robofrontier.suite.dial_motion_edit import (
    DialReplayTrack, build_dial_edit, classify, command_evidence, event_evidence,
    process_covered,
)
from robofrontier.workstation import Workstation


class DialMotionEditTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body><joint name="A_joint1"/><geom size=".1"/></body>
          <body pos="1 0 0"><joint name="A_finger_joint1" type="slide"/><geom size=".1"/></body>
          <body pos="2 0 0"><freejoint name="module"/><geom size=".1"/></body>
          <body pos="3 0 0"><joint name="D01_hinge"/><geom size=".1"/></body>
          </worldbody></mujoco>''')
        self.times = np.arange(1081)/60
        self.q = np.tile(self.model.qpos0,(len(self.times),1))

    def classify(self,drives=(),events=(),complete=True):
        return classify(self.model,self.times,self.q,drives,events,evidence_complete=complete)

    def test_large_repetitive_rotor_motion_without_progress_is_cut(self):
        self.q[:,-1] = .15*np.sin(2*np.pi*self.times)
        with patch('mujoco.mj_step',side_effect=AssertionError('No classifier physics')):
            keep,reasons = self.classify()
        self.assertGreater(np.sum(~keep),600)
        self.assertIn('oscillating_without_progress',reasons[~keep])

    def test_slow_real_drift_and_monotonic_spring_return_remain(self):
        for values in (.0003*self.times, .05*np.exp(-self.times/4)):
            self.q[:,-1] = 0
            self.q[:,2] = 2+values
            keep,_ = self.classify()
            self.assertTrue(keep.all())

    def test_slow_and_high_frequency_oscillation_are_not_bound_to_one_window(self):
        for frequency in (1/6,20):
            self.q[:,-1] = .15*np.sin(2*np.pi*frequency*self.times)
            keep,_ = self.classify()
            self.assertGreater(np.sum(~keep),300)

    def test_oscillation_with_a_slow_moving_centre_is_preserved(self):
        self.q[:,2] = 2+.02*np.sin(2*np.pi*self.times)+.001*self.times
        self.assertTrue(self.classify()[0].all())

    def test_long_active_action_can_have_an_idle_tail(self):
        self.q[:,-1] = .15*np.sin(2*np.pi*self.times)
        drives = [dict(start=0,end=18,kind='effective',progress_times=[0,.2,.4])]
        keep,_ = self.classify(drives=drives,events=[0,18])
        self.assertTrue(keep[:30].all())
        self.assertGreater(np.sum(~keep),300)

    def test_effective_small_gripper_operation_and_unknown_evidence_stay(self):
        self.q[:,1] = self.times*.000005
        keep,_ = self.classify(drives=[dict(start=0,end=18,kind='effective')])
        self.assertTrue(keep.all())
        self.assertTrue(self.classify(complete=False)[0].all())
        self.assertTrue(self.classify(drives=[dict(start=0,end=18,kind='unknown')])[0].all())

    def test_drop_with_lateral_oscillation_is_not_erased(self):
        self.q[:,2] += .02*np.sin(2*np.pi*self.times)
        self.q[:,4] -= .005*self.times
        self.assertTrue(self.classify()[0].all())

    def test_new_attempt_with_unchanged_best_and_support_changes_are_events(self):
        state = dict(display=2,valid_trials=1,private_support_ready=[True],input_ready=True)
        process = [dict(sim_time_s=0,result=state),dict(sim_time_s=4,result=dict(state,valid_trials=2)),
            dict(sim_time_s=6,result=dict(state,private_support_ready=[False]))]
        stamps = event_evidence([],process)
        self.assertEqual(stamps,[0,4,6])
        self.q[:,-1] = .15*np.sin(2*np.pi*self.times)
        keep,_ = self.classify(events=stamps)
        self.assertTrue(keep[240] and keep[360])

    def test_process_gaps_are_not_claimed_as_known_stationarity(self):
        process = [dict(sim_time_s=t,result={}) for t in range(19)]
        self.assertTrue(process_covered(process,self.times))
        self.assertFalse(process_covered(process[:2]+process[10:],self.times))
        self.assertFalse(process_covered([],self.times))

    def test_independent_tracks_skip_different_times_and_preserve_feedback_on_seek(self):
        root = Path(__file__).resolve().parents[1]
        row = json.loads((root/'configs/task_families/dial/best5-r11.json').read_text())['cases'][0]
        source = Workstation(root/row['source_directory'])
        with tempfile.TemporaryDirectory() as td,patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')):
            path = Path(td)/'motion'
            writer = MotionWriter(path,source.path,source.model,source.data)
            for i in range(1,721):
                source.data.time=i/60
                if i==360:source.model.geom_rgba[0,0]=.123
                writer.capture(source.data)
            writer.close()
            # A complete no-action trace is valid evidence for a static scratch
            # recording. Process result changes still protect the new trial.
            (Path(td)/'process.jsonl').write_text(''.join(json.dumps(dict(sim_time_s=t,result=dict(valid_trials=0)))+'\n' for t in range(13)))
            edit = build_dial_edit(path)
            write_json(Path(td)/'edit.json',edit)
            first = DialReplayTrack(path,edit)
            self.assertTrue(first.at(0)['appearance_reset'])
            second_edit = json.loads(json.dumps(edit))
            second_edit.update(retained_frames=721,removed_frames=0,retained_spans=[dict(first_frame=0,stop_frame=721)])
            second = DialReplayTrack(path,second_edit)
            self.assertNotEqual(first.at(10)['source_time_s'],second.at(10)['source_time_s'])
            last = first.at(len(first.indices)-1)
            self.assertEqual(last['source_frame'],720)
            self.assertTrue(last['appearance_changes'])
            rewind = first.at(0)
            self.assertFalse(rewind['appearance_changes'])
            self.assertTrue(rewind['appearance_reset'])
            self.assertFalse(first.at(0)['appearance_reset'])
            self.assertTrue(first.at(len(first.indices)+10)['completed'])
            first.close();second.close()
            (Path(td)/'process.jsonl').write_text('{}\n')
            with self.assertRaisesRegex(ValueError,'evidence changed'):
                DialReplayTrack(path,edit)

    def test_repeat_hold_is_distinct_from_force_change_or_tiny_real_closure(self):
        root = Path(__file__).resolve().parents[1]
        row = json.loads((root/'configs/task_families/dial/best5-r11.json').read_text())['cases'][0]
        source = Workstation(root/row['source_directory'])
        q = np.tile(source.initial_qpos,(121,1))
        times = np.arange(121)/60
        def event(start,end,force):
            return [dict(event='action_start',sim_time_s=start,label=str(start),commands={'A':['gripper',dict(mode='grasp',width_m=.02,force_n=force)]}),
                dict(event='action_end',sim_time_s=end,label=str(start),results={'A':dict(status='completed')})]
        events = event(0,.5,30)+event(.5,1,30)+event(1,1.5,70)
        drives,_,complete = command_evidence(source.model,times,q,events,Path('/unused'))
        self.assertTrue(complete)
        self.assertEqual([d['kind'] for d in drives],['effective','hold','effective'])


if __name__ == '__main__':unittest.main()
