import unittest
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import numpy as np
from robofrontier.station_edit import classify_window, build, StationEdit


class MotionClassificationTests(unittest.TestCase):
    def sample(self):
        q=np.zeros((321,8));q[:,4]=1
        joints=[dict(id='A_joint1',type=3,qpos_address=0),
                dict(id='cube_free',type=0,qpos_address=1)]
        return q,joints

    def test_static_and_large_passive_repetition_are_cut_candidates(self):
        q,j=self.sample();self.assertEqual(classify_window(q,j),'quiet')
        q[:,1]=.15*np.sin(np.linspace(0,8*np.pi,len(q)))
        self.assertEqual(classify_window(q,j),'repeated_passive')

    def test_real_stream_cuts_idle_but_preserves_fail_recover_progress_and_appearance(self):
        from robofrontier.task_families import ROOT,RELEASE,read
        from robofrontier.workstation import Workstation
        from robofrontier.motion_recording import MotionWriter,MotionReader,digest
        spec=read(ROOT/RELEASE)['families']['transport']['oracle'][0]
        source=Workstation(ROOT/spec['directory'])
        with tempfile.TemporaryDirectory() as tmp, patch('mujoco.mj_step',side_effect=AssertionError('No physics')):
            root=Path(tmp);recording=root/'motion'
            writer=MotionWriter(recording,source.path,source.model,source.data)
            for frame in range(1,2401):
                source.data.time=frame/60;writer.capture(source.data)
            writer.event(8.,'rgba',geoms={source.manifest['geometries'][0]['id']:[1,0,0,1]})
            writer.event(12.,'drop');writer.event(18.,'recovery')
            writer.close(source.data)
            (root/'actions.json').write_text(json.dumps([dict(start_sim_s=26.,end_sim_s=30.)]))
            (root/'process.jsonl').write_text(json.dumps(dict(sim_time_s=32.,result=dict(task_success=True)))+'\n')
            old=root/'old.json';old.write_text('{}')
            new_spec=dict(spec,recording=str(recording),edit=str(old),edit_sha256=digest(old))
            with patch('robofrontier.station_edit._base_keep',side_effect=lambda spec,reader,times:(np.ones(len(times),bool),len(times)-1)):
                build(new_spec,root/'edited')
            reader=MotionReader(recording);edited=StationEdit(root/'edited/edit.json',reader)
            self.assertFalse(edited.keep[240]) # Ordinary idle at four seconds.
            for seconds in (0,8,12,18,26,30,32,40):
                self.assertTrue(edited.keep[round(seconds*60)],seconds)
            self.assertGreater((~edited.keep).sum(),600)
            (root/'actions.json').write_text('[]')
            with self.assertRaisesRegex(ValueError,'semantic evidence'):
                StationEdit(root/'edited/edit.json',reader)

    def test_robot_work_slow_drift_and_full_continuous_rotations_survive(self):
        q,j=self.sample();q[:,0]=np.linspace(0,.0001,len(q))
        self.assertEqual(classify_window(q,j),'keep')
        q,j=self.sample();q[:,1]=np.linspace(0,.001,len(q))
        self.assertEqual(classify_window(q,j),'keep')
        q,j=self.sample();angle=np.linspace(0,4*np.pi,len(q));q[:,4]=np.cos(angle/2);q[:,7]=np.sin(angle/2)
        self.assertEqual(classify_window(q,j),'keep')

    def test_passive_rotational_oscillation_is_distinguished_from_rotation(self):
        q,j=self.sample();angle=.3*np.sin(np.linspace(0,8*np.pi,len(q)))
        q[:,4]=np.cos(angle/2);q[:,7]=np.sin(angle/2)
        self.assertEqual(classify_window(q,j),'repeated_passive')

    def test_slow_progress_under_large_repetition_is_preserved(self):
        q,j=self.sample();t=np.arange(len(q))*.1
        q[:,1]=.2*np.sin(2*np.pi*.31*t)+t*.0001
        self.assertEqual(classify_window(q,j),'keep')
        q,j=self.sample();angle=.3*np.sin(2*np.pi*.31*t)+t*.002
        q[:,4]=np.cos(angle/2);q[:,7]=np.sin(angle/2)
        self.assertEqual(classify_window(q,j),'keep')


if __name__=='__main__':unittest.main()
