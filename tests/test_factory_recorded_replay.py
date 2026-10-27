import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.motion_recording import MotionWriter,MotionReader
from robofrontier.workstation import Workstation
from robofrontier.suite.v2 import ROOT,cases,directory

class FactoryRecordedReplayTests(unittest.TestCase):
 def test_moving_robot_module_and_display_reconstruct_without_physics(self):
  scene=ROOT/'outputs/robo-factory/v6-workstations/scene.xml'
  if not scene.exists():self.skipTest('Generated factory not present')
  source=Workstation(directory(next(c for c in cases() if c['id']=='dial-n04-d3')))
  with tempfile.TemporaryDirectory() as td,patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')):
   writer=MotionWriter(Path(td)/'motion',source.path,source.model,source.data,hz=60)
   robot=source.model.joint('A_joint1').qposadr[0];body=source.model.joint('D01_module_free').qposadr[0];gid=source.model.geom('score_0_0').id
   for i in range(1,61):
    source.data.time=i/60;source.data.qpos[robot]+=.0001;source.data.qpos[body]+=.0001
    if i==30:source.model.geom_rgba[gid]=[.1,.2,.3,.75]
    writer.capture(source.data)
   writer.close();reader=MotionReader(Path(td)/'motion');frozen=Workstation(reader.path/'source');play=FactoryPlayback(scene);last=None
   for stamp,q in reader.frames():last=play.apply(reader.meta['case_id'],frozen,q,stamp)
   names=play.manifest['workstation_names'][reader.meta['case_id']]
   for name,target in names['geom_names'].items():np.testing.assert_allclose(play.data.geom_xpos[play.model.geom(target).id],last['geometries'][name]['position_m'],atol=1e-6)
   changes=[e for e in reader.events() if e['event']=='rgba'];self.assertEqual(len(changes),1);play.appearance(reader.meta['case_id'],changes[0]['geoms'])
   target=names['geom_names']['score_0_0'];np.testing.assert_allclose(play.model.geom_rgba[play.model.geom(target).id],[.1,.2,.3,.75])
   frozen.manifest['scene_sha256']='different-source-revision'
   with self.assertRaisesRegex(ValueError,'source revision differs'):play.apply(reader.meta['case_id'],frozen,q,stamp)

 def test_shared_edit_replays_same_poses_and_skipped_appearance_without_physics(self):
  from robofrontier.motion_edit import build_timeline
  from scripts.replay_motion_in_factory import replay
  from scripts.export_factory_recorded_pose import export
  from robofrontier.robot_control import quat_matrix
  scene=ROOT/'outputs/robo-factory/v6-workstations/scene.xml'
  if not scene.exists():self.skipTest('Generated factory not present')
  source=Workstation(directory(next(c for c in cases() if c['id']=='dial-n04-d3')))
  with tempfile.TemporaryDirectory() as td,patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')):
   root=Path(td);writer=MotionWriter(root/'motion',source.path,source.model,source.data,hz=60)
   robot=source.model.joint('A_joint1').qposadr[0];gid=source.model.geom('score_0_0').id
   for i in range(1,121):
    source.data.time=i/60;source.data.qpos[robot]=source.initial_qpos[robot]+i*.0001
    if i==30:source.model.geom_rgba[gid]=[.1,.2,.3,.75]
    writer.capture(source.data)
   writer.close();reader=MotionReader(root/'motion');frames=list(reader.frames())
   self.assertEqual(len(frames),121)
   edit=build_timeline(reader,[t for t,_ in frames],[0,30,60,90,120],[True,False,True,False,True])
   path=root/'edit.json';path.write_text(json.dumps(edit))
   report=replay(root/'motion',scene,root/'checked',edit_timeline=path)
   self.assertTrue(report['passed']);self.assertEqual(report['states_checked'],3)
   self.assertEqual(report['replayed_source_timestamps_s'],[0.,1.,2.])
   pose=export(root/'motion',scene,root/'pose.json',edit_timeline=path,output_frame=1)
   self.assertEqual(pose['selected_source_frame'],60);self.assertEqual(pose['selected_recorded_time_s'],1.)
   display=next(g for g in pose['geoms'] if g['name'].endswith('__score_0_0'))
   np.testing.assert_allclose(display['rgba'],[.1,.2,.3,.75],atol=1e-7)
   frozen=Workstation(reader.path/'source');q=frames[60][1]
   baseline=frozen.frame(q);shift=np.array([4.,-3.,.2]);yaw=.7
   rotated=frozen.frame(q,1.,shift,yaw);rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
   for name,original in baseline['geometries'].items():
    np.testing.assert_allclose(rotated['geometries'][name]['position_m'],rotation@original['position_m']+shift,atol=1e-10)
    np.testing.assert_allclose(quat_matrix(rotated['geometries'][name]['quaternion_wxyz']),rotation@quat_matrix(original['quaternion_wxyz']),atol=1e-10)

if __name__=='__main__':unittest.main()
