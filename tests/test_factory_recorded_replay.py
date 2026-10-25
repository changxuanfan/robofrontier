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

if __name__=='__main__':unittest.main()
