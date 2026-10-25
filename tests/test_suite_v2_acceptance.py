import json,unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace
import mujoco,numpy as np
from robofrontier.suite.v2 import cases,directory
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.evaluate_v2 import placement_candidates
from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.workstation import Workstation

class V2AcceptanceTests(unittest.TestCase):
 def setup_world(self,case_id):
  source=Workstation(directory(next(c for c in cases() if c['id']==case_id)));source.frame();task=json.loads((source.path/'task.json').read_text())
  world=SimpleNamespace(model=source.model,data=source.data,task=task,arms={a:PandaArm(source.model,source.data,a) for a in task['arms']})
  return world,Monitor(world,secret=[1]*len(task['dials']) if task['family']=='dial' else None)
 def test_long_blade_acceptance_uses_actual_socket_geometry(self):
  world,mon=self.setup_world('transport-a04-d2');g=world.task['goals'][0];obj=next(o for o in mon.objects if o['identity']==g['requires']['identity']);j=mon.names.index(obj['id']);board=world.data.body(g['board']);rotation=board.xmat.reshape(3,3).copy()
  joint=world.data.joint(obj['id']+'_free');joint.qpos[:]=np.r_[g['position_m'],matrix_quat(rotation)];mujoco.mj_forward(world.model,world.data)
  stable=[False]*len(mon.names);stable[j]=True
  def eligible():return placement_candidates(mon,np.array([world.data.body(n).xpos for n in mon.names]),stable)[0,j]
  self.assertTrue(eligible());joint.qpos[:3]+=rotation[:,0]*.002;mujoco.mj_forward(world.model,world.data);self.assertFalse(eligible())
 def test_pocket_accepts_off_center_seating_with_complete_footprint(self):
  world,mon=self.setup_world('mosaic-a04-6x12-d1');g=world.task['goals'][0];obj=next(o for o in mon.objects if o['color']==g['requires']['color']);j=mon.names.index(obj['id']);joint=world.data.joint(obj['id']+'_free');joint.qpos[:]=np.r_[np.asarray(g['position_m'])+[.008,0,0],[1,0,0,0]];mujoco.mj_forward(world.model,world.data)
  stable=[False]*len(mon.names);stable[j]=True
  def eligible():return placement_candidates(mon,np.array([world.data.body(n).xpos for n in mon.names]),stable)[0,j]
  self.assertTrue(eligible());joint.qpos[0]+=.005;mujoco.mj_forward(world.model,world.data);self.assertFalse(eligible())
 def test_multiple_payloads_need_independent_completion(self):
  world,mon=self.setup_world('relay-a08-d3');mon.physical_step();result=mon.result()
  self.assertEqual(result['total_payloads'],2);self.assertEqual(result['completed_payloads'],0);self.assertFalse(result['task_success'])
 def test_flexible_handover_requires_overlap_and_tracks_drop_recovery(self):
  world,mon=self.setup_world('relay-a08-d1');relay=mon.relay_v2;ident=next(iter(relay.states));relay.states={ident:relay.states[ident]}
  support={'A','B'}
  with patch.object(mon,'bilateral',side_effect=lambda *a,**k:set(support)),patch.object(mon,'bottom',return_value=world.task['table_height_m']+.2),patch.object(relay,'inside',return_value=False),patch.object(relay,'environment_contact',return_value=False):
   world.data.time=.1;relay.sample();support.update(('C','D'));world.data.time=.2;relay.sample();world.data.time=.5;relay.sample();support={'C','D'};world.data.time=.6;relay.sample()
  state=relay.states[ident];self.assertEqual(state['custody'],{'C','D'});self.assertEqual(len(state['transitions']),1);self.assertFalse(mon.violations)
  support={'E','F'}
  with patch.object(mon,'bilateral',return_value=support),patch.object(mon,'bottom',return_value=world.task['table_height_m']+.2),patch.object(relay,'inside',return_value=False),patch.object(relay,'environment_contact',return_value=True):
   world.data.time=.7;relay.sample()
  self.assertTrue(any(v['kind']=='receiver_recovered_drop' for v in mon.violations))

if __name__=='__main__':unittest.main()
