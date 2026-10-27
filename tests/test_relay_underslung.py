"""Contracts for the explicitly requested r16 apparatus, independent of Oracle success."""
import math,unittest
import numpy as np
from robofrontier.suite.relay_underslung import design, attachment_point, transverse_axis
from scripts.verify_floating_dial_oracle import axis_rotation

class UnderslungDesignTests(unittest.TestCase):
 def test_arm_specific_span_and_reduced_bending(self):
  for arms in [8,12,16]:
   for level in [2,3]:
    m=design(arms,level);self.assertAlmostEqual(m['chord_m'],1.2 if arms==16 else 1.15)
    if level==2:self.assertAlmostEqual(m['circular_angle_deg'],63.)
    else:self.assertAlmostEqual(m['interior_angle_deg'],148.5)
 def test_mount_is_one_hundred_mm_along_the_actual_beam_from_tip(self):
  for arms in [8,12,16]:
   for level in [2,3]:
    m=design(arms,level);point=attachment_point(m,level)
    self.assertAlmostEqual(point[2],-.0225)
    if level==2:
     angle=math.asin(point[1]/m['radius_m']);distance=m['radius_m']*(math.radians(m['circular_angle_deg']/2)-angle)
    else:distance=np.linalg.norm(point[:2]-[0,m['chord_m']/2])
    self.assertAlmostEqual(distance,.1)
 def test_transverse_hinge_preserves_horizontal_projection_angle(self):
  m=design(8,3);n=transverse_axis(m);h=m['chord_m']/2;left=np.array([-m['bend_rise_m'],-h,0]);right=np.array([-m['bend_rise_m'],h,0])
  self.assertAlmostEqual(np.dot(n,right),0.)
  for degrees in [-25,0,25]:
   vector=axis_rotation(n,math.radians(degrees))@right
   angle=math.degrees(math.acos(np.dot(left[:2],vector[:2])/np.linalg.norm(left[:2])/np.linalg.norm(vector[:2])))
   self.assertAlmostEqual(angle,148.5)
   if degrees:self.assertGreater(abs(vector[2]),.2)


class UnderslungBuiltSceneTests(unittest.TestCase):
 def scene(self,arms,level):
  import json,mujoco
  from robofrontier.workstation import Workstation
  from robofrontier.suite.v2 import cases,directory
  c=next(c for c in cases() if c['id']==f'relay-a{arms:02d}-d{level}')
  c=dict(c,scene_revision=16) # Manufactured geometry contract predates the r17 release clarification.
  self.assertEqual(c['scene_revision'],16);w=Workstation(directory(c));mujoco.mj_forward(w.model,w.data)
  return w,json.loads((w.path/'task.json').read_text())
 def test_all_six_are_contact_linked_and_preserve_robots_and_completion(self):
  import mujoco,json
  from robofrontier.workstation import Workstation
  from robofrontier.suite.v2 import cases,directory
  from robofrontier.suite.relay_underslung import linking_number
  for arms in [8,12,16]:
   for level in [2,3]:
    with self.subTest(arms=arms,level=level):
     w,t=self.scene(arms,level);m,d=w.model,w.data;oldcase=next(c for c in cases() if c['id']==t['id']);old=Workstation(directory(dict(oldcase,scene_revision=15 if arms==12 else 13)));baseline=json.loads((old.path/'task.json').read_text())
     self.assertEqual(t['arms'],baseline['arms']);self.assertEqual(t['completion'],baseline['completion']);self.assertEqual(t['route'],baseline['route']);self.assertEqual(m.nu,8*arms)
     self.assertTrue(any('After pickup keep the whole transported assembly clear of table/floor until final landing.' in rule for rule in t['rules']))
     for field in ['actuator_gainprm','actuator_biasprm','jnt_range','dof_damping']:np.testing.assert_array_equal(getattr(m,field),getattr(old.model,field))
     self.assertFalse(any('hook_bracket' in (m.geom(g).name or '') for g in range(m.ngeom)))
     for a in t['mechanism']['payload_assemblies']:
      i=a['id'];self.assertAlmostEqual(abs(linking_number(m,d,i)),1,delta=.002)
      shaft=m.geom(i+'_weight_shaft');self.assertEqual(int(shaft.type[0]),mujoco.mjtGeom.mjGEOM_CYLINDER);self.assertAlmostEqual(float(shaft.size[1])*2,.17)
      self.assertEqual(a['attachment']['face'],'underside');self.assertEqual(a['attachment']['bracket_stem_m'],0.)
      self.assertEqual(int(m.joint(i+'_weight_free').type[0]),mujoco.mjtJoint.mjJNT_FREE)
 def test_real_mujoco_hinge_axis_and_projection_match_the_manufactured_bearing(self):
  import mujoco
  from robofrontier.robot_control import matrix_quat
  for arms in [8,12,16]:
   w,t=self.scene(arms,3);m,d=w.model,w.data;h=t['mechanism']['chord_m']/2;rise=t['mechanism']['bend_rise_m']
   for a in t['mechanism']['payload_assemblies']:
    ident=a['id'];joint=m.joint(ident+'_flexion');axis=transverse_axis(t['mechanism']);np.testing.assert_allclose(m.jnt_axis[joint.id],axis)
    d.joint(ident+'_free').qpos[3:]=matrix_quat(np.eye(3))
    for degrees in [-25,0,25]:
     d.joint(joint.name).qpos[0]=math.radians(degrees);mujoco.mj_forward(m,d);root=d.body(ident);child=d.body(ident+'_distal');pivot=root.xpos+root.xmat.reshape(3,3)@np.array([rise,0,.014]);left=root.xpos+root.xmat.reshape(3,3)@np.array([0,-h,0])-pivot;right=child.xpos+child.xmat.reshape(3,3)@np.array([-rise,h,-.014])-pivot
     angle=math.degrees(math.acos(np.clip(np.dot(left[:2],right[:2])/np.linalg.norm(left[:2])/np.linalg.norm(right[:2]),-1,1)));self.assertAlmostEqual(angle,148.5,places=8)
     housing=m.geom(ident+'_hinge_housing').id;np.testing.assert_allclose(d.geom_xmat[housing].reshape(3,3)[:,2],d.xaxis[joint.id],atol=1e-12)
 def test_current_grasp_geometry_follows_new_arc_and_refuses_outside_endpoints(self):
  from types import SimpleNamespace
  from robofrontier.suite import relay_oracle_v2 as ref
  w,t=self.scene(8,2);trial=SimpleNamespace(task=t,data=w.data);r=t['mechanism']['radius_m']
  self.assertAlmostEqual(ref.cross_x(t,0),r*(1-math.cos(math.radians(31.5))))
  with self.assertRaisesRegex(ValueError,'inside the timber'):ref.grasp_targets(trial,'payload_01',np.zeros(3),np.eye(3),.58)

if __name__=='__main__':unittest.main()
