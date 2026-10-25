"""Independent timestep convergence/cost probe, not a task Oracle.

Source task scenes/profile are never edited. Two fixtures share geometry,
material, mass and initial state across timesteps. Contact fixture adds a peg.
"""
from copy import deepcopy
import json,math,time
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from robofrontier.suite.cable import PROFILE,install
from robofrontier.physics_health import require_valid_step
from robofrontier.run_setup import write_json


def trial(dt,peg=False,seconds=8.):
 root=ET.Element('mujoco');option=ET.SubElement(root,'option',timestep=str(dt),integrator='implicitfast',solver='Newton',iterations='100',noslip_iterations='10');ET.SubElement(option,'flag',autoreset='disable')
 world=ET.SubElement(root,'worldbody');ET.SubElement(root,'equality');contact=ET.SubElement(root,'contact')
 anchor=np.array([0.,0.,.6]);body=ET.SubElement(world,'body',name='anchor',pos='0 0 .6');direction=np.array([math.sin(.25),0,-math.cos(.25)])
 profile=deepcopy(PROFILE);profile['timestep_s']=dt;n=profile['segments'];points=anchor+np.linspace(0,profile['length_m'],n+1)[:,None]*direction
 center=points[-1]-[0,0,.025];weight=ET.SubElement(world,'body',name='weight',pos=' '.join(map(str,center)));ET.SubElement(weight,'freejoint',name='weight_free');ET.SubElement(weight,'geom',name='weight_geom',type='cylinder',size='.025 .025',mass='.25');ET.SubElement(weight,'site',name='weight_top',pos='0 0 .025')
 tip=install(root,body,anchor,np.eye(3),points,'weight_top','cord',profile);ET.SubElement(contact,'exclude',body1=tip,body2='weight')
 if peg:ET.SubElement(world,'geom',name='peg',type='cylinder',pos='0 0 .5',size='.015 .04',quat=f'{math.sqrt(.5)} {math.sqrt(.5)} 0 0',friction='.6 .003 .0001')
 model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 times=[];poses=[];max_residual=0.;max_penetration=0.;first_contact=None;error=None;sample_every=round(.01/dt);started=time.perf_counter()
 try:
  for i in range(round(seconds/dt)):
   before=float(data.time);mujoco.mj_step(model,data);require_valid_step(data,before)
   if peg:
    for c in data.contact:
     names=[model.geom(int(g)).name for g in c.geom]
     if 'peg' in names and any(name.startswith('cord_geom') for name in names):
      if first_contact is None:first_contact=float(data.time)
      max_penetration=max(max_penetration,float(-c.dist))
   if (i+1)%sample_every==0:
    mujoco.mj_kinematics(model,data);max_residual=max(max_residual,float(np.linalg.norm(data.site('cord_end').xpos-data.site('weight_top').xpos)));times.append(float(data.time));poses.append(data.body('weight').xpos.copy().tolist())
 except RuntimeError as failure:error=str(failure)
 elapsed=time.perf_counter()-started
 return dict(timestep_ms=dt*1000,requested_simulation_s=seconds,completed_simulation_s=float(data.time),physics_steps=round(data.time/dt),wall_s=elapsed,stable=error is None,error=error,maximum_endpoint_error_mm=max_residual*1000,maximum_rope_peg_penetration_mm=max_penetration*1000,first_peg_contact_s=first_contact,sample_times_s=times,weight_positions_m=poses)


if __name__=='__main__':
 output=Path('docs/generated/frontier-v2/mechanism-checks/timestep-comparison');output.mkdir(parents=True,exist_ok=True);results={}
 for name,peg in [('free_swing',False),('peg_contact',True)]:
  rows=[]
  for dt in (.0005,.0002,.0001):
   repeats=[trial(dt,peg) for _ in range(3)];row=repeats[0];row['median_wall_s']=float(np.median([r['wall_s'] for r in repeats]));row['all_repeats_stable']=all(r['stable'] for r in repeats);rows.append(row)
   print(json.dumps({k:v for k,v in row.items() if k not in ('sample_times_s','weight_positions_m')}),flush=True)
  reference=rows[-1];ref=np.asarray(reference['weight_positions_m'])
  for row in rows:
   if row['stable'] and reference['stable'] and len(row['weight_positions_m'])==len(ref):
    delta=np.linalg.norm(np.asarray(row['weight_positions_m'])-ref,axis=1);row['rms_position_difference_to_0p1ms_mm']=float(np.sqrt(np.mean(delta**2))*1000);row['maximum_position_difference_to_0p1ms_mm']=float(max(delta)*1000)
   row['median_wall_ratio_to_0p5ms']=row['median_wall_s']/rows[0]['median_wall_s']
  results[name]=rows
 report=dict(scope='Two isolated passive cable fixtures, 8 seconds each, 3 repeats per timestep; no robot task, no formal parameter changes. Trajectory samples aligned every10ms. Small-fixture timings do not predict full-scene costs.',profile=PROFILE,fixtures=results,formal_parameters_changed=False,oracle_started=False)
 write_json(output/'report.json',report);print(output/'report.json',flush=True)
