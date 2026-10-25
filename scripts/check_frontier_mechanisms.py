"""Independent passive fixtures only; no manipulation or full-task Oracle."""
import json,math,time
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from robofrontier.suite.cable import PROFILE,install
from robofrontier.run_setup import write_json

def cable_fixture(segments,dt,seconds=4):
 root=ET.Element('mujoco');ET.SubElement(root,'option',timestep=str(dt),integrator='implicitfast',solver='Newton',iterations='100',noslip_iterations='10')
 world=ET.SubElement(root,'worldbody');ET.SubElement(root,'equality');ET.SubElement(root,'contact');anchor=np.array([0,0,.6]);body=ET.SubElement(world,'body',name='anchor',pos='0 0 .6')
 direction=np.array([math.sin(.1),0,-math.cos(.1)]);points=anchor+np.linspace(0,.2,segments+1)[:,None]*direction
 center=points[-1]-[0,0,.025];weight=ET.SubElement(world,'body',name='weight',pos=' '.join(map(str,center)));ET.SubElement(weight,'freejoint',name='weight_free')
 ET.SubElement(weight,'geom',name='weight_geom',type='cylinder',size='.025 .025',mass='.25');ET.SubElement(weight,'site',name='weight_top',pos='0 0 .025')
 profile=deepcopy(PROFILE);profile['segments']=segments
 tip=install(root,body,anchor,np.eye(3),points,'weight_top','cord',profile);ET.SubElement(root.find('contact'),'exclude',body1=tip,body2='weight')
 model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 max_residual=0.;positions=[];times=[];started=time.monotonic()
 for i in range(round(seconds/dt)):
  before=float(data.time);mujoco.mj_step(model,data)
  if data.time<=before:raise RuntimeError('Numerical failure reset fixture time')
  if i%max(1,round(.01/dt))==0:
   mujoco.mj_kinematics(model,data);positions.append(float(data.body('weight').xpos[0]));times.append(float(data.time));max_residual=max(max_residual,float(np.linalg.norm(data.site('cord_end').xpos-data.site('weight_top').xpos)))
  if not np.isfinite(data.qpos).all():raise ValueError('Nonfinite passive cable fixture')
 crossings=[times[i] for i in range(1,len(times)) if positions[i-1]>=0 and positions[i]<0]
 period=float(np.mean(np.diff(crossings))) if len(crossings)>1 else None
 return dict(segments=segments,timestep_s=dt,simulation_s=seconds,wall_s=time.monotonic()-started,total_cord_mass_kg=sum(float(model.body(i).mass[0]) for i in range(model.nbody) if model.body(i).name.startswith('cord_link')),period_s=period,point_pendulum_reference_s=2*math.pi*math.sqrt(.225/9.81),maximum_end_constraint_error_m=max_residual,finite=bool(np.isfinite(data.qpos).all()),passed=bool(period and .7<period<1.2 and max_residual<.003))

def spring_fixture():
 from robofrontier.suite.v2 import cases,directory
 from robofrontier.robot_control import PandaArm,matrix_quat
 folder=directory(next(c for c in cases() if c['id']=='transport-a04-d3'));task=json.loads((folder/'task.json').read_text());model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
 # Declared fixture initialization, not an executed task solution.
 goals=[g for g in task['goals'] if g['board']=='board_A'];rotation=data.body('board_A').xmat.reshape(3,3).copy()
 for k,g in enumerate(goals):data.joint(f'A_item{k+1}_free').qpos[:]=np.r_[g['position_m'],matrix_quat(rotation)]
 mujoco.mj_forward(model,data);arms={r:PandaArm(model,data,r) for r in task['arms']}
 for i in range(round(3/model.opt.timestep)):
  if i%round(.02/model.opt.timestep)==0:
   for arm in arms.values():arm.control(.02)
  mujoco.mj_step(model,data)
 sag=float(data.joint('board_A_z').qpos[0]);expected=(float(model.body('board_A').mass[0])+3*float(model.body('A_item1').mass[0]))*9.81/1400
 return dict(scope='Three initialized passive loads on actual spring board; no robot task commands',sag_m=sag,linear_static_estimate_m=expected,passed=bool(abs(sag-expected)<.003 and np.isfinite(data.qpos).all()))

def dial_fixture():
 from robofrontier.suite.v2 import cases,directory
 folder=directory(next(c for c in cases() if c['id']=='dial-n04-d3'));xml=ET.parse(folder/'scene.xml').getroot();original=xml.find("worldbody/body[@name='D01_module']")
 body=deepcopy(original);body.remove(body.find('freejoint'));body.set('pos','0 0 .6')
 # Fixed support, with disk vertical: test spring return under gravity, not a
 # robot grasp or complete airborne puzzle. Physics/rotor geometry unchanged.
 body.set('quat',f'{math.sqrt(.5)} {math.sqrt(.5)} 0 0')
 world=xml.find('worldbody');world.clear();world.append(body)
 for section in ('actuator','tendon','equality','contact','keyframe'):
  value=xml.find(section)
  if value is not None:xml.remove(value)
 compiler=xml.find('compiler');compiler.set('meshdir',str((folder/compiler.get('meshdir','')).resolve()))
 for e in xml.findall('asset/*[@file]'):
  if e.tag!='mesh':e.set('file',str((folder/e.get('file')).resolve()))
 rows=[]
 for degrees in (90,180,-90):
  model=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));data=mujoco.MjData(model);data.joint('D01_hinge').qpos[0]=math.radians(degrees)
  for _ in range(round(8/model.opt.timestep)):mujoco.mj_step(model,data)
  angle=math.degrees(float(data.joint('D01_hinge').qpos[0]));rows.append(dict(start_deg=degrees,end_deg=angle,passed=bool(abs(angle)<4 and np.isfinite(data.qpos).all())))
 return dict(scope='Isolated crank on fixed vertical support; no robot grasp or puzzle solution',trials=rows,passed=all(r['passed'] for r in rows))

if __name__=='__main__':
 out=Path('docs/generated/frontier-v2/mechanism-checks');out.mkdir(exist_ok=True)
 rows=[]
 for n,dt in ((8,.0005),(16,.0005),(16,.00025)):
  row=cable_fixture(n,dt);rows.append(row);print(json.dumps(row),flush=True)
 report=dict(scope='Passive mechanism engineering fixtures only; not a manipulation Oracle, real-material calibration or task success',profile=PROFILE,cables=rows,spring=spring_fixture(),dial=dial_fixture())
 report['passed']=all(r['passed'] for r in rows) and report['spring']['passed'] and report['dial']['passed']
 write_json(out/'report.json',report);print(json.dumps(report),flush=True)
