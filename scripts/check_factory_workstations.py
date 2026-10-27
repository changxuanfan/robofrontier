"""Check all-source FK identity and make static views; never step task physics."""
import argparse,json,math,time
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.factory_playback import current_scene
from robofrontier.workstation import Workstation
from robofrontier.run_setup import write_json

def check(scene,output,render=False):
 output=Path(output);output.mkdir(parents=True,exist_ok=True);play=FactoryPlayback(scene);rows=[]
 for place in play.manifest['placements']:
  source=Workstation(place['source_directory']);frame=play.apply(place['case_id'],source,source.initial_qpos,0);names=play.manifest['workstation_names'][place['case_id']]['geom_names'];position=0.;rotation=0.
  for name,target in names.items():
   gid=play.model.geom(target).id;pose=frame['geometries'][name];position=max(position,float(np.linalg.norm(play.data.geom_xpos[gid]-pose['position_m'])))
   actual=np.empty(4);mujoco.mju_mat2Quat(actual,play.data.geom_xmat[gid]);rotation=max(rotation,1-abs(float(actual@pose['quaternion_wxyz'])))
  rows.append(dict(case_id=place['case_id'],geometries=len(names),maximum_position_error_m=position,maximum_quaternion_dot_error=rotation,passed=position<1e-6 and rotation<1e-9))
  print(place['case_id'],position,flush=True)
 report=dict(scope='All36 static source-to-factory poses, no physics or Oracle',passed=all(r['passed'] for r in rows),workstations=len(rows),physical_dofs=play.model.nv,actuators=play.model.nu,geometries=play.model.ngeom,rows=rows)
 write_json(output/'geometry-check.json',report)
 if render:
  del play
  from robofrontier.factory_view import FactoryRenderer
  r=FactoryRenderer(scene,width=1920,height=1200)
  try:
   for name in ('top','from_south','interior_diagonal'):
    r.set_camera(next(c for c in r.manifest['cameras'] if c['id']==name));Image.fromarray(r.render()).save(output/(name+'.png'));print('rendered',name,flush=True)
   for region in r.manifest['regions']:
    x0,y0,x1,y1=region['bounds_xy_m'];cx,cy=(x0+x1)/2,(y0+y1)/2
    r.set_camera(dict(position_m=[cx,cy,24],target_m=[cx,cy,0],projection='orthographic',orthographic_height_m=27,hidden_layers=['roof','north','south','west','east']))
    Image.fromarray(r.render()).save(output/(region['id']+'-top.png'));print('rendered',region['id'],flush=True)
  finally:r.close()
 if not report['passed']:raise SystemExit('Factory pose mismatch')
 return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,default=current_scene());p.add_argument('--output',type=Path,required=True);p.add_argument('--render',action='store_true');a=p.parse_args();check(a.scene,a.output,a.render)
