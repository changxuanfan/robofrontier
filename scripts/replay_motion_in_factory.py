"""Read stored motion into a factory. No commands, IK or physics are executed."""
import argparse,json
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from robofrontier.motion_recording import MotionReader
from robofrontier.workstation import Workstation
from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.robot_control import quat_matrix
from robofrontier.recording import Video
from robofrontier.run_setup import write_json

def video_sample(stamp,deadline,rate=30):
 if stamp+1e-9<deadline:return False,deadline
 while deadline<=stamp+1e-9:deadline+=1/rate
 return True,deadline

def replay(recording,factory,output,render=False):
 reader=MotionReader(recording);source=Workstation(reader.path/'source');output=Path(output);output.mkdir(parents=True,exist_ok=False)
 factory_renderer=None;source_renderer=None;videos={};play=None
 if render:
  from robofrontier.factory_view import FactoryRenderer
  factory_renderer=FactoryRenderer(factory,width=1280,height=800)
  play=FactoryPlayback(factory,model=factory_renderer.model,data=factory_renderer.data)
  place=next(p for p in play.manifest['placements'] if p['case_id']==reader.meta['case_id']);yaw=place['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
  camera=source.model.camera('global').id;position=source.model.cam_pos[camera];r=quat_matrix(source.model.cam_quat[camera]);shift=np.asarray(place['translation_m'])
  factory_renderer.set_camera(dict(position_m=(rot@position+shift).tolist(),target_m=(rot@(position-r[:,2])+shift).tolist(),projection='perspective',vertical_fov_deg=float(source.model.cam_fovy[camera]),hidden_layers=['roof']))
  source_renderer=mujoco.Renderer(source.model,width=1280,height=800,max_geom=max(5000,source.model.ngeom+100))
  videos={name:Video(output/(name+'.mp4'),1280,800,30) for name in ('simple','factory')}
 else:play=FactoryPlayback(factory)
 case=reader.meta['case_id'];names=play.manifest['workstation_names'][case]['geom_names'];initial=reader.meta['initial_rgba'];source.model.geom_rgba[:]=initial
 geometry_ids={g['id']:g['source_index'] for g in source.manifest['geometries']}
 play.appearance(case,{name:initial[index] for name,index in geometry_ids.items()})
 events=iter(reader.events());event=next(events,None);frames=0;maximum_error=0.;image_times=[];next_image=0.
 try:
  for stamp,qpos in reader.frames():
   while event is not None and event['sim_time_s']<=stamp+1e-9:
    if event['event']=='rgba':
     play.appearance(case,event['geoms'])
     for name,value in event['geoms'].items():
      try:source.model.geom_rgba[geometry_ids[name]]=value
      except KeyError:raise ValueError('Unknown recorded appearance geometry')
    event=next(events,None)
   expected=play.apply(case,source,qpos,stamp);frames+=1
   for name,target in names.items():maximum_error=max(maximum_error,float(np.linalg.norm(play.data.geom_xpos[play.model.geom(target).id]-expected['geometries'][name]['position_m'])))
   selected,next_image=video_sample(stamp,next_image)
   if render and selected:
    source_renderer.update_scene(source.data,camera='global');simple=source_renderer.render();industrial=factory_renderer.render()
    for name,pixels in [('simple',simple),('factory',industrial)]:
     if not videos[name].frame(pixels,block=True):raise RuntimeError('Offline encoder could not preserve every selected frame')
     if not image_times:Image.fromarray(pixels).save(output/(name+'-first.png'))
    image_times.append(stamp)
   if frames%60==0:print('checked states',frames,flush=True)
 finally:
  checks={name:video.close() for name,video in videos.items()}
  if source_renderer:source_renderer.close()
  if factory_renderer:factory_renderer.close()
 report=dict(scope='Recorded-state FK + rigid factory placement; no physics, commands, IK or resimulation',case_id=case,states_checked=frames,maximum_geometry_position_error_m=maximum_error,passed=maximum_error<1e-6,record_status=reader.meta['status'],videos=checks,video_source_timestamps_s=image_times)
 write_json(output/'report.json',report)
 if not report['passed'] or any(c['error'] for c in checks.values()):raise RuntimeError('Recorded-state replay verification failed')
 print(json.dumps(report),flush=True);return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('recording',type=Path);p.add_argument('--factory',type=Path,default=Path('outputs/robo-factory/v6-workstations/scene.xml'));p.add_argument('--output',type=Path,required=True);p.add_argument('--render',action='store_true');a=p.parse_args();replay(a.recording,a.factory,a.output,a.render)
