"""Recorded-state progress movie: all actuator intervals, low-motion waits compressed."""
import argparse,json,math,subprocess,time
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from robofrontier.motion_recording import MotionReader

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--speed',type=float,default=4.);p.add_argument('--every',type=int,default=16);p.add_argument('--user-goal-complete',action='store_true');p.add_argument('--completion-sim-s',type=float);p.add_argument('--encoder',choices=['libx264','h264_videotoolbox'],default='libx264');p.add_argument('--ignore-idle-robot-jitter',action='store_true');p.add_argument('--no-shadows',action='store_true');p.add_argument('--no-reflections',action='store_true');p.add_argument('--idle-threshold-scale',type=float,default=1.);a=p.parse_args();run=a.run.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);task=json.loads((run/'task.json').read_text());reader=MotionReader(run/'motion');end=reader.meta['chunks'][-1]['end_sim_s'];intervals=[];start=None
 terminal=None
 if a.user_goal_complete:
  user_goal_report=json.loads((run/'user-goal-report.json').read_text())
  assert user_goal_report['passed']
  tolerance_mm=1000*user_goal_report['boundary_tolerance_m']
  if a.completion_sim_s is not None:
   assert np.isfinite(a.completion_sim_s) and 0<a.completion_sim_s<=end
   verified=json.loads((run/'RECORDING-VERIFIED-THROUGH-USER-GOAL.json').read_text())
   assert verified['passed'] and abs(verified['completion_cutoff_s']-a.completion_sim_s)<1e-7
   assert verified['case_id']==task['id'] and verified['source_scene_sha256']==reader.meta['source_scene_sha256'] and verified['world_id']==user_goal_report['world_id']
   end=a.completion_sim_s
  else:
   assert reader.meta['status']!='recording' and (run/'report.json').exists()
   terminal_time=json.loads((run/'report.json').read_text())['evaluation']['sim_time_s']
   with np.load(run/'event-states.npz',allow_pickle=False) as final_events:terminal=(terminal_time,final_events['qpos'][-1].copy())
   interval=math.ceil(1./reader.meta['sample_hz']/reader.meta['physics_timestep_s'])*reader.meta['physics_timestep_s']
   assert end-1e-7<=terminal_time<=end+interval+1e-7
   end=terminal_time
 elif a.completion_sim_s is not None:raise ValueError('A completion cutoff requires a verified user-goal result')
 assert a.idle_threshold_scale>0 and a.speed>0 and a.every>0
 for raw in (run/'trace.jsonl').open():
  try:v=json.loads(raw)
  except json.JSONDecodeError:continue
  t=v.get('sim_time_s',0.)
  if t>end:break
  if v.get('event')=='action_start':start=t
  elif v.get('event')=='action_end' and start is not None:intervals.append((max(0,start-.4),min(end,t+.8)));start=None
 if start is not None:intervals.append((max(0,start-.4),end))
 intervals.append((0.,min(end,1.)))
 intervals.sort();merged=[]
 for lo,hi in intervals:
  if merged and lo<=merged[-1][1]:merged[-1]=(merged[-1][0],max(merged[-1][1],hi))
  else:merged.append((lo,hi))
 model=mujoco.MjModel.from_xml_path(str(run/'scene.xml'));data=mujoco.MjData(model);model.geom_rgba[:]=reader.meta['initial_rgba'];events=iter(reader.events());event=next(events,None);registry=json.loads((reader.path/'source/workstation.json').read_text());geomids={g['id']:g['source_index'] for g in registry['geometries']};font=ImageFont.truetype('/System/Library/Fonts/STHeiti Medium.ttc',22)
 hz=reader.meta['sample_hz'];fps=a.speed*hz/a.every;dest=out/'global.mp4';codec=['-c:v','libx264','-threads','1','-crf','22','-preset','fast'] if a.encoder=='libx264' else ['-c:v','h264_videotoolbox','-threads','1','-b:v','5M','-allow_sw','0'];encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-filter_threads','1','-filter_complex_threads','1','-f','rawvideo','-pix_fmt','rgb24','-s','1280x800','-r',str(fps),'-i','-','-an',*codec,'-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],stdin=subprocess.PIPE)
 spans=[];previous=None;last_emitted=None;idx=0;rendered=0;read=0;skipped=0;cuts=[];last=None;started=time.monotonic();free=[];hinges=[];slides=[]
 for j in range(model.njnt):
  q=int(model.jnt_qposadr[j]);kind=int(model.jnt_type[j])
  if kind==mujoco.mjtJoint.mjJNT_FREE:free.append(q)
  elif kind==mujoco.mjtJoint.mjJNT_SLIDE:slides.append(q)
  else:hinges.append(q)
 if a.ignore_idle_robot_jitter:
  robot_qids={int(model.jnt_qposadr[j]) for j in range(model.njnt) if (model.joint(j).name or '').split('_',1)[0] in task['arms']}
  hinges[:]=[q for q in hinges if q not in robot_qids];slides[:]=[q for q in slides if q not in robot_qids]
 def moving(t,q):
  if previous is None:return True
  stamp,old=previous;dt=t-stamp
  if dt<=0:return False
  if hinges and np.max(abs(q[hinges]-old[hinges]))/dt>.01*a.idle_threshold_scale:return True
  if slides and np.max(abs(q[slides]-old[slides]))/dt>.001*a.idle_threshold_scale:return True
  for k in free:
   if np.linalg.norm(q[k:k+3]-old[k:k+3])/dt>.002*a.idle_threshold_scale:return True
   first=q[k+3:k+7];second=old[k+3:k+7];cos=abs(float(np.dot(first,second)))/(np.linalg.norm(first)*np.linalg.norm(second));angle=2*math.acos(float(np.clip(cos,-1,1)))
   if angle/dt>.05*a.idle_threshold_scale:return True
  return False
 try:
  with mujoco.Renderer(model,width=1280,height=800,max_geom=max(5000,model.ngeom+1000)) as renderer:
   def emit(stamp,q,note=''):
    nonlocal rendered,last_emitted
    title='原进程全程 · 按用户白框标准完成' if a.user_goal_complete else '新版进度回放 · 尚未整项通过'
    if a.completion_sim_s is not None:title='原任务全程 · 验收完成（原进程继续运行）'
    subtitle=f'保留中间失误和最终修正；松爪、白框边界容差{tolerance_mm:g}mm；旧日志保留' if a.user_goal_complete else '保留全部指令执行段和明显被动物体运动；不是成功视频或新物理模拟'
    data.qpos[:]=q;data.time=stamp;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data);renderer.update_scene(data,camera='global')
    if a.no_shadows:renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
    if a.no_reflections:renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
    im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((10,10,1268,105),fill='#f3f5f7');draw.text((23,17),f'{task["id"]} · {title}',font=font,fill='#1d3449');draw.text((23,46),f'实测记录 FK | sim {stamp:.2f}s / 截止 {end:.2f}s | 动作 {a.speed:g}× | 低运动等待压缩',font=font,fill='#35516a');draw.text((23,75),note or subtitle,font=font,fill='#607284');encoder.stdin.write(im.tobytes());rendered+=1;last_emitted=stamp
    if rendered==1:im.save(out/'first.png')
    if abs(stamp-end)<.04:im.save(out/'last.png')
    if rendered%1000==0:print(task['id'],'rendered',rendered,'frames; source sim',round(stamp,2),'/',round(end,2),flush=True)
   for number,(stamp,q) in enumerate(reader.frames()):
    if stamp>end+1e-8:break
    read+=1;last=(stamp,q)
    while event is not None and event['sim_time_s']<=stamp+1e-9:
     if event['event']=='rgba':
      for name,value in event['geoms'].items():model.geom_rgba[geomids[name]]=value
     event=next(events,None)
    if number%a.every:continue
    while idx<len(merged) and merged[idx][1]<stamp:idx+=1
    active=idx<len(merged) and merged[idx][0]<=stamp<=merged[idx][1];motion=moving(stamp,q);previous=(stamp,q.copy())
    if active or motion or last_emitted is None:
     note=''
     if last_emitted is not None and stamp-last_emitted>2*a.every/hz:
      note=f'低运动等待压缩：sim {last_emitted:.2f} → {stamp:.2f}s';cuts.append(dict(first_s=last_emitted,last_s=stamp))
     emit(stamp,q,note)
    else:skipped+=1
   if last is not None and (last_emitted is None or last[0]>last_emitted+1e-8):emit(last[0],last[1],'最终记录帧；按用户白框标准完成' if a.user_goal_complete else '截至本次完整采样块；试跑继续')
   if terminal is not None and terminal[0]>last_emitted+1e-8:emit(*terminal,'原进程结束事件的精确最终状态；按用户白框标准完成')
 finally:encoder.stdin.close();code=encoder.wait()
 if code:raise RuntimeError('Progress video encoding failed')
 check=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(dest),'-filter_threads','1','-f','null','-'],capture_output=True,text=True);assert check.returncode==0,check.stderr
 scope='Full closed original physical episode from canonical start to final recorded frame; user-final-in-Goal acceptance, legacy strict native result retained.' if a.user_goal_complete else 'Full recorded progress from canonical start to declared snapshot cutoff; not a full-task success movie.'
 if a.completion_sim_s is not None:scope='Full original physical task record from canonical start through verified user-goal completion cutoff. Original physics process continues; this is not a closed episode. Final pose is an authentic recording sample, not a fabricated terminal event.'
 manifest=dict(case_id=task['id'],source=str(run),source_scene_sha256=reader.meta['source_scene_sha256'],source_stream_snapshot=reader.meta,source_frames_read=read,end_sim_s=end,last_source_sample_s=None if last is None else last[0],completion_cutoff_s=a.completion_sim_s,encoder=a.encoder,video=str(dest),rendered_frames=rendered,fps=fps,speed=a.speed,idle_threshold_scale=a.idle_threshold_scale,low_motion_samples_omitted=skipped,cuts=cuts,retained_all_actuator_intervals=True,user_goal_complete=a.user_goal_complete,scope=scope+' Low-motion waiting compressed. FK rendering only, no new integration.',decode_passed=True,wall_s=time.monotonic()-started)
 manifest['presentation_options']=dict(ignore_idle_robot_jitter=a.ignore_idle_robot_jitter,no_shadows=a.no_shadows,no_reflections=a.no_reflections)
 (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2));print(json.dumps({k:manifest[k] for k in ['case_id','video','end_sim_s','rendered_frames','wall_s']},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
