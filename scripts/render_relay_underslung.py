"""Render r16 initial environments and explicitly labelled FK inspection poses."""
import json,math
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from robofrontier.suite.v2 import ROOT,cases,directory
from robofrontier.workstation import Workstation
from robofrontier.robot_control import matrix_quat
from robofrontier.suite.scenes import rz
from robofrontier.suite.relay_underslung import render_views,linking_number

OUT=ROOT/'outputs/design/relay-r16-actual';FONT='/System/Library/Fonts/STHeiti Medium.ttc'
def labelled(image,title,subtitle):
 canvas=Image.new('RGB',(image.width,image.height+90),'#f4f6f9');canvas.paste(image,(0,90));draw=ImageDraw.Draw(canvas);draw.text((22,12),title,font=ImageFont.truetype(FONT,30),fill='#20374b');draw.text((22,53),subtitle,font=ImageFont.truetype(FONT,21),fill='#607284');return canvas

def main():
 OUT.mkdir(parents=True,exist_ok=True);selected=[dict(c,scene_revision=16) for c in cases() if c['family']=='relay' and c['level']>1];manifest=[]
 for c in selected:
  source=directory(c);target=OUT/c['id'];target.mkdir(exist_ok=True);w=Workstation(source);m,d=w.model,w.data;mujoco.mj_forward(m,d);task=json.loads((source/'task.json').read_text());ident='payload_01';assembly=task['mechanism']['payload_assemblies'][0];hook=np.array(assembly['attachment']['fixed_eye_center_local_m']);root=d.body(ident);R=root.xmat.reshape(3,3).copy();yaw=math.atan2(R[1,0],R[0,0]);m.vis.global_.offwidth=1600;m.vis.global_.offheight=1000
  with mujoco.Renderer(m,width=1600,height=1000,max_geom=max(5000,m.ngeom+1000)) as renderer:
   for camera in ['global','top','global_rear']:
    renderer.update_scene(d,camera=camera);im=labelled(Image.fromarray(renderer.render()),f"{c['arms']}臂 · L{c['level']} · 新版真实初态",f"跨度 {task['mechanism']['chord_m']:.2f}m | 无短柱 · 底面挂环 · 木吊杆170mm | MuJoCo静态渲染，未步进")
    im.save(target/(camera+'.png'))
   # Tabletop attachment closeup, actual initial state unchanged.
   camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.lookat[:]=root.xpos+R@(hook+np.array([-.075,0,-.005]));camera.azimuth=math.degrees(yaw)+90;camera.elevation=-6;camera.distance=.65
   renderer.update_scene(d,camera=camera);labelled(Image.fromarray(renderer.render()),f"L{c['level']} 桌面初态：下表面挂点近景",'来自实际初始关键帧；吊杆与砝码自然卧放，固定环和杆端接触桌面').save(target/'initial-attachment-close.png')
   # A separate FK inspection pose makes the hidden underside visible.
   data=mujoco.MjData(m);data.qpos[:]=w.initial_qpos;pose=data.joint(ident+'_free').qpos;pose[2]=1.35;pose[3:]=matrix_quat(rz(yaw))
   if c['level']==3:data.joint(ident+'_flexion').qpos[0]=0.
   eye=pose[:3]+rz(yaw)@(hook+np.array([-.014,0,-.018]));data.joint(ident+'_weight_free').qpos[:]=np.r_[eye,matrix_quat(rz(yaw))];mujoco.mj_kinematics(m,data);mujoco.mj_comPos(m,data);mujoco.mj_camlight(m,data)
   link=linking_number(m,data,ident);assert abs(abs(link)-1)<.03
   option=mujoco.MjvOption();option.sitegroup[:]=0;option.flags[mujoco.mjtVisFlag.mjVIS_TENDON]=0;option.flags[mujoco.mjtVisFlag.mjVIS_JOINT]=0
   for gid in range(m.ngeom):
    body=m.body(int(m.geom_bodyid[gid])).name or ''
    if not(body==ident or body.startswith(ident+'_')):m.geom_group[gid]=5
   option.geomgroup[5]=0
   camera.lookat[:]=(data.body(ident).xpos+rz(yaw)@hook+data.body(ident+'_weight').xpos+rz(yaw)@np.array([0,0,-.218]))/2
   camera.distance=.69;camera.azimuth=math.degrees(yaw)+90;camera.elevation=4
   renderer.update_scene(data,camera=camera,scene_option=option);labelled(Image.fromarray(renderer.render()),f"L{c['level']} 底面铁环 → 木吊杆 → 600g砝码",'悬空结构检查姿态：仅保存模型的FK摆放，不是机械臂执行记录').save(target/'hanging-attachment-close.png')
   camera.lookat[:]=data.body(ident).xpos+[0,0,-.10];camera.distance=1.75;camera.azimuth=math.degrees(yaw)+65;camera.elevation=-22
   renderer.update_scene(data,camera=camera,scene_option=option);labelled(Image.fromarray(renderer.render()),f"L{c['level']} 完整装置结构检查",'仅FK摆放，无物理步进；实际环境初态见全景与俯视图').save(target/'hanging-assembly.png')
   if c['level']==3:
    for gid in range(m.ngeom):
     if m.body(int(m.geom_bodyid[gid])).name==ident+'_weight':m.geom_group[gid]=5
    pivot=data.body(ident+'_distal').xpos.copy();axis=rz(yaw)@np.array(assembly['elastic_hinge']['axis_local_xyz']);camera.lookat[:]=pivot;camera.distance=.52;camera.azimuth=math.degrees(math.atan2(axis[1],axis[0]));camera.elevation=-8
    for angle in [-25,0,25]:
     data.joint(ident+'_flexion').qpos[0]=math.radians(angle);mujoco.mj_kinematics(m,data);mujoco.mj_comPos(m,data);mujoco.mj_camlight(m,data);renderer.update_scene(data,camera=camera,scene_option=option)
     labelled(Image.fromarray(renderer.render()),f'L3转轴近景 · {angle:+d}°','FK角度检查，未步进；主杆水平时俯视中心线夹角保持148.5°').save(target/f'hinge-FK-{angle}.png')
  manifest.append(dict(case_id=c['id'],source=str(source),scene_sha256=w.manifest['scene_sha256'],chord_m=task['mechanism']['chord_m'],hanging_fk_link_number=link,initial_views=['global','top','global_rear','initial-attachment-close'],inspection_views_fk_only=['hanging-attachment-close','hanging-assembly']))
  print('RENDERED',c['id'],flush=True)
 for view in ['global','top']:
  cw,ch=1000,681;sheet=Image.new('RGB',(3080,2*ch+170),'#e9eef3');draw=ImageDraw.Draw(sheet);draw.text((24,16),f'R16 六个新版环境 · {"全景" if view=="global" else "俯视"}',font=ImageFont.truetype(FONT,40),fill='#20374b');draw.text((26,76),'上排L2 / 下排L3；从左至右8 / 12 / 16臂。真实MuJoCo初态静态渲染。',font=ImageFont.truetype(FONT,25),fill='#607284')
  for c in selected:
   im=Image.open(OUT/c['id']/(view+'.png'));im.thumbnail((cw,ch),Image.Resampling.LANCZOS);x=20+[8,12,16].index(c['arms'])*1020;y=140+(c['level']-2)*ch;sheet.paste(im,(x,y))
  sheet.save(OUT/(view+'-six.png'))
 for name in ['hanging-attachment-close','initial-attachment-close','hanging-assembly']:
  ims=[Image.open(OUT/f'relay-a08-d{l}'/(name+'.png')) for l in [2,3]];sheet=Image.new('RGB',(3200,1090),'#f4f6f9')
  for i,im in enumerate(ims):sheet.paste(im,(1600*i,0))
  sheet.save(OUT/(name+'-L2-L3.png'))
 hinge=Image.new('RGB',(4800,1090),'#f4f6f9')
 for i,angle in enumerate([-25,0,25]):hinge.paste(Image.open(OUT/'relay-a08-d3'/f'hinge-FK-{angle}.png'),(1600*i,0))
 hinge.save(OUT/'hinge-out-of-plane.png')
 (OUT/'manifest.json').write_text(json.dumps(dict(scope='Static real MuJoCo initial scenes; separately labelled FK inspection poses. No Oracle success claimed.',physics_steps=0,cases=manifest),ensure_ascii=False,indent=2));print(OUT,flush=True)

if __name__=='__main__':main()
