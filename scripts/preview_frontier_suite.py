"""Derived previews and static robot/camera guides; no scene mutation."""
import argparse,json,math
import mujoco,numpy as np
from PIL import Image,ImageDraw
from robofrontier.suite.scenes import catalog,case_directory
from robofrontier.cube_transport import _font
from robofrontier.run_setup import write_json

def preview(case):
    folder=case_directory(case);env=json.loads((folder/'environment.json').read_text());model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    with mujoco.Renderer(model,height=800,width=1280) as renderer:
        for view in ('global','global_rear','top'):
            renderer.update_scene(data,camera=view);Image.fromarray(renderer.render()).save(folder/('preview-'+view+'.png'))
    image=Image.new('RGB',(1200,900),(233,241,247));draw=ImageDraw.Draw(image);font=_font(30);small=_font(21)
    xy=np.array([s['base_position_m'][:2] for s in env['stations']]);lo=xy.min(axis=0)-.35;hi=xy.max(axis=0)+.35;scale=min(1060/(hi[0]-lo[0]),680/(hi[1]-lo[1]));mid=(lo+hi)/2
    def pos(p):return (600+(p[0]-mid[0])*scale,480-(p[1]-mid[1])*scale)
    for table in env['tables']:
        cx,cy,_=table['center_m'];hx,hy,_=table['half_size_m'];a=pos([cx-hx,cy+hy]);b=pos([cx+hx,cy-hy]);draw.rectangle((*a,*b),fill=(203,172,144),outline=(144,110,85),width=2)
    if case['family']=='mosaic':
        half=np.array([case['columns']*.051/2,7*.051/2]);a=pos([-half[0],half[1]]);b=pos([half[0],-half[1]]);draw.rectangle((*a,*b),fill=(244,241,233),outline=(92,107,117),width=2);draw.text((600,480),'ASSEMBLY',font=small,anchor='mm',fill=(23,32,43))
    for index,s in enumerate(env['stations']):
        x,y=pos(s['base_position_m']);r=25;draw.ellipse((x-r,y-r,x+r,y+r),fill=(245,242,235),outline=(23,32,43),width=3);draw.text((x,y),s['id'],font=font,anchor='mm',fill=(23,32,43));a=s['yaw_rad'];end=(x+52*math.cos(a),y-52*math.sin(a));draw.line((x,y,*end),fill=(23,32,43),width=3)
    draw.rectangle((0,0,1200,112),fill=(244,241,233));draw.text((40,30),case['id'],font=_font(34),fill=(23,32,43));draw.text((40,80),'Fixed robot bases and facing directions | +X east, +Y north',font=small,fill=(23,32,43));draw.text((40,855),'Mounting envelope only; no live object poses or hidden puzzle state.',font=small,fill=(23,32,43));image.save(folder/'robot-layout.png')
    cameras=[]
    for index in range(model.ncam):cameras.append(dict(name=model.camera(index).name,position_world_m=model.cam_pos[index].tolist(),quaternion_wxyz=model.cam_quat[index].tolist(),vertical_fov_deg=float(model.cam_fovy[index]),fixed=True))
    write_json(folder/'camera-guide.json',dict(scope='Fixed camera calibration; no object tracking',robot_layout='robot-layout.png',cameras=cameras));print(case['id'],flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');a=p.parse_args()
    for case in catalog()['cases']:
        if a.case is None or case['id'] in a.case:preview(case)
