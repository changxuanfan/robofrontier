"""Post-run global snapshot views from recorded observations, never physics resimulation.

Robot arm joints are recorded observations. Module poses are time-interpolated
between private 10Hz samples; finger positions use the measured total aperture
split symmetrically. No robot trajectories are interpolated or simulated.
"""
import argparse
from bisect import bisect_left
import json,math,sqlite3,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from robofrontier.cube_transport import _camera


def slerp(a,b,t):
    a=np.asarray(a);b=np.asarray(b);dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    if dot>.9995:q=a+t*(b-a);return q/np.linalg.norm(q)
    theta=math.acos(np.clip(dot,-1,1));return (math.sin((1-t)*theta)*a+math.sin(t*theta)*b)/math.sin(theta)


def render(run):
    run=Path(run).resolve();out=run/'private/outputs';db=sqlite3.connect(run/'private/state/runtime.sqlite');observations={}
    def collect(value):
        if isinstance(value,dict):
            if 'sim_time_s' in value and isinstance(value.get('robot'),dict) and len(value['robot'])==8:observations[float(value['sim_time_s'])]=value
            for v in value.values():collect(v)
        elif isinstance(value,list):
            for v in value:collect(v)
    for raw, in db.execute('select result from calls where result is not null'):
        collect(json.loads(raw))
    if (out/'seal.json').exists():collect(json.loads((out/'seal.json').read_text()))
    samples=[]
    for line in (out/'spring-dial-process.jsonl').open():
        v=json.loads(line)
        if v.get('event')=='sample':samples.append(v)
    ts=[v['sim_time_s'] for v in samples]
    model=mujoco.MjModel.from_xml_path(str(run/'private/inputs/scene/scene.xml'));data=mujoco.MjData(model)
    # Read a symmetric opposite-side camera from the frozen global calibration.
    scene=ET.Element('worldbody');front=model.camera('global').id;pos=model.cam_pos[front].copy()
    center=np.array([model.geom('table_0').pos[0],0,.55]);pos[1]=-pos[1]
    _camera(scene,'back',pos,center,float(model.cam_fovy[front]));e=scene[0]
    # Use a fixed opposite-side camera; free-camera azimuth has a different convention.
    axes=np.fromstring(e.get('xyaxes'),sep=' ');rotation=np.column_stack((axes[:3],axes[3:],np.cross(axes[:3],axes[3:])))
    quaternion=np.zeros(4);mujoco.mju_mat2Quat(quaternion,rotation.ravel())
    rear_id=model.camera('global_side').id;model.cam_pos[rear_id]=pos;model.cam_quat[rear_id]=quaternion;model.cam_fovy[rear_id]=model.cam_fovy[front]
    rear='global_side'
    fps=2;size=(1280,860);paths={name:out/(name+'.mp4') for name in ('global-front-observation-replay','global-rear-observation-replay')};encoders={}
    for name,path in paths.items():
        encoders[name]=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{size[0]}x{size[1]}','-r',str(fps),'-i','-','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    try:font=ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc',21,index=1)
    except OSError:font=ImageFont.load_default(size=21)
    ledger=[]
    try:
        with mujoco.Renderer(model,height=800,width=1280) as renderer:
            for t,obs in sorted(observations.items()):
                if t<ts[0] or t>ts[-1]+.101:continue
                j=min(len(ts)-1,bisect_left(ts,t));i=max(0,j-1)
                if t>=ts[-1]:i=j=len(ts)-1
                fraction=0. if i==j else (t-ts[i])/(ts[j]-ts[i]);fraction=float(np.clip(fraction,0,1))
                mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id)
                for a,r in obs['robot'].items():
                    for k,q in enumerate(r['joint_rad'],1):data.joint(a+'_joint'+str(k)).qpos[0]=q
                    for suffix in ('finger_joint1','finger_joint2'):data.joint(a+'_'+suffix).qpos[0]=r['gripper_width_m']/2
                for k,a in enumerate('ABCD'):
                    p=samples[i]['apparatus']['module_pose_world'][k];q=samples[j]['apparatus']['module_pose_world'][k];joint=data.joint('dial_'+a+'_module_free').qpos
                    joint[:3]=(1-fraction)*np.asarray(p['position_m'])+fraction*np.asarray(q['position_m']);joint[3:]=slerp(p['quaternion_wxyz'],q['quaternion_wxyz'],fraction)
                    data.joint('dial_'+a+'_hinge').qpos[0]=(1-fraction)*samples[i]['angle_rad'][k]+fraction*samples[j]['angle_rad'][k]
                mujoco.mj_forward(model,data)
                from robofrontier.spring_dial import update_display
                update_display(model,samples[i]['feedback'])
                for name,camera,label in [('global-front-observation-replay','global','FRONT'),('global-rear-observation-replay',rear,'REAR')]:
                    renderer.update_scene(data,camera=camera);canvas=Image.new('RGB',size,'#f3f6fa');canvas.paste(Image.fromarray(renderer.render()),(0,60));draw=ImageDraw.Draw(canvas)
                    draw.text((18,6),f'{label} | RECORDED OBSERVATION SNAPSHOTS | NOT CONTINUOUS FOOTAGE',font=font,fill='#172b3f')
                    draw.text((18,32),f'sim t={t:.2f}s | robot motion between observations is omitted | object poses: 10Hz interpolation',font=font,fill='#172b3f')
                    for _ in range(2):encoders[name].stdin.write(canvas.tobytes())
                    if t==max(observations):canvas.save(out/('final-'+label.lower()+'-state-reconstruction.png'))
                ledger.append(dict(observation_sim_time_s=t,module_sample_interval_s=ts[j]-ts[i],module_pose_interpolated=i!=j and fraction not in (0,1)))
    finally:
        for p in encoders.values():p.stdin.close()
        codes=[p.wait() for p in encoders.values()]
    if any(codes):raise RuntimeError('Video encoding failed')
    report=dict(videos={k:str(v) for k,v in paths.items()},snapshot_count=len(ledger),duration_s=len(ledger),rear_camera_position_world_m=pos.tolist(),rear_camera_quaternion_wxyz=quaternion.tolist(),source='actual full-robot observations and private apparatus samples from this run',limitations=['Not an original rear recording; discrete state reconstruction only','No robot trajectory interpolation or physics stepping','Module poses time-interpolated between recorded 10Hz samples','Each measured aperture is split symmetrically between fingers; individual finger positions were not logged'],frames=ledger)
    (out/'observation-replay-manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='frames'}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');render(p.parse_args().run)
