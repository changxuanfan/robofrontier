"""Rendering, camera inspection and browser viewing for the static factory."""
from __future__ import annotations

import io
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import resource
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import mujoco
import numpy as np
from PIL import Image

from .factory_scene import ROOT, LAYERS


def _rss_mib():
    peak=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return peak/(2**20 if sys.platform=='darwin' else 1024)


class FactoryRenderer:
    """One retained factory scene; camera/layer changes never step physics."""
    def __init__(self, scene, renderer='classic', width=1280, height=720):
        self.path=Path(scene).resolve();self.width,self.height=width,height
        self.manifest=json.loads((self.path.parent/'manifest.json').read_text())
        self.scene_hash=hashlib.sha256(self.path.read_bytes()).hexdigest()
        if self.scene_hash!=self.manifest['scene_sha256']:raise ValueError('Scene/manifest mismatch; rebuild the factory before rendering')
        self.model=mujoco.MjModel.from_xml_path(str(self.path));self.data=mujoco.MjData(self.model)
        # Presentation has no physical dynamics. FK avoids allocating a
        # collision pipeline for thousands of visual-only workstation bodies.
        mujoco.mj_kinematics(self.model,self.data)
        mujoco.mj_comPos(self.model,self.data)
        mujoco.mj_camlight(self.model,self.data)
        self.renderer_name=renderer;self.options=mujoco.MjvOption();self.camera_id='interior_south'
        self.camera=next(c.copy() for c in self.manifest['cameras'] if c['id']==self.camera_id)
        self.render_times=[];self.last_pixels=None
        if renderer!='classic':raise ValueError('Interactive renderer must be classic; use Blender for Cycles/Eevee')
        self.renderer=mujoco.Renderer(self.model,height=height,width=width,max_geom=max(self.model.ngeom+1000,5000))
        self.set_camera(self.camera)
        # Settle initial material compilation/post-processing before inspecting output.
        for _ in range(12):self.render()
        # Shader compilation is startup cost, not the viewer's steady frame time.
        self.render_times.clear()

    def set_camera(self, values):
        state=self.camera.copy();state.update(values)
        p=np.asarray(state['position_m'],float);target=np.asarray(state['target_m'],float)
        if p.shape!=(3,) or target.shape!=(3,) or not np.isfinite([*p,*target]).all() or np.linalg.norm(p-target)<.01:raise ValueError('Invalid camera position/target')
        fov=state.get('vertical_fov_deg')
        if state.get('projection')!='orthographic' and (fov is None or not 10<=float(fov)<=110):raise ValueError('FOV must be between 10 and 110 degrees')
        hidden=state.get('hidden_layers',[])
        if any(g not in LAYERS for g in hidden):raise ValueError('Unknown hidden layer')
        self.camera=state;self.options.geomgroup[:]=1
        for g in hidden:self.options.geomgroup[LAYERS[g]]=0
        # The interactive camera is a world-root camera; model state is otherwise fixed.
        idx=self.model.camera('factory_camera_interactive').id
        z=p-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z)
        if np.linalg.norm(x)<1e-7:x=np.array([1.,0,0])
        x/=np.linalg.norm(x);y=np.cross(z,x);rot=np.column_stack([x,y,z]);quat=np.empty(4)
        mujoco.mju_mat2Quat(quat,rot.ravel());self.model.cam_pos[idx]=p;self.model.cam_quat[idx]=quat
        self.model.cam_fovy[idx]=state.get('orthographic_height_m') if state.get('projection')=='orthographic' else float(fov)
        self.model.cam_projection[idx]=1 if state.get('projection')=='orthographic' else 0
        mujoco.mj_camlight(self.model,self.data)
        vcam=mujoco.MjvCamera();vcam.type=mujoco.mjtCamera.mjCAMERA_FIXED;vcam.fixedcamid=idx;self.vcam=vcam

    def select(self, name):
        camera=next((c for c in self.manifest['cameras'] if c['id']==name),None)
        if camera is None:raise ValueError('Unknown camera '+name)
        self.camera_id=name;self.set_camera(camera)

    def manipulate(self, op, values):
        p=np.array(self.camera['position_m'],float);target=np.array(self.camera['target_m'],float)
        if op=='orbit':
            d=p-target;r=np.linalg.norm(d);yaw=math.atan2(d[1],d[0])+float(values.get('dx',0))*.005
            elev=math.asin(np.clip(d[2]/r,-1,1))+float(values.get('dy',0))*.005;elev=np.clip(elev,-.05,1.5)
            p=target+r*np.array([math.cos(elev)*math.cos(yaw),math.cos(elev)*math.sin(yaw),math.sin(elev)])
        elif op=='zoom':
            d=p-target;distance=np.clip(np.linalg.norm(d)*math.exp(float(values.get('delta',0))*.001),.25,180)
            p=target+d/np.linalg.norm(d)*distance
        elif op=='pan':
            forward=target-p;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1])
            if np.linalg.norm(right)<1e-7:right=np.array([1.,0,0])
            right/=np.linalg.norm(right);up=np.cross(right,forward)
            scale=max(.01,np.linalg.norm(target-p)*.0015)
            shift=(-right*float(values.get('dx',0))+up*float(values.get('dy',0)))*scale;p+=shift;target+=shift
        elif op=='move':
            f=target-p;f[2]=0
            if np.linalg.norm(f)<1e-7:f=np.array([0,1.,0])
            f/=np.linalg.norm(f);right=np.cross(f,[0,0,1])
            shift=f*float(values.get('forward',0))+right*float(values.get('right',0))+np.array([0,0,float(values.get('up',0))]);p+=shift;target+=shift
        else:raise ValueError('Unknown camera operation')
        p[2]=max(.12,p[2]);self.camera_id='custom'
        self.set_camera(dict(position_m=p.tolist(),target_m=target.tolist(),projection='perspective',vertical_fov_deg=self.camera.get('vertical_fov_deg') or 55))

    def render(self):
        start=time.perf_counter()
        self.renderer.update_scene(self.data,camera=self.vcam,scene_option=self.options)
        pixels=self.renderer.render()
        self.render_times.append(time.perf_counter()-start);self.last_pixels=pixels.copy()
        return pixels

    def report(self):
        times=self.render_times[-30:]
        return dict(renderer=self.renderer_name,mujoco_version=mujoco.__version__,platform=sys.platform,source_scene_sha256=self.scene_hash,resolution=[self.width,self.height],dimensions_m=self.manifest['dimensions_m'],camera=self.camera_id,camera_state=self.camera,frames=len(self.render_times),mean_frame_ms=1000*float(np.mean(times)) if times else None,peak_process_rss_mib=_rss_mib(),model_buffer_mib=self.model.nbuffer/2**20,decoded_texture_mib=self.model.ntexdata/2**20,geoms=self.model.ngeom,bodies=self.model.nbody,dynamic_dofs=self.model.nv,actuators=self.model.nu,physics_stepped=False)

    def close(self):
        self.renderer.close()


def render_views(scene, output, renderer='classic', width=1600, height=1000, views=None, benchmark_frames=8):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    r=FactoryRenderer(scene,renderer,width,height);reports=[]
    try:
        for name in views or ['interior_south','interior_north','interior_diagonal','detail_storage','detail_ceiling','top','from_south','from_north','from_east','from_west']:
            r.select(name)
            for _ in range(3):r.render()
            Image.fromarray(r.render()).save(output/(name+'.png'))
            start=len(r.render_times)
            for _ in range(benchmark_frames):r.render()
            stats=r.report();times=r.render_times[start:];stats['measured_view_mean_ms']=1000*float(np.mean(times)) if times else None
            stats['image']=str((output/(name+'.png')).resolve());reports.append(stats)
            print(json.dumps({'view':name,'renderer':renderer,'mean_ms':stats['measured_view_mean_ms'],'image':stats['image']}),flush=True)
        (output/'render-report.json').write_text(json.dumps(dict(scope='Static factory renders; no physics, GPT or task objects',views=reports),indent=2)+'\n')
    finally:r.close()
    return reports


def serve(scene, renderer='classic', width=1280, height=720, port=0, play=False):
    """GPU calls stay on the main thread; HTTP handlers only enqueue camera edits."""
    r=FactoryRenderer(scene,renderer,width,height);commands=queue.Queue(maxsize=128);lock=threading.Lock()
    timeline=None;playback_started=time.perf_counter();playback_state=None
    if play:
        from .factory_playback import FactoryTimeline
        from .task_families import RELEASE,read
        timeline=FactoryTimeline(scene,read(ROOT/RELEASE),model=r.model,data=r.data)
    shared={'jpeg':None,'status':{},'error':None};captures=ROOT/'outputs/robo-factory/captures';captures.mkdir(parents=True,exist_ok=True)
    page=(ROOT/'assets/robo_factory/viewer.html').read_bytes();stop=threading.Event()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,data,kind='application/json'):
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            route=self.path.split('?')[0]
            if route=='/':self.reply(200,page,'text/html; charset=utf-8')
            elif route=='/frame.jpg':
                with lock:frame=shared['jpeg']
                self.reply(200,frame,'image/jpeg') if frame else self.reply(503,b'{}')
            elif route=='/state':
                with lock:data=json.dumps(shared['status']).encode()
                self.reply(200,data)
            elif route=='/cameras':self.reply(200,json.dumps(r.manifest['cameras']).encode())
            else:self.reply(404,b'{}')
        def do_POST(self):
            if self.path!='/camera':self.reply(404,b'{}');return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=16384:raise ValueError('Invalid request size')
                values=json.loads(self.rfile.read(length));commands.put_nowait(values);self.reply(202,b'{"queued":true}')
            except (ValueError,TypeError,queue.Full):self.reply(400,b'{"error":"Invalid camera command"}')
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    url=f'http://127.0.0.1:{server.server_port}/';connection=ROOT/'outputs/robo-factory/viewer-connection.json'
    connection.write_text(json.dumps(dict(url=url,pid=os.getpid(),renderer=renderer,scope='Static factory camera viewer; no robot actions'),indent=2)+'\n')
    print(json.dumps({'url':url,'ready':True,'renderer':renderer}),flush=True)
    profile=json.loads((r.path.parent/'build-config.json').read_text())['render_profiles']['preview']
    interval=1/max(1,min(60,float(profile['fps_cap'])));next_render=0.;dirty=True
    try:
        while not stop.is_set():
            while not commands.empty():
                cmd=commands.get_nowait()
                try:
                    op=cmd.get('op','set')
                    if op=='preset':r.select(cmd['name'])
                    elif op=='set':r.set_camera({k:v for k,v in cmd.items() if k!='op'})
                    elif op in ['orbit','pan','zoom','move']:r.manipulate(op,cmd)
                    elif op=='frame_region':
                        region=next(z for z in r.manifest['regions'] if z['id']==cmd['name']);x0,y0,x1,y1=region['bounds_xy_m'];cx,cy=(x0+x1)/2,(y0+y1)/2;span=max(x1-x0,y1-y0)
                        r.set_camera(dict(position_m=[cx,cy-span,span*.85],target_m=[cx,cy,0],vertical_fov_deg=52,projection='perspective',hidden_layers=['roof','south'] if cy<0 else ['roof','north']))
                    elif op=='capture':
                        stamp=time.strftime('%Y%m%d-%H%M%S');Image.fromarray(r.render()).save(captures/(stamp+'.png'));(captures/(stamp+'.json')).write_text(json.dumps(r.report(),indent=2)+'\n')
                    else:raise ValueError('Unknown operation')
                    shared['error']=None;dirty=True
                except (ValueError,TypeError,KeyError,StopIteration) as error:
                    with lock:
                        shared['error']=str(error);shared['status']['error']=str(error)
            if (dirty or timeline is not None) and time.perf_counter()>=next_render:
                if timeline is not None:
                    playback_state=timeline.at(int((time.perf_counter()-playback_started)*timeline.fps))
                    pixels=r.render()
                else:
                    for _ in range(3):pixels=r.render()
                buffer=io.BytesIO();Image.fromarray(pixels).save(buffer,format='JPEG',quality=89)
                with lock:shared['jpeg']=buffer.getvalue();shared['status']={**r.report(),'error':shared['error'],'playback':playback_state}
                dirty=False;next_render=time.perf_counter()+interval
            time.sleep(.015)
    except KeyboardInterrupt:pass
    finally:
        if timeline is not None:timeline.close()
        server.shutdown();r.close()

