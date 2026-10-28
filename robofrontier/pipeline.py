"""One MuJoCo world, independently controlled Panda arms, file-backed mailbox.

The server owns physics/GL. Policy clients never mutate model/data directly.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import math
import os
import queue
import secrets
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import glfw
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .scene import ROOT, vector
from .sim import RobotIK
from .recording import Video

from .task_config import load_task, action_limits, panda_control_profile
from .robot_control import PandaArm
from .scene_task import camera_names
from .runtime_service import RuntimeService
from .motion_check import (CONTROL_DT, JOINT_SPEED_LIMIT, JOINT_ACCEL_LIMIT,
                           TRACKING_LIMIT, RobotGeometry, prepare_check, state_id)

_DEFAULT_TASK = load_task()
ARMS = _DEFAULT_TASK['arms']
OBJECTS = [(o['id'],o['color'],o['pattern'],o['region'],o['initial_position_m'][1]-ARMS[o['region']]) for o in _DEFAULT_TASK['objects']]
COLORS = {"gray": (.65,.67,.70,1), "blue": (.08,.32,.95,1),
          "red": (.9,.08,.08,1), "green": (.08,.7,.2,1)}
TASK = {a:[g['requires'] for g in goals] for a,goals in _DEFAULT_TASK['goals'].items()}


def atomic_json(path, value):
    from .run_setup import write_json
    write_json(path,value)


def camera(world, name, position, target, fovy=48):
    z = np.array(position, dtype=float)-np.array(target)
    z /= np.linalg.norm(z)
    x = np.cross([0,0,1], z); x /= np.linalg.norm(x)
    y = np.cross(z,x)
    ET.SubElement(world, 'camera', name=name, pos=vector(position),
                  xyaxes=vector([*x,*y]), fovy=str(fovy))


def build_world(directory, task=None):
    task = task or load_task()
    arms = task["arms"]
    objects = task["objects"]
    height = task["table_height_m"]
    half = task["cube_size_m"] / 2
    directory = Path(directory)
    asset_dir = directory/'private/outputs/scene-assets'
    asset_dir.mkdir(parents=True, exist_ok=True)
    root = ET.parse(ROOT/'vendor/panda/panda.xml').getroot()
    root.set('model', 'RoboFrontier shared pipeline')
    root.find('compiler').set('meshdir', str(ROOT/'vendor/panda/assets'))
    root.remove(root.find('keyframe'))
    root.find('option').attrib.update(timestep='.002', gravity='0 0 -9.81',
        integrator='implicitfast', solver='Newton', iterations='100', noslip_iterations=str(task['noslip_iterations']))
    visual = ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1280',offheight='800')
    ET.SubElement(visual,'headlight',ambient='.35 .35 .35',diffuse='.7 .7 .7')
    ET.SubElement(root, 'statistic', center='.35 0 .65', extent='2.5')
    world = root.find('worldbody')
    original = copy.deepcopy(world.find('body'))
    world.clear()
    sections = {}
    for tag in ('tendon','equality','actuator','contact'):
        section = root.find(tag)
        sections[tag] = list(copy.deepcopy(section))
        section.clear()
    named = {e.get('name') for e in original.iter() if e.get('name')}
    for children in sections.values():
        named.update(e.get('name') for child in children for e in child.iter() if e.get('name'))
    def renamed(element, prefix):
        element=copy.deepcopy(element)
        for e in element.iter():
            for key,value in list(e.attrib.items()):
                if (key == 'name' or key in ('joint','joint1','joint2','tendon','body1','body2')) and value in named:
                    e.set(key,prefix+value)
        return element
    for arm, base_y in arms.items():
        body=renamed(original,arm+'_'); body.set('pos',vector([0,base_y,height]))
        # The rail's 0..40 mm range is unchanged. Firmer limit constraints keep
        # an off-centre contact from crossing the fingers and wedging their
        # collision meshes; friction, actuator force and NoSlip stay unchanged.
        for suffix in ('finger_joint1','finger_joint2'):
            joint=body.find(f".//joint[@name='{arm}_{suffix}']")
            joint.set('solreflimit','0.004 1')
            joint.set('solimplimit','0.99 0.999 0.0001 0.5 2')
        hand=body.find(f".//body[@name='{arm}_hand']")
        ET.SubElement(hand,'site',name=arm+'_tcp',pos='0 0 .1029',size='.004',rgba='1 .8 0 1')
        world.append(body)
        for tag,children in sections.items():
            for child in children: root.find(tag).append(renamed(child,arm+'_'))
    ET.SubElement(world,'light',pos='.2 0 3',dir='0 0 -1',diffuse='.85 .85 .85')
    ET.SubElement(world,'geom',name='floor',type='plane',size='4 4 .1',rgba='.13 .16 .20 1')
    ET.SubElement(world,'geom',name='table',type='box',pos=vector([.30,(min(arms.values())+max(arms.values()))/2,height-.025]),size=vector([.52,(max(arms.values())-min(arms.values()))/2+.47,.025]),
                  rgba='.45 .36 .27 1',friction=vector(task['contact_friction']),condim='4')
    asset=root.find('asset')
    for arm,y in arms.items():
        for i,goal in enumerate(task['goals'][arm]):
            xyz=goal['position_m']
            ET.SubElement(world,'geom',name=f'{arm}_slot{i+1}',type='box',size='.032 .032 .0003',
                          pos=vector([xyz[0],xyz[1],height+.0004]),rgba='.86 .83 .62 1',contype='0',conaffinity='0')
        for dy in (-.325,.325):
            ET.SubElement(world,'geom',type='box',size='.40 .001 .0003',pos=vector([.35,y+dy,height+.0004]),
                          rgba='.25 .55 .95 1',contype='0',conaffinity='0')
        # Static public station label, no state leakage.
        im=Image.new('RGB',(256,128),(65,75,90)); d=ImageDraw.Draw(im)
        try: font=ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc',90)
        except OSError: font=ImageFont.load_default()
        d.text((85,12),arm,font=font,fill='white')
        p=asset_dir/f'label_{arm}.png'; im.save(p)
        ET.SubElement(asset,'texture',name='label_'+arm,type='2d',file=str(p))
        ET.SubElement(asset,'material',name='label_'+arm,texture='label_'+arm)
        ET.SubElement(world,'geom',type='box',size='.045 .07 .0003',pos=vector([.16,y,height+.0005]),
                      material='label_'+arm,contype='0',conaffinity='0')
        camera(world,arm+'_front',[1.35,y,1.3],[.40,y,.46],45)
        camera(world,arm+'_side',[.8,y-.95,1.15],[.4,y,.47],45)
    for obj in objects:
        name,color,pattern,arm = (obj[k] for k in ('id','color','pattern','region'))
        attrs={'rgba':vector(COLORS[color])}
        if pattern!='plain':
            im=Image.new('RGB',(128,128),tuple(int(v*255) for v in COLORS[color][:3])); d=ImageDraw.Draw(im)
            if pattern=='checker':
                for i in range(4):
                    for j in range(4):
                        if (i+j)%2==0:d.rectangle((i*32,j*32,i*32+31,j*32+31),fill=(220,230,255))
            else:
                for i in range(4):
                    for j in range(4): d.ellipse((i*32+8,j*32+8,i*32+22,j*32+22),fill='white')
            p=asset_dir/(name+'.png');im.save(p)
            ET.SubElement(asset,'texture',name=name,type='cube',file=str(p))
            ET.SubElement(asset,'material',name=name,texture=name)
            attrs={'material':name,'rgba':'1 1 1 1'}
        body=ET.SubElement(world,'body',name=name,pos=vector(obj['initial_position_m']))
        ET.SubElement(body,'freejoint',name=name+'_free')
        ET.SubElement(body,'geom',name=name+'_geom',type='box',size=vector([half]*3),mass=str(task['cube_mass_kg']),
                      friction=vector(task['contact_friction']),condim='4',**attrs)
    camera(world,'global',[2.3,0,2.65],[.25,0,.5],51)
    camera(world,'global_side',[1.6,-2.4,2.3],[.3,0,.5],51)
    xml=ET.tostring(root,encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml),xml


class Arm(PandaArm):
    def __init__(self,model,data,name,base_y=None,initialized=False):
        self.base_y=float(model.body(name+'_link0').pos[1]) if base_y is None else base_y
        super().__init__(model,data,name,None if initialized else [.40,self.base_y,.68])


class World:
    def __init__(self,directory,task=None):
        self.directory=Path(directory);self.task=task or load_task()
        self.command_bounds=copy.deepcopy(self.task.get('workspace_world_m'))
        if self.task.get('scene_asset'):
            from collections import deque
            folder=self.directory/Path(self.task['scene_asset']).parent
            root=ET.parse(self.directory/self.task['scene_asset']).getroot()
            compiler=root.find('compiler')
            compiler.set('meshdir',str((folder/compiler.get('meshdir','')).resolve()) if (folder/'workstation.json').exists() else str(folder/'model-assets'))
            for texture in root.findall('asset/texture'):
                if texture.get('file'):texture.set('file',str(folder/texture.get('file')))
            self.xml=ET.tostring(root,encoding='unicode');self.model=mujoco.MjModel.from_xml_string(self.xml)
            self.data=mujoco.MjData(self.model)
            mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id);mujoco.mj_forward(self.model,self.data)
            self.arms={a:Arm(self.model,self.data,a,initialized=True) for a in self.task['arms']}
            self.evaluation_samples=deque(maxlen=math.ceil(self.task['completion']['stable_duration_sim_s']/.02)+3)
            self.object_names={o['id'] for o in self.task['objects']}
            self.robot_bodies={b for ids in RobotGeometry(self.model,self.arms).bodies.values() for b in ids}
        else:
            self.model,self.xml=build_world(self.directory,self.task);self.data=mujoco.MjData(self.model)
            self.arms={a:Arm(self.model,self.data,a,y) for a,y in self.task['arms'].items()}
            mujoco.mj_forward(self.model,self.data)
            for _ in range(500):mujoco.mj_step(self.model,self.data)
            self.data.time=0
        mujoco.mj_forward(self.model,self.data)
        hand_cfg=self.directory/'private/inputs/harness.json'
        profile=json.loads(hand_cfg.read_text()).get('gripper_position_servo','legacy') if hand_cfg.exists() else 'legacy'
        for arm in self.arms.values():arm.configure_hand_position_servo(profile)
        self.beam_sampler=None;self.spring_sampler=None;self.suite_sampler=None;self.motion_recorder=None
        if self.task.get('kind')=='spring_dial':
            from .spring_dial import update_display
            update_display(self.model,dict(ready=False,correct_count=None,won=False))
        output=self.directory/'private/outputs';output.mkdir(parents=True,exist_ok=True)
        (output/'scene.xml').write_text(self.xml)

    @property
    def time(self):return float(self.data.time)

    def tcp_command_bounds(self,actor,cfg):
        bounds=getattr(self,'command_bounds',None)
        if bounds:return np.array(bounds['lower']),np.array(bounds['upper'])
        y=self.arms[actor].base_y
        return (np.array([cfg['workspace_x_m'][0],y-cfg['workspace_y_radius_m'],cfg['workspace_z_m'][0]]),
                np.array([cfg['workspace_x_m'][1],y+cfg['workspace_y_radius_m'],cfg['workspace_z_m'][1]]))

    def robot_facts(self,cfg):
        """Small static planning context, without querying live MjData or objects."""
        robots={}
        for name,arm in self.arms.items():
            base=self.model.body(name+'_link0').id
            if self.model.body_parentid[base]!=0 or self.model.body_jntnum[base]!=0:
                raise ValueError('This adapter requires fixed world-root robot bases')
            lower,upper=self.tcp_command_bounds(name,cfg)
            robots[name]={'base_pose_world':{'position_m':self.model.body_pos[base].tolist(),
                                             'quaternion_wxyz':self.model.body_quat[base].tolist()},
                          'arm_dof':len(arm.qids),'grippers':1,
                          'gripper_width_range_m':[arm.grip_min,arm.grip_max],
                          'tcp_command_bounds_world_m':{'lower':lower.tolist(),'upper':upper.tolist()}}
        width_ranges={tuple(r['gripper_width_range_m']) for r in robots.values()}
        shared_width=list(next(iter(width_ranges))) if len(width_ranges)==1 else None
        return {'coordinate_frame':'world','base_poses':'fixed throughout this run',
                'shared_capabilities':{'move':'TCP position and optional quaternion_wxyz using robot-only IK, or joint_rad mode; omitted orientation holds the current reference',
                    'tcp_reference':'move.target_m and observation.robot[robot].tcp_m refer to the same world-frame center between the fingertip pads. The controller already applies the hand-to-TCP offset. This is a robot point, not a measured object center.',
                    'robot_status':'moving/holding describe controller motion or pose holding, not whether an object is grasped. Verify payload with RGB; gripper width alone is not grasp proof.',
                    'gripper':{'state':['open','close'],
                        'width_m':shared_width,
                        'width_limits_source':'Each robots entry gives its installed mechanical gripper_width_range_m; use that robot-specific range.',
                        'speed_m_s':action_limits(cfg)['allowed_gripper_speed_m_s'],
                        'mode':['move','grasp'],'grasp':{'force_n':action_limits(cfg)['allowed_grasp_force_n'],
                            'width_m':'expected object width, not zero-close','epsilon_inner_m':{'min_exclusive':0,'max':.01,'default':.005},'epsilon_outer_m':{'min_exclusive':0,'max':.01,'default':.005},
                            'force_convention':'total opposing finger drive; each finger receives half; no measured contact-force feedback',
                            'is_grasped_source':'drive width/settling inference, not verified payload support',
                            'measured_force_sensor_available':False,
                            'holding':'continues after completed grasp and during arm moves until another gripper command'}},
                    'stop':'Bounded braking of the named own call; query its terminal result',
                    'quaternion_convention':'world-frame wxyz; normalized on input',
                    'angular_speed_rad_s':action_limits(cfg)['allowed_angular_speed_rad_s'],
                    'joint_rad':{'count':7,'speed_limit_rad_s':JOINT_SPEED_LIMIT},
                    'action_contract':'pose-aperture-grasp-v2','force_control':False,
                    'force_control_scope':'continuous arm torque/wrench; Hand grasp force is separately supported',
                    'gripper_force_control':True,'robot_gravity_feedforward':True,'max_in_flight_physical_actions_per_robot':1,
                    'interface_allows_different_robot_actions_concurrently':True,'nonblocking_submission':{'wait':False}},
                'robots':robots,'control_profile':panda_control_profile(cfg),
                'camera_configuration':{'render_size_pixel':[1280,800],'cameras':{self.model.camera(i).name:{'position_world_m':self.model.cam_pos[i].tolist(),'quaternion_wxyz':self.model.cam_quat[i].tolist(),'fovy_deg':float(self.model.cam_fovy[i])} for i in range(self.model.ncam)}},
                'bounds_scope':'Admitted TCP command bounds, not whole-link envelopes, guaranteed IK reachability or collision-free zones.',
                'state_sources':'Current robot state: observe; work/commitments: role notes and messages. No object state, occupancy or task plan is included.'}

    def robot_model(self,cfg):
        return RobotGeometry(self.model,self.arms).describe(self,cfg)

    def check_motion(self,parameters,cfg):
        return prepare_check(self,parameters,cfg)

    def motion_state_id(self):
        return state_id(self)

    def step(self):
        substeps=max(1,round(.02/self.model.opt.timestep))
        control_dt=substeps*self.model.opt.timestep
        for arm in self.arms.values():arm.control(control_dt)
        for _ in range(substeps):
            for arm in self.arms.values():
                if arm.grasp_force is not None:arm.physics_control()
            before=float(self.data.time);mujoco.mj_step(self.model,self.data)
            from .physics_health import require_valid_step
            require_valid_step(self.data,before)
            if getattr(self,'suite_sampler',None) is not None:self.suite_sampler.physical_step()
            if getattr(self,'beam_sampler',None) is not None:self.beam_sampler.check_collisions()
            if getattr(self,'spring_sampler',None) is not None:self.spring_sampler.sample()
            if getattr(self,'motion_recorder',None) is not None and not self.motion_recorder.closed:self.motion_recorder.capture(self.data)
        mujoco.mj_forward(self.model,self.data)
        if getattr(self,'beam_sampler',None) is not None:
            self.beam_sampler.sample()
        elif self.task.get('kind')=='shared_grid':
            from .scene_task import sample_shared
            sample_shared(self)

    def start_trial(self):
        if self.task.get('execution_readiness')=='environment_preview_only':
            raise ValueError('This v2 scene is an environment preview. Mechanism-specific evaluation and recording commissioning are required before starting a policy trial.')
        if self.task.get('execution_readiness')=='prepared_policy_trial':
            from .task_families import validate_frozen_policy
            validate_frozen_policy(self.task,self.directory)
        if self.task.get('kind')=='frontier_suite':
            from .suite.evaluate import Monitor
            secret=json.loads((self.directory/'private/inputs/suite-code.json').read_text())['code_symbols'] if self.task['family']=='dial' else None
            self.suite_sampler=Monitor(self,secret=secret,log_path=self.directory/'private/outputs/suite-process.jsonl')
            source=self.directory/Path(self.task['scene_asset']).parent
            if (source/'workstation.json').exists():
                from .motion_recording import MotionWriter
                self.motion_recorder=MotionWriter(self.directory/'private/outputs/motion',source,self.model,self.data,cache=ROOT/'runs/state-assets')
        if self.task.get('kind')=='spring_dial':
            from .spring_dial_task import SpringDialSampler
            self.spring_sampler=SpringDialSampler(self)
        if self.task.get('kind')=='beam_handoff':
            from .beam_task import BeamSampler
            self.beam_sampler=BeamSampler(self)
            self.beam_sampler.sample()

    def end_trial(self):
        if getattr(self,'suite_sampler',None) is not None:self.suite_sampler.finish()
        sampler=getattr(self,'spring_sampler',None)
        if sampler is not None:sampler.finish()
        if getattr(self,'motion_recorder',None) is not None:self.motion_recorder.close(self.data)

    def close(self):
        if getattr(self,'motion_recorder',None) is not None and not self.motion_recorder.closed:self.motion_recorder.close(self.data,status='interrupted')
        if getattr(self,'suite_sampler',None) is not None:self.suite_sampler.close()
        if getattr(self,'beam_sampler',None) is not None:self.beam_sampler.close()
        if getattr(self,'spring_sampler',None) is not None:self.spring_sampler.close()

    def validate_action(self,actor,op,p,cfg):
        lower,upper=self.tcp_command_bounds(actor,cfg)
        return self.arms[actor].validate(op,p,cfg,lower,upper)

    def start_action(self,actor,call_id,command):
        self.arms[actor].start(call_id,command,self.time)

    def request_stop(self,actor,call_id):
        arm=self.arms[actor]
        if arm.job!=call_id:raise ValueError('Stop target is not the active action on this robot')
        arm.request_stop(self.time)

    def _stop_action(self,arm,status,reason=None):
        return arm.end(status,reason)

    def poll_actions(self,timeout):
        return [r for arm in self.arms.values() if (r:=arm.poll(self.time,timeout)) is not None]

    def abort_actions(self,reason):
        return [a.end('failed',reason) for a in self.arms.values() if a.job]

    def evaluation_evidence(self):
        """Private bounded final window, for audit only; never policy feedback."""
        if getattr(self,'suite_sampler',None) is not None:return {'result':self.suite_sampler.result(),'process_ref':'private/outputs/suite-process.jsonl'}
        if getattr(self,'spring_sampler',None) is not None:
            return {'result':self.spring_sampler.result(),'process_ref':'private/outputs/spring-dial-process.jsonl'}
        if getattr(self,'beam_sampler',None) is not None:return self.beam_sampler.evaluator.result()
        samples=getattr(self,'evaluation_samples',None)
        return None if samples is None else {'samples':copy.deepcopy(list(samples)),
            'scope':'Private final sampled physics window; not a restart checkpoint or policy observation'}

    def evaluate(self):
        if self.task.get('kind')=='frontier_suite':
            if self.suite_sampler is None:return {'all_goals_met':False,'task_success':False,'scope':'Trial not started'}
            return self.suite_sampler.result()
        if self.task.get('kind')=='spring_dial':
            if self.spring_sampler is None:return {'all_goals_met':False,'task_success':False,'scope':'Trial not started'}
            return self.spring_sampler.result()
        if self.task.get('kind')=='beam_handoff':
            if self.beam_sampler is None:return {'all_goals_met':False,'scope':'Trial not started'}
            return self.beam_sampler.evaluator.result()
        if self.task.get('scene_asset'):
            from .scene_task import evaluate_shared
            return evaluate_shared(self)
        goals={};done=True
        for actor,slots in self.task['goals'].items():
            goals[actor]=[]
            for slot in slots:
                hits=[]
                for obj in self.task['objects']:
                    pos=self.data.body(obj['id']).xpos;delta=np.abs(pos-np.array(slot['position_m']))
                    ev=self.task['evaluation']
                    if delta[0]<ev['xy_tolerance_m'] and delta[1]<ev['xy_tolerance_m'] and delta[2]<ev['z_tolerance_m'] and all(obj.get(k)==v for k,v in slot['requires'].items()):hits.append(obj['id'])
                goals[actor].append(hits);done=done and len(hits)==1
        return {'all_goals_met':bool(done),'slots':goals,'sim_time_s':self.time}


class Service(RuntimeService):
    def __init__(self,directory,hidden=False):
        self.hidden=hidden;self.obs=0
        self.perf={'world_step_wall_s':0.,'world_steps':0,'capture_wall_s':0.,'captures':0,'pump_wall_s':0.}
        cfg = json.loads((Path(directory)/'private/inputs/harness.json').read_text())
        video_factory = (lambda:Video(Path(directory).resolve()/'private/outputs'/
                         (self.task.get('recording_camera','global')+'-realtime.mp4'),1280,800,self.config['video_fps'])) \
                         if cfg.get('record_realtime_video', True) else None
        super().__init__(directory,World,self.capture,video_factory)

    def capture(self,cameras):
        capture_start=time.perf_counter()
        self.obs+=1;paths={}
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN,self.context)
        viewport=mujoco.MjrRect(0,0,1280,800)
        for name in cameras:
            cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FIXED;cam.fixedcamid=self.world.model.camera(name).id
            mujoco.mjv_updateScene(self.world.model,self.world.data,self.options,None,cam,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
            mujoco.mjr_render(viewport,self.scene,self.context)
            rgb=np.empty((800,1280,3),dtype=np.uint8);mujoco.mjr_readPixels(rgb,None,viewport,self.context)
            path=self.directory/'public/observations'/f'obs-{uuid.uuid4().hex[:8]}-{self.obs:05d}-{name}.png'
            Image.fromarray(rgb[::-1]).save(path);paths[name]=str(path)
        if not getattr(self,'offscreen_context',None):
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW,self.context)
        result={'sim_time_s':self.world.time,'captured_at':time.time(),'images':paths,
                'robot':{a:r.observe() for a,r in self.world.arms.items()}}
        self.perf['capture_wall_s']+=time.perf_counter()-capture_start;self.perf['captures']+=1
        return result

    def run(self):
        from http.cookies import SimpleCookie
        from urllib.parse import urlsplit
        service=self
        observer=secrets.token_urlsafe(24)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                path=urlsplit(self.path).path
                if path=='/viewer/'+observer:
                    self.send_response(302);self.send_header('Set-Cookie','rf_observer='+observer+'; HttpOnly; SameSite=Strict; Path=/')
                    self.send_header('Location','/');self.end_headers();return
                cookie=SimpleCookie();cookie.load(self.headers.get('Cookie',''))
                if not cookie.get('rf_observer') or cookie['rf_observer'].value!=observer:
                    self.send_error(403,'Use the observer link saved for Meta');return
                if path=='/':raw=(ROOT/'assets/pipeline_dashboard.html').read_bytes();kind='text/html; charset=utf-8'
                elif path=='/state':
                    with service.lock:raw=json.dumps(service.snapshot(),ensure_ascii=False).encode()
                    kind='application/json'
                else:self.send_error(404);return
                self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(raw)
            def do_POST(self):
                if self.path!='/op':self.send_error(404);return
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=65536:raise ValueError('Invalid request size')
                    body=json.loads(self.rfile.read(length));actor=body['actor']
                    if service.tokens.get(actor)!=self.headers.get('X-Token'):self.send_error(403);return
                    if body.get('run_id')!=service.directory.name or body.get('schema_version')!=3:raise ValueError('Wrong run/protocol')
                    job=service.accept(actor,body['op'],body.get('data',{}),body['call_id'],body.get('ack_delivery'))
                    answer=job['result'] if job['done'].wait(40) else {'call_id':job['id'],'execution_status':'pending','note':'Query/retry original call ID; do not create duplicate motion'}
                    raw=json.dumps(answer,ensure_ascii=False).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(raw)
                except (ValueError,KeyError,TypeError) as error:self.send_error(400,str(error))
                except (BrokenPipeError,ConnectionResetError):pass
        try:
            if self.hidden and os.environ.get('MUJOCO_GL','').lower()=='osmesa':
                from mujoco.osmesa import GLContext
                self.offscreen_context=GLContext(1280,800)
                self.offscreen_context.make_current()
            else:
                if not glfw.init():raise RuntimeError('GLFW init failed')
                glfw.window_hint(glfw.VISIBLE,not self.hidden)
                self.window=glfw.create_window(1280,800,'RoboFrontier — Shared World',None,None)
                if not self.window:raise RuntimeError('GLFW window creation failed')
                glfw.make_context_current(self.window);glfw.swap_interval(1)
            self.context=mujoco.MjrContext(self.world.model,mujoco.mjtFontScale.mjFONTSCALE_150)
            self.scene=mujoco.MjvScene(self.world.model,maxgeom=5000);self.options=mujoco.MjvOption();self.options.geomgroup[3]=0
            self.cam=mujoco.MjvCamera();self.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
            self.cam.fixedcamid=self.world.model.camera(self.task.get('view_camera','global')).id
            self.record_cam=mujoco.MjvCamera();self.record_cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
            self.record_cam.fixedcamid=self.world.model.camera(self.task.get('recording_camera','global')).id
            self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.server.daemon_threads=True
            threading.Thread(target=self.server.serve_forever,daemon=True).start()
            url=f'http://127.0.0.1:{self.server.server_port}'
            for actor in self.roles:
                p=self.directory/'private/state/connections'/f'{actor}.json'
                connection=json.loads(p.read_text());connection['url']=url;atomic_json(p,connection);p.chmod(0o600)
            atomic_json(self.directory/'private/state/observer.json',{'url':url+'/viewer/'+observer,'read_only':True})
            print(json.dumps({'url':url,'directory':str(self.directory),'ready':True}),flush=True)
            previous=time.monotonic();acc=0.;next_frame=previous;next_state=previous;next_record=previous;next_perf=previous
            perf_file=self.directory/'private/outputs/cpu-performance.jsonl'
            while not self.shutdown and (not hasattr(self,'window') or not glfw.window_should_close(self.window)):
                now=time.monotonic();acc+=min(now-previous,.1);previous=now
                if hasattr(self,'window'):glfw.poll_events()
                with self.lock:
                    tick=time.perf_counter();self.pump();self.perf['pump_wall_s']+=time.perf_counter()-tick
                    while acc>=.02 and not self.ended:
                        tick=time.perf_counter();self.world.step();self.perf['world_step_wall_s']+=time.perf_counter()-tick
                        self.perf['world_steps']+=1;acc-=.02
                    if now>=next_state:self.periodic();next_state=now+1
                    if now>=next_perf:
                        with perf_file.open('a') as stream:stream.write(json.dumps({'at':time.time(),**self.perf})+'\n')
                        next_perf=now+5
                    if not self.hidden and now>=next_frame:
                        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW,self.context)
                        w,h=glfw.get_framebuffer_size(self.window);viewport=mujoco.MjrRect(0,0,w,h)
                        mujoco.mjv_updateScene(self.world.model,self.world.data,self.options,None,self.cam,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
                        mujoco.mjr_render(viewport,self.scene,self.context);glfw.swap_buffers(self.window);next_frame=now+1/30
                    if self.video and now>=next_record:
                        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN,self.context);viewport=mujoco.MjrRect(0,0,1280,800)
                        mujoco.mjv_updateScene(self.world.model,self.world.data,self.options,None,self.record_cam,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
                        mujoco.mjr_render(viewport,self.scene,self.context)
                        rgb=np.empty((800,1280,3),dtype=np.uint8);mujoco.mjr_readPixels(rgb,None,viewport,self.context)
                        frame=np.ascontiguousarray(rgb[::-1]);fps=self.config['video_fps']
                        self.video.frame(frame);next_record+=1/fps
                        while next_record<=now:self.video.frame(frame);next_record+=1/fps
                    elif not self.video:next_record=now
                time.sleep(.001)
        finally:
            if hasattr(self,'context'):
                with (self.directory/'private/outputs/cpu-performance.jsonl').open('a') as stream:
                    stream.write(json.dumps({'at':time.time(),**self.perf})+'\n')
            if hasattr(self,'server'):self.server.shutdown()
            if hasattr(self,'offscreen_context'):
                if hasattr(self,'context'):self.context.free()
                self.offscreen_context.free()
            if hasattr(self,'window'):glfw.destroy_window(self.window)
            if not hasattr(self,'offscreen_context'):glfw.terminate()
            self.close()


def main():
    parser=argparse.ArgumentParser(description='Shared MuJoCo owner; starting it is an explicit engineering action')
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--hidden',action='store_true')
    args=parser.parse_args();Service(args.run_dir,args.hidden).run()

if __name__=='__main__':main()
