"""Oracle engineering witness for the beam scene, not a policy/model result.

Reads timber pose and contacts for targets/assertions. After initial reset only
the PandaArm actuators move the live world, using the declared grasp-force drive
mode; no object pose correction, weld, mass/friction edit or external object force.
Collision/contact checks run every 2 ms.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import glfw
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.beam_handoff import DIRECTORY
from robofrontier.cube_transport import _font
from robofrontier.cube_transport_presentation import annotate
from robofrontier.recording import Video
from robofrontier.robot_control import PandaArm,rotation_vector
from robofrontier.task_config import load_harness
from robofrontier.pipeline import World
from robofrontier.motion_check import prepare_check


def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


class Trial:
    def __init__(self, output, visible=False, record=True, speed=.04, grasp_height_offset=.012, force=50., inner_grip=.16, plan=None):
        self.output = Path(output)
        self.output.mkdir(parents=True,exist_ok=False)
        self.env = json.loads((DIRECTORY/'environment.json').read_text())
        root = ET.parse(DIRECTORY/'scene.xml').getroot()
        root.find('compiler').set('meshdir',str((DIRECTORY/root.find('compiler').get('meshdir')).resolve()))
        for texture in root.findall('asset/texture'):
            if texture.get('file'):
                texture.set('file',str((DIRECTORY/texture.get('file')).resolve()))
        xml = ET.tostring(root,encoding='unicode')
        (self.output/'scene.xml').write_text(xml)
        self.model = mujoco.MjModel.from_xml_string(xml); self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id)
        mujoco.mj_forward(self.model,self.data)
        self.arms = {s['id']:PandaArm(self.model,self.data,s['id']) for s in self.env['stations']}
        self.cfg = load_harness(); self.speed = speed
        self.grasp_height_offset = grasp_height_offset
        self.force = force
        self.inner_grip = inner_grip
        self.plan=plan or {};self.outer_grip=float(self.plan.get('outer_grip_m',.47))
        if plan:
            if not plan.get('passed') or plan['source_scene_sha256']!=hashlib.sha256((DIRECTORY/'scene.xml').read_bytes()).hexdigest():
                raise ValueError('Nominal plan does not match this source scene')
            self.inner_grip=float(plan['inner_grip_m'])
            write(self.output/'nominal-plan.json',plan)
        self.grip_offsets=[self.outer_grip if ((i%2==0)==self.plan.get('start_outer',True)) else self.inner_grip for i in range(4)]
        self.home = {a:np.array(s['home_tcp_m']) for a,s in zip(self.arms,self.env['stations'])}
        self.home_q = {a:self.data.qpos[arm.qids].copy() for a,arm in self.arms.items()}
        self.timber = self.model.body('timber').id
        self.timber_geom = self.model.geom('timber_geom').id
        self.names = list(self.arms)
        roots = {self.model.body(a+'_link0').id:i for i,a in enumerate(self.names)}
        self.owners = np.full(self.model.ngeom,-1,dtype=int)
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            while body and body not in roots:
                body = int(self.model.body_parentid[body])
            if body in roots:
                self.owners[geom] = roots[body]
        self.trace = (self.output/'trace.jsonl').open('w')
        self.number = 0; self.stage = 'initial'; self.checks = 0
        self.airborne = False; self.allow_table = True; self.support_gap = 0.
        self.custody = ['A','B']; self.transfers = []; self.samples = deque(maxlen=100)
        self.contact_events = []; self.path_times = []; self.path_qpos = []; self.path_ctrl = []
        self.robot_screens = []
        self.next_sample = 0.; self.next_frame = 0.; self.min_airborne_bottom = float('inf')
        self.video = None; self.window = None; self.context = None
        self.renderer = None; self.width = 1280; self.height = 900
        if visible or record:
            if not glfw.init():
                raise RuntimeError('GLFW initialization failed')
            glfw.window_hint(glfw.VISIBLE,visible)
            self.window = glfw.create_window(self.width,self.height,'RoboFrontier | ORACLE beam handoff',None,None)
            if not self.window:
                raise RuntimeError('Reference window unavailable')
            glfw.make_context_current(self.window); glfw.swap_interval(0)
            self.context = mujoco.MjrContext(self.model,mujoco.mjtFontScale.mjFONTSCALE_150)
            self.scene = mujoco.MjvScene(self.model,maxgeom=10000); self.options = mujoco.MjvOption()
            self.camera = mujoco.MjvCamera(); self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera.fixedcamid = self.model.camera('global').id
            if record:
                self.video = Video(self.output/'oracle-sim-time.mp4',1536,1152,15)
        write(self.output/'parameters.json',json.loads((DIRECTORY/'parameters.json').read_text()))
        (self.output/'executor-source.py').write_text(Path(__file__).read_text())
        from robofrontier import robot_control
        (self.output/'robot-control-source.py').write_text(Path(robot_control.__file__).read_text())
        write(self.output/'drive-profile.json',dict(version='pose-aperture-grasp-v2',
            force_n=force,convention='total opposing finger drive, half per finger',
            force_law='F=-F_set-(2*F_set/total_aperture_speed)*tendon_velocity',
            original_position_mode='restored on opening/cancelled grasp; unchanged legacy gain and bias',
            unchanged='object/table geometry, body inertia/mass, friction, NoSlip, joint ranges; grasp soft output clamp tightens within original actuator ceilings'))
        self.source_sha = hashlib.sha256((DIRECTORY/'scene.xml').read_bytes()).hexdigest()

    @property
    def now(self):
        return float(self.data.time)

    def event(self, kind, **values):
        self.trace.write(json.dumps(dict(event=kind,sim_time_s=self.now,stage=self.stage,**values))+'\n')
        self.trace.flush()

    def snapshot(self):
        body = self.data.body('timber'); rot = body.xmat.reshape(3,3)
        half = self.model.geom_size[self.timber_geom]
        bottom = float(body.xpos[2]-np.sum(np.abs(rot[2])*half))
        fingers = {a:set() for a in self.arms}; forces = {a:0. for a in self.arms}
        table = False; forbidden = []
        for i,c in enumerate(self.data.contact):
            if self.timber_geom not in (c.geom1,c.geom2):
                continue
            other = int(c.geom2 if c.geom1==self.timber_geom else c.geom1)
            body_name = self.model.body(self.model.geom_bodyid[other]).name
            geom_name = self.model.geom(other).name
            force = np.zeros(6); mujoco.mj_contactForce(self.model,self.data,i,force)
            if (geom_name or '').startswith('table_') and c.dist<=.0001:
                table = True
            owner = int(self.owners[other])
            if owner>=0:
                a = self.names[owner]
                if body_name in (a+'_left_finger',a+'_right_finger'):
                    if force[0]>.05:
                        fingers[a].add(body_name.rsplit('_',2)[-2])
                        forces[a] += float(force[0])
                elif c.dist<-.0001:
                    forbidden.append(dict(robot=a,body=body_name,depth_m=float(-c.dist)))
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.timber,velocity,0)
        return dict(position_m=body.xpos.tolist(),quaternion_wxyz=body.xquat.tolist(),
                    bottom_z_m=bottom,table_contact=table,
                    gripping_robots=[a for a,v in fingers.items() if len(v)==2],
                    finger_contacts={a:sorted(v) for a,v in fingers.items() if v},
                    summed_normal_contact_force_n={a:v for a,v in forces.items() if v},
                    unexpected_robot_contact=forbidden,
                    angular_velocity_rad_s=velocity[:3].tolist(),linear_velocity_m_s=velocity[3:].tolist())

    def frame(self, camera='global', save=None, encode=True):
        if not self.context:
            return
        glfw.poll_events()
        if glfw.window_should_close(self.window):
            raise RuntimeError('Reference window closed')
        self.camera.fixedcamid = self.model.camera(camera).id
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN,self.context)
        viewport = mujoco.MjrRect(0,0,self.width,self.height)
        mujoco.mjv_updateScene(self.model,self.data,self.options,None,self.camera,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
        mujoco.mjr_render(viewport,self.scene,self.context)
        pixels = np.empty((self.height,self.width,3),dtype=np.uint8)
        mujoco.mjr_readPixels(pixels,None,viewport,self.context)
        image, _ = annotate(Image.fromarray(pixels[::-1]),self.model,self.data,self.env,camera)
        draw = ImageDraw.Draw(image); draw.rectangle((0,0,image.width,45),fill=(23,48,75))
        draw.text((16,8),f'ORACLE / PROGRAM | {self.stage} | sim={self.now:.2f}s | 1x simulation time',font=_font(22),fill='white')
        if save:
            image.save(self.output/save)
        if self.video and encode:
            if self.video.error:
                raise RuntimeError(self.video.error)
            self.video.queue.put(np.asarray(image).tobytes(),timeout=30)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW,self.context)
        mujoco.mjr_render(viewport,self.scene,self.context); glfw.swap_buffers(self.window)
        return image

    def tick(self):
        for arm in self.arms.values():
            arm.control(.02)
        for _ in range(10):
            mujoco.mj_step(self.model,self.data); self.checks += 1
            contacts = self.data.contact
            if len(contacts):
                first, second = self.owners[contacts.geom1], self.owners[contacts.geom2]
                hit = np.flatnonzero((first>=0)&(second>=0)&(first!=second)&(contacts.dist<0))
                if len(hit):
                    c = contacts[int(hit[0])]
                    detail = dict(robots=[self.names[first[hit[0]]],self.names[second[hit[0]]]],
                        bodies=[self.model.body(self.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)],
                        penetration_m=float(-c.dist))
                    self.contact_events.append(dict(detail,sim_time_s=self.now))
                    self.event('inter_robot_contact',**detail)
                    raise AssertionError('inter_robot_contact: '+str(detail))
        mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all():
            raise AssertionError('Nonfinite physics state')
        s = self.snapshot()
        if self.airborne:
            self.min_airborne_bottom = min(self.min_airborne_bottom,s['bottom_z_m'])
            if not self.allow_table and (s['table_contact'] or s['bottom_z_m']<.405):
                self.event('drop',custody=self.custody,timber=s)
                raise AssertionError('Timber contacted support during airborne transport')
            self.support_gap = self.support_gap+.02 if len(s['gripping_robots'])<2 else 0.
            if self.support_gap>.20:
                self.event('lost_dual_support',custody=self.custody,timber=s)
                raise AssertionError('Two-robot opposing-finger grip lost for >0.20 s')
        if s['unexpected_robot_contact']:
            self.event('unexpected_payload_contact',timber=s)
            raise AssertionError('Timber hit a non-finger robot body')
        self.samples.append(dict(sim_time_s=self.now,**s))
        if self.now+1e-9>=self.next_sample:
            self.event('sample',timber=s,robots={a:r.observe() for a,r in self.arms.items()})
            self.path_times.append(self.now); self.path_qpos.append(self.data.qpos.copy()); self.path_ctrl.append(self.data.ctrl.copy())
            self.next_sample += .1
        if self.now+1e-9>=self.next_frame:
            self.frame(); self.next_frame += 1/15

    def hold(self, seconds):
        for _ in range(round(seconds/.02)):
            self.tick()

    def commands(self, commands):
        prepared = {a:self.arms[a].validate(op,p,self.cfg,np.array([-1.8,-1.1,.405]),np.array([1.8,1.1,1.2]))
                    for a,(op,p) in commands.items()}
        active = {}
        for a,(op,p) in commands.items():
            self.number += 1; ident = f'oracle-{self.number}-{a}'
            self.arms[a].start(ident,prepared[a],self.now); active[ident] = a
            self.event('action_started',call_id=ident,robot=a,op=op,parameters=p)
        while active:
            self.tick()
            for a in self.arms:
                result = self.arms[a].poll(self.now,self.cfg['action_timeout_sim_s'])
                if result:
                    self.event('action_finished',robot=a,result=result,robot_state=self.arms[a].observe())
                    active.pop(result['call_id'],None)
                    if result['status']!='completed':
                        raise AssertionError(f'{a}: '+str(result))

    def moves(self, points, speed=None):
        commands={}
        for a,p in points.items():
            parameters=dict(target_m=np.asarray(p).tolist(),speed_m_s=speed or self.speed)
            try:
                self.arms[a].validate('move',parameters,self.cfg,np.array([-1.8,-1.1,.405]),np.array([1.8,1.1,1.2]))
            except ValueError as error:
                if a in self.custody or 'IK solution' not in str(error):raise
                # Oracle motion authoring only: an unloaded arm can use
                # the same public joint-target API when current-seed IK stalls.
                arm=self.arms[a];q,e=arm.ik.solve(np.asarray(p),arm.rotation,self.home_q[a],500)
                _,r=arm.ik.pose(q)
                if e>.002 or np.linalg.norm(rotation_vector(arm.rotation,r))>.02:raise
                parameters=dict(joint_rad=q.tolist(),joint_speed_rad_s=.18)
                self.event('oracle_unloaded_joint_route',robot=a,target_m=np.asarray(p).tolist(),endpoint_error_m=e)
            commands[a]=('move',parameters)
        # Same read-only robot-only checker exposed to policy roles. The
        # Oracle's object-derived targets do not enter its input except as
        # candidate TCP commands; payload/world truth is not inspected by it.
        preview=World.__new__(World);preview.model=self.model;preview.data=self.data;preview.arms=self.arms
        preview.command_bounds=dict(lower=[-1.8,-1.1,.405],upper=[1.8,1.1,1.2]);preview.task={}
        report=[r for r in prepare_check(preview,dict(motions=[dict(actor=a,**p) for a,(_,p) in commands.items()],
                    horizon_s=15.,sample_dt_s=.1),self.cfg) if r is not None][-1]
        self.robot_screens.append(report);self.event('robot_only_motion_screen',report=report)
        if report['assessment']=='conflict':
            raise AssertionError('Candidate violates reference clearance margin: '+str(report['closest']))
        self.commands(commands)

    def grips(self, arms, width):
        parameters=(dict(mode='grasp',width_m=self.env['beam']['width_m'],force_n=self.force,
                         speed_m_s=.04,epsilon_inner_m=.003,epsilon_outer_m=.003)
                    if width==0 else dict(width_m=width,speed_m_s=.06))
        self.commands({a:('gripper',dict(parameters)) for a in arms})

    def stage_event(self, name):
        self.stage = name; self.event('milestone',timber=self.snapshot(),custody=self.custody)
        s=self.snapshot()
        print(json.dumps(dict(stage=name,sim_s=round(self.now,2),timber_center_m=s['position_m'],
            gripping_robots=s['gripping_robots'],table_contact=s['table_contact'])),flush=True)

    def require_grips(self, names):
        s = self.snapshot()
        if not set(names).issubset(s['gripping_robots']):
            self.event('grasp_check_failed',required=names,timber=s)
            raise AssertionError('Opposing-finger grasp missing: '+str(names))

    def grasp_points(self, names, offset):
        center = self.data.body('timber').xpos.copy(); R = self.data.body('timber').xmat.reshape(3,3)
        return {a:center+R@np.array([0,(-1 if self.env['stations'][self.names.index(a)]['side']=='south' else 1)*offset,self.grasp_height_offset]) for a in names}

    def carry_to(self, names, x, z=None):
        self.require_grips(names)
        center = self.data.body('timber').xpos.copy()
        target = np.array([x,0.,center[2] if z is None else z])
        self.moves({a:self.data.site_xpos[self.arms[a].site].copy()+target-center for a in names})
        self.require_grips(names)

    def run(self):
        self.stage_event('initial')
        pair = ['A','B']; points = self.grasp_points(pair,self.grip_offsets[0])
        self.stage = 'A+B approach'
        self.moves({a:p+np.array([0,0,.18]) for a,p in points.items()})
        self.moves(points,speed=.025); self.grips(pair,0.); self.hold(.5)
        self.require_grips(pair); self.stage_event('A+B opposing-finger contacts confirmed')
        self.stage = 'A+B lift'; self.carry_to(pair,-1.25,.60); self.hold(1.)
        self.airborne = True; self.allow_table = False
        self.stage_event('A+B airborne dual support')
        self.frame(camera='pair_1',save='initial-lift-close.png',encode=False)
        for index,receiver in enumerate((['C','D'],['E','F'],['G','H'])):
            x = -.72+index*.72
            self.stage = '+'.join(pair)+' carry to '+ '+'.join(receiver)
            self.carry_to(pair,x); self.hold(.5)
            self.stage_event('handoff '+str(index+1)+' donor at overlap')
            offset = self.grip_offsets[index+1]
            points = self.grasp_points(receiver,offset)
            self.stage = '+'.join(receiver)+' receive approach'
            self.moves({a:p+np.array([0,0,.10]) for a,p in points.items()})
            self.moves(points,speed=.025); self.grips(receiver,0.); self.hold(.6)
            self.require_grips(pair+receiver)
            self.stage_event('handoff '+str(index+1)+' all four grips verified')
            self.frame(camera=f'pair_{index+1}',save=f'handoff-{index+1}-four-grips.png',encode=False)
            self.stage = '+'.join(pair)+' release'
            self.grips(pair,.075); self.hold(.5); self.require_grips(receiver)
            self.custody = receiver[:]
            self.transfers.append(dict(from_robots=pair[:],to_robots=receiver[:],sim_time_s=self.now,timber=self.snapshot()))
            self.stage_event('handoff '+str(index+1)+' custody transferred')
            self.stage = '+'.join(pair)+' withdraw'
            retreat=(self.plan['retreats'][index] if self.plan else
                     dict(outward_y_m=.16 if self.grip_offsets[index]==self.outer_grip else 0.,up_z_m=.08 if self.grip_offsets[index]==self.outer_grip else .16))
            self.moves({a:self.data.site_xpos[self.arms[a].site].copy()+np.array([0,(-1 if self.env['stations'][self.names.index(a)]['side']=='south' else 1)*retreat['outward_y_m'],retreat['up_z_m']]) for a in pair})
            self.moves({a:self.home[a] for a in pair}); self.require_grips(receiver)
            pair = receiver
        self.stage = 'G+H carry to right finish'
        self.carry_to(pair,self.env['goal']['center_xy_m'][0]); self.hold(.5)
        self.stage = 'G+H final placement'; self.allow_table = True
        bottom_goal = .4+self.env['beam']['height_m']/2
        self.carry_to(pair,self.env['goal']['center_xy_m'][0],bottom_goal+.0001)
        self.hold(.5); self.airborne = False
        self.grips(pair,.075)
        self.custody=[]
        self.moves({a:self.data.site_xpos[self.arms[a].site].copy()+np.array([0,0,.18]) for a in pair})
        self.moves({a:self.home[a] for a in pair}); self.hold(2.)
        self.stage_event('final stable released')
        self.frame(camera='top',save='final-top.png',encode=False)
        self.frame(save='final-global.png',encode=False)
        s = self.snapshot(); target = np.array([*self.env['goal']['center_xy_m'],bottom_goal])
        err = float(np.linalg.norm(np.array(s['position_m'])-target))
        stable = len(self.samples)>=50 and all(np.linalg.norm(v['linear_velocity_m_s'])<.01
                    and np.linalg.norm(v['angular_velocity_rad_s'])<.05 and v['table_contact'] and not v['gripping_robots'] for v in list(self.samples)[-50:])
        if not (err<.015 and s['table_contact'] and not s['gripping_robots'] and stable and len(self.transfers)==3):
            raise AssertionError('Final placement/handovers/stability check failed: '+str(s))
        return dict(final_center_error_m=err,final=s,stable_last_sim_s=1.)

    def close(self):
        np.savez_compressed(self.output/'trajectory.npz',sim_times=np.array(self.path_times),qpos=np.array(self.path_qpos),ctrl=np.array(self.path_ctrl))
        self.trace.close(); video = self.video.close() if self.video else None
        if self.context:
            self.context.free(); glfw.destroy_window(self.window); glfw.terminate()
        return video


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--visible',action='store_true')
    parser.add_argument('--no-record',action='store_true')
    parser.add_argument('--speed',type=float,default=.04)
    parser.add_argument('--force',type=float,default=50.)
    parser.add_argument('--inner-grip',type=float,default=.16,help='Oracle reference grasp location, not a policy requirement')
    parser.add_argument('--plan',type=Path,help='Engineering-only nominal whole-chain candidate plan')
    parser.add_argument('--grasp-height-offset',type=float,default=.012,
                        help='Oracle-selected TCP height relative to the observed beam center; does not change physics')
    args = parser.parse_args(); trial = None; wall = time.monotonic()
    report = dict(passed=False,scope='Oracle engineering witness; not GPT performance',
                  grip_evidence='Bilateral positive-force finger contacts are an indicator only; lifting and sustained support are checked separately.',
                  object_truth_used_for=['grasp targeting','translation offset','contact support assertions','final evaluation'],
                  mutations='Declared force/speed motor-controller mode via existing actuators. No weld, object repositioning, mass/friction changes or external object force. Hardware effort ceilings never increased; grasp drive uses a 70 N software envelope.',
                  failure_policy='Stop at the first collision, unsupported carry, drop or failed command. No silent restart or receiver recovery.')
    try:
        plan=json.loads(args.plan.read_text()) if args.plan else None
        trial = Trial(args.output.resolve(),args.visible,not args.no_record,args.speed,args.grasp_height_offset,args.force,args.inner_grip,plan)
        report['reference_choices'] = dict(tcp_speed_m_s=args.speed,grasp_height_offset_m=args.grasp_height_offset,grasp_force_n=args.force,grip_offsets_by_pair_m=trial.grip_offsets,receiver_approach_clearance_m=.10,nominal_plan_ref=str(args.plan) if args.plan else None)
        report.update(trial.run()); report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        if trial:
            trial.event('trial_failed',error=repr(error),timber=trial.snapshot())
            try:
                trial.frame(save='failure-global.png',encode=False)
                trial.frame(camera='top',save='failure-top.png',encode=False)
            except Exception as capture_error:
                report['capture_error'] = repr(capture_error)
    finally:
        if trial:
            report.update(stage=trial.stage,sim_time_s=trial.now,actions_started=trial.number,
                          handovers=trial.transfers,inter_robot_contacts=trial.contact_events,
                          robot_only_motion_screens=trial.robot_screens,
                          collision_checks=trial.checks,source_scene_sha256=trial.source_sha,
                          minimum_airborne_bottom_z_m=trial.min_airborne_bottom if np.isfinite(trial.min_airborne_bottom) else None,
                          last_timber=trial.snapshot(),video=trial.close())
            report['wall_time_s'] = time.monotonic()-wall
            write(trial.output/'report.json',report)
        print(json.dumps({k:v for k,v in report.items() if k not in ('robot_only_motion_screens','handovers','last_timber','final')},ensure_ascii=False),flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
