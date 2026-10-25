"""Private engineering execution through the same bounded PandaArm controller.

Live apparatus state is read for planning/evaluation. Only actuator commands
affect the live world after its initial keyframe reset. Scratch kinematics are
not physical execution and are recorded separately when used for screening.
"""
from copy import deepcopy,copy
import hashlib,json,math,shutil,time,os
from os.path import relpath
from pathlib import Path
import xml.etree.ElementTree as ET
from itertools import permutations,product
import mujoco,numpy as np
from PIL import Image
from ..robot_control import PandaArm,matrix_quat,rotation_vector
from ..task_config import load_harness,physical_control_fingerprint
from ..run_setup import write_json
from .evaluate import Monitor
from .provenance import bundle_manifest,environment_logic_manifest
from .joint_route import connect_route

class ReferenceStopped(RuntimeError):pass
class ProcessViolation(RuntimeError):pass

class PlacementPlanningError(RuntimeError):
    """Finite search failure, not a statement of physical impossibility."""
    def __init__(self,robot,name,diagnostics):
        self.stage='placement_search';self.diagnostics=diagnostics
        # Retain the legacy prefix for callers that preserve failed jobs.
        super().__init__('No common reachable transit height '+robot+'/'+name+'; stages='+str(diagnostics))

def joint_edge_clear(start,goal,check,step_rad=.025):
    """Sample a static joint segment; this is not continuous collision proof."""
    if not math.isfinite(step_rad) or not .0001<=step_rad<=.025:raise ValueError('Route sample joint step must be within 0.0001..0.025 rad')
    start,goal=np.asarray(start),np.asarray(goal);distance=float(np.max(abs(goal-start)))
    for resolution in ((.025,step_rad) if step_rad<.025 else (step_rad,)):
        count=max(2,math.ceil(distance/resolution))
        if not all(check(start+(goal-start)*f) for f in np.linspace(0,1,count+1)[1:]):return False
    return True

CUBE_ROTATIONS=tuple(matrix for axes in permutations(range(3)) for signs in product((-1,1),repeat=3)
    if round(np.linalg.det(matrix:=np.eye(3)[:,axes]*np.asarray(signs)))==1)

class Trial:
    def __init__(self,scene,output,seed=0,frames=True,sources=None,controller_cfg=None,record_hz=60.,monitor_factory=None):
        self.scene=Path(scene).resolve();self.directory=Path(output).resolve();self.directory.mkdir(parents=True,exist_ok=False)
        self.stop_requested=None
        self.abort_on_process_violation=False
        self.task=json.loads((self.scene/'task.json').read_text());self.env=json.loads((self.scene/'environment.json').read_text());self.seed=seed
        self.source_bundle=bundle_manifest(self.scene);write_json(self.directory/'source-assets.json',self.source_bundle)
        def immutable_copy(a,b):
            try:os.link(a,b)
            except OSError:shutil.copy2(a,b)
            return b
        shutil.copytree(self.scene/'assets',self.directory/'assets',copy_function=immutable_copy);root=ET.parse(self.scene/'scene.xml').getroot()
        if not (self.scene/'workstation.json').exists():root.find('compiler').set('meshdir',relpath((self.scene/root.find('compiler').get('meshdir')).resolve(),self.directory))
        ET.ElementTree(root).write(self.directory/'scene.xml',encoding='unicode')
        self.model=mujoco.MjModel.from_xml_path(str(self.directory/'scene.xml'));self.data=mujoco.MjData(self.model);self._live_data=self.data;mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id);mujoco.mj_forward(self.model,self.data)
        self.initial_perturbation=dict(seed=seed,scope='Declared verification initialization only; no changes after the first physics step')
        if seed:
            rng=np.random.default_rng(19071+seed)
            relay_v2=self.task.get('suite_version')==2 and self.task['family']=='relay'
            for obj in self.task['objects']:
                if relay_v2:continue
                joint=self.data.joint(obj['id']+'_free');joint.qpos[:2]+=rng.uniform(-.001,.001,2)
                angle=float(rng.uniform(-.012,.012));old=self.data.body(obj['id']).xmat.reshape(3,3)
                from .scenes import rz
                joint.qpos[3:]=matrix_quat(rz(angle)@old)
            if self.task['family']=='dial':
                for dial in self.task['dials']:
                    self.data.joint(dial['id']+'_hinge').qpos[0]+=float(rng.uniform(-.008,.008))
            if relay_v2:
                from .scenes import rz
                from ..robot_control import quat_matrix
                for assembly in self.task['mechanism']['payload_assemblies']:
                    origin=self.data.joint(assembly['id']+'_free').qpos[:3].copy();shift=np.r_[rng.uniform(-.001,.001,2),0];rotation=rz(float(rng.uniform(-.012,.012)))
                    for name in (assembly['id'],assembly.get('weight_body'),assembly.get('tray_body')):
                        if name:
                            q=self.data.joint(name+'_free').qpos;q[:3]=origin+rotation@(q[:3]-origin)+shift;q[3:]=matrix_quat(rotation@quat_matrix(q[3:]))
            elif self.task['family']=='relay' and self.task['level']==3:
                a=float(rng.uniform(-.025,.025));self.data.joint('suspension_joint_0').qpos[:]=[math.cos(a/2),math.sin(a/2),0,0]
            self.model.key_qpos[self.model.key('initial').id]=self.data.qpos.copy();root.find("keyframe/key[@name='initial']").set('qpos',' '.join(repr(float(x)) for x in self.data.qpos));ET.ElementTree(root).write(self.directory/'scene.xml',encoding='unicode');mujoco.mj_forward(self.model,self.data)
        self.arms={r:PandaArm(self.model,self.data,r) for r in self.task['arms']};self.cfg=deepcopy(controller_cfg) if controller_cfg is not None else load_harness();self.tick=0;self.stride=round(.02/self.model.opt.timestep)
        for arm in self.arms.values():arm.configure_hand_position_servo(self.cfg.get('gripper_position_servo','legacy'))
        self.home={r:(self.data.site_xpos[a.site].copy(),self.data.site_xmat[a.site].reshape(3,3).copy()) for r,a in self.arms.items()}
        self.home_q={r:self.data.qpos[a.qids].copy() for r,a in self.arms.items()}
        self.screen_model=copy(self.model)
        self.screen_model.geom_margin[:]=np.maximum(self.screen_model.geom_margin,.004)
        self.screen_data=mujoco.MjData(self.screen_model);self.carry_geoms={};self.hand_geoms={}
        self.stations={s['id']:s for s in self.env['stations']};self.actions=[];self.events=[];self.event_qpos=[];self.event_qvel=[];self.frame_qpos=[];self.frame_times=[];self.frame_scores=[];self.next_frame=0;self.record_frames=frames
        self.trace=(self.directory/'trace.jsonl').open('w');self.started=time.monotonic();self.renderer=None;self.held={};self.frame_grip_force=[];self.frame_grip_setpoint=[];self.record_interval=1./record_hz
        secret=None
        if self.task['family']=='dial' and monitor_factory is None:
            # Oracle verification seeds are not public scene seeds. This code
            # belongs only to the evaluator; the feedback decoder never reads it.
            from .dial_feedback import alphabet,settings
            rng=np.random.default_rng(710391+seed)
            secret=(rng.choice(alphabet(self.task),len(self.task['dials'])) if settings(self.task) else rng.integers(1,4,len(self.task['dials']))).tolist()
            write_json(self.directory/'private-code.json',dict(code_symbols=secret,scope='Oracle verification only; not a policy input'))
        shared_motion=bool(frames and (self.scene/'workstation.json').exists())
        self.monitor=monitor_factory(self) if monitor_factory else Monitor(self,secret=secret,log_path=None if shared_motion else self.directory/'process.jsonl')
        self.motion=None
        self.process_events=None
        if shared_motion:
            from ..motion_recording import MotionWriter
            from .scenes import ROOT
            self.motion=MotionWriter(self.directory/'motion',self.scene,self.model,self.data,hz=record_hz,cache=ROOT/'runs/state-assets')
            from .oracle_process_log import OracleProcessLog
            self.process_events=OracleProcessLog(self.directory/'process.jsonl')
        source_dir=self.directory/'executor-source';source_dir.mkdir();self.executor_sha256=None;self.physical_control_sha256=None;self.environment_logic_sha256=None;self.environment_logic_provenance=None
        if sources is not None:
            digest=hashlib.sha256()
            for name,payload in sorted(sources.items()):
                destination=source_dir/name;destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(payload);digest.update(name.encode()+b'\0'+payload)
            self.executor_sha256=digest.hexdigest()
            hashes={name:hashlib.sha256(payload).hexdigest() for name,payload in sources.items()}
            self.physical_control_sha256=physical_control_fingerprint(hashes,self.cfg);self.environment_logic_provenance=environment_logic_manifest(hashes,self.task);self.environment_logic_sha256=self.environment_logic_provenance['sha256']
        else:
            for source in list(Path(__file__).parent.glob('*.py'))+[Path(__file__).parent.parent/'robot_control.py',Path(__file__).parent.parent/'sim.py',Path(__file__).parent.parent/'task_config.py']:shutil.copy2(source,source_dir/source.name)
        write_json(source_dir/'effective-harness.json',self.cfg)
        self.executor_launch=None
        frozen_root=os.environ.get('ROBOFRONTIER_ORACLE_FROZEN_ROOT')
        if frozen_root:
            payload=(Path(frozen_root)/'snapshot.json').read_bytes()
            (self.directory/'executor-launch.json').write_bytes(payload)
            self.executor_launch=dict(mode='frozen_process',manifest='executor-launch.json',sha256=hashlib.sha256(payload).hexdigest())
        write_json(self.directory/'task.json',self.task);write_json(self.directory/'environment.json',self.env)
        self.event('initial',scene_sha256=self.task['scene_sha256'],seed=seed)
    @property
    def time(self):return float(self.data.time)
    def check_stop(self):
        if self.stop_requested is not None and self.stop_requested():raise ReferenceStopped('Engineering stop requested; this world is not resumed')
    def event(self,kind,**fields):
        i=len(self.event_qpos);self.event_qpos.append(self.data.qpos.copy());self.event_qvel.append(self.data.qvel.copy());entry=dict(event=kind,sim_time_s=self.time,wall_elapsed_s=time.monotonic()-self.started,state_index=i,**fields);self.events.append(entry);self.trace.write(json.dumps(entry,allow_nan=False)+'\n');self.trace.flush()
        if getattr(self,'motion',None) is not None:self.motion.event(self.time,kind,**fields)
        self._heartbeat(kind)
    def _heartbeat(self,phase='executing'):
        if self.data is not getattr(self,'_live_data',self.data):return # Scratch candidates must never become live status/images.
        stamp=time.monotonic()
        if getattr(self,'live_preview',False) and self.renderer is not None and stamp-getattr(self,'last_preview',-float('inf'))>=5:
            self.renderer.update_scene(self.data,camera='global')
            temporary=self.directory/'live.tmp.png';Image.fromarray(self.renderer.render()).save(temporary)
            temporary.replace(self.directory/'live.png');self.last_preview=stamp
        write_json(self.directory/'status.json',dict(phase=phase,simulation_time_s=self.time,
            wall_elapsed_s=time.monotonic()-self.started,action_batches=len(getattr(self,'actions',[])),
            updated_at_unix_s=time.time(),active_robots=[r for r,a in self.arms.items() if a.job is not None]))
        self.last_heartbeat=stamp
    def step(self):
        if self.data is not getattr(self,'_live_data',self.data):raise RuntimeError('Planning scratch cannot step a physical world')
        guard=getattr(self,'reference_step_guard',None)
        if guard is not None:guard()
        if self.tick%self.stride==0:
            self.check_stop()
            for arm in self.arms.values():arm.control(.02)
        for arm in self.arms.values():
            if arm.grasp_force is not None:arm.physics_control()
        before=self.time;mujoco.mj_step(self.model,self.data)
        from ..physics_health import require_valid_step
        require_valid_step(self.data,before)
        self.tick+=1;self.monitor.physical_step()
        if self.process_events is not None:self.process_events.record(self.time,self.monitor.result_cache,self.monitor.violations)
        if self.abort_on_process_violation and self.monitor.violations:
            if self.motion is not None:self.motion.capture(self.data,force=True)
            self.event('irreversible_process_violation',violation=self.monitor.violations[-1],active_robots=[r for r,a in self.arms.items() if a.job is not None])
            raise ProcessViolation('Irreversible process violation: '+str(self.monitor.violations[-1]))
        if (time.monotonic()-getattr(self,'last_heartbeat',0)>5 or
            (getattr(self,'live_preview',False) and time.monotonic()-getattr(self,'last_preview',0)>5)):self._heartbeat()
        if self.motion is not None:self.motion.capture(self.data)
        if self.tick%self.stride==0:mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all():raise RuntimeError('Nonfinite physical state')
        if self.record_frames and self.motion is None and self.time>=self.next_frame-1e-8:
            self.next_frame=self.time+self.record_interval;self.frame_qpos.append(self.data.qpos.copy());self.frame_times.append(self.time);self.frame_scores.append(self.monitor.game.display if self.task['family']=='dial' and self.monitor.game.display is not None else -1)
            self.frame_grip_force.append([float(self.data.actuator_force[a.grip]) for a in self.arms.values()]);self.frame_grip_setpoint.append([a.grasp_force if a.grasp_force is not None else -1. for a in self.arms.values()])
    def wait(self,seconds):
        for _ in range(round(seconds/self.model.opt.timestep)):self.step()
    def execute(self,label,commands,allow_failure=False):
        if self.data is not getattr(self,'_live_data',self.data):raise RuntimeError('Planning scratch cannot execute robot actions')
        self.check_stop()
        if not commands:return {}
        lower=np.array(self.task['workspace_world_m']['lower']);upper=np.array(self.task['workspace_world_m']['upper'])
        validated={}
        for r,(op,p) in commands.items():
            try:validated[r]=self.arms[r].validate(op,p,self.cfg,lower,upper)
            except ValueError as error:
                self.event('validation_rejected',label=label,robot=r,command=[op,p],reason=str(error));raise
        index=len(self.actions);self.event('action_start',label=label,commands=commands)
        for r,c in validated.items():self.arms[r].start(f'oracle-{index}-{r}',c,self.time)
        pending=set(commands);results={};start=self.time
        try:
            while pending:
                self.step()
                if self.tick%self.stride:continue
                for r in list(pending):
                    result=self.arms[r].poll(self.time,15)
                    if result:results[r]=result;pending.remove(r)
                if self.time-start>605:raise RuntimeError('Unresolved action: '+label)
        except Exception as error:
            # Preserve submitted calls and advance IDs before live recovery.
            # Unresolved calls stay uncertain until an actual stop/poll event.
            interrupted={r:results.get(r,dict(call_id=f'oracle-{index}-{r}',status='uncertain',reason=str(error))) for r in commands}
            self.actions.append(dict(label=label,commands=commands,results=interrupted,start_sim_s=start,end_sim_s=self.time,interrupted=True))
            self.event('action_interrupted',label=label,results=interrupted)
            raise
        rec=dict(label=label,commands=commands,results=results,start_sim_s=start,end_sim_s=self.time);self.actions.append(rec);self.event('action_end',label=label,results=results)
        if index%10==0:print(json.dumps(dict(case=self.task['id'],batch=index,label=label,sim_time_s=self.time,status='working')),flush=True)
        if any(v['status']!='completed' for v in results.values()) and not allow_failure:raise RuntimeError(label+': '+str(results))
        return results
    def pose_command(self,robot,target,rotation,speed=.10,omega=.6,joints=False):
        if not joints:return ('move',dict(target_m=np.asarray(target).tolist(),quaternion_wxyz=matrix_quat(rotation).tolist(),speed_m_s=speed,angular_speed_rad_s=omega))
        arm=self.arms[robot];q,error=arm.ik.solve(np.asarray(target),rotation,arm.qref,250);_,reached=arm.ik.pose(q)
        if error>.0015 or np.linalg.norm(rotation_vector(rotation,reached))>.02:
            q,error=arm.ik.solve(np.asarray(target),rotation,self.home_q[robot],350);_,reached=arm.ik.pose(q)
        if error>.0015 or np.linalg.norm(rotation_vector(rotation,reached))>.02:raise ValueError('No verified robot IK for '+robot)
        return ('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.65))
    def move(self,robot,target,rotation=None,label='move',precise=False,speed_m_s=None):
        arm=self.arms[robot];rotation=self.data.site_xmat[arm.site].reshape(3,3).copy() if rotation is None else rotation
        if label.startswith('transit-'):
            if speed_m_s is not None:raise ValueError('Use move_free joint_speed_rad_s for routed transit')
            self.move_free(robot,target,rotation,label);return self.data.site_xpos[arm.site].copy()
        straight=robot in self.held or any(x in label for x in ('contact','lift','prealign','seat','withdraw'))
        try:self.execute(label,{robot:self.pose_command(robot,target,rotation,speed=(.035 if robot in self.held else .10) if speed_m_s is None else speed_m_s,joints=not straight and speed_m_s is None)})
        except ValueError as error:
            if robot not in self.held and 'no verified IK solution' in str(error):self.move_free(robot,target,rotation,label)
            else:raise
        if precise:
            for i in range(3):
                actual=self.data.site_xpos[arm.site].copy();current=self.data.site_xmat[arm.site].reshape(3,3).copy();error=np.asarray(target)-actual
                if np.linalg.norm(error)<.0006 and np.linalg.norm(rotation_vector(rotation,current))<.006:break
                self.execute(label+'-refine'+str(i),{robot:self.pose_command(robot,np.asarray(target)+error,rotation,speed=.02,omega=.2)})
                self.wait(.12)
        return self.data.site_xpos[arm.site].copy()
    def screen_ik(self,robot,target,rotation,seed=None):
        self.check_stop()
        if getattr(self,'live_preview',False) and self.data is getattr(self,'_live_data',None) and time.monotonic()-getattr(self,'last_heartbeat',0)>5:self._heartbeat('planning')
        arm=self.arms[robot];q,e=arm.ik.solve(np.asarray(target),rotation,arm.qref if seed is None else seed,220);_,r=arm.ik.pose(q)
        return q if e<.0018 and np.linalg.norm(rotation_vector(rotation,r))<.02 else None
    def pose_clear(self,robot,q,allowed_objects=(),clearance_m=.003):
        """Scratch endpoint check, including every arm link and world geometry.

        Target finger contact is permitted, never palm/arm contact. The live
        physics and continuous process monitor remain the execution evidence.
        """
        self.check_stop();scratch=self.screen_data;scratch.qpos[:]=self.data.qpos;scratch.qpos[self.arms[robot].qids]=q
        mujoco.mj_kinematics(self.screen_model,scratch)
        tcp=scratch.site_xpos[self.arms[robot].site];workspace=self.task['workspace_world_m']
        if np.any(tcp<workspace['lower']) or np.any(tcp>workspace['upper']):return False
        carried=self.held.get(robot);carried_geoms=set()
        if carried is not None:
            arm=self.arms[robot];old=self.data.body(carried);old_r=old.xmat.reshape(3,3);old_tcp_r=self.data.site_xmat[arm.site].reshape(3,3)
            relative=old_tcp_r.T@old_r;new_r=scratch.site_xmat[arm.site].reshape(3,3)@relative
            offset=old_r.T@(self.data.site_xpos[arm.site]-old.xpos);new_p=scratch.site_xpos[arm.site]-new_r@offset
            if carried not in self.carry_geoms:self.carry_geoms[carried]=set(self.monitor.geoms[carried] if carried in self.monitor.geoms else self.monitor.descendant_geoms(self.model.body(carried).id))
            scratch.joint(carried+'_free').qpos[:]=np.r_[new_p,matrix_quat(new_r)];carried_geoms=self.carry_geoms[carried];mujoco.mj_kinematics(self.screen_model,scratch)
        mujoco.mj_comPos(self.screen_model,scratch);mujoco.mj_collision(self.screen_model,scratch);owner=self.monitor.geom_owner;rid=list(self.arms).index(robot)
        allowed=set(g for name in allowed_objects for g in self.monitor.geoms[name])
        allowed|=carried_geoms
        for c in scratch.contact:
            if c.dist>=clearance_m:continue
            a,b=map(int,c.geom)
            if owner[a]==rid and owner[b]==rid:
                names=[self.model.body(int(self.model.geom_bodyid[g])).name for g in (a,b)]
                if c.dist<-.001 and not all('finger' in name for name in names):return False
                continue
            if a in carried_geoms or b in carried_geoms:
                obj,other=(a,b) if a in carried_geoms else (b,a)
                if other in carried_geoms:continue
                body=self.model.body(int(self.model.geom_bodyid[other])).name
                if owner[other]==rid and body in (robot+'_left_finger',robot+'_right_finger'):continue
                return False
            if owner[a]==rid and owner[b]!=rid:a,b=b,a
            if owner[b]!=rid or owner[a]==rid:continue
            body=self.model.body(int(self.model.geom_bodyid[b])).name
            if body==robot+'_link0':continue # Fixed, bolted mounting contact.
            if a in allowed and body in (robot+'_left_finger',robot+'_right_finger'):continue
            return False
        return True
    def move_free(self,robot,target,rotation,label='free-approach',continuation=None,preferred_seed=None,target_joints=None,continuation_clearance=.001,joint_speed_rad_s=None,route_joint_step_rad=.025):
        """Finite robot-joint route, with static full-scene collision screening
        and a kinematic carried-object prediction when applicable. That
        prediction exists only in scratch data; live loads are never attached.
        Other arms are held during this reference move.
        Screens are not certificates: the live monitor still audits execution.
        """
        arm=self.arms[robot];start=self.data.qpos[arm.qids].copy();seeds=([np.asarray(preferred_seed)] if preferred_seed is not None else [])+[arm.qref,self.home_q[robot]]
        for delta in (-1.,-.5,.5,1.):
            q=self.home_q[robot].copy();q[0]+=delta;q[2]-=delta;seeds.append(np.clip(q,*arm.ik.limits.T))
        if target_joints is not None:
            fixed=np.asarray(target_joints);p,r=arm.ik.pose(fixed)
            if np.linalg.norm(p-target)>.003 or np.linalg.norm(rotation_vector(rotation,r))>.02:raise ValueError('Exact joint goal differs from the declared TCP goal')
            seeds=[fixed]
        def edge(a,b):
            return joint_edge_clear(a,b,lambda q:self.pose_clear(robot,q),route_joint_step_rad)
        goals=[];path=None
        for seed in seeds:
            q=seed.copy() if target_joints is not None else self.screen_ik(robot,target,rotation,seed)
            if q is None or not self.pose_clear(robot,q):continue
            if continuation is not None:
                end,r,allowed=continuation;next_q=q.copy();safe=True
                for f in np.linspace(0,1,5)[1:]:
                    next_q=self.screen_ik(robot,np.asarray(target)*(1-f)+np.asarray(end)*f,r,seed=next_q)
                    if next_q is None or not self.pose_clear(robot,next_q,allowed,clearance_m=continuation_clearance):safe=False;break
                if not safe:continue
            if all(np.linalg.norm(q-old)>.10 for old in goals):goals.append(q)
            if edge(start,q):path=[start,q];break
        if not goals:raise RuntimeError('No clear free-arm IK '+robot+'/'+label)
        goals.sort(key=lambda q:np.linalg.norm(q-start))
        if path is None:
            rng=np.random.default_rng(9901+len(self.actions))
            for goal in goals[:3]:
                path=connect_route(start,goal,arm.ik.limits,edge,rng,iterations=800 if any(tag in label for tag in ('receive-selected','send-clear','park')) else 300)
                if path is not None:break
            if path is None:raise RuntimeError('No collision-free sampled free-arm route '+robot+'/'+label)
        self.event('free_joint_route',robot=robot,label=label,waypoints=len(path),max_joint_sample_step_rad=route_joint_step_rad,scope='Private static full-scene sampled screen; not continuous collision proof or live object modification')
        speed=((.45 if self.task['family'] in ('transport','mosaic') else .20) if robot in self.held else .65) if joint_speed_rad_s is None else joint_speed_rad_s
        for index,q in enumerate(path[1:]):self.execute(label+'-'+str(index),{robot:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=speed))})
    def snapshot(self,label,cameras=('global','global_rear','top')):
        if self.renderer is None:self.renderer=mujoco.Renderer(self.model,height=800,width=1280)
        for cam in cameras:
            self.renderer.update_scene(self.data,camera=cam);Image.fromarray(self.renderer.render()).save(self.directory/(label+'-'+cam+'.png'))
    def finger_contacts(self,robot,name):return robot in self.monitor.bilateral(self.monitor.geoms[name])
    def release_clearance_height(self,robot,tool_rotation,grasp_height):
        """Known hand envelope above neighboring 40 mm cubes, not a policy rule."""
        arm=self.arms[robot];old_r=self.data.site_xmat[arm.site].reshape(3,3);origin=self.data.site_xpos[arm.site];minimum=0.
        if robot not in self.hand_geoms:self.hand_geoms[robot]=[g for g in range(self.model.ngeom) if self.model.body(int(self.model.geom_bodyid[g])).name in (robot+'_hand',robot+'_left_finger',robot+'_right_finger') and (self.model.geom_contype[g] or self.model.geom_conaffinity[g])]
        for geom in self.hand_geoms[robot]:
            body=self.model.body(int(self.model.geom_bodyid[geom])).name
            r=self.data.geom_xmat[geom].reshape(3,3);aabb=self.model.geom_aabb[geom];local=old_r.T@(self.data.geom_xpos[geom]+r@aabb[:3]-origin)
            if body.endswith(('_left_finger','_right_finger')):
                suffix='finger_joint1' if body.endswith('_left_finger') else 'finger_joint2';joint=self.model.joint(robot+'_'+suffix).id
                opening=min(arm.grip_max,float(self.task.get('cube_size_m') or .04)+.012)
                local+=(old_r.T@self.data.xaxis[joint])*(opening/2-float(self.data.joint(robot+'_'+suffix).qpos[0]))
            projected=tool_rotation@local;extent=abs(tool_rotation@old_r.T@r)@aabb[3:];minimum=min(minimum,float(projected[2]-extent[2]))
        return max(.005,self.task['cube_size_m']/2+.005-grasp_height-minimum)
    def pick(self,robot,name,height=.006,force=50.,placement_hint=None,aperture=None):
        arm=self.arms[robot]
        width=float(self.task.get('cube_size_m') or .04)
        if aperture is None:aperture=min(arm.grip_max,width+.012)
        # A 52 mm approach aperture clears a 40 mm cube without sweeping the
        # full 80 mm opening through neighboring stock. This is an ordinary
        # robot command, checked by the same exact-aperture feedback.
        if arm.grasp_force is not None or abs(arm.observe()['gripper_width_m']-aperture)>.0005:self.execute('open-for-pick',{robot:('gripper',dict(mode='move',width_m=aperture,speed_m_s=.05))})
        obj=self.data.body(name);p=obj.xpos.copy();r=obj.xmat.reshape(3,3).copy();candidate=None;choices=[]
        up_index=int(np.argmax(abs(r[2])));up=r[:,up_index]*np.sign(r[2,up_index])
        for axis in (i for i in range(3) if i!=up_index):
            for sign in (1,-1):
                z=-up;y=r[:,axis]*sign;rr=np.column_stack((np.cross(y,z),y,z));point=p+up*height
                preference=1
                if placement_hint is not None:
                    destination,orientation=placement_hint;offset=r.T@(point-p);relative=r.T@rr
                    from .scenes import rz
                    symmetries=CUBE_ROTATIONS if self.task['family']=='mosaic' else tuple(rz(turn) for turn in ((0,math.pi) if self.task['level']>1 else (0,math.pi/2,math.pi,-math.pi/2)))
                    for symmetry in symmetries:
                        goal_r=orientation@symmetry;goal=np.asarray(destination)+goal_r@offset;tool_r=goal_r@relative
                        q=self.screen_ik(robot,goal,tool_r)
                        if q is not None and self.screen_ik(robot,goal+np.array([0,0,.14]),tool_r,seed=q) is not None:preference=0;break
                choices.append((preference,point,rr))
        for _,point,rr in sorted(choices,key=lambda item:item[0]):
            try:self.move_free(robot,point+np.array([0,0,.15]),rr,'pick-approach',continuation=(point,rr,(name,)),continuation_clearance=-.0002 if aperture<=.042 else .001)
            except RuntimeError as error:
                if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                raise
            candidate=(point,rr);break
        if candidate is None:raise RuntimeError('No pickup pose '+robot+'/'+name)
        point,rr=candidate
        p=self.data.body(name).xpos.copy();r=self.data.body(name).xmat.reshape(3,3).copy();up_index=int(np.argmax(abs(r[2])));up=r[:,up_index]*np.sign(r[2,up_index]);point=p+up*height
        self.move(robot,point,rr,'pick-contact',precise=True)
        command=dict(mode='grasp',width_m=width,force_n=force,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008) if force is not None else dict(mode='move',width_m=0.,speed_m_s=.04)
        self.execute('pick-grasp',{robot:('gripper',command)});self.wait(.15)
        if not self.finger_contacts(robot,name):raise RuntimeError('Bilateral cube contact missing '+name)
        self.move(robot,self.data.site_xpos[arm.site]+np.array([0,0,.14]),rr,'pick-lift');self.wait(.1)
        if self.data.body(name).xpos[2]<p[2]+.08:raise RuntimeError('Payload not retained '+name)
        self.held[robot]=name;self.event('picked',robot=robot,object=name)
    def place(self,robot,position,object_rotation=None,precise=False,pose_provider=None):
        name=self.held[robot];arm=self.arms[robot];obj=self.data.body(name);old_r=obj.xmat.reshape(3,3).copy();tcp_r=self.data.site_xmat[arm.site].reshape(3,3).copy()
        offset=old_r.T@(self.data.site_xpos[arm.site]-obj.xpos);relative=old_r.T@tcp_r
        from .scenes import rz
        base_r=object_rotation if object_rotation is not None else old_r
        mosaic=self.task['family']=='mosaic';pocket=mosaic and self.task['level']==1
        socket_goal=self.task['family']=='transport' and self.task['level']>1 and precise
        symmetries=CUBE_ROTATIONS if self.task['family']=='mosaic' else tuple(rz(turn) for turn in ((0,math.pi) if socket_goal else (0,math.pi/2,math.pi,-math.pi/2)))
        rotations=sorted((base_r@symmetry for symmetry in symmetries),key=lambda candidate:np.linalg.norm(rotation_vector(candidate,old_r)))
        chosen=None;level=None;rejections={};examples={}
        def rejected(stage,reason=None):
            rejections[stage]=rejections.get(stage,0)+1
            if reason is not None:examples.setdefault(stage,reason)
        for r in rotations:
            target=np.asarray(position)+r@offset;rr=r@relative
            if mosaic:
                release_height=self.release_clearance_height(robot,rr,float((r@offset)[2]));release=target+np.array([0,0,release_height])
                if not pocket:release[:2]=np.asarray(position)[:2]
                for seed in (arm.qref,self.home_q[robot]):
                    q=self.screen_ik(robot,release,rr,seed=seed)
                    if q is None or not self.pose_clear(robot,q):continue
                    try:self.move_free(robot,release,rr,'mosaic-release-approach',target_joints=q)
                    except RuntimeError as error:
                        if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                        raise
                    chosen=(target,rr,r);level=float(release[2]);break
                if chosen is not None:break
                continue
            candidate_normal=np.array([0,0,1.]) if self.task['family']=='mosaic' else r[:,2]
            hover=target+candidate_normal*.015
            pre=None;near=None
            for seed in (arm.qref,self.home_q[robot]):
                pre=self.screen_ik(robot,target+np.array([0,0,.14]),rr,seed=seed)
                if pre is None:rejected('approach_ik')
                near=None if pre is None else self.screen_ik(robot,hover,rr,seed=pre)
                if pre is not None and near is None:rejected('near_slot_ik')
                if near is not None:break
            if near is None:continue
            # Free transit is screened below. A dense assembly contact phase
            # is not required to be contact-free: the actual solver and final
            # neighbor/goal checks decide whether the attempt is acceptable.
            if self.task['family']!='mosaic' and not self.pose_clear(robot,near,clearance_m=.003):
                contacts=[];rid=list(self.arms).index(robot)
                for contact in self.screen_data.contact:
                    a,b=map(int,contact.geom)
                    if contact.dist<.003 and rid in (self.monitor.geom_owner[a],self.monitor.geom_owner[b]):
                        contacts.append(dict(bodies=[self.model.body(int(self.model.geom_bodyid[g])).name for g in (a,b)],distance_m=float(contact.dist)))
                rejected('near_slot_clearance',sorted(contacts,key=lambda c:c['distance_m'])[:4]);continue
            try:
                # A loaded arm near maximum extension may need to retract
                # before it can rise. The full joint-space screen can find
                # that route; do not require an arbitrary high transit plane.
                self.move_free(robot,target+np.array([0,0,.14]),rr,'transit-direct',preferred_seed=pre)
                level=float(target[2]+.14);chosen=(target,rr,r);break
            except RuntimeError as error:
                if not str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):raise
                rejected('direct_transit',str(error))
            for offset in (.50,.45,.40,.35,.30):
                z=self.task['table_height_m']+offset
                if z<target[2]+.14:continue
                current=self.data.site_xpos[arm.site].copy();current[2]=z;current_r=self.data.site_xmat[arm.site].reshape(3,3).copy();high=target.copy();high[2]=z
                try:
                    self.move_free(robot,current,current_r,'transit-lift')
                    self.move_free(robot,high,rr,'transit-across')
                except RuntimeError as error:
                    if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):
                        rejected('raised_transit',str(error));continue
                    raise
                level=z;chosen=(target,rr,r);break
            if chosen is not None:break
        if level is None:
            diagnostics=dict(counts=rejections,examples=examples,scope='Finite tested poses/routes; not physical impossibility')
            self.event('placement_search_rejected',robot=robot,object=name,diagnostics=diagnostics)
            raise PlacementPlanningError(robot,name,diagnostics)
        target,rr,r=chosen;symmetry=base_r.T@r
        normal=np.array([0,0,1.]) if self.task['family']=='mosaic' else r[:,2]
        if not mosaic:self.move(robot,target+np.array([0,0,.14]),rr,'place-approach')
        buffer=self.task['family']=='transport' and not precise
        heights=() if mosaic else (.07,.035,.015) if buffer else (.07,.035,.015,.005,0)
        # A pocket rim does not admit the complete finger envelope. Release
        # just above the real rim and let gravity perform the final settling.
        for k,height in enumerate(heights):
            if not self.finger_contacts(robot,name):raise RuntimeError('Grasp lost during placement '+name)
            if pose_provider is not None:
                position,reference_r=pose_provider();r=reference_r@symmetry
            normal=np.array([0,0,1.]) if self.task['family']=='mosaic' else r[:,2]
            obj=self.data.body(name);current_r=obj.xmat.reshape(3,3).copy();offset=current_r.T@(self.data.site_xpos[arm.site]-obj.xpos);relative=current_r.T@self.data.site_xmat[arm.site].reshape(3,3)
            rr=r@relative
            if self.task['family']=='mosaic':height=max(height,self.release_clearance_height(robot,rr,float((r@offset)[2])))
            target=np.asarray(position)+r@offset+normal*height
            self.move(robot,target,rr,'place-prealign' if height else 'place-seat',precise=precise)
        self.wait(.15)
        if self.task['family']=='mosaic' and not pocket and np.linalg.norm(self.data.site_xpos[arm.site,:2]-np.asarray(position)[:2])>.002:
            # A cube can settle along a fingertip during a joint-space carry.
            # In the final free drop, it slides back under the pad centre as
            # squeeze is removed. Present that nominal landing axis over the
            # cell rather than preserving a transient tangential grasp offset.
            # This is an Oracle motion choice, not a controller attachment or
            # a change to object state, physics or the completion tolerance.
            landing=self.data.site_xpos[arm.site].copy();landing[:2]=np.asarray(position)[:2]
            self.move(robot,landing,rr,'place-release-axis',precise=True)
            self.event('reference_release_axis',robot=robot,object=name,landing_xy_m=np.asarray(position)[:2].tolist())
        # Grasp drive and aperture move have different semantics. Removing the
        # force hold is not itself evidence of release: retreat and evaluate.
        open_width=min(arm.grip_max,float(self.task.get('cube_size_m') or .04)+.012)
        width=open_width if pocket or buffer or self.task['family']=='transport' else float(np.clip(arm.observe()['gripper_width_m']+.001,0,arm.grip_max));relaxed=self.execute('place-relax',{robot:('gripper',dict(mode='move',width_m=width,speed_m_s=.10 if pocket else .05))},allow_failure=True)[robot]
        if arm.grasp_force is not None or relaxed['status']=='uncertain' or (relaxed['status']=='failed' and relaxed.get('reason') not in ('Requested aperture exceeded','Requested opening not reached')):raise RuntimeError('Cannot safely release force hold: '+str(relaxed))
        self.move(robot,self.data.site_xpos[arm.site]+normal*.10,rr,'place-withdraw')
        if abs(arm.observe()['gripper_width_m']-open_width)>.0005:self.execute('place-open-clear',{robot:('gripper',dict(mode='move',width_m=open_width,speed_m_s=.08))})
        self.wait(.12)
        if not self.monitor.released(self.monitor.geoms[name]):raise RuntimeError('Payload still contacting robot after release '+name)
        self.held.pop(robot);self.event('placed',robot=robot,object=name,target_m=np.asarray(position).tolist())
    def close(self,report):
        self.monitor.close();self.event('trial_end',result=report);self.trace.close()
        if self.process_events is not None:
            self.process_events.close();report['process_recording']=dict(schema=self.process_events.schema,pose_stream='motion/stream.json',process_events='process.jsonl')
        if self.motion is not None:report['motion_recording']=self.motion.close(None if report.get('error') else self.data,status='interrupted' if report.get('interrupted') else 'failed' if report.get('error') else 'complete')
        if self.renderer:self.renderer.close()
        np.savez_compressed(self.directory/'frame-states.npz',qpos=np.asarray(self.frame_qpos),time_s=np.asarray(self.frame_times),display=np.asarray(self.frame_scores),robot_ids=np.asarray(list(self.arms)),gripper_actuator_force_n=np.asarray(self.frame_grip_force),gripper_force_setpoint_n=np.asarray(self.frame_grip_setpoint))
        np.savez_compressed(self.directory/'event-states.npz',qpos=np.asarray(self.event_qpos),qvel=np.asarray(self.event_qvel))
        report.update(scene_sha256=self.task['scene_sha256'],scene_bundle_sha256=self.source_bundle['sha256'],oracle_scene_sha256=hashlib.sha256((self.directory/'scene.xml').read_bytes()).hexdigest(),executor_sha256=self.executor_sha256,verification_initialization=self.initial_perturbation,llm_used=bool(getattr(self,'oracle_repair_interventions',0)),live_object_state_writes_after_initialization=False,robot_action_count=sum(len(a['commands']) for a in self.actions),failed_robot_actions=sum(r['status']=='failed' for a in self.actions for r in a['results'].values()),action_batches=len(self.actions),wall_time_s=time.monotonic()-self.started,finished_at_unix_s=time.time())
        write_json(self.directory/'actions.json',self.actions)
        report.update(physical_control_sha256=self.physical_control_sha256,environment_logic_sha256=self.environment_logic_sha256,environment_logic_provenance=self.environment_logic_provenance)
        if self.executor_launch:report['executor_launch']=self.executor_launch
        write_json(self.directory/'report.json',report)
