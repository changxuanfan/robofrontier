"""Robot-only bounded Panda servo. No object lookup or task decisions."""
import math
from collections import deque
import mujoco
import numpy as np

from .sim import RobotIK
from .motion_check import JOINT_SPEED_LIMIT, JOINT_ACCEL_LIMIT, TRACKING_LIMIT
from .task_config import action_limits, execution_timing


def hand_position_profile(profile='legacy'):
    if profile not in ('legacy','pd1000-v1'):raise ValueError('Unknown hand position-servo profile')
    return dict(id=profile,stiffness_n_m=1000. if profile=='pd1000-v1' else None,
                damping='Critical from robot motor mass, minus existing passive damping' if profile=='pd1000-v1' else 'Source actuator',
                max_motor_force_n=70. if profile=='pd1000-v1' else None,
                scope='Frozen benchmark driver tuning; not a real Franka firmware calibration')


def matrix_quat(rotation):
    q=np.empty(4);mujoco.mju_mat2Quat(q,np.asarray(rotation).ravel());return q


def quat_matrix(value):
    q=np.asarray(value,dtype=float)
    norm=float(np.linalg.norm(q))
    if q.shape!=(4,) or not np.isfinite(q).all() or not math.isfinite(norm) or norm<1e-9:
        raise ValueError('quaternion_wxyz must be four finite values with nonzero norm')
    q=q/norm;r=np.empty(9);mujoco.mju_quat2Mat(r,q);return r.reshape(3,3)


def rotation_vector(target,current):
    q=matrix_quat(target@current.T)
    if q[0]<0:q=-q
    n=np.linalg.norm(q[1:])
    return np.zeros(3) if n<1e-12 else q[1:]*(2*math.atan2(n,max(0.,q[0]))/n)


def advance_rotation(current,target,angle):
    vector=rotation_vector(target,current);length=np.linalg.norm(vector)
    if length<=angle:return target.copy()
    q=np.r_[math.cos(angle/2),vector/length*math.sin(angle/2)]
    return quat_matrix(q)@current


def finite_vector(value,length,label):
    vector=np.asarray(value,dtype=float)
    if vector.shape!=(length,) or not np.isfinite(vector).all():raise ValueError('Finite '+label+' required')
    return vector


def bounded(value,limits,label):
    value=float(value)
    if not math.isfinite(value) or not limits[0]<=value<=limits[1]:raise ValueError(label+' outside run limits')
    return value


class PandaArm:
    def __init__(self,model,data,name,home=None):
        self.model,self.data,self.name=model,data,name
        self.ik=RobotIK(model,name+'_');self.qids=self.ik.qids;self.dofs=self.ik.dofs;self.site=self.ik.site
        self.act=np.array([model.actuator(f'{name}_actuator{i}').id for i in range(1,8)])
        self.grip=model.actuator(name+'_actuator8').id
        # Robot-owned mechanical profile. Finger travel is unchanged; a
        # manufactured offset fingertip changes the opposing contact-face gap.
        numeric=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_NUMERIC,name+'_gripper_gap_offset')
        self._grip_min=float(model.numeric_data[model.numeric_adr[numeric]]) if numeric>=0 else 0.
        self._grip_stroke=sum(float(model.jnt_range[model.joint(name+'_'+suffix).id,1]-model.jnt_range[model.joint(name+'_'+suffix).id,0]) for suffix in ('finger_joint1','finger_joint2'))
        if self.grip_min<0 or self.grip_stroke<=0:raise ValueError('Invalid robot gripper geometry profile')
        if home is not None:
            seed=np.array([0,0,0,-np.pi/2,0,np.pi/2,np.pi/4]);_,rotation=self.ik.pose(seed)
            q,error=self.ik.solve(np.asarray(home),rotation,seed,200)
            if error>.003:raise RuntimeError((name,'unreachable home',error))
            data.qpos[self.qids]=q;data.ctrl[self.act]=q;data.ctrl[self.grip]=255
            for suffix in ('finger_joint1','finger_joint2'):data.joint(name+'_'+suffix).qpos[0]=.04
        self.qref=data.ctrl[self.act].copy();_,self.rotation=self.ik.pose(data.qpos[self.qids])
        self.home_rotation=self.rotation.copy();self.rotation_goal=None;self.joint_goal=None
        self.velocity=np.zeros(7);self.tcp_ref=None;self.cartesian_integral=np.zeros(3)
        self.goal=None;self.speed=.06;self.angular_speed=.6;self.joint_speed=JOINT_SPEED_LIMIT
        self.gripper=float(data.ctrl[self.grip]);self.grip_goal=self.gripper;self.grip_speed=.08
        self.grip_initial_width=self.grip_min+sum(float(data.joint(name+'_'+s).qpos[0]) for s in ('finger_joint1','finger_joint2'))
        self.grip_direction='holding'
        self.grasp_force=None;self.grasp_width=None;self.grasp_epsilon=(.005,.005);self.grasp_confirmed=False
        self.gripper_command_mode='move'
        self.grip_position_parameters=tuple(x.copy() for x in
            (model.actuator_gainprm[self.grip],model.actuator_biasprm[self.grip],model.actuator_ctrlrange[self.grip],model.actuator_forcerange[self.grip]))
        self._source_grip_position_parameters=tuple(x.copy() for x in self.grip_position_parameters)
        self.grasp_hold_velocity_bias=float(self.grip_position_parameters[1][2]);self.hand_position_servo='legacy'
        self.grasp_drive_force=0.
        self.grasp_phase=None;self.grasp_force_started=None;self.grasp_contact_width=None
        finger_dofs=[model.jnt_dofadr[model.joint(self.name+'_'+s).id] for s in ('finger_joint1','finger_joint2')]
        self.grasp_motor_mass=float(np.sum(model.dof_M0[finger_dofs]));self.grasp_passive_damping=float(np.sum(model.dof_damping[finger_dofs]))
        self.grasp_settling=deque()
        self.job=None;self.job_op=None;self.stable=0;self.stopping=False;self.status='holding'
        self.execution_timeout_s=None;self.stop_reason=None;self.stop_details={};self.uncertainty_reported=False

    def configure_hand_position_servo(self,profile):
        """Engineering initialization only; no Agent action or object-state use.

        Preserve legacy inputs and force-hold damping. The calibrated position
        drive uses the same motor, encoder calibration and existing force bound.
        """
        hand_position_profile(profile)
        if self.job is not None or self.grasp_force is not None:raise ValueError('Configure hand driver before execution')
        params=tuple(x.copy() for x in self._source_grip_position_parameters)
        if profile=='pd1000-v1':
            stiffness=1000.;damping=max(0.,2*math.sqrt(stiffness*self.grasp_motor_mass)-self.grasp_passive_damping)
            params[0][0]=stiffness*self.grip_stroke/(2*255)
            params[1][1:3]=[-stiffness,-damping]
            params[3][0]=max(params[3][0],-70.);params[3][1]=min(params[3][1],70.)
        self.grip_position_parameters=params;self.hand_position_servo=profile
        for destination,value in zip((self.model.actuator_gainprm,self.model.actuator_biasprm,self.model.actuator_ctrlrange,self.model.actuator_forcerange),params):destination[self.grip]=value

    @property
    def grip_min(self):return getattr(self,'_grip_min',0.)

    @property
    def grip_stroke(self):return getattr(self,'_grip_stroke',.08)

    @property
    def grip_max(self):return self.grip_min+self.grip_stroke

    def grip_to_width(self,control):return self.grip_min+float(control)*self.grip_stroke/255

    def width_to_grip(self,width):return float(np.clip((float(width)-self.grip_min)/self.grip_stroke,0,1))*255

    def observe(self):
        fingers=[self.data.joint(self.name+'_'+s) for s in ('finger_joint1','finger_joint2')]
        width=self.grip_min+sum(float(j.qpos[0]) for j in fingers)
        grasped=bool(self.grasp_force is not None and self.grasp_confirmed and width>self.grip_min+.001 and
                     self.grasp_width-self.grasp_epsilon[0]<width<self.grasp_width+self.grasp_epsilon[1])
        return {'tcp_m':self.data.site_xpos[self.site].tolist(),
                'tcp_quaternion_wxyz':matrix_quat(self.data.site_xmat[self.site].reshape(3,3)).tolist(),
                'joint_rad':self.data.qpos[self.qids].tolist(),'joint_velocity_rad_s':self.data.qvel[self.dofs].tolist(),
                'gripper_width_m':width,
                'gripper_width_range_m':[self.grip_min,self.grip_max],
                'gripper_velocity_m_s':sum(float(j.qvel[0]) for j in fingers),
                'gripper_target_width_m':self.grip_to_width(self.grip_goal),'status':self.status,
                'hand_position_servo':self.hand_position_servo,
                'gripper_mode':'grasp' if self.grasp_force is not None else 'move',
                'grasp_drive_phase':self.grasp_phase,
                'grasp_force_setpoint_n':self.grasp_force,'is_grasped':grasped,
                'measured_grasp_force_n':None}

    def validate(self,op,p,cfg,lower,upper):
        if self.job:raise ValueError('Arm already has an execution')
        limits=action_limits(cfg)
        if op=='gripper':
            mode=p.get('mode','move')
            if mode=='grasp':
                fields={'mode','width_m','speed_m_s','force_n','epsilon_inner_m','epsilon_outer_m'}
                if set(p)-fields or not {'width_m','force_n'}<=set(p):
                    raise ValueError('Grasp requires width_m and force_n, optional speed and width tolerances')
                width=bounded(p['width_m'],[self.grip_min+.001,self.grip_max],'Grasp object width')
                force=bounded(p['force_n'],limits['allowed_grasp_force_n'],'Grasp force')
                if force>min(-self.grip_position_parameters[3][0],self.grip_position_parameters[3][1]):
                    raise ValueError('Grasp force exceeds configured drive capacity')
                epsilon=[bounded(p.get(k,.005),[0,.01],'Grasp width tolerance') for k in ('epsilon_inner_m','epsilon_outer_m')]
                if not all(e>0 for e in epsilon):raise ValueError('Grasp width tolerances must be positive')
                actual=self.observe()['gripper_width_m']
                if width>actual+epsilon[1]:raise ValueError('Grasp is a closing operation; open with move first')
                speed=bounded(p.get('speed_m_s',limits['default_gripper_speed_m_s']),limits['allowed_gripper_speed_m_s'],'Gripper speed')
                return self.timed({'op':op,'mode':mode,'width':width,'force':force,'epsilon':epsilon,'speed':speed},actual/speed,cfg)
            if mode!='move':raise ValueError('Gripper mode must be move or grasp')
            p={k:v for k,v in p.items() if k!='mode'}
            if set(p)-{'state','width_m','speed_m_s'} or ('state' in p)==('width_m' in p):
                raise ValueError('Choose gripper state or width_m, with optional speed_m_s')
            if 'state' in p and p['state'] not in ('open','close'):raise ValueError('Gripper state must be open or close')
            width=(self.grip_max if p['state']=='open' else self.grip_min) if 'state' in p else bounded(p['width_m'],[self.grip_min,self.grip_max],'Gripper width')
            speed=bounded(p.get('speed_m_s',limits['default_gripper_speed_m_s']),limits['allowed_gripper_speed_m_s'],'Gripper speed')
            travel=max(abs(width-self.grip_to_width(self.gripper)),abs(width-self.observe()['gripper_width_m']))
            return self.timed({'op':op,'width':width,'speed':speed},travel/speed,cfg)
        if set(p)-{'target_m','delta_m','quaternion_wxyz','speed_m_s','angular_speed_rad_s','joint_rad','joint_speed_rad_s'}:
            raise ValueError('Unsupported move parameter')
        if 'joint_rad' in p:
            if set(p)-{'joint_rad','joint_speed_rad_s'}:raise ValueError('Joint and Cartesian move parameters are mutually exclusive')
            q=finite_vector(p['joint_rad'],7,'seven joint angles')
            if np.any(q<self.ik.limits[:,0]) or np.any(q>self.ik.limits[:,1]):raise ValueError('Joint target outside soft limits')
            tcp,_=self.ik.pose(q)
            if np.any(tcp<lower) or np.any(tcp>upper):raise ValueError('Joint target TCP outside admitted workspace')
            speed=bounded(p.get('joint_speed_rad_s',JOINT_SPEED_LIMIT),[.01,JOINT_SPEED_LIMIT],'Joint speed')
            duration=max(np.max(np.abs(q-self.qref)),np.max(np.abs(q-self.data.qpos[self.qids])))/speed
            return self.timed({'op':op,'joint_goal':q,'joint_speed':speed},duration,cfg)
        if 'joint_speed_rad_s' in p:raise ValueError('Joint speed requires joint_rad')
        if 'target_m' in p and 'delta_m' in p:raise ValueError('Choose target_m or delta_m')
        actual=self.data.site_xpos[self.site]
        if 'target_m' in p:goal=finite_vector(p['target_m'],3,'xyz')
        elif 'delta_m' in p:goal=actual+finite_vector(p['delta_m'],3,'xyz delta')
        elif 'quaternion_wxyz' in p:goal=actual.copy()
        else:raise ValueError('Move requires a position, delta or orientation')
        if np.any(goal<lower) or np.any(goal>upper):raise ValueError('TCP workspace target outside admitted bounds')
        rotation=quat_matrix(p['quaternion_wxyz']) if 'quaternion_wxyz' in p else self.rotation.copy()
        speed=bounded(p.get('speed_m_s',cfg['default_tcp_speed_m_s']),cfg['allowed_tcp_speed_m_s'],'TCP speed')
        angular=bounded(p.get('angular_speed_rad_s',limits['default_angular_speed_rad_s']),limits['allowed_angular_speed_rad_s'],'Angular speed')
        q,error=self.ik.solve(goal,rotation,self.qref,250);_,reached=self.ik.pose(q)
        if error>.0025 or np.linalg.norm(rotation_vector(rotation,reached))>.02:
            raise ValueError('Requested pose has no verified IK solution; change pose or allocation')
        current_r=self.data.site_xmat[self.site].reshape(3,3)
        duration=max(np.linalg.norm(goal-actual)/speed,np.linalg.norm(rotation_vector(rotation,current_r))/angular,
                     np.max(np.abs(q-self.qref))/JOINT_SPEED_LIMIT)
        return self.timed({'op':op,'goal':goal,'rotation':rotation,'speed':speed,'angular_speed':angular},duration,cfg)

    def timed(self,command,duration,cfg):
        timing=execution_timing(cfg);budget=timing['base_timeout_sim_s']+timing['reference_time_factor']*float(duration)
        if not math.isfinite(budget) or budget>timing['max_timeout_sim_s']:
            raise ValueError('Estimated action duration exceeds maximum execution budget; choose another speed or segment')
        return {**command,'timing':{'estimated_reference_s':float(duration),'timeout_sim_s':budget}}

    def start(self,call_id,command,now):
        self.stopping=False;self.joint_goal=None;self.rotation_goal=None;self.goal=None
        if command['op']=='gripper':
            self.gripper_command_mode=command.get('mode','move')
            self.grip_initial_width=self.observe()['gripper_width_m']
            if command.get('mode')=='grasp':
                previous_drive=self.grasp_drive_force if self.grasp_force is not None else 0.
                retained=self.grasp_confirmed and command['width']-command['epsilon'][0]<self.grip_initial_width<command['width']+command['epsilon'][1]
                self.grasp_force=command['force'];self.grasp_width=command['width']
                self.grasp_epsilon=tuple(command['epsilon']);self.grasp_confirmed=False
                # Closing velocity control and settled force holding are
                # separate driver phases. A high approach velocity damper must
                # not remain saturated while another robot perturbs the load.
                self.model.actuator_gainprm[self.grip,:]=0
                self.model.actuator_gainprm[self.grip,0]=1
                self.model.actuator_biasprm[self.grip,:]=0
                ceiling=min(70.,-self.grip_position_parameters[3][0],self.grip_position_parameters[3][1])
                self.model.actuator_forcerange[self.grip]=[-ceiling,ceiling]
                self.model.actuator_ctrlrange[self.grip]=[-ceiling,ceiling]
                self.grasp_drive_force=min(previous_drive,command['force'],ceiling-1e-6)
                self.grasp_phase='force' if retained else 'approach';self.grasp_contact_width=self.grip_initial_width if retained else None
                self.grasp_force_started=now-.4*self.grasp_drive_force/command['force'] if retained else None
                self.data.ctrl[self.grip]=-self.grasp_drive_force
                self.gripper=self.width_to_grip(self.grip_initial_width)
            elif self.grasp_force is not None:
                self.hold_aperture()
            delta=command['width']-self.grip_initial_width
            self.grip_direction='opening' if delta>1e-6 else 'closing' if delta< -1e-6 else 'holding'
            self.grip_goal=self.width_to_grip(command['width']);self.grip_speed=command['speed']
        elif 'joint_goal' in command:self.joint_goal=command['joint_goal'].copy();self.joint_speed=command['joint_speed']
        else:
            self.goal=command['goal'].copy();self.rotation_goal=command['rotation'].copy()
            self.speed=command['speed'];self.angular_speed=command['angular_speed']
        self.job=call_id;self.job_op=command['op'];self.job_start=now;self.stable=0
        self.grasp_settling.clear()
        self.execution_timeout_s=command.get('timing',{}).get('timeout_sim_s')
        self.uncertainty_reported=False;self.stop_reason=None;self.stop_details={}

    def hold_aperture(self):
        """Cancel a force-producing close without continuing its old target."""
        width=self.observe()['gripper_width_m']
        if self.grasp_force is not None:
            for destination,value in zip((self.model.actuator_gainprm,self.model.actuator_biasprm,self.model.actuator_ctrlrange,self.model.actuator_forcerange),self.grip_position_parameters):
                destination[self.grip]=value
        self.grasp_force=None;self.grasp_width=None;self.grasp_confirmed=False
        self.grasp_phase=None;self.grasp_force_started=None;self.grasp_contact_width=None
        self.gripper=self.width_to_grip(width);self.grip_goal=self.gripper
        self.data.ctrl[self.grip]=self.gripper

    def request_stop(self,now,reason='stopped_by_agent',details=None):
        if self.stopping:return # Repeated requests must not extend the stop deadline.
        if self.job_op=='gripper':self.hold_aperture()
        self.stop_started_at=now
        self.stopping=True;self.goal=None;self.joint_goal=None;self.rotation_goal=None
        self.tcp_ref=None;self.cartesian_integral[:]=0
        self.grip_goal=self.gripper if self.grasp_force is None else self.width_to_grip(self.grasp_width);self.stable=0
        self.status='stopping'
        self.stop_reason=reason;self.stop_details=details or {}

    def control(self,dt=.02):
        if self.stopping:requested=np.zeros(7)
        elif self.joint_goal is not None:
            delta=self.joint_goal-self.qref
            requested=delta*min(6.,self.joint_speed/max(np.max(abs(delta)),1e-12));self.status='moving joints'
        elif self.goal is not None:
            actual=self.data.site_xpos[self.site];actual_r=self.data.site_xmat[self.site].reshape(3,3)
            if self.tcp_ref is None:self.tcp_ref=actual.copy();self.cartesian_integral[:]=0
            lag=self.tcp_ref-actual
            self.tcp_ref=actual+lag*min(1.,.003/max(np.linalg.norm(lag),1e-12))
            delta=self.goal-self.tcp_ref;self.tcp_ref+=delta*min(1.,self.speed*dt/max(np.linalg.norm(delta),1e-9))
            self.cartesian_integral=np.clip(self.cartesian_integral+(self.tcp_ref-actual)*dt*1.5,-.003,.003)
            self.rotation=advance_rotation(actual_r,self.rotation,.03)
            self.rotation=advance_rotation(self.rotation,self.rotation_goal,self.angular_speed*dt)
            q,error=self.ik.solve(self.tcp_ref+self.cartesian_integral,self.rotation,self.qref,12)
            if error>.002:q,error=self.ik.solve(self.tcp_ref+self.cartesian_integral,self.rotation,self.qref,50)
            requested=6*(q-self.qref)
            requested*=min(1.,JOINT_SPEED_LIMIT/max(np.max(abs(requested)),1e-12))
            self.status='moving' if error<.005 else 'IK limited'
        else:self.tcp_ref=None;requested=np.zeros(7)
        lower,upper=self.ik.limits.T
        requested=np.clip(requested,-np.sqrt(2*JOINT_ACCEL_LIMIT*np.maximum(self.qref-lower,0)),
                           np.sqrt(2*JOINT_ACCEL_LIMIT*np.maximum(upper-self.qref,0)))
        change=requested-self.velocity
        if (self.joint_goal is not None or self.goal is not None) and not self.stopping:
            # One shared progress rate and acceleration scale preserve the
            # joint-space segment, instead of letting short axes finish early
            # and sweeping the arm along an unplanned curved path.
            self.velocity+=change*min(1.,JOINT_ACCEL_LIMIT*dt/max(np.max(abs(change)),1e-12))
        else:self.velocity+=np.clip(change,-JOINT_ACCEL_LIMIT*dt,JOINT_ACCEL_LIMIT*dt)
        self.qref=np.clip(self.qref+self.velocity*dt,*self.ik.limits.T)
        actualq=self.data.qpos[self.qids]
        if np.max(np.abs(self.qref-actualq))>TRACKING_LIMIT:
            self.qref=np.clip(self.qref,actualq-TRACKING_LIMIT,actualq+TRACKING_LIMIT);self.velocity[:]=0;self.status='tracking limited'
        # Robot gravity/Coriolis feedforward through the same bounded position
        # actuators; no payload attachment or object-state lookup.
        bias=self.data.qfrc_bias[self.dofs]/self.model.actuator_gainprm[self.act,0]
        velocity_ff=-self.model.actuator_biasprm[self.act,2]*self.velocity/self.model.actuator_gainprm[self.act,0]
        self.data.ctrl[self.act]=np.clip(self.qref+bias+velocity_ff,self.model.actuator_ctrlrange[self.act,0],self.model.actuator_ctrlrange[self.act,1])
        if self.grasp_force is not None:self.physics_control()
        else:
            change=255*dt*self.grip_speed/self.grip_stroke
            self.gripper+=float(np.clip(self.grip_goal-self.gripper,-change,change));self.data.ctrl[self.grip]=self.gripper

    def physics_control(self):
        """Fast hand drive, using encoders and robot parameters only.

        No passive inertia/damping, material or force limit is increased. The
        same bounded actuator carries both speed approach and pressure hold.
        """
        if self.grasp_force is None:return
        fingers=[self.data.joint(self.name+'_'+s) for s in ('finger_joint1','finger_joint2')]
        width=self.grip_min+sum(float(j.qpos[0]) for j in fingers)
        if self.grasp_phase in ('force','hold') and width<self.grasp_width-self.grasp_epsilon[0] and min(float(j.qvel[0]) for j in fingers)<-.05:
            self.grasp_phase='approach';self.grasp_confirmed=False;self.grasp_drive_force=0.;self.grasp_force_started=None;self.grasp_contact_width=None;self.grasp_settling.clear()
        if self.grasp_phase=='approach':
            damping=min(2*self.grasp_force/self.grip_speed,self.grasp_motor_mass/self.model.opt.timestep)
            self.model.actuator_biasprm[self.grip,2]=-damping
            self.data.ctrl[self.grip]=-(damping+self.grasp_passive_damping)*self.grip_speed/2
            self.grasp_drive_force=0.
        else:
            self.model.actuator_biasprm[self.grip,2]=self.grasp_hold_velocity_bias
            ceiling=min(-self.model.actuator_forcerange[self.grip,0],self.model.actuator_forcerange[self.grip,1])
            target=min(self.grasp_force,ceiling-1e-6)
            self.grasp_drive_force=min(target,max(0.,float(self.data.time)-self.grasp_force_started)*self.grasp_force/.4)
            self.data.ctrl[self.grip]=-self.grasp_drive_force

    def finger_stationary(self,now):
        positions=np.array([float(self.data.joint(self.name+'_'+s).qpos[0]) for s in ('finger_joint1','finger_joint2')])
        if not self.grasp_settling or now>self.grasp_settling[-1][0]+1e-9:self.grasp_settling.append((now,positions))
        while len(self.grasp_settling)>1 and now-self.grasp_settling[1][0]>=.18:self.grasp_settling.popleft()
        elapsed=now-self.grasp_settling[0][0];span=np.ptp(np.array([x[1] for x in self.grasp_settling]),axis=0);drift=abs(positions-self.grasp_settling[0][1])/max(elapsed,1e-9)
        return bool(elapsed>=.18-1e-9 and max(span)<.00015 and max(drift)<.001)

    def poll(self,now,timeout):
        if not self.job:return None
        if self.uncertainty_reported:return None
        age=now-self.job_start;feedback=self.observe();details={}
        fingers_settled=self.finger_stationary(now) if self.job_op=='gripper' or self.stopping else False
        if self.job_op=='gripper' and not self.stopping:
            fingers=[float(self.data.joint(self.name+'_'+s).qpos[0]) for s in ('finger_joint1','finger_joint2')]
            if any(q<-.001 or q>.041 for q in fingers):
                self.request_stop(now,'Finger joint outside configured range');return None
        if self.stopping:
            reached=(np.max(np.abs(self.velocity))<1e-5 and (fingers_settled if self.job_op=='gripper' else np.max(np.abs(self.data.qvel[self.dofs]))<.03))
            reason=self.stop_reason;details=self.stop_details
        elif self.job_op=='move':
            if self.joint_goal is not None:reached=np.max(np.abs(self.data.qpos[self.qids]-self.joint_goal))<.004
            else:
                reached=(np.linalg.norm(self.goal-self.data.site_xpos[self.site])<.0025 and
                         np.linalg.norm(rotation_vector(self.rotation_goal,self.data.site_xmat[self.site].reshape(3,3)))<.02)
            reason=None
        elif self.grasp_force is not None:
            width=feedback['gripper_width_m']
            # A contact solver can produce alternating sub-step velocities while
            # each finger position remains stable within a few micrometres.
            # Check both fingers independently over a time window, so opposing
            # motion cannot cancel in the summed width. This is drive settling,
            # never evidence that an object is retained or supported.
            stationary=fingers_settled
            valid=width>self.grip_min+.001 and self.grasp_width-self.grasp_epsilon[0]<width<self.grasp_width+self.grasp_epsilon[1]
            if stationary and valid and self.grasp_phase=='approach':
                self.grasp_phase='force';self.grasp_force_started=now;self.grasp_contact_width=width;self.grasp_settling.clear();self.stable=0
                stationary=False
            reached=stationary and (not valid or self.grasp_drive_force>=self.grasp_force-1e-3)
            reason=None if valid else 'Grasp stopped outside expected object-width tolerance'
            details={'target_width_m':self.grasp_width,'actual_width_m':width,'force_setpoint_n':self.grasp_force,'drive_phase':self.grasp_phase,
                     'is_grasped':valid and stationary,'grasp_verified':False,'measured_grasp_force_n':None,
                     'grasp_state_source':'drive width/settling inference, not verified payload support'}
        else:
            width=feedback['gripper_width_m'];target=self.grip_to_width(self.grip_goal)
            reached=(abs(self.gripper-self.grip_goal)<1e-5 and fingers_settled)
            details={'target_width_m':target,'actual_width_m':width,'target_width_reached':abs(width-target)<.002,
                     'initial_width_m':self.grip_initial_width,'direction':self.grip_direction,'grasp_verified':False}
            if self.grip_direction=='closing':
                valid=target-.002<=width<=self.grip_initial_width+.002
                reason=None if valid else 'Closing aperture tracking error'
            else:
                reason=None if abs(width-target)<.002 else ('Requested opening not reached' if width<target else 'Requested aperture exceeded')
        self.stable=self.stable+1 if reached else 0
        if self.stable>=(3 if self.stopping else 10 if self.job_op=='gripper' else 12):
            return self.end('failed' if reason else 'completed',reason,details)
        expired=(now-self.stop_started_at>3.) if self.stopping else age>(self.execution_timeout_s or timeout)
        if expired:
            if self.stopping:
                self.uncertainty_reported=True
                return {'call_id':self.job,'status':'uncertain','reason':'Stop did not settle; engineering review required',
                        'stop_reason':self.stop_reason,**details}
            self.request_stop(now,'Action timeout; stopped at current reference',details)
        return None

    def end(self,status,reason=None,details=None):
        result={'call_id':self.job,'status':status,'reason':reason,**(details or {})}
        if self.job_op=='gripper' and self.gripper_command_mode=='grasp' and status!='completed':
            result.update(is_grasped=False,grasp_verified=False,measured_grasp_force_n=None)
        if self.job_op=='gripper' and self.grasp_force is not None:
            if status=='completed':self.grasp_confirmed=True;self.grasp_phase='hold'
            else:self.hold_aperture()
        elif self.job_op=='gripper' and status!='completed':self.hold_aperture()
        if self.job_op=='gripper' and self.gripper_command_mode=='move' and self.hand_position_servo=='pd1000-v1' and self.grip_direction=='closing' and status=='completed' and not (details or {}).get('target_width_reached',False):self.hold_aperture()
        self.job=None;self.goal=None;self.joint_goal=None;self.rotation_goal=None
        self.grip_goal=self.gripper if self.grasp_force is None else self.width_to_grip(self.grasp_width)
        self.tcp_ref=None;self.cartesian_integral[:]=0;self.stopping=False;self.status='holding'
        # Preserve bounded braking rather than discontinuously zeroing velocity.
        self.rotation=self.data.site_xmat[self.site].reshape(3,3).copy()
        return result
