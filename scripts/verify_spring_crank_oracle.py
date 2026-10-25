"""Engineering-only contact test for four fixed-dial crank handles; no LLM, no hinge writes."""
import argparse,json,math,subprocess
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from robofrontier.robot_control import PandaArm,matrix_quat,rotation_vector
from robofrontier.task_config import load_harness
from robofrontier.scene import ROOT
from scripts.build_spring_crank_dial import DEFAULT,CRANK


def rz(deg):
    t=math.radians(deg);c,s=math.cos(t),math.sin(t)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def axis_rotation(axis,angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis);x,y,z=axis;c,s,v=math.cos(angle),math.sin(angle),1-math.cos(angle)
    return np.array([[c+x*x*v,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,c+y*y*v,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,c+z*z*v]])

def grasp_R(side):
    sign=1 if side=='south' else -1
    return np.column_stack(([0,0,sign],[1,0,0],[0,sign,0]))

class Oracle:
    def __init__(self,out):
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=False);self.frames=self.out/'frames';self.frames.mkdir()
        self.model=mujoco.MjModel.from_xml_path(str(DEFAULT/'scene.xml'));self.data=mujoco.MjData(self.model);mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id);mujoco.mj_forward(self.model,self.data)
        self.arms={a:PandaArm(self.model,self.data,a) for a in 'ABCD'};self.cfg=load_harness();self.actions=[];self.collisions=[];self.times=[];self.qpos=[];self.index=0;self.renderer=mujoco.Renderer(self.model,height=500,width=800);self.video_every=round(.2/self.model.opt.timestep)
        self.centers={a:self.data.body('dial_'+a).xpos.copy() for a in 'ABCD'}
        self.hingeq={a:self.model.jnt_qposadr[self.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'}
        self.hinged={a:self.model.jnt_dofadr[self.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'}
        self.handles={a:self.model.geom('dial_'+a+'_hand_grip').id for a in 'ABCD'}
        self.grasp_evidence={a:set() for a in 'ABCD'};self.grip_offsets_local={};self.monitor_grips={a:False for a in 'ABCD'};self.grip_gap={a:0. for a in 'ABCD'};self.max_grip_gap={a:0. for a in 'ABCD'}
    @property
    def t(self):return float(self.data.time)
    def step(self):
        if self.index%10==0:
            for arm in self.arms.values():arm.control(.02)
        mujoco.mj_step(self.model,self.data);self.index+=1
        for a in 'ABCD':
            if self.monitor_grips[a]:
                if len(self.fingers(a))<2:self.grip_gap[a]+=self.model.opt.timestep
                else:self.grip_gap[a]=0.
                self.max_grip_gap[a]=max(self.max_grip_gap[a],self.grip_gap[a])
        for c in self.data.contact:
            if c.dist>0:continue
            b=[self.model.body(self.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
            owner=[x.split('_')[0] for x in b]
            if all(x in self.arms for x in owner) and owner[0]!=owner[1]:self.collisions.append(dict(time_s=self.t,bodies=b,penetration_m=float(-c.dist)))
        if self.index%self.video_every==0:
            self.renderer.update_scene(self.data,camera='global');Image.fromarray(self.renderer.render()).save(self.frames/f'frame_{len(self.times):05d}.png');self.qpos.append(self.data.qpos.copy());self.times.append(self.t)
    def wait(self,seconds):
        for _ in range(round(seconds/self.model.opt.timestep)):self.step()
    def fingers(self,a):
        found=set()
        for c in self.data.contact:
            for fg,og in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
                body=self.model.body(self.model.geom_bodyid[fg]).name;other=self.model.geom(og).name
                if body in (a+'_left_finger',a+'_right_finger') and other==f'dial_{a}_hand_grip' and c.dist<=.0005:found.add(body)
        return found
    def action(self,a,label,op,params):
        command=self.arms[a].validate(op,params,self.cfg,np.array([-.95,-1.05,.425]),np.array([.95,1.05,1.30]));call=f'{a}-{label}-{len(self.actions):03d}';self.arms[a].start(call,command,self.t);start=self.t
        for _ in range(round(75/self.model.opt.timestep)):
            self.step()
            if self.index%10:continue
            result=self.arms[a].poll(self.t,15)
            if result:
                rec=dict(robot=a,label=label,started_sim_s=start,finished_sim_s=self.t,command=params,result=result,angle_deg=math.degrees(self.data.qpos[self.hingeq[a]]));self.actions.append(rec)
                if result.get('status')!='completed':raise RuntimeError(f'{a}/{label}: {result}')
                return rec
        raise RuntimeError(f'{a}/{label}: action timeout')
    def batch_moves(self,label,commands):
        active=set(commands);start=self.t;results={}
        for a,p in commands.items():
            c=self.arms[a].validate('move',p,self.cfg,np.array([-.95,-1.05,.425]),np.array([.95,1.05,1.30]));self.arms[a].start(f'{a}-{label}-{len(self.actions):03d}',c,self.t)
        for _ in range(round(75/self.model.opt.timestep)):
            self.step()
            if self.index%10:continue
            for a in list(active):
                r=self.arms[a].poll(self.t,15)
                if r:
                    active.remove(a);results[a]=r;rec=dict(robot=a,label=label,started_sim_s=start,finished_sim_s=self.t,command=commands[a],result=r,angle_deg=math.degrees(self.data.qpos[self.hingeq[a]]));self.actions.append(rec)
                    if r.get('status')!='completed':raise RuntimeError(f'{a}/{label}: {r}')
            if not active:return results
        raise RuntimeError(f'{label}: unresolved arms {sorted(active)}')
    def tcp(self,a):return self.data.site_xpos[self.arms[a].site].copy()
    def handle(self,a):return self.data.geom(self.handles[a]).xpos.copy()
    def moveparams(self,a,pos,rot,speed=.05,omega=.5):return dict(target_m=np.asarray(pos).tolist(),quaternion_wxyz=matrix_quat(rot).tolist(),speed_m_s=speed,angular_speed_rad_s=omega)
    def snapshot(self,label):
        self.renderer.update_scene(self.data,camera='global');Image.fromarray(self.renderer.render()).save(self.out/f'{label}.png')
    def preflight_grasp(self,a):
        center=self.centers[a];offset=np.array([0.,CRANK['radius_m']-.006,.039]);grip=center+offset
        approach=grip+np.array([0.,0.,.075]);seed=self.arms[a].qref.copy()
        homeR=self.data.site_xmat[self.arms[a].site].reshape(3,3).copy();axis=homeR[:,2]
        for twist in range(0,360,30):
            R=axis_rotation(axis,math.radians(twist))@homeR;q=seed.copy();waypoints=[(approach,R),(grip,R)]
            # Screen the whole physical dial arc from the observed starting angle.
            angles=list(range(15,91,15))
            for deg in angles:waypoints.append((center+rz(-deg)@offset,R))
            for point,rot in waypoints:
                q,error=self.arms[a].ik.solve(point,rot,q,250);_,actual=self.arms[a].ik.pose(q)
                if error>.0025 or np.linalg.norm(rotation_vector(rot,actual))>.02:break
            else:return dict(rotation=R,twist_deg=twist,approach_m=approach.tolist(),arc_samples=len(angles))
        raise RuntimeError(f'{a}: no overhead grasp orientation has a kinematically reachable crank arc')
    def grasp_one(self,a,plan):
        R=plan['rotation'];grip=self.centers[a]+np.array([0.,CRANK['radius_m']-.006,.039]);pre=np.asarray(plan['approach_m'])
        self.action(a,'overhead-approach','move',self.moveparams(a,pre,R,.06,.5))
        grip=self.data.geom(self.handles[a]).xpos.copy()
        self.action(a,'overhead-descend','move',self.moveparams(a,grip,R,.025,.3))
        result=self.action(a,'grasp','gripper',dict(mode='grasp',width_m=.022,force_n=50.,speed_m_s=.025,epsilon_inner_m=.006,epsilon_outer_m=.006))
        self.wait(.3);self.grasp_evidence[a]=self.fingers(a)
        if len(self.grasp_evidence[a])<2:raise RuntimeError(f'{a}: handle did not contact both fingers; evidence={self.grasp_evidence[a]}')
        result['two_finger_contact']=sorted(self.grasp_evidence[a])
        rotor=self.data.xmat[self.model.body(f'dial_{a}').id].reshape(3,3);self.grip_offsets_local[a]=rotor.T@(self.tcp(a)-self.handle(a))
        self.monitor_grips[a]=True;return plan
    def turn_batch(self,target_deg,Rs):
        for segment in range(18):
            commands={}
            for a in 'ABCD':
                target=math.radians(target_deg);now=self.data.qpos[self.hingeq[a]];err=(target-now+math.pi)%(2*math.pi)-math.pi
                if abs(math.degrees(err))<2.0:continue
                step=float(np.clip(err,-math.radians(15),math.radians(15)))
                next_rad=now+step;rot=rz(-math.degrees(next_rad));target_pos=self.centers[a]+rot@np.array([0.,CRANK['radius_m']-.006,.0435])+rot@self.grip_offsets_local[a]
                rr=rot@Rs[a]
                commands[a]=self.moveparams(a,target_pos,rr,.025,.25)
            if not commands:return
            self.batch_moves(f'turn-{target_deg}-{segment+1}',commands);self.wait(.15)
        raise RuntimeError(f'Failed to settle at {target_deg}; angles={[math.degrees(self.data.qpos[self.hingeq[a]]) for a in "ABCD"]}')
    def encode(self):
        path=self.out/'oracle-crank-four-dials.mp4'
        subprocess.run(['ffmpeg','-y','-v','error','-framerate','5','-i',str(self.frames/'frame_%05d.png'),'-c:v','libx264','-preset','veryfast','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],check=True)
        return path

def run(out):
    o=Oracle(out);report={'llm_used':False,'direct_hinge_writes_during_trial':False,'targets':[]}
    try:
        o.wait(.2);o.snapshot('00-initial')
        plans={a:o.preflight_grasp(a) for a in 'ABCD'};report['ik_preflight']={a:{'twist_deg':p['twist_deg'],'arc_samples':p['arc_samples']} for a,p in plans.items()}
        for a in 'ABCD':o.grasp_one(a,plans[a])
        for target in (90,):
            o.turn_batch(target,{a:plans[a]['rotation'] for a in 'ABCD'})
            angles=[math.degrees(o.data.qpos[o.hingeq[a]]) for a in 'ABCD'];row=dict(target_deg=target,angles_deg=angles,errors_deg=[((x-target+180)%360)-180 for x in angles]);report['targets'].append(row);o.snapshot(f'target-{target}')
            if max(abs(x) for x in row['errors_deg'])>4:raise RuntimeError(f'Angle tolerance exceeded: {row}')
        report['rotation_passed']=all(max(abs(x) for x in row['errors_deg'])<=4 for row in report['targets'])
        report['handle_finger_contacts']={a:sorted(o.grasp_evidence[a]) for a in 'ABCD'}
        report['max_finger_contact_gap_s']=o.max_grip_gap
        report['inter_robot_contacts']=o.collisions
        o.snapshot('all-four-held-at-90')
        report['release_passed']=False
        try:
            for a in 'ABCD':
                o.monitor_grips[a]=False
                o.action(a,'release','gripper',dict(mode='move',width_m=.08,speed_m_s=.04))
            report['release_passed']=True;o.wait(2);returned=[math.degrees(o.data.qpos[o.hingeq[a]]) for a in 'ABCD'];report['spring_return_angles_deg']=returned
        except Exception as release_error:
            report['release_error']=type(release_error).__name__+': '+str(release_error)
        report['passed']=bool(report['rotation_passed'] and not o.collisions and max(o.max_grip_gap.values())<=.02 and all(len(o.grasp_evidence[a])==2 for a in 'ABCD'))
        o.snapshot('final')
    except Exception as e:report.update(passed=False,error=type(e).__name__+': '+str(e))
    finally:
        report.update(actions=o.actions,action_count=len(o.actions),sim_time_s=o.t,inter_robot_contacts=o.collisions)
        o.renderer.close();np.savez_compressed(o.out/'frame-states.npz',qpos=np.array(o.qpos),time_s=np.array(o.times));report['video']=str(o.encode());(o.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='actions'},indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'runs/spring-crank-oracle-20261023/r1');args=p.parse_args()
    if not run(args.output)['passed']:raise SystemExit(1)
