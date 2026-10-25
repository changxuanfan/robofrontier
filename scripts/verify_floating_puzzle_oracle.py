"""Engineering-only eight-arm contact test. Oracle geometry/angles never become policy inputs."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import mujoco
import numpy as np
from PIL import Image

from robofrontier.robot_control import PandaArm, matrix_quat, rotation_vector
from robofrontier.task_config import load_harness
from robofrontier.floating_dial_task import FloatingState, make_task
from robofrontier.spring_dial import DialFeedback, update_display
from scripts.build_floating_dial_puzzle import DIRECTORY
from scripts.verify_floating_dial_oracle import axis_rotation, knob_grasp_rotation


class Trial:
    def __init__(self,output,scene_directory=DIRECTORY):
        self.directory=Path(output);self.directory.mkdir(parents=True,exist_ok=False)
        scene_directory=Path(scene_directory).resolve();self.scene_directory=scene_directory
        self.model=mujoco.MjModel.from_xml_path(str(scene_directory/'scene.xml'));self.data=mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('initial').id);mujoco.mj_forward(self.model,self.data)
        self.arms={a:PandaArm(self.model,self.data,a) for a in 'ABCDEFGH'};self.cfg=load_harness();self.task=make_task(scene_directory)
        self.app=FloatingState(self);self.tick=0;self.actions=[];self.robot_contacts={};self.environment_contacts={};self.min_clear=[math.inf]*4;self.monitor=False;self.maximum_handle_gap=[0.]*4;self.gaps=[0.]*4
        self.qids=[self.model.jnt_qposadr[self.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'];self.dofs=[self.model.jnt_dofadr[self.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
        self.game=DialFeedback(self.task['feedback'],secret=[90]*4);self.states=[];self.times=[];self.last_capture=-10
        self.renderer=mujoco.Renderer(self.model,height=600,width=1280)
        self.encoder=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x600','-r','4','-i','-','-c:v','libx264','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+frag_keyframe+empty_moov',str(self.directory/'oracle-eight-arm.mp4')],stdin=subprocess.PIPE)
    @property
    def time(self):return float(self.data.time)
    def contacts(self,arm,prefix):
        result=set()
        for c in self.data.contact:
            for fg,og in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
                b=self.model.body(self.model.geom_bodyid[fg]).name
                if b in (arm+'_left_finger',arm+'_right_finger') and (self.model.geom(og).name or '').startswith(prefix) and c.dist<=.0002:result.add(b)
        return result
    def step(self):
        if self.tick%20==0:
            for arm in self.arms.values():arm.control(.02)
        mujoco.mj_step(self.model,self.data);self.tick+=1
        eligible=self.app.sample(self.data.qpos[self.qids],audit=not self.game.won);feedback=self.game.update(self.time,self.data.qpos[self.qids],self.data.qvel[self.dofs],eligible=eligible);update_display(self.model,feedback)
        for c in self.data.contact:
            b=[self.model.body(self.model.geom_bodyid[g]).name for g in (c.geom1,c.geom2)]
            owners=[name.split('_')[0] if name and name.split('_')[0] in self.arms else None for name in b]
            if all(owners) and owners[0]!=owners[1] and c.dist<=0:
                key=' / '.join(sorted(b));self.robot_contacts[key]=max(self.robot_contacts.get(key,0),float(-c.dist))
            if any(owners) and not all(owners) and c.dist<-.0002:
                other=c.geom2 if owners[0] else c.geom1;name=self.model.geom(other).name or ''
                if name.startswith(('table_','score_')) or name=='floor':
                    key=' / '.join([str(self.model.geom(c.geom1).name),str(self.model.geom(c.geom2).name)])
                    self.environment_contacts[key]=max(self.environment_contacts.get(key,0),float(-c.dist))
        if self.monitor:
            for i,a in enumerate('ABCD'):
                self.min_clear[i]=min(self.min_clear[i],self.app.snapshot['clearance_m'][i])
                if len(self.contacts(chr(66+2*i),'dial_'+a+'_module_stem'))<2:self.gaps[i]+=.001
                else:self.gaps[i]=0
                self.maximum_handle_gap[i]=max(self.maximum_handle_gap[i],self.gaps[i])
        if self.time-self.last_capture>=.25-1e-8:
            self.last_capture=self.time;self.renderer.update_scene(self.data,camera='global');self.encoder.stdin.write(self.renderer.render().tobytes());self.states.append(self.data.qpos.copy());self.times.append(self.time)
    def wait(self,s):
        for _ in range(round(s/.001)):self.step()
    def execute(self,label,commands):
        pending=set(commands);result={}
        for a,(op,p) in commands.items():
            c=self.arms[a].validate(op,p,self.cfg,np.array(self.task['workspace_world_m']['lower']),np.array(self.task['workspace_world_m']['upper']));self.arms[a].start(label+'-'+a,c,self.time)
        start=self.time
        while pending and self.time-start<120:
            self.step()
            if self.tick%20:continue
            for a in list(pending):
                r=self.arms[a].poll(self.time,15)
                if r:result[a]=r;pending.remove(a)
        record=dict(label=label,time_s=self.time,commands=commands,results=result,angles_deg=np.degrees(self.data.qpos[self.qids]).tolist(),clearances_m=self.app.snapshot['clearance_m'])
        self.actions.append(record);(self.directory/'actions.json').write_text(json.dumps(self.actions,indent=2))
        print(json.dumps({k:record[k] for k in ('label','time_s','angles_deg','clearances_m')}),flush=True)
        if pending or any(r['status']!='completed' for r in result.values()):raise RuntimeError(label+': '+str(result))
    def move(self,p,r,s=.05,omega=.4):return ('move',dict(target_m=np.asarray(p).tolist(),quaternion_wxyz=matrix_quat(r).tolist(),speed_m_s=s,angular_speed_rad_s=omega))
    def snapshot(self,name):
        for camera in ('global','dial_A','dial_A_side','board'):
            self.renderer.update_scene(self.data,camera=camera);Image.fromarray(self.renderer.render()).save(self.directory/(name+'-'+camera+'.png'))
    def close(self):
        self.encoder.stdin.close();self.encoder.wait();self.renderer.close();np.savez_compressed(self.directory/'frame-states.npz',qpos=np.array(self.states),time_s=self.times)


def run(output,scene_directory=DIRECTORY,support_strategy='roll'):
    t=Trial(output,scene_directory);report=dict(llm_used=False,direct_module_or_dial_writes_during_trial=False,passed=False,support_strategy=support_strategy)
    try:
        t.wait(.25);t.snapshot('00-initial');targets={};support_r={}
        for i,a in enumerate('ABCD'):
            b=chr(66+2*i);g=t.data.geom('dial_'+a+'_module_stem');targets[b]=g.xpos.copy()+g.xmat.reshape(3,3)@np.array([0,0,.07]);support_r[b]=t.data.site_xmat[t.arms[b].site].reshape(3,3).copy()
        t.execute('handle-approach',{b:t.move(p+np.array([0,0,.10]),support_r[b]) for b,p in targets.items()})
        t.execute('handle-descend',{b:t.move(p,support_r[b],omega=.25) for b,p in targets.items()})
        t.execute('handle-grasp',{b:('gripper',dict(mode='grasp',width_m=.016,force_n=70.,speed_m_s=.02,epsilon_inner_m=.005,epsilon_outer_m=.005)) for b in targets})
        t.wait(.35)
        for i,a in enumerate('ABCD'):
            if len(t.contacts(chr(66+2*i),'dial_'+a+'_module_stem'))<2:raise RuntimeError('Two-finger support missing '+a)
        lifts={b:np.array(t.arms[b].observe()['tcp_m'])+np.array([0,0,.18]) for b in targets}
        t.execute('lift',{b:t.move(p,support_r[b],omega=.25) for b,p in lifts.items()});t.wait(.35);t.monitor=True
        if support_strategy=='roll':
            t.execute('support-roll',{b:t.move(lifts[b]+np.array([0,0,.10]),axis_rotation([0,1,0],math.pi/2)@t.data.site_xmat[t.arms[b].site].reshape(3,3),.06,.6) for b in targets})
        t.wait(.5);t.snapshot('01-airborne')
        twists={};pre={};rot={}
        for i,a in enumerate('ABCD'):
            arm=chr(65+2*i);center=t.data.geom('dial_'+a+'_red_knob').xpos.copy();mr=t.data.body('dial_'+a+'_module').xmat.reshape(3,3).copy()
            # Select a seed with the required signed half-turn range, not just an endpoint.
            for deg in range(0,360,45):
                r=knob_grasp_rotation(mr,math.radians(deg));q=t.arms[arm].qref.copy()
                for pos,rr in [(center-r[:,2]*.08,r),(center,r)]+[(center,axis_rotation(-mr[:,2],math.radians(d))@r) for d in (30,60,90,120,150,175)]:
                    q,error=t.arms[arm].ik.solve(pos,rr,q,250);_,actual=t.arms[arm].ik.pose(q)
                    if error>.0025 or np.linalg.norm(rotation_vector(rr,actual))>.025:break
                else:twists[a]=deg;rot[arm]=r;pre[arm]=center-r[:,2]*.08;break
            if a not in twists:raise RuntimeError('No half-turn IK seed '+a)
        t.execute('knob-approach',{arm:t.move(pre[arm],r,.06,.4) for arm,r in rot.items()})
        t.execute('knob-prealign',{chr(65+2*i):t.move(t.data.geom('dial_'+a+'_red_knob').xpos-rot[chr(65+2*i)][:,2]*.08,rot[chr(65+2*i)]) for i,a in enumerate('ABCD')})
        t.execute('knob-contact',{chr(65+2*i):t.move(t.data.geom('dial_'+a+'_red_knob').xpos,rot[chr(65+2*i)],.08,.35) for i,a in enumerate('ABCD')})
        t.execute('knob-grasp',{arm:('gripper',dict(mode='grasp',width_m=.056,force_n=70.,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008)) for arm in rot});t.wait(.35)
        for i,a in enumerate('ABCD'):
            if len(t.contacts(chr(65+2*i),'dial_'+a+'_red_knob'))<2:raise RuntimeError('Two-finger knob contact missing '+a)
        report['twists_deg']=twists;report['targets']=[]
        # Test symbol 1 then 2, return to zero physically, then symbol 3.
        for target in (90,177,0,-90):
            for segment in range(14):
                commands={}
                for i,a in enumerate('ABCD'):
                    error=math.radians(target)-t.data.qpos[t.qids[i]]
                    if abs(math.degrees(error))<=2.3:continue
                    arm=chr(65+2*i);axis=t.data.xaxis[t.model.joint('dial_'+a+'_hinge').id].copy();r=t.data.site_xmat[t.arms[arm].site].reshape(3,3).copy();delta=float(np.clip(error,-math.radians(25),math.radians(25)))
                    commands[arm]=t.move(t.data.geom('dial_'+a+'_red_knob').xpos,axis_rotation(axis,delta)@r,.025,.25)
                if not commands:break
                t.execute(f'turn-{target}-{segment}',commands);t.wait(.3)
            t.wait(.6);angles=np.degrees(t.data.qpos[t.qids]).tolist();report['targets'].append(dict(target_deg=target,angles_deg=angles,clearance_m=t.app.snapshot['clearance_m']))
            t.snapshot('target-'+str(target))
            if any(abs(x-target)>4 for x in angles):raise RuntimeError('Angle target failed '+str(angles))
        t.execute('release-knobs',{arm:('gripper',dict(mode='move',width_m=.08,speed_m_s=.04)) for arm in rot});t.wait(2)
        report.update(spring_return_angles_deg=np.degrees(t.data.qpos[t.qids]).tolist(),minimum_clearance_m=t.min_clear,maximum_handle_contact_gap_s=t.maximum_handle_gap,inter_robot_contacts=t.robot_contacts)
        report['passed']=not t.app.contact_episodes and all(x>=.05 for x in t.min_clear) and not t.robot_contacts and not t.environment_contacts and all(abs(x)<=4 for x in report['spring_return_angles_deg']) and max(t.maximum_handle_gap)<=.02
        t.snapshot('final')
    except Exception as e:report['error']=type(e).__name__+': '+str(e)
    finally:
        report.update(action_count=len(t.actions),sim_time_s=t.time,inter_robot_contacts=t.robot_contacts,minimum_clearance_m=t.min_clear,premature_knob_contacts=t.app.contact_episodes,scene_directory=str(t.scene_directory),arm_environment_penetrations=t.environment_contacts,legal_feedback_win=t.game.won)
        t.close();(t.directory/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--scene-directory',type=Path,default=DIRECTORY);p.add_argument('--support-strategy',choices=('roll','pinch'),default='roll');a=p.parse_args()
    if not run(a.output,a.scene_directory,a.support_strategy)['passed']:raise SystemExit(1)
