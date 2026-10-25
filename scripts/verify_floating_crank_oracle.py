"""Engineering-only physical lifting and eccentric-crank test; never a policy recipe.

Private scene poses/hinge angles guide robot Cartesian commands. No apparatus
pose writes, hinge actuation, welds or grasp attachments occur during the trial.
"""
import argparse,json,math,subprocess
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from scripts.verify_floating_puzzle_oracle import Trial
from scripts.verify_floating_dial_oracle import axis_rotation,knob_grasp_rotation
from scripts.build_floating_crank import DIRECTORY
from robofrontier.robot_control import rotation_vector


class CrankTrial(Trial):
    def __init__(self,output,scene_directory):
        super().__init__(output,scene_directory)
        self.rear=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x600','-r','4','-i','-','-c:v','libx264','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+frag_keyframe+empty_moov',str(self.directory/'oracle-global-rear.mp4')],stdin=subprocess.PIPE)
        self.rotor_monitor=False;self.rotor_gaps=[0.]*4;self.max_rotor_gaps=[0.]*4
    def step(self):
        n=len(self.states);super().step()
        if self.rotor_monitor:
            for i,a in enumerate('ABCD'):
                if len(self.contacts(chr(65+2*i),'dial_'+a+'_hand_grip'))<2:self.rotor_gaps[i]+=.001
                else:self.rotor_gaps[i]=0.
                self.max_rotor_gaps[i]=max(self.max_rotor_gaps[i],self.rotor_gaps[i])
        if len(self.states)!=n:
            self.renderer.update_scene(self.data,camera='global_rear');self.rear.stdin.write(self.renderer.render().tobytes())
    def snapshot(self,name):
        super().snapshot(name)
        self.renderer.update_scene(self.data,camera='global_rear');Image.fromarray(self.renderer.render()).save(self.directory/(name+'-global_rear.png'))
    def close(self):
        self.rear.stdin.close();self.rear.wait();super().close()


def run(output,scene_directory=DIRECTORY):
    t=CrankTrial(output,scene_directory);report=dict(passed=False,llm_used=False,direct_apparatus_writes_during_trial=False,targets=[])
    try:
        t.wait(.25);t.snapshot('00-initial');targets={};support_r={}
        for i,a in enumerate('ABCD'):
            b=chr(66+2*i);g=t.data.geom('dial_'+a+'_module_stem')
            targets[b]=g.xpos.copy()+g.xmat.reshape(3,3)@np.array([0,0,.07])
            support_r[b]=t.data.site_xmat[t.arms[b].site].reshape(3,3).copy()
        t.execute('blue-approach',{b:t.move(p+np.array([0,0,.1]),support_r[b]) for b,p in targets.items()})
        t.execute('blue-descend',{b:t.move(p,support_r[b],omega=.25) for b,p in targets.items()})
        t.execute('blue-grasp',{b:('gripper',dict(mode='grasp',width_m=.016,force_n=70.,speed_m_s=.02,epsilon_inner_m=.005,epsilon_outer_m=.005)) for b in targets})
        t.wait(.35)
        for i,a in enumerate('ABCD'):
            if len(t.contacts(chr(66+2*i),'dial_'+a+'_module_stem'))<2:raise RuntimeError('Blue support missing '+a)
        t.execute('lift',{b:t.move(t.data.site_xpos[t.arms[b].site]+np.array([0,0,.24]),support_r[b],omega=.25) for b in targets})
        t.wait(.35);t.monitor=True;t.snapshot('01-lift')
        # Physically tilt the frame toward face-up to reduce eccentric gravity torque.
        relations={};centers={};angles={}
        for i,a in enumerate('ABCD'):
            b=chr(66+2*i);body=t.data.body('dial_'+a+'_module');mr=body.xmat.reshape(3,3).copy()
            relations[b]=(mr.T@(t.data.site_xpos[t.arms[b].site]-body.xpos),mr.T@t.data.site_xmat[t.arms[b].site].reshape(3,3))
            centers[b]=body.xpos.copy();angles[b]=math.atan2(mr[2,1],mr[1,1])
        for k in range(1,9):
            commands={}
            for b,(offset,relative_r) in relations.items():
                fraction=k/8;mr=axis_rotation([1,0,0],angles[b]*(1-fraction)+math.radians(20)*fraction);center=centers[b].copy();center[2]=(1-fraction)*center[2]+fraction*.75
                commands[b]=t.move(center+mr@offset,mr@relative_r,.045,.3)
            t.execute('support-face-up-'+str(k),commands)
        t.wait(.5);t.snapshot('02-face-up')
        report['supported_module_pose']={a:dict(position=t.data.body('dial_'+a+'_module').xpos.tolist(),rotation=t.data.body('dial_'+a+'_module').xmat.tolist()) for a in 'ABCD'}
        # Grasp candidates must support the whole signed semicircle, not just endpoints.
        plans={}
        for i,a in enumerate('ABCD'):
            arm=chr(65+2*i);mr=t.data.body('dial_'+a+'_module').xmat.reshape(3,3);grip=t.data.geom('dial_'+a+'_hand_grip').xpos.copy()
            anchor=t.data.xanchor[t.model.joint('dial_'+a+'_hinge').id].copy();axis=t.data.xaxis[t.model.joint('dial_'+a+'_hinge').id].copy()
            for twist in (90,270,0,180,45,135,225,315):
                r=knob_grasp_rotation(mr,math.radians(twist));q=t.arms[arm].qref.copy()
                waypoints=[(grip-r[:,2]*.09,r),(grip,r)]
                waypoints += [(anchor+axis_rotation(axis,math.radians(d))@(grip-anchor),axis_rotation(axis,math.radians(d))@r) for d in range(15,178,15)]
                for p,rr in waypoints:
                    q,error=t.arms[arm].ik.solve(p,rr,q,250);_,actual=t.arms[arm].ik.pose(q)
                    if error>.0025 or np.linalg.norm(rotation_vector(rr,actual))>.025:break
                else:plans[a]=dict(arm=arm,r=r,twist_deg=twist);break
            if a not in plans:raise RuntimeError('No full-arc IK grasp '+a)
        report['grasp_twists_deg']={a:p['twist_deg'] for a,p in plans.items()}
        t.execute('crank-approach',{p['arm']:t.move(t.data.geom('dial_'+a+'_hand_grip').xpos-p['r'][:,2]*.09,p['r'],.05,.4) for a,p in plans.items()})
        t.execute('crank-contact',{p['arm']:t.move(t.data.geom('dial_'+a+'_hand_grip').xpos,p['r'],.025,.25) for a,p in plans.items()})
        t.execute('crank-grasp',{p['arm']:('gripper',dict(mode='grasp',width_m=.022,force_n=50.,speed_m_s=.025,epsilon_inner_m=.006,epsilon_outer_m=.006)) for p in plans.values()})
        t.wait(.35)
        for a,p in plans.items():
            if len(t.contacts(p['arm'],'dial_'+a+'_hand_grip'))<2:raise RuntimeError('Two-finger crank grasp missing '+a)
        t.rotor_monitor=True;t.snapshot('03-grasp')
        for target in (90,177,0,-90):
            for segment in range(35):
                commands={}
                for i,a in enumerate('ABCD'):
                    error=math.radians(target)-t.data.qpos[t.qids[i]]
                    if abs(math.degrees(error))<2.:continue
                    arm=plans[a]['arm'];jid=t.model.joint('dial_'+a+'_hinge').id
                    anchor=t.data.xanchor[jid];axis=t.data.xaxis[jid];delta=float(np.clip(error,-math.radians(12),math.radians(12)))
                    dr=axis_rotation(axis,delta);tcp=t.data.site_xpos[t.arms[arm].site];r=t.data.site_xmat[t.arms[arm].site].reshape(3,3)
                    commands[arm]=t.move(anchor+dr@(tcp-anchor),dr@r,.025,.25)
                if not commands:break
                t.execute(f'turn-{target}-{segment}',commands);t.wait(.12)
            t.wait(.5);angles=np.degrees(t.data.qpos[t.qids]).tolist();mark=180 if target==177 else target
            errors=[(x-mark+180)%360-180 for x in angles]
            report['targets'].append(dict(command_target_deg=target,mark_deg=mark,angles_deg=angles,errors_deg=errors))
            t.snapshot('target-'+str(target))
            if max(abs(x) for x in errors)>4:raise RuntimeError('Marked angle failed '+str(angles))
        report['rotation_passed']=True;t.rotor_monitor=False
        # Stop the force-held grasp without claiming the cylinder has cleared.
        # Axial withdrawal releases contact; full opening is verified afterwards.
        t.execute('relax-cranks',{p['arm']:('gripper',dict(mode='move',width_m=t.arms[p['arm']].observe()['gripper_width_m'],speed_m_s=.04)) for p in plans.values()})
        t.execute('withdraw-cranks',{p['arm']:t.move(t.data.site_xpos[t.arms[p['arm']].site]+t.data.body('dial_'+a+'_module').xmat.reshape(3,3)[:,2]*.08,t.data.site_xmat[t.arms[p['arm']].site].reshape(3,3),.04,.25) for a,p in plans.items()})
        t.execute('open-clear',{p['arm']:('gripper',dict(mode='move',width_m=.08,speed_m_s=.04)) for p in plans.values()})
        t.wait(3);returned=np.degrees(t.data.qpos[t.qids]).tolist()
        report['spring_return_angles_deg']=returned
        report['release_passed']=max(abs(x) for x in returned)<=4 and all(not t.contacts(p['arm'],'dial_'+a+'_') for a,p in plans.items())
        report['passed']=bool(report['release_passed'] and not t.robot_contacts and not t.environment_contacts and not t.app.contact_episodes and min(t.min_clear)>=.02 and max(t.maximum_handle_gap)<=.02 and max(t.max_rotor_gaps)<=.02)
        t.snapshot('final')
    except Exception as e:report['error']=type(e).__name__+': '+str(e)
    finally:
        report.update(scene_directory=str(scene_directory),action_count=len(t.actions),sim_time_s=t.time,inter_robot_contacts=t.robot_contacts,arm_environment_penetrations=t.environment_contacts,premature_crank_contacts=t.app.contact_episodes,minimum_clearance_m=t.min_clear,maximum_blue_contact_gap_s=t.maximum_handle_gap,maximum_crank_contact_gap_s=t.max_rotor_gaps,legal_feedback_win=t.game.won)
        t.close();(t.directory/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--scene-directory',type=Path,default=DIRECTORY);a=p.parse_args()
    if not run(a.output,a.scene_directory)['passed']:raise SystemExit(1)
