"""Private actuator reference with a count-only code decoder.

The decoder receives only attempted symbols and aggregate scores. Physical
execution reads exact apparatus pose, as an explicitly privileged Oracle.
"""
from itertools import product
import math
import numpy as np
from ..robot_control import rotation_vector
from .scenes import rz
from scripts.verify_floating_dial_oracle import axis_rotation,knob_grasp_rotation

class Decoder:
    def __init__(self,n):self.candidates=np.array(list(product((1,2,3),repeat=n)),dtype=np.int8);self.seen=set();self.n=n
    def update(self,guess,count):
        key=(tuple(guess),int(count))
        if key in self.seen:return
        self.seen.add(key);self.candidates=self.candidates[np.sum(self.candidates==np.asarray(guess),axis=1)==count]
        if not len(self.candidates):raise RuntimeError('Aggregate feedback contradicts the candidate model')
    def choose(self,current):
        if len(self.candidates)==1:return self.candidates[0].tolist()
        pool=self.candidates[np.linspace(0,len(self.candidates)-1,min(96,len(self.candidates))).astype(int)];best=None
        for guess in pool:
            scores=np.sum(self.candidates==guess,axis=1);counts=np.bincount(scores,minlength=self.n+1);prob=counts[counts>0]/len(scores);entropy=float(-np.sum(prob*np.log2(prob)))
            movement=sum(a!=b for a,b in zip(guess,current));rank=entropy-.015*movement
            if best is None or rank>best[0]:best=(rank,guess.copy())
        return best[1].tolist()

class Manipulator:
    def __init__(self,t):
        self.t=t;self.dials=t.task['dials'];self.grips={};self.supports={};self.decoder=Decoder(len(self.dials))
        self.rest_frames={a['id']:(t.data.body(a['id']+'_module').xpos.copy(),t.data.body(a['id']+'_module').xmat.reshape(3,3).copy()) for a in self.dials}
    @property
    def angle_step_deg(self):
        return self.t.task.get('mechanism',{}).get('feedback',{}).get('step_deg',90.)
    def target_angles(self,symbol):
        angle=(float(symbol)*self.angle_step_deg+180)%360-180
        return [math.radians(179.5),math.radians(-179.5)] if abs(angle+180)<1e-8 else [math.radians(angle)]
    def input_settled(self,symbols):
        return self.t.monitor.game.display is not None
    def arc_joint_clear(self,robot,start,goal):
        return True # Legacy reference; versioned references add sampled checks.
    def execute_turn(self,label,commands):
        return self.t.execute(label,commands)
    def maintain_holds(self,symbols):
        pass
    def update_decoder(self,guess):
        # The physical Oracle may measure apparatus pose. Its code solver gets
        # only its requested input and the currently visible count, never the
        # evaluator's history of unobserved intermediate configurations.
        count=self.t.monitor.game.display
        if count is not None and count!=len(self.dials):self.decoder.update(guess,count)
    def lift(self):
        t=self.t
        if t.task['level']==1:return
        commands={};side_access=bool(t.task['mechanism'].get('parking_fixture'))
        for a in self.dials:
            robot=a['nominal_robots'][1];g=t.data.geom(a['id']+'_module_stem');mr=g.xmat.reshape(3,3).copy();bar_point=g.xpos.copy()+mr@np.array([0,0,.09 if side_access else .10]);point=bar_point.copy();r=t.home[robot][1]
            if side_access:
                selected=False
                for sign in (1,-1):
                    candidate=np.column_stack((sign*mr[:,2],mr[:,1],-sign*mr[:,0]))
                    point=bar_point.copy()
                    try:t.move_free(robot,point-candidate[:,2]*.10,candidate,'blue-side-approach',continuation=(point,candidate,()))
                    except RuntimeError as error:
                        if str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):continue
                        raise
                    r=candidate;selected=True;break
                if not selected:raise RuntimeError('No side-access blue grasp '+a['id'])
            else:commands[robot]=t.pose_command(robot,point+np.array([0,0,.10]),r,.06,.4)
            self.supports[a['id']]=(robot,point,r)
        if commands:t.execute('blue-approach',commands)
        t.execute('blue-contact',{r:t.pose_command(r,p,rot,.04,.3) for r,p,rot in self.supports.values()})
        t.execute('blue-grasp',{r:('gripper',dict(mode='grasp',width_m=.020 if side_access else .016,force_n=70.,speed_m_s=.02,epsilon_inner_m=.005,epsilon_outer_m=.005)) for r,_,_ in self.supports.values()});t.wait(.35)
        for a in self.dials:
            robot=self.supports[a['id']][0]
            if robot not in t.monitor.bilateral([g for g,index in t.monitor.blue.items() if index==a['index']]):raise RuntimeError('Blue support absent '+a['id'])
            t.held[robot]=a['id']+'_module'
        lift=.14 if side_access else .24
        t.execute('lift-all',{r:t.pose_command(r,t.data.site_xpos[t.arms[r].site]+np.array([0,0,lift]),rotation,.05,.3) for r,_,rotation in self.supports.values()});t.wait(.35)
        if side_access:
            t.wait(.3);t.snapshot('airborne')
            if not all(x>=.02 for x in t.monitor.result()['private_clearance_m']):raise RuntimeError('Side grasp did not lift all complete modules')
            return
        # Let a lower finger bear the weight. Incremental physical motion is
        # used here because a frictional grasp can settle relative to the tool;
        # a rigid carried-object prediction is not a contact-dynamics proof.
        frames={}
        for a in self.dials:
            robot=self.supports[a['id']][0];arm=t.arms[robot];frames[robot]=(a,t.data.site_xpos[arm.site].copy(),t.data.site_xmat[arm.site].reshape(3,3).copy())
        for step in range(1,9):
            commands={};f=step/8
            for robot,(a,point,rotation) in frames.items():
                axis=rz(a['layout_yaw_rad'])@np.array([0,1.,0]);r=axis_rotation(axis,math.pi/2*f)@rotation
                commands[robot]=t.pose_command(robot,point+np.array([0,0,.10*f]),r,.045,.3)
            t.execute('support-bearing-'+str(step),commands)
        t.wait(.5);t.snapshot('airborne')
        if not all(x>=.02 for x in t.monitor.result()['private_clearance_m']):raise RuntimeError('Not all complete modules are airborne')
    def red_geoms(self,a):return [g for g,i in self.t.monitor.red.items() if i==a['index']]
    def red_pose(self,a,twist):
        t=self.t;name=a['id'];mr=t.data.body(name+'_module').xmat.reshape(3,3);crank=t.task['level']==3
        point=t.data.geom(name+('_hand_grip' if crank else '_red_knob')).xpos.copy()
        return point,knob_grasp_rotation(mr,math.radians(twist))
    def wait_support(self,entries):
        if self.t.task['level']==1 or not entries:return
        for _ in range(150):
            if all(self.t.monitor.dial_ready[a['index']] for a in entries):return
            self.t.wait(.02)
        raise RuntimeError('Blue grasp did not requalify before red operation: '+','.join(a['id'] for a in entries if not self.t.monitor.dial_ready[a['index']]))
    def present(self,entries):
        if not self.t.task['mechanism'].get('parking_fixture'):return
        t=self.t
        for a in entries:
            if a['id'] in self.grips:raise RuntimeError('Release red control before moving its supporting frame')
            robot=a['nominal_robots'][1];arm=t.arms[robot];wanted,r=self.rest_frames[a['id']];wanted=wanted+np.array([0,0,.14])
            for _ in range(2):
                body=t.data.body(a['id']+'_module');old_r=body.xmat.reshape(3,3).copy()
                if np.linalg.norm(body.xpos-wanted)<.003 and np.linalg.norm(rotation_vector(r,old_r))<.04:break
                offset=old_r.T@(t.data.site_xpos[arm.site]-body.xpos);relative=old_r.T@t.data.site_xmat[arm.site].reshape(3,3)
                t.move_free(robot,wanted+r@offset,r@relative,'present-device');t.wait(.3)
    def release(self,entries):
        t=self.t;entries=[a for a in entries if a['id'] in self.grips]
        if not entries:return
        opening=.060 if t.task['level']==3 else .075
        # Release by opening, not by holding a tightly closed aperture while
        # waiting for a spring-loaded crank to settle against the fingertips.
        result=t.execute('red-release-open',{self.grips[a['id']]['robot']:('gripper',dict(mode='move',width_m=opening,speed_m_s=.10)) for a in entries},allow_failure=True)
        for robot,value in result.items():
            if t.arms[robot].grasp_force is not None or value['status']=='uncertain' or (value['status']=='failed' and value.get('reason') not in ('Requested aperture exceeded','Requested opening not reached')):raise RuntimeError('Red force release unresolved '+robot)
        blocked=[a for a in entries if result[self.grips[a['id']]['robot']]['status']!='completed']
        if blocked:
            # A known aperture failure does not permit dragging a still-captured
            # rotor away from its supporting robot. Try one ordinary wider move
            # within the same robot limits, then require real contact release.
            wider=t.execute('red-release-wide-retry',{self.grips[a['id']]['robot']:('gripper',dict(mode='move',width_m=min(.08,t.arms[self.grips[a['id']]['robot']].grip_max),speed_m_s=.05)) for a in blocked},allow_failure=True)
            for a in blocked:
                robot=self.grips[a['id']]['robot']
                if wider[robot]['status']!='completed' or not t.monitor.released(self.red_geoms(a)):
                    raise RuntimeError('Cannot safely withdraw red capture after aperture failure '+robot)
        commands={}
        for a in entries:
            robot=self.grips[a['id']]['robot'];arm=t.arms[robot];rotation=t.data.site_xmat[arm.site].reshape(3,3).copy();normal=-rotation[:,2]
            for distance in ((.06,.05,.04) if t.task['level']==3 else (.045,.04,.035)):
                point=t.data.site_xpos[arm.site]+normal*distance
                if t.screen_ik(robot,point,rotation) is not None:commands[robot]=t.pose_command(robot,point,rotation,.04,.3);break
            if robot not in commands:raise RuntimeError('No reachable axial red release '+robot)
        t.execute('red-withdraw',commands)
        opened=t.execute('red-open-clear',{self.grips[a['id']]['robot']:('gripper',dict(mode='move',width_m=opening,speed_m_s=.05)) for a in entries},allow_failure=True)
        for a in entries:
            robot=self.grips[a['id']]['robot'];value=opened[robot]
            if value['status']=='completed':continue
            if value['status']=='uncertain' or value.get('reason')!='Requested opening not reached':raise RuntimeError('Unresolved red opening '+str(value))
            enough=(.022 if t.task['level']==3 else .056)+.008
            if t.arms[robot].observe()['gripper_width_m']>=enough and t.monitor.released(self.red_geoms(a)):
                t.event('reference_accept_released_aperture',robot=robot,actual_width_m=t.arms[robot].observe()['gripper_width_m'],scope='Known exact-opening failure remains recorded; the hand is already clear of the rotor')
                continue
            arm=t.arms[robot];normal=t.data.body(a['id']+'_module').xmat.reshape(3,3)[:,2]
            t.execute('red-extra-withdraw',{robot:t.pose_command(robot,t.data.site_xpos[arm.site]+normal*.035,t.data.site_xmat[arm.site].reshape(3,3),.025,.25)})
            t.execute('red-reopen-clear',{robot:('gripper',dict(mode='move',width_m=.08,speed_m_s=.05))})
        for a in entries:
            robot=self.grips[a['id']]['robot']
            if getattr(self,'home_after_release',True):t.move_free(robot,*t.home[robot],label='red-reset-posture')
        for a in entries:self.grips.pop(a['id'])
        t.wait(1.)
        for _ in range(50):
            if all(abs(t.data.joint(a['id']+'_hinge').qpos[0])<math.radians(2) for a in entries):break
            t.wait(.1)
        if any(abs(t.data.joint(a['id']+'_hinge').qpos[0])>math.radians(4) for a in entries):raise RuntimeError('Spring return failed')
    def grasp(self,entries,symbols,retries=2,avoid=None):
        t=self.t;plans={}
        self.present(entries)
        self.wait_support(entries)
        for a in entries:
            robot=a['nominal_robots'][0];name=a['id'];module=t.data.body(name+'_module');mr=module.xmat.reshape(3,3).copy();crank=t.task['level']==3
            center=t.data.geom(name+('_hand_grip' if crank else '_red_knob')).xpos.copy()
            jid=t.model.joint(name+'_hinge').id;anchor=t.data.xanchor[jid].copy();axis=t.data.xaxis[jid].copy();symbol=symbols[a['index']]
            angles=self.target_angles(symbol)
            angles.sort(key=lambda target:abs(target-t.data.joint(name+'_hinge').qpos[0]))
            twists=(0,180,45,225,315,135,270,90) if crank else (270,90,0,180,45,135,225,315)
            for twist,target in product(twists,angles):
                if avoid and avoid.get(name)==twist:continue
                r=knob_grasp_rotation(mr,math.radians(twist));waypoints=[(center-r[:,2]*.10,r),(center,r)]
                travel=target-float(t.data.joint(name+'_hinge').qpos[0])
                for delta in np.linspace(0,travel,max(2,math.ceil(abs(travel)/math.radians(15))))[1:]:
                    dr=axis_rotation(axis,delta);waypoints.append((anchor+dr@(center-anchor),dr@r))
                end_p,end_r=waypoints[-1];waypoints.append((end_p+mr[:,2]*(.06 if crank else .045),end_r))
                for seed in (t.arms[robot].qref,t.home_q[robot]):
                    q=seed.copy();approach_seed=None
                    for step,(p,rr) in enumerate(waypoints):
                        q=t.screen_ik(robot,p,rr,seed=q)
                        if q is None:break
                        if step==0:
                            if not t.pose_clear(robot,q):break
                            approach_seed=q.copy()
                    else:plans[name]=dict(robot=robot,r=r,twist=twist,approach_seed=approach_seed.tolist(),target_angle_rad=target);break
                if name in plans:break
            if name not in plans:raise RuntimeError('No complete red-grasp arc '+name)
        for name,plan in list(plans.items()):
            center=t.data.geom(name+('_hand_grip' if t.task['level']==3 else '_red_knob')).xpos
            try:t.move_free(plan['robot'],center-plan['r'][:,2]*.10,plan['r'],'red-approach',preferred_seed=plan['approach_seed'])
            except RuntimeError as error:
                if not retries or not str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):raise
                affected=next(a for a in entries if a['id']==name)
                t.event('reference_replan_approach',robot=plan['robot'],dial=name,scope='Planning failed before execution; choose another grasp instead of treating a single IK branch as impossibility')
                self.grasp([affected],symbols,retries-1,avoid={name:plan['twist']})
                plans.pop(name);entries=[a for a in entries if a['id']!=name]
        if not plans:return
        for refinement in range(3):
            self.wait_support(entries)
            commands={}
            for a in entries:
                plan=plans[a['id']];robot=plan['robot'];arm=t.arms[robot];point,r=self.red_pose(a,plan['twist'])
                error=point-t.data.site_xpos[arm.site]
                if np.linalg.norm(error)>.001 or np.linalg.norm(rotation_vector(r,t.data.site_xmat[arm.site].reshape(3,3)))>.01:commands[robot]=t.pose_command(robot,point+(np.clip(error,-.002,.002) if refinement else 0),r,.025,.25)
            if not commands:break
            t.execute('red-contact-'+str(refinement),commands);t.wait(.10)
        closed=t.execute('red-grasp',{p['robot']:('gripper',dict(mode='grasp',width_m=.022 if t.task['level']==3 else .056,force_n=30.,speed_m_s=.08,epsilon_inner_m=.004 if t.task['level']==3 else .008,epsilon_outer_m=.004 if t.task['level']==3 else .008)) for p in plans.values()},allow_failure=True);t.wait(.25)
        if t.monitor.violations:raise RuntimeError('Process violation during red capture '+str(t.monitor.violations))
        bad=[]
        for a in entries:
            robot=plans[a['id']]['robot'];value=closed[robot]
            if value['status']=='uncertain':raise RuntimeError('Red grasp result unresolved '+robot)
            geoms=[t.model.geom(a['id']+'_hand_grip').id] if t.task['level']==3 else self.red_geoms(a)
            if value['status']!='completed' or robot not in t.monitor.bilateral(geoms):bad.append(a)
        self.grips.update(plans)
        if bad:
            if not retries:raise RuntimeError('Bilateral red regrasp exhausted '+','.join(a['id'] for a in bad))
            t.event('reference_red_regrasp',devices=[a['id'] for a in bad],remaining=retries)
            previous={a['id']:plans[a['id']]['twist'] for a in bad};self.release(bad);self.grasp(bad,symbols,retries-1,avoid=previous)
        self.wait_support(entries)
    def prepare_symbols(self,entries,symbols):
        self.release(entries);self.grasp(entries,symbols)
    def set_symbols(self,symbols,finish_when_solved=False):
        t=self.t;entries=[]
        for a,s in zip(self.dials,symbols):
            theta=float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0]))%360
            if a['id'] not in self.grips or abs((theta-self.angle_step_deg*s+180)%360-180)>3:entries.append(a)
        self.prepare_symbols(entries,symbols)
        # A digit is a maintained condition. An unchanged request does not
        # imply its free supporting frame has stayed still while other disks
        # were manipulated. Track every held input, not only changed digits.
        tracking=self.dials
        step_degrees=6 if t.task['level']==3 else 12;stalls={a['id']:0 for a in tracking};regrasps={a['id']:0 for a in tracking}
        for segment in range((math.ceil(180/step_degrees)+4)*3):
            if finish_when_solved and t.monitor.game.display==len(self.dials):return
            self.wait_support(tracking)
            self.maintain_holds(symbols)
            commands={};before={}
            for a in tracking:
                index=a['index'];target=self.grips[a['id']]['target_angle_rad'];now=t.data.joint(a['id']+'_hinge').qpos[0];err=target-now
                if abs(err)<math.radians(1.4):continue
                robot=self.grips[a['id']]['robot'];arm=t.arms[robot];jid=t.model.joint(a['id']+'_hinge').id;anchor=t.data.xanchor[jid]
                dr=axis_rotation(t.data.xaxis[jid],float(np.clip(err,-math.radians(step_degrees),math.radians(step_degrees))))
                commands[robot]=t.pose_command(robot,anchor+dr@(t.data.site_xpos[arm.site]-anchor),dr@t.data.site_xmat[arm.site].reshape(3,3),.025,.25);before[a['id']]=abs(err)
            if not commands:
                t.wait(.3)
                if all(abs(self.grips[a['id']]['target_angle_rad']-t.data.joint(a['id']+'_hinge').qpos[0])<math.radians(2) for a in tracking):break
                continue
            try:self.execute_turn('dial-turn-'+str(segment),commands)
            except ValueError as error:
                if 'no verified IK solution' not in str(error):raise
                rejection=t.events[-1];robot=rejection.get('robot') if rejection['event']=='validation_rejected' else None
                affected=next((a for a in tracking if self.grips[a['id']]['robot']==robot),None)
                if affected is None or regrasps[affected['id']]>=2:raise
                regrasps[affected['id']]+=1;t.event('reference_replan_rejected_arc',robot=robot,dial=affected['id'],scope='Admission rejected before actions started; no unknown execution is retried')
                old={affected['id']:self.grips[affected['id']]['twist']};self.release([affected]);self.grasp([affected],symbols,avoid=old);continue
            except RuntimeError:
                last=t.actions[-1] if t.actions else None
                if last is None or last['label']!='dial-turn-'+str(segment) or t.monitor.violations:raise
                results=last['results'];failed=[robot for robot,value in results.items() if value['status']=='failed']
                if not failed or any(value['status'] not in ('completed','failed') or (value['status']=='failed' and value.get('reason')!='Action timeout; stopped at current reference') for value in results.values()):raise
                affected=[a for a in tracking if self.grips[a['id']]['robot'] in failed]
                if any(regrasps[a['id']]>=2 for a in affected):raise
                self.wait_support(affected)
                for a in affected:regrasps[a['id']]+=1
                t.event('reference_replan_stopped_arc',robots=failed,scope='Known terminal controlled stops only; failed actions stay recorded, uncertain executions are never replayed')
                old={a['id']:self.grips[a['id']]['twist'] for a in affected};self.release(affected);self.grasp(affected,symbols,avoid=old);continue
            t.wait(.10)
            if t.monitor.violations:raise RuntimeError('Process violation during red turn '+str(t.monitor.violations))
            if finish_when_solved and t.monitor.game.display==len(self.dials):
                t.event('visible_win_reached',scope='Logical search stops on the visible latched win, after mechanical mark verification');return
            bad=[]
            for a in tracking:
                name=a['id']
                if name not in before:continue
                goal=self.grips[name]['target_angle_rad'];error=abs(goal-float(t.data.joint(name+'_hinge').qpos[0]));robot=self.grips[name]['robot']
                geoms=[t.model.geom(name+'_hand_grip').id] if t.task['level']==3 else self.red_geoms(a)
                engaged=robot in t.monitor.bilateral(geoms);stalls[name]=stalls[name]+1 if before[name]-error<math.radians(.3) else 0
                if error>math.radians(2) and (not engaged or stalls[name]>=3):bad.append(a)
            if bad:
                if any(regrasps[a['id']]>=2 for a in bad):raise RuntimeError('Red turn/regrasp budget exhausted '+','.join(a['id'] for a in bad))
                previous={a['id']:self.grips[a['id']]['twist'] for a in bad}
                for a in bad:regrasps[a['id']]+=1;stalls[a['id']]=0
                t.event('reference_turn_regrasp',devices=[a['id'] for a in bad]);self.release(bad);self.grasp(bad,symbols,avoid=previous)
        t.wait(.1)
        angles=np.degrees(t.data.qpos[t.monitor.qids])%360;errors=abs((angles-np.asarray(symbols)*self.angle_step_deg+180)%360-180)
        t.event('dial_configuration',requested=symbols,actual_angles_deg=angles.tolist(),errors_deg=errors.tolist(),score=t.monitor.game.display,candidates=len(self.decoder.candidates))
        if max(errors)>4:raise RuntimeError('Red mark tracking failed '+str(errors.tolist()))
        if t.monitor.violations:raise RuntimeError('Process violation '+str(t.monitor.violations))
        for _ in range(30):
            if self.input_settled(symbols):break
            self.wait_support(self.dials);t.wait(.1)
        if not self.input_settled(symbols):raise RuntimeError('No stable visible count for the commanded input')
        self.update_decoder(symbols)

def solve(t):
    m=Manipulator(t);m.lift();n=len(m.dials)
    for symbol in (1,2,3):m.set_symbols([symbol]*n)
    m.release(m.dials);t.event('mechanical_marks_and_return_passed',dial_count=n)
    for attempt in range(24):
        if t.monitor.game.won:break
        current=[int(round(float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0]))/90))%4 for a in m.dials];guess=m.decoder.choose(current)
        m.set_symbols(guess,finish_when_solved=True);print(f"{t.task['id']} feedback trial {t.monitor.game.queries}, candidates {len(m.decoder.candidates)}",flush=True)
    if not t.monitor.game.won:raise RuntimeError('Feedback decoder did not finish within its engineering query bound')
    t.event('decoder_solved',trials=t.monitor.game.queries,candidates=len(m.decoder.candidates));t.wait(.3)
