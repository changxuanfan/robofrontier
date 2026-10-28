"""Private process and goal evaluation. Nothing here is a policy observation."""
from collections import deque
import json,math
from pathlib import Path
import mujoco,numpy as np

DIGITS={0:'abcdef',1:'bc',2:'abged',3:'abgcd',4:'fgbc',5:'afgcd',6:'afgedc',7:'abc',8:'abcdefg',9:'abfgcd'}

def geom_extent(model,data,g):
    r=data.geom_xmat[g].reshape(3,3);size=model.geom_size[g];typ=model.geom_type[g]
    if typ==mujoco.mjtGeom.mjGEOM_BOX:return abs(r)@size
    if typ==mujoco.mjtGeom.mjGEOM_SPHERE:return np.full(3,size[0])
    if typ==mujoco.mjtGeom.mjGEOM_CYLINDER:return abs(r[:,2])*size[1]+size[0]*np.sqrt(np.maximum(0,1-r[:,2]**2))
    if typ==mujoco.mjtGeom.mjGEOM_CAPSULE:return abs(r[:,2])*size[1]+size[0]
    aabb=model.geom_aabb[g];return abs(r)@aabb[3:]

class CountFeedback:
    """N-digit aggregate feedback; private secret, no individual correctness."""
    def __init__(self,secret):
        self.__secret=tuple(secret)
        if not self.__secret or any(x not in (1,2,3) for x in self.__secret):raise ValueError('Nonzero ternary code required')
        self.n=len(secret);self.candidate=None;self.since=None;self.submitted=False;self.won=False;self.queries=0;self.last=None;self.trials=[];self.display=None
    def update(self,t,angles,speeds,eligible):
        degrees=np.degrees(angles)%360;marks=np.rint(degrees/90).astype(int)%4
        # The device recognizes a detent band held continuously for 250 ms.
        # Small in-band vibration must not invalidate the displayed input via
        # an additional instantaneous solver-velocity threshold.
        valid=eligible and np.all(marks!=0) and np.all(abs((degrees-marks*90+180)%360-180)<=4)
        guess=tuple(int(x) for x in marks) if valid else None
        if guess!=self.candidate or not valid:self.candidate=guess;self.since=t if valid else None;self.submitted=False
        if self.last is not None and t-self.last>.030001:self.since=t if valid else None
        self.last=t;ready=valid and t-self.since>=.25-1e-8
        score=sum(x==y for x,y in zip(guess,self.__secret)) if ready else None
        if ready and not self.submitted and not self.won:self.queries+=1;self.trials.append(dict(index=self.queries,sim_time_s=t,input_symbols=list(guess),correct_count=score));self.submitted=True
        if score==self.n:self.won=True
        self.display=self.n if self.won else score
        return self.display

class SupportQualification:
    """Initial dwell and requalification after a sustained support loss.

    A sub-grace contact gap makes the instantaneous input ineligible, but does
    not create a fresh 250 ms unsupported interval after contact returns.
    """
    def __init__(self,dwell=.25,gap=.05):self.dwell=dwell;self.gap=gap;self.since=None;self.missing=None;self.qualified=False;self.last=None
    def update(self,t,present):
        if self.last is not None and t-self.last>.030001:self.since=None;self.missing=None;self.qualified=False
        self.last=t
        if present:
            self.missing=None
            if self.since is None:self.since=t
            if t-self.since>=self.dwell-1e-9:self.qualified=True
        else:
            if self.missing is None:self.missing=t
            self.since=None
            if t-self.missing>self.gap+1e-9:self.qualified=False
        return bool(present and self.qualified)

class Monitor:
    def __init__(self,world,secret=None,log_path=None):
        self.w=world;self.m=world.model;self.d=world.data;self.task=world.task;self.family=self.task['family'];self.robots=list(world.arms);self.robot_index={r:i for i,r in enumerate(self.robots)}
        self.geom_owner=np.full(self.m.ngeom,-1,dtype=int);self.finger=np.full(self.m.ngeom,-1,dtype=int)
        roots={self.m.body(r+'_link0').id:i for i,r in enumerate(self.robots)}
        for g in range(self.m.ngeom):
            body=int(self.m.geom_bodyid[g]);name=self.m.body(body).name or ''
            while body and body not in roots:body=int(self.m.body_parentid[body])
            if body in roots:
                owner=roots[body];self.geom_owner[g]=owner
                if name in (self.robots[owner]+'_left_finger',self.robots[owner]+'_right_finger'):self.finger[g]=owner*2+(name.endswith('right_finger'))
        physical=(self.m.geom_contype!=0)|(self.m.geom_conaffinity!=0)
        self.finger_geoms={code:np.flatnonzero((self.finger==code)&physical).tolist() for code in range(len(self.robots)*2)}
        self.objects=self.task['objects'];self.names=[o['id'] for o in self.objects];self.bodies=[self.m.body(n).id for n in self.names]
        self.geoms={n:self.descendant_geoms(self.m.body(n).id) for n in self.names}
        self.geom_object=np.full(self.m.ngeom,-1,int)
        for i,n in enumerate(self.names):self.geom_object[self.geoms[n]]=i
        self.contact_pairs=[];self.self_contact_pairs=[];self.first_collisions={};self.violations=[];self.next_sample=-1;self.next_log=-1;self.last_time=None;self.max_sample_gap=0.;self.samples=0;self.max_active=0;self.overlap_s=0.;self.holding_s=0.;self.previous_sample=None
        from .contact_episodes import ContactEpisodes
        self.contact_episodes = ContactEpisodes()
        self.motion_overlap_s=0.;self.any_motion_s=0.;self.robot_motion_s=0.;self.max_moving=0
        self.log=Path(log_path).open('w') if log_path else None;self.active=True
        self.result_cache={};self.goal_since=None;self.since=np.full((len(self.task['goals']),len(self.names)),np.nan)
        if self.family=='dial':
            self.dials=self.task['dials'];n=len(self.dials);self.blue={};self.red={};self.module_geoms=[];self.blue_qualification=[SupportQualification() for _ in range(n)];self.dial_ready=[False]*n;self.previous_red=set();self.unsupported_since={}
            for i,a in enumerate(self.dials):
                name=a['id']
                if self.task['level']>1:
                    self.blue[self.m.geom(name+'_module_stem').id]=i
                    for g in self.descendant_geoms(self.m.body(name+'_module').id):
                        if (self.m.geom(g).name or '').startswith(name+'_handle_shoulder'):self.blue[g]=i
                rotor=self.m.body(name+'_rotor').id;self.red.update({g:i for g in self.descendant_geoms(rotor)})
                self.module_geoms.append(self.descendant_geoms(self.m.body(name+'_module').id))
            self.qids=[self.m.jnt_qposadr[self.m.joint(x['id']+'_hinge').id] for x in self.dials];self.dofs=[self.m.jnt_dofadr[self.m.joint(x['id']+'_hinge').id] for x in self.dials]
            if secret is None:raise ValueError('Private dial code must be explicitly supplied to evaluator')
            if self.task.get('mechanism',{}).get('feedback'):
                from .dial_feedback import from_task
                self.game=from_task(self.task,secret)
            else:self.game=CountFeedback(secret)
            self.minimum_clearance=np.full(n,math.inf);self.blue_gaps=np.zeros(n);self.blue_max_gaps=np.zeros(n);self.last_dial_angles=None
        if self.family=='relay' and self.task.get('suite_version')==2:
            from .evaluate_v2 import Relay
            self.relay_v2=Relay(self)
        elif self.family=='relay':
            self.support_geoms=[self.m.geom('payload_beam').id]
            self.route=self.task['route'];self.pairs=self.task['pairs'];self.route_index=0;self.custody=set(self.pairs[0]['robots']);self.airborne=False;self.dropped=False;self.drop_count=0;self.overlap_since=None;self.overlap_qualified=False;self.support_gap_since=None;self.transitions=[];self.final_supported=False;self.landed=False;self.initial_z=float(self.d.body('payload').xpos[2]);self.minimum_payload_clearance=math.inf
    def descendant_geoms(self,root):
        out=[]
        for g in range(self.m.ngeom):
            if not (self.m.geom_contype[g] or self.m.geom_conaffinity[g]):continue
            b=int(self.m.geom_bodyid[g])
            while b and b!=root:b=int(self.m.body_parentid[b])
            if b==root:out.append(g)
        return out
    def violation(self,kind,**details):
        if not any(v['kind']==kind and v.get('key')==details.get('key') for v in self.violations):self.violations.append(dict(kind=kind,sim_time_s=float(self.d.time),**details))
    def terminal_scoring(self):
        return self.task.get('run_protocol', {}).get('evaluation_semantics') == 'terminal_outcome_v2'
    def contacts(self):
        if not self.d.ncon:return np.empty((0,2),int),np.empty(0)
        return np.asarray(self.d.contact.geom),np.asarray(self.d.contact.dist)
    def bilateral(self,target_geoms,positive_force=False):
        """Opposed-jaw proximity with actual contact on at least one side.

        For load support, a real force-bearing contact is required per gripper;
        it need not be distributed positively over both individual pads.
        """
        target=set(target_geoms);found={};loaded=set()
        for i,c in enumerate(self.d.contact):
            if c.dist>.0002:continue
            for finger,part in ((int(c.geom1),int(c.geom2)),(int(c.geom2),int(c.geom1))):
                code=int(self.finger[finger])
                if code<0 or part not in target:continue
                if positive_force:
                    force=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,i,force)
                    if force[0]>=.02:loaded.add(self.robots[code//2])
                found.setdefault(self.robots[code//2],set()).add(code%2)
        if found:
            # With zero physics margin, the contact list contains no positive
            # gaps at all. Implement the declared 0.2 mm geometric tolerance by
            # querying distance, without adding contacts or forces to physics.
            for robot,parts in found.items():
                if len(parts)==2:continue
                missing=1-next(iter(parts));code=self.robot_index[robot]*2+missing
                for finger in self.finger_geoms[code]:
                    for part in target:
                        compatible=(self.m.geom_contype[finger]&self.m.geom_conaffinity[part]) or (self.m.geom_contype[part]&self.m.geom_conaffinity[finger])
                        if compatible and mujoco.mj_geomDistance(self.m,self.d,finger,part,.00021,None)<.0002:
                            parts.add(missing);break
                    if missing in parts:break
        return {r for r,parts in found.items() if len(parts)==2 and (not positive_force or r in loaded)}
    def released(self,target_geoms):
        pairs,dist=self.contacts()
        if not len(pairs):return True
        target=np.zeros(self.m.ngeom,bool);target[list(target_geoms)]=True
        return not np.any((dist<=.0002)&((target[pairs[:,0]]&(self.geom_owner[pairs[:,1]]>=0))|(target[pairs[:,1]]&(self.geom_owner[pairs[:,0]]>=0))))
    def bottom(self,gs):return min(float((self.d.geom_xpos[g]+self.d.geom_xmat[g].reshape(3,3)@self.m.geom_aabb[g,:3])[2]-geom_extent(self.m,self.d,g)[2]) for g in gs)
    def velocity(self,body):
        v=np.zeros(6);mujoco.mj_objectVelocity(self.m,self.d,mujoco.mjtObj.mjOBJ_BODY,body,v,0);return v
    def physical_step(self):
        if not self.active:return
        pairs,dist=self.contacts()
        episode_contacts=set()
        if len(pairs):
            owners=self.geom_owner[pairs];bad=(owners[:,0]>=0)&(owners[:,1]>=0)&(owners[:,0]!=owners[:,1])&(dist<-.0002)
            for a,b in pairs[bad]:
                names=tuple(sorted((self.m.body(self.m.geom_bodyid[a]).name,self.m.body(self.m.geom_bodyid[b]).name)));key=' / '.join(names)
                episode_contacts.add(('inter_robot_contact',key))
                if key not in self.first_collisions:self.first_collisions[key]=float(self.d.time);self.violation('inter_robot_contact',key=key)
            same=(owners[:,0]>=0)&(owners[:,0]==owners[:,1])&(dist<-.001)
            for a,b in pairs[same]:
                bodies=[self.m.body(self.m.geom_bodyid[g]).name or '' for g in (a,b)]
                if not all('finger' in x for x in bodies):
                    key=' / '.join(sorted(bodies))
                    episode_contacts.add(('self_robot_contact',key))
                    if key not in self.self_contact_pairs:self.self_contact_pairs.append(key);self.violation('self_robot_contact',key=key)
        t=float(self.d.time)
        self.contact_episodes.sample(t,episode_contacts)
        if t+1e-9<self.next_sample:return
        self.next_sample=t+.02;self.samples+=1
        if self.last_time is not None:self.max_sample_gap=max(self.max_sample_gap,t-self.last_time)
        dt=0 if self.last_time is None else t-self.last_time;self.last_time=t
        active=sum(arm.job is not None for arm in self.w.arms.values());self.max_active=max(self.max_active,active)
        if active>=2:self.overlap_s+=dt
        sample=np.asarray([self.d.qpos[arm.qids].copy() for arm in self.w.arms.values()])
        if self.previous_sample is not None and dt>0:
            # Position change over the complete sampling window suppresses
            # solver sub-step velocity chatter. Holding is counted separately.
            moving=int(np.sum(np.max(abs(sample-self.previous_sample),axis=1)/dt>.02))
            self.max_moving=max(self.max_moving,moving);self.robot_motion_s+=moving*dt
            if moving:self.any_motion_s+=dt
            if moving>=2:self.motion_overlap_s+=dt
        self.previous_sample=sample
        if self.family=='dial':self.sample_dial(dt)
        elif self.family=='relay':self.relay_v2.sample() if hasattr(self,'relay_v2') else self.sample_relay()
        else:self.sample_goals()
        if self.log and t>=self.next_log:
            self.next_log=t+.10;record=dict(sim_time_s=t,result=self.result(),objects={n:dict(position_m=self.d.body(n).xpos.tolist(),quaternion_wxyz=self.d.body(n).xquat.tolist()) for n in self.names})
            self.log.write(json.dumps(record,allow_nan=False)+'\n');self.log.flush()
    def sample_goals(self):
        n=len(self.names);goals=self.task['goals'];eligible=np.zeros((len(goals),n),bool)
        positions=np.array([self.d.body(n).xpos for n in self.names]);vel=[self.velocity(b) for b in self.bodies]
        released=np.ones(n,bool);pairs,dist=self.contacts()
        for left,right in ((0,1),(1,0)):
            ids=self.geom_object[pairs[:,left]];mask=(ids>=0)&(self.geom_owner[pairs[:,right]]>=0)&(dist<=.0002);released[ids[mask]]=False
        stable=[np.linalg.norm(v[3:])<.01 and np.linalg.norm(v[:3])<.1 and released[j] for j,v in enumerate(vel)]
        if self.task.get('suite_version')==2:
            from .evaluate_v2 import placement_candidates
            eligible=placement_candidates(self,positions,stable)
        elif self.family=='mosaic':
            targets=np.asarray([g['position_m'] for g in goals]);delta=positions[None,:,:]-targets[:,None,:]
            eligible=(abs(delta[:,:,2])<.003)
            if self.task['level']!=1:eligible&=np.sum(delta[:,:,:2]**2,axis=2)<.005**2
            colors=np.asarray([o['color'] for o in self.objects]);wanted=np.asarray([g['requires']['color'] for g in goals]);eligible&=wanted[:,None]==colors[None,:]
            rotations=np.asarray([self.d.body(n).xmat.reshape(3,3) for n in self.names]);aligned=np.min(np.max(abs(rotations),axis=1),axis=1)>=math.cos(math.radians(8))
            eligible&=(np.asarray(stable)&aligned)[None,:]
            if self.task['level']==1:
                extents=.02*np.sum(abs(rotations),axis=2)
                eligible&=np.all(abs(delta[:,:,:2])+extents[None,:,:2]<=.0253,axis=2)
                eligible&=(abs(positions[:,2]-extents[:,2]-self.task['grid']['floor_z_m'])<=.0008)[None,:]
        for i,g in (enumerate(goals) if self.family!='mosaic' and self.task.get('suite_version')!=2 else []):
            if self.family=='transport':
                board=self.d.body(g['board']);r=board.xmat.reshape(3,3);target=np.asarray(g['local_position_m']);bv=self.velocity(board.id)
                if np.linalg.norm(bv[3:])>.01 or np.linalg.norm(bv[:3])>.1:continue
            for j,o in enumerate(self.objects):
                if not stable[j] or any(o.get(k)!=v for k,v in g['requires'].items()):continue
                obj=self.d.body(o['id']);orot=obj.xmat.reshape(3,3)
                if self.family=='transport':
                    p=r.T@(positions[j]-board.xpos);rr=r.T@orot
                    if np.linalg.norm(p[:2]-target[:2])>(.005 if self.task['level']==1 else .0014) or abs(p[2]-.02)>.003:continue
                    if abs(rr[2,2])<math.cos(math.radians(5)):continue
                    if self.task['level']>1:
                        tips=[p+rr@np.array([x,0,z]) for x in (-.010,.010) for z in (-.02,-.03)]
                        if any(min(np.linalg.norm(tip[:2]-[x,target[1]]) for x in (-.010,.010))>.0014 for tip in tips):continue
                        if any(tip[2]>-.008 for tip in (tips[1],tips[3])):continue
                else:
                    target=np.asarray(g['position_m']);p=positions[j]
                    if np.linalg.norm(p[:2]-target[:2])>.005 or abs(p[2]-target[2])>.003:continue
                    if np.min(np.max(abs(orot),axis=0))<math.cos(math.radians(8)):continue
                    if self.task['level']==1:
                        extent=.02*np.sum(abs(orot),axis=1)
                        if np.any(abs(p[:2]-target[:2])+extent[:2]>.0253) or abs(p[2]-extent[2]-self.task['grid']['floor_z_m'])>.0008:continue
                eligible[i,j]=True
        t=float(self.d.time);self.since=np.where(eligible,np.where(np.isnan(self.since),t,self.since),np.nan)
        qualified=eligible&((t-self.since)>=self.task['completion']['stable_duration_sim_s']-1e-8);matched={}
        def assign(i,seen):
            for j in np.flatnonzero(qualified[i]):
                if j in seen:continue
                seen.add(j)
                if j not in matched or assign(matched[j],seen):matched[j]=i;return True
            return False
        for i in range(len(goals)):assign(i,set())
        complete=len(matched)==len(goals)
        self.result_cache=dict(all_goals_met=complete,task_success=complete and not self.violations,matched_goals=len(matched),total_goals=len(goals),assignments={goals[i]['goal_id']:self.names[j] for j,i in matched.items()})
        if Monitor.terminal_scoring(self):
            self.result_cache.update(task_success=complete, terminal_goal_met=complete,
                                     strict_task_success=complete and not self.violations,
                                     scoring_version='terminal_outcome_v2')
    def sample_dial(self,dt):
        n=len(self.dials);supporters=[];ready=[];clear=[];red=set();pairs,dist=self.contacts()
        for i,a in enumerate(self.dials):
            if self.task['level']==1:support=set();ok=True;c=math.inf
            else:
                support=self.bilateral([g for g,index in self.blue.items() if index==i]);c=self.bottom(self.module_geoms[i])-self.task.get('table_height_m',.4)
                own=np.zeros(self.m.ngeom,bool);own[self.module_geoms[i]]=True
                external=bool(np.any((dist<=.0002)&((own[pairs[:,0]]&~own[pairs[:,1]]&(self.geom_owner[pairs[:,1]]<0))|(own[pairs[:,1]]&~own[pairs[:,0]]&(self.geom_owner[pairs[:,0]]<0))))) if len(pairs) else False
                ok=c>=.02 and bool(support) and not external
                if ok:self.holding_s+=dt
                ok=self.blue_qualification[i].update(float(self.d.time),ok)
                self.minimum_clearance[i]=min(self.minimum_clearance[i],c) if ok else self.minimum_clearance[i]
            supporters.append(support);ready.append(ok);clear.append(None if not math.isfinite(c) else c)
        for pair,distance in zip(pairs,dist):
            if distance>.0002:continue
            for robot,part in ((int(pair[0]),int(pair[1])),(int(pair[1]),int(pair[0]))):
                if self.geom_owner[robot]>=0 and part in self.red:red.add((self.red[part],self.robots[self.geom_owner[robot]]))
        if not self.game.won and self.task['level']>1:
            for i,robot in red-self.previous_red:
                if not ready[i] or not (supporters[i]-{robot}):self.violation('premature_red_contact',key=self.dials[i]['id']+'/'+robot)
            for i,robot in red:
                arm=self.w.arms[robot];moving=arm.job is not None and arm.job_op=='move' and np.max(abs(self.d.qvel[arm.dofs]))>.03
                key=(i,robot)
                if not ready[i] and moving:
                    if key not in self.unsupported_since:self.unsupported_since[key]=self.d.time
                    if self.d.time-self.unsupported_since[key]>.05:self.violation('driven_red_without_support',key=self.dials[i]['id']+'/'+robot)
                else:self.unsupported_since.pop(key,None)
        self.dial_ready=ready;self.previous_red=red;eligible=all(ready);red_ready=None
        spec=self.task.get('mechanism',{}).get('feedback')
        if spec and spec.get('require_red_grasp'):
            red_ready=[]
            for i,a in enumerate(self.dials):
                geoms=[self.m.geom(a['id']+'_hand_grip').id] if self.task['level']==3 else [g for g,index in self.red.items() if index==i]
                rotating={r for r in self.bilateral(geoms) if self.w.arms[r].grasp_force is not None}
                if self.task['level']>1:rotating={r for r in rotating if supporters[i]-{r}}
                red_ready.append(bool(rotating))
            eligible=eligible and all(red_ready)
        score=self.game.update(float(self.d.time),self.d.qpos[self.qids],self.d.qvel[self.dofs],eligible)
        if spec:
            from .dial_feedback import paint_display
            paint_display(self.m,self.game,spec.get('display_digits',2))
        else:
            show='--' if score is None else f'{score:02d}'
            for digit,char in enumerate(show):
                for k,segment in enumerate('abcdefg'):
                    gid=self.m.geom(f'score_{digit}_{k}').id;self.m.geom_rgba[gid]=[.68,.13,.07,1 if segment in (DIGITS[int(char)] if char!='-' else 'g') else 0]
        self.result_cache=dict(all_goals_met=self.game.won,task_success=self.game.won and not self.violations,solved=self.game.won,valid_trials=self.game.queries,display=score,private_angles_deg=np.degrees(self.d.qpos[self.qids]).tolist(),private_clearance_m=clear,private_support_ready=ready,private_angle_speeds_rad_s=self.d.qvel[self.dofs].tolist(),private_feedback_dwell_s=None if self.game.since is None else float(self.d.time)-self.game.since)
        if Monitor.terminal_scoring(self):
            self.result_cache.update(task_success=self.game.won, terminal_goal_met=self.game.won,
                                     strict_task_success=self.game.won and not self.violations,
                                     scoring_version='terminal_outcome_v2')
        if spec:self.result_cache.update(feedback_mode=self.game.mode,input_ready=self.game.ready,private_red_hold_ready=red_ready)
    def sample_relay(self):
        t=float(self.d.time);gs=self.geoms['payload'];support=self.bilateral(self.support_geoms,positive_force=True);position=self.d.body('payload').xpos.copy();contact_env=False;cradle=False
        for c in self.d.contact:
            if c.dist>0:continue
            for a,b in ((int(c.geom1),int(c.geom2)),(int(c.geom2),int(c.geom1))):
                if a not in gs:continue
                name=self.m.geom(b).name or ''
                if name.startswith(('table_','floor')):contact_env=True
                if name.startswith(('start_rail','finish_rail')):cradle=True
        if not self.airborne:
            if position[2]>self.initial_z+.025:
                if len(support&self.custody)>=2:self.airborne=True
                else:self.violation('unsupported_initial_lift')
        if self.airborne:
            self.minimum_payload_clearance=min(self.minimum_payload_clearance,self.bottom(gs)-self.task.get('table_height_m',.4))
            if len(support)<2:
                if self.support_gap_since is None:self.support_gap_since=t
            else:self.support_gap_since=None
            final=self.route_index==len(self.route)-1
            frame=self.route[-1];rot=self.d.body('payload').xmat.reshape(3,3);desired=np.array([-math.sin(frame['yaw_rad']),math.cos(frame['yaw_rad']),0])
            inside=np.linalg.norm(position[:2]-np.asarray(frame['center_m'][:2]))<.035 and abs(position[2]-frame['center_m'][2])<.008 and abs(rot[:,1]@desired)>math.cos(.08)
            cradle_release=final and inside and cradle and self.final_supported
            if (contact_env or (self.support_gap_since is not None and t-self.support_gap_since>.20 and not self.landed and not cradle_release)) and not self.dropped:
                self.dropped=True;self.drop_count+=1
            if self.dropped:
                if support-self.custody:self.violation('receiver_recovered_drop',key=str(self.route_index))
                if len(support&self.custody)>=2 and not contact_env and position[2]>self.initial_z+.025:self.dropped=False;self.support_gap_since=None
            elif not final:
                next_pair=set(self.pairs[(self.route_index+1)%len(self.pairs)]['robots'])
                if next_pair<=support and support&self.custody:
                    if self.overlap_since is None:self.overlap_since=t
                    if t-self.overlap_since>=.20:self.overlap_qualified=True
                elif not self.overlap_qualified:self.overlap_since=None
                if not (support&self.custody) and len(support)>=2:
                    if self.overlap_qualified and next_pair<=support:
                        self.route_index+=1;self.transitions.append(dict(route_index=self.route_index,time_s=t));self.custody=next_pair;self.overlap_since=None;self.overlap_qualified=False
                    else:self.violation('unverified_handover',key=str(self.route_index))
            final=self.route_index==len(self.route)-1
            frame=self.route[-1];rot=self.d.body('payload').xmat.reshape(3,3);desired=np.array([-math.sin(frame['yaw_rad']),math.cos(frame['yaw_rad']),0])
            inside=np.linalg.norm(position[:2]-np.asarray(frame['center_m'][:2]))<.035 and abs(position[2]-frame['center_m'][2])<.008 and abs(rot[:,1]@desired)>math.cos(.08)
            if final and inside and cradle and len(support&self.custody)>=2:self.final_supported=True
            released=self.released(gs)
            if final and inside and cradle and released and self.final_supported:self.landed=True;self.dropped=False
            if final and inside and cradle and not support and not self.final_supported:self.violation('unsupported_final_landing')
            v=self.velocity(self.m.body('payload').id);done=final and self.landed and inside and released and np.linalg.norm(v[3:])<.01 and np.linalg.norm(v[:3])<.1 and not contact_env
            if done:
                if self.goal_since is None:self.goal_since=t
            else:self.goal_since=None
        complete=self.goal_since is not None and t-self.goal_since>=1.
        self.result_cache=dict(all_goals_met=complete,task_success=complete and not self.violations,completed_handovers=self.route_index,required_handovers=len(self.route)-1,drops=self.drop_count,custody=sorted(self.custody),transitions=self.transitions)
    def result(self):
        result = dict(self.result_cache,process_compliant=not self.violations,violations=list(self.violations),sim_time_s=float(self.d.time),samples=self.samples,max_sample_gap_s=self.max_sample_gap,inter_robot_contacts=len(self.first_collisions),self_contacts=list(self.self_contact_pairs),peak_concurrent_actions=self.max_active,execution_overlap_sim_s=self.overlap_s,motion_overlap_sim_s=self.motion_overlap_s,any_motion_sim_s=self.any_motion_s,robot_motion_s=self.robot_motion_s,peak_moving_robots=self.max_moving,motion_definition='max joint displacement / 20 ms window > 0.02 rad/s; distinct from active jobs and useful holding',useful_blue_support_robot_s=self.holding_s)
        if self.terminal_scoring(): result['robot_contact_episodes'] = self.contact_episodes.report()
        return result
    def close(self):
        if self.log:self.log.close()
    def finish(self):self.active=False
