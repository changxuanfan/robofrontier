"""R10 family-owned actuator reference; shared execution stays unchanged.

The decoder consumes attempted symbols and visible aggregate counts only.
Exact apparatus geometry is privileged engineering input, never a GPT recipe.
No simulator state, physics parameters, scoring rules or private code are set.
"""
from itertools import product
import math
import numpy as np
from ..robot_control import rotation_vector
from .dial_oracle import Decoder, Manipulator as BaselineManipulator
from scripts.verify_floating_dial_oracle import axis_rotation, knob_grasp_rotation


def grip_width(trial, dial):
    """Physical rotor grasp diameter, read from the frozen model."""
    suffix = '_hand_grip' if trial.task['level'] == 3 else '_red_knob'
    geom = trial.model.geom(dial['id'] + suffix)
    width = float(2 * geom.size[0])
    if not 0 < width < .072:
        raise RuntimeError('Rotor diameter outside supported Panda grasp range')
    return width


class Manipulator(BaselineManipulator):
    # Private, physically commissioned capture choice; not an Agent recipe.
    red_capture_offset_m=0.
    red_grasp_force_n=30.

    def red_pose(self,a,twist):
        point,rotation=super().red_pose(a,twist)
        if self.t.task['level']==3:point=point+rotation[:,0]*self.red_capture_offset_m
        return point,rotation

    # The legacy reference predates 27.5 mm cranks. Keep its proven state
    # machine, overriding only dimension-dependent red capture planning.
    def present(self, entries):
        if self.t.task['level'] == 3:
            # Stable carried frames need not be reset to a historical pose.
            # Red arc planning below uses the current measured module frame;
            # its finite IK search can reject it without issuing live motion.
            self.wait_support(entries)
            return
        return super().present(entries)

    def clear_released_crank(self, dial, point, rotation):
        """Choose a forward-only clear exit after real release, not a fixed reach."""
        from .oracle import joint_edge_clear
        t=self.t;robot=self.grips[dial['id']]['robot'];arm=t.arms[robot]
        if (arm.job is not None or arm.grasp_force is not None or
                not t.monitor.released(self.red_geoms(dial))):
            raise RuntimeError('Crank clear requires terminal opening and real separation '+robot)
        if arm.observe()['gripper_width_m']<arm.grip_max-.002:
            raise RuntimeError('Crank clear requires measured maximum aperture '+robot)
        point=np.asarray(point);rotation=np.asarray(rotation)
        distance=float(np.dot(t.data.site_xpos[arm.site]-point,-rotation[:,2]))
        for length in (.06,.055,.05,.045,.04,.035,.03):
            if length<distance+.001:continue
            target=point-rotation[:,2]*length
            try:
                # Use the same strict private joint-command IK as execution.
                command=t.pose_command(robot,target,rotation,joints=True)
            except ValueError as error:
                if 'No verified robot IK' not in str(error):raise
                continue
            q=np.asarray(command[1]['joint_rad'])
            if not joint_edge_clear(t.data.qpos[arm.qids],q,
                                    lambda q:t.pose_clear(robot,q),.005):continue
            t.event('reference_released_crank_exit',robot=robot,dial=dial['id'],
                    selected_distance_m=length,previous_distance_m=distance,
                    scope='Forward-only exact joint-command IK and sampled full-scene clearance; physical release already observed')
            t.execute('crank-open-clear',{robot:command})
            return length
        if t.pose_clear(robot,t.data.qpos[arm.qids]):
            t.event('reference_released_crank_already_clear',robot=robot,dial=dial['id'],
                    scope='No extra displacement required; actual separated hand is already clear for the screened home route')
            return None
        raise RuntimeError('No reachable clear released-crank exit '+robot)

    def wide_open_sweep_is_axially_clear(self, dial):
        """Bound hand opening outside the rotor's axial swept envelope.

        Rotation about the hinge leaves rotor axial extents unchanged. Finger
        slides are linear, so current/max apertures bound the opening sweep.
        This is static geometry, never a substitute for observed separation.
        """
        t=self.t;robot=self.grips[dial['id']]['robot'];arm=t.arms[robot]
        normal=t.data.body(dial['id']+'_module').xmat.reshape(3,3)[:,2]
        top=-float('inf');bottom=float('inf');red=set(self.red_geoms(dial))
        for geom in range(t.model.ngeom):
            if not (t.model.geom_contype[geom] or t.model.geom_conaffinity[geom]):continue
            r=t.data.geom_xmat[geom].reshape(3,3);aabb=t.model.geom_aabb[geom]
            center=t.data.geom_xpos[geom]+r@aabb[:3];extent=float(abs(normal@r)@aabb[3:])
            if geom in red:top=max(top,float(normal@center)+extent)
            body=t.model.body(int(t.model.geom_bodyid[geom])).name
            if body not in (robot+'_hand',robot+'_left_finger',robot+'_right_finger'):continue
            centers=[center]
            if body.endswith(('_left_finger','_right_finger')):
                suffix='finger_joint1' if body.endswith('_left_finger') else 'finger_joint2'
                joint=t.model.joint(robot+'_'+suffix).id
                delta=(arm.grip_max-arm.grip_min)/2-float(t.data.joint(robot+'_'+suffix).qpos[0])
                centers.append(center+t.data.xaxis[joint]*delta)
            bottom=min(bottom,*(float(normal@p)-extent for p in centers))
        return math.isfinite(top) and math.isfinite(bottom) and bottom-top>=.003

    def release(self, entries):
        if self.t.task['level'] != 3:
            return super().release(entries)
        for retry in range(3):
            try:
                return self._release_cranks_once(entries)
            except RuntimeError as error:
                message=str(error);t=self.t
                active=[a for a in entries if a['id'] in self.grips]
                known=message.startswith(('Crank clear requires terminal opening and real separation',
                                          'No reachable clear released-crank exit'))
                if message.startswith('crank-open-axial-') and t.actions:
                    last=t.actions[-1]
                    known=last['label'].startswith('crank-open-axial-') and all(
                        v['status']=='completed' or (v['status']=='failed' and
                        v.get('reason')=='Action timeout; stopped at current reference')
                        for v in last['results'].values())
                if (retry==2 or not known or not active or self.rule_failure() or
                    any(arm.job is not None for arm in t.arms.values()) or
                    not all(t.monitor.dial_ready[a['index']] for a in active)):
                    raise
                t.event('reference_forward_release_retry',retry=retry+1,reason=message,
                    devices=[a['id'] for a in active],
                    scope='Known terminal result and current blue support; recompute from measured current state, never restore poses or replay unknown work')

    def _release_cranks_once(self, entries):
        """Physically open the crank capture, then clear its return sweep.

        Aperture completion, force-mode exit and physical separation are
        distinct facts. A known blocked opening can be followed by a separate
        bounded freeing action with real opposite support; the opening failure
        stays recorded. Module position is free under the game rules, but blue
        grasp, airborne requirements and the process audit must stay valid.
        No force limit, solver, object pose, or acceptance parameter is modified.
        """
        if self.t.task['level'] != 3:
            return super().release(entries)
        t=self.t
        active=[a for a in entries if a['id'] in self.grips]
        if not active:return
        self.wait_support(active)
        # An extra loaded return arc can destabilize the blue bearing. Open
        # first, retaining support; only actual contact requires a finite axial
        # freeing motion. Fresh six-dial all-mark commissioning verifies this
        # closure, not a general guarantee for every grasp or layout.
        opened=t.execute('crank-open-maximum',{
            self.grips[a['id']]['robot']:('gripper',dict(mode='move',
                width_m=t.arms[self.grips[a['id']]['robot']].grip_max,speed_m_s=.05))
            for a in active},allow_failure=True)
        starts={}
        for a in active:
            robot=self.grips[a['id']]['robot'];arm=t.arms[robot]
            if (opened[robot]['status']=='uncertain' or arm.grasp_force is not None
                    or (opened[robot]['status']=='failed' and opened[robot].get('reason') not in ('Requested opening not reached','Requested aperture exceeded'))):
                raise RuntimeError('Crank release action unresolved '+robot)
            starts[a['id']]=(t.data.site_xpos[arm.site].copy(),
                t.data.site_xmat[arm.site].reshape(3,3).copy(),
                t.data.body(a['id']+'_module').xpos.copy())
        t.event('reference_supported_crank_freeing',devices=[a['id'] for a in active],initial_opening_results=opened,
            scope='Known opening failures remain failures. Force mode is off; small axial freeing may move the supported module. Blue support, airborne prerequisites and the process audit must remain valid; no release is claimed before actual separation')
        for leg in range(1,21):
            touching=[a for a in active if not t.monitor.released(self.red_geoms(a)) or
                (t.arms[self.grips[a['id']]['robot']].observe()['gripper_width_m']<
                 t.arms[self.grips[a['id']]['robot']].grip_max-.002 and
                 not self.wide_open_sweep_is_axially_clear(a))]
            if not touching:break
            commands={}
            for a in touching:
                point,rotation,_=starts[a['id']];robot=self.grips[a['id']]['robot']
                target=point-rotation[:,2]*(.003*leg)
                if t.screen_ik(robot,target,rotation) is None:
                    raise RuntimeError('No reachable small open-crank exit '+robot)
                commands[robot]=t.pose_command(robot,target,rotation,.01,.2)
            t.execute('crank-open-axial-'+str(leg),commands)
            self.wait_support(active)
            t.event('reference_supported_freeing_progress',leg=leg,
                module_displacement_m={a['id']:float(np.linalg.norm(t.data.body(a['id']+'_module').xpos-starts[a['id']][2])) for a in active})
        if not all(t.monitor.released(self.red_geoms(a)) for a in active):
            raise RuntimeError('Open crank exit exhausted before rotor separation')
        t.execute('crank-reopen-clear',{self.grips[a['id']]['robot']:('gripper',dict(mode='move',width_m=t.arms[self.grips[a['id']]['robot']].grip_max,speed_m_s=.05)) for a in active})
        for a in active:
            robot=self.grips[a['id']]['robot'];point,rotation,_=starts[a['id']]
            self.clear_released_crank(a,point,rotation)
            if getattr(self,'home_after_release',True):t.move_free(robot,*t.home[robot],label='crank-open-home')
            self.grips.pop(a['id'])
        t.wait(2.)
        self.wait_support(active)
        t.event('reference_crank_separation_observed',devices=[a['id'] for a in active],
            angles_deg=[float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0])) for a in active],
            scope='Actual release and continuing blue support; next grasp uses measured pose, no artificial zero reset')

    def grasp(self,entries,symbols,retries=2,avoid=None):
        t=self.t;plans={}
        excluded={name:({value} if np.isscalar(value) else set(value))
                  for name,value in (avoid or {}).items()}
        self.present(entries)
        self.wait_support(entries)
        for a in entries:
            robot=a['nominal_robots'][0];name=a['id'];module=t.data.body(name+'_module');mr=module.xmat.reshape(3,3).copy();crank=t.task['level']==3
            jid=t.model.joint(name+'_hinge').id;anchor=t.data.xanchor[jid].copy();axis=t.data.xaxis[jid].copy();symbol=symbols[a['index']]
            angles=self.target_angles(symbol)
            angles.sort(key=lambda target:abs(target-t.data.joint(name+'_hinge').qpos[0]))
            twists=(0,180,45,225,315,135,270,90) if crank else (270,90,0,180,45,135,225,315)
            for twist,target in product(twists,angles):
                if twist in excluded.get(name,set()):continue
                center,r=self.red_pose(a,twist);waypoints=[(center-r[:,2]*.10,r),(center,r)]
                travel=target-float(t.data.joint(name+'_hinge').qpos[0])
                for delta in np.linspace(0,travel,max(2,math.ceil(abs(travel)/math.radians(15))))[1:]:
                    dr=axis_rotation(axis,delta);waypoints.append((anchor+dr@(center-anchor),dr@r))
                if getattr(self,'require_grasp_exit',True):
                    end_p,end_r=waypoints[-1];waypoints.append((end_p+mr[:,2]*(.06 if crank else .045),end_r))
                for seed in (t.arms[robot].qref,t.home_q[robot]):
                    q=seed.copy();approach_seed=None
                    for step,(p,rr) in enumerate(waypoints):
                        previous=q.copy();q=t.screen_ik(robot,p,rr,seed=q)
                        if q is None:break
                        if not self.arc_joint_clear(robot,previous,q):break
                        if step==0:
                            if not t.pose_clear(robot,q):break
                            approach_seed=q.copy()
                    else:plans[name]=dict(robot=robot,r=r,twist=twist,approach_seed=approach_seed.tolist(),target_angle_rad=target);break
                if name in plans:break
            if name not in plans:raise RuntimeError('No complete red-grasp arc '+name)
        for name,plan in list(plans.items()):
            affected=next(a for a in entries if a['id']==name);center,_=self.red_pose(affected,plan['twist'])
            try:t.move_free(plan['robot'],center-plan['r'][:,2]*.10,plan['r'],'red-approach',preferred_seed=plan['approach_seed'])
            except RuntimeError as error:
                if not retries or not str(error).startswith(('No clear free-arm IK','No collision-free sampled free-arm route')):raise
                affected=next(a for a in entries if a['id']==name)
                t.event('reference_replan_approach',robot=plan['robot'],dial=name,scope='Planning failed before execution; choose another grasp instead of treating a single IK branch as impossibility')
                self.grasp([affected],symbols,retries-1,
                           avoid={name:excluded.get(name,set())|{plan['twist']}})
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
        closed=t.execute('red-grasp',{p['robot']:('gripper',dict(mode='grasp',width_m=grip_width(t, next(a for a in entries if a['id']==name)),force_n=self.red_grasp_force_n if t.task['level']==3 else 30.,speed_m_s=.08,epsilon_inner_m=.004 if t.task['level']==3 else .008,epsilon_outer_m=.004 if t.task['level']==3 else .008)) for name,p in plans.items()},allow_failure=True);t.wait(.25)
        if self.rule_failure():raise RuntimeError('Process violation during red capture '+str(t.monitor.violations))
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
            previous={a['id']:excluded.get(a['id'],set())|{plans[a['id']]['twist']} for a in bad}
            self.release(bad);self.grasp(bad,symbols,retries-1,avoid=previous)
        self.wait_support(entries)

def solve(t):
    if t.task.get('mechanism',{}).get('feedback'):
        raise ValueError('This legacy three-symbol current-count reference cannot execute the versioned BEST task')
    m=Manipulator(t);n=len(m.dials)
    # Preserve existing four/six-dial witnesses. The eight-dial D03 centered
    # 30 N local closure avoids the offset capture's radial displacement and
    # width failures. Mixed-input blue support still needs full qualification.
    if t.task['level']==3 and n>4:
        m.red_capture_offset_m=0. if n==8 else .004;m.red_grasp_force_n=30. if n==8 else 50.
    m.lift()
    for symbol in (1,2,3):m.set_symbols([symbol]*n)
    m.release(m.dials);t.event('mechanical_marks_and_controlled_unload_passed' if t.task['level']==3 else 'mechanical_marks_and_return_passed',dial_count=n,passive_free_return_verified=t.task['level']!=3)
    for attempt in range(24):
        if t.monitor.game.won:break
        current=[int(round(float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0]))/90))%4 for a in m.dials];guess=m.decoder.choose(current)
        m.set_symbols(guess,finish_when_solved=True);print(f"{t.task['id']} feedback trial {t.monitor.game.queries}, candidates {len(m.decoder.candidates)}",flush=True)
    if not t.monitor.game.won:raise RuntimeError('Feedback decoder did not finish within its engineering query bound')
    t.event('decoder_solved',trials=t.monitor.game.queries,candidates=len(m.decoder.candidates));t.wait(.3)


def passive_release_probe(t, angle_deg=15.):
    """Independent low-angle free-release check, not a task success witness."""
    if t.task['level'] != 3 or not 5 <= angle_deg <= 30:
        raise ValueError('Probe needs a crank scene and a declared low angle')
    m = Manipulator(t)
    m.lift()
    a = m.dials[min(2, len(m.dials)-1)]
    m.grasp([a], [1]*len(m.dials))
    robot = m.grips[a['id']]['robot']
    arm = t.arms[robot]
    target = math.radians(angle_deg)
    for segment in range(12):
        m.wait_support([a])
        angle = float(t.data.joint(a['id']+'_hinge').qpos[0])
        if abs(target-angle) < math.radians(1):
            break
        jid = t.model.joint(a['id']+'_hinge').id
        anchor = t.data.xanchor[jid].copy()
        dr = axis_rotation(t.data.xaxis[jid], float(np.clip(target-angle, -math.radians(5), math.radians(5))))
        t.execute('passive-probe-set-'+str(segment), {robot:t.pose_command(
            robot, anchor+dr@(t.data.site_xpos[arm.site]-anchor),
            dr@t.data.site_xmat[arm.site].reshape(3,3), .025,.25)})
        t.wait(.08)
    else:
        raise RuntimeError('Low-angle probe positioning budget exhausted')
    before = float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0]))
    # Deliberately bypass controlled unloading to test the actual free release.
    BaselineManipulator.release(m, [a])
    after = float(np.degrees(t.data.joint(a['id']+'_hinge').qpos[0]))
    released = t.monitor.released(m.red_geoms(a))
    result = dict(dial=a['id'],angle_before_release_deg=before,angle_after_release_deg=after,
                  red_contact_released=bool(released),support_ready=bool(t.monitor.dial_ready[a['index']]),
                  scope='Independent low-angle passive release only, not all-angle passive return or game completion')
    result['probe_passed']=abs(before-angle_deg)<2 and abs(after)<4 and released and result['support_ready'] and not t.monitor.violations
    t.event('passive_low_angle_probe',**result)
    return result
