"""Beam task adapter and private, incremental process evaluation; no task solver.

Only public_beam_task crosses into policy inputs. Contact/pose records below are
private audit evidence, never robot observations or motion-check inputs.
"""
from copy import deepcopy
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .scene_task import ROOT, validate_scene_asset


HARD_FAILURES={
    'inter_robot_collision':'Cross-robot penetration beyond the declared tolerance invalidates compliance.',
    'incomplete_process_sampling':'Missing process evidence prevents a successful score.',
    'payload_nonfinger_contact':'Payload penetration into a non-finger robot body invalidates compliance.',
    'pickup_by_non_sender':'Initial pickup must use the declared initial sending team.',
    'unsupported_initial_lift':'Establish the required support before the initial lift.',
    'recovery_by_non_sender':'Only the established sending team may retrieve a drop; existing receivers may release, not assist recovery.',
    'unverified_handover':'A replacement supporting team must establish the declared overlap before the retiring sender releases.',
    'unsupported_final_landing':'Maintain required support until landing in the finish zone.',
}
PLANNING_HINTS=[
    'Collision risk includes elbows, wrists and links, not only fingertips; assess full-arm motion and payload clearance.',
    'Current grasp locations, payload pose and joint configurations affect the next team\'s approach, simultaneous support and sender withdrawal. Choose alternatives yourself; no grasp spacing, grouping or route is prescribed.',
]


def evaluation_rules():
    return dict(invalidating_events=deepcopy(HARD_FAILURES),
                recoverable_events={'drop_attempt':'Counts as a failed transfer attempt; legal sender recovery and retry may still complete the episode.'},
                finish='Meet the declared whole-object containment, release, stable-duration and handover criteria without any invalidating event.',
                evidence='Bilateral positive-force contact plus retained airborne motion is a support proxy, not measured per-robot load. Rules/thresholds are public; live evaluator state is private.')


def make_beam_task(directory):
    directory=Path(directory).resolve()
    env=json.loads((directory/'environment.json').read_text())
    draft=json.loads((directory/'task-draft.json').read_text())
    task=dict(kind='beam_handoff',id=env['id']+'-policy-v1',status='policy task v1; engineering integration, not a model result',
              scene_asset=str(directory.relative_to(ROOT)),scene_sha256=hashlib.sha256((directory/'scene.xml').read_bytes()).hexdigest(),
              arms={s['id']:{k:s[k] for k in ('base_position_m','yaw_rad')} for s in env['stations']},
              objects=[dict(id=env['beam']['id'],shape='box',dimensions_m=[env['beam']['width_m'],env['beam']['length_m'],env['beam']['height_m']],
                            mass_kg=env['beam']['mass_kg'],initial_position_m=env['beam']['initial_center_world_m'])],
              beam=deepcopy(env['beam']),goal=deepcopy(env['goal']),table_height_m=env['stations'][0]['base_position_m'][2],
              initial_custody=['A','B'],rules=deepcopy(draft['rules']),coordinate_frame=draft['coordinate_frame'],
              workspace_world_m=dict(lower=[-1.8,-1.1,.405],upper=[1.8,1.1,1.2]),
              cameras=[c.get('name') for c in ET.parse(directory/'scene.xml').findall('worldbody/camera')],
              observation_contract=dict(initial_positions_public=True,initial_geometry_public=True,
                                        policy=['RGB','robot_proprioception','action_status'],live_object_truth_for_policy=False,
                                        motion_screen='Optional robot-only nominal geometry; no payload/table/dynamics guarantee'),
              completion=dict(stable_duration_sim_s=1.,max_sample_gap_sim_s=.021,
                              max_linear_speed_m_s=.01,max_angular_speed_rad_s=.05,
                              containment_tolerance_m=.001,minimum_handovers=1),
              process_criteria=dict(minimum_grippers=2,minimum_finger_normal_force_n=.05,
                                    support_gap_tolerance_s=.20,handover_overlap_s=.20,
                                    contact_tolerance_m=.0001,airborne_clearance_m=.005,
                                    collision_penetration_tolerance_m=.0001))
    task.update(evaluation_rules=evaluation_rules(),planning_hints=list(PLANNING_HINTS))
    return validate_beam_task(task)


def validate_beam_task(task):
    if task.get('kind')!='beam_handoff':raise ValueError('Expected beam_handoff')
    folder,env=validate_scene_asset(task)
    beam=task['beam'];obj=task['objects']
    expected=dict(id=beam['id'],shape='box',dimensions_m=[beam['width_m'],beam['length_m'],beam['height_m']],
                  mass_kg=beam['mass_kg'],initial_position_m=beam['initial_center_world_m'])
    if beam!=env['beam'] or obj!=[expected] or task['goal']!=env['goal']:
        raise ValueError('Beam/goal differs from generated scene')
    if task['table_height_m']!=env['stations'][0]['base_position_m'][2]:raise ValueError('Table height mismatch')
    geom=ET.parse(folder/'scene.xml').find(f"worldbody/body[@name='{beam['id']}']/geom")
    dimensions=[] if geom is None else [float(x)*2 for x in geom.get('size','').split()]
    if len(dimensions)!=3 or not all(math.isclose(a,b,rel_tol=0,abs_tol=1e-9) for a,b in zip(dimensions,expected['dimensions_m'])):
        raise ValueError('Beam collision dimensions mismatch')
    if abs(float(geom.get('mass'))-beam['mass_kg'])>1e-9:raise ValueError('Beam mass mismatch')
    custody=task['initial_custody'];p=task['process_criteria'];c=task['completion']
    if not isinstance(custody,list) or len(custody)!=len(set(custody)) or not set(custody)<=set(task['arms']):
        raise ValueError('Invalid initial custody')
    if type(p['minimum_grippers']) is not int or not 2<=p['minimum_grippers']<=len(custody):raise ValueError('Invalid support count')
    if type(c['minimum_handovers']) is not int or c['minimum_handovers']<1:raise ValueError('Require an airborne handover')
    for section,excluded in ((p,{'minimum_grippers'}),(c,{'minimum_handovers'})):
        for k,v in section.items():
            if k not in excluded and (isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0):
                raise ValueError('Invalid beam criterion: '+k)
    if not .02<=c['max_sample_gap_sim_s']<=p['support_gap_tolerance_s']:raise ValueError('Invalid sampling contract')
    if c['containment_tolerance_m']>.002 or p['collision_penetration_tolerance_m']>.001:
        raise ValueError('Excessive numerical tolerance')
    if task['observation_contract'].get('live_object_truth_for_policy') is not False:
        raise ValueError('Beam policy cannot receive live object truth')
    if task.get('evaluation_rules')!=evaluation_rules():raise ValueError('Public scoring rules differ from the evaluator')
    hints=task.get('planning_hints',[])
    if not isinstance(hints,list) or any(not isinstance(h,str) for h in hints):raise ValueError('Planning hints must be text')
    return task


def public_beam_task(task):
    keys=('kind','id','status','arms','objects','beam','goal','table_height_m','initial_custody','rules',
          'coordinate_frame','workspace_world_m','cameras','observation_contract','completion','process_criteria','evaluation_rules')
    result={k:deepcopy(task[k]) for k in keys}
    result['planning_hints']=deepcopy(task.get('planning_hints',[]))
    if not task['observation_contract']['initial_positions_public']:
        result['objects'][0].pop('initial_position_m',None)
        for k in ('initial_center_world_m','initial_center_xy_m'):result['beam'].pop(k,None)
    result['evaluation_scope']='Private process scoring. Bilateral positive-force contact plus retained airborne motion is a support proxy, not measured per-robot load. No live score/pose/contact feedback.'
    result['objective']='Complete the airborne cooperative relay correctly, then reduce elapsed time, avoidable motion, holding and decision/coordination overhead. No prescribed grasp spacing, grouping or route.'
    return result


class BeamEvaluator:
    """Evaluate observations, not Agent stage declarations or a fixed relay order."""
    def __init__(self,task):
        self.task=task;self.p=task['process_criteria'];self.c=task['completion']
        self.custody=set(task['initial_custody']);self.airborne=False;self.recovering=False
        self.last=None;self.ready={};self.overlap={};self.gap_since=None;self.stable_since=None
        self.recovery_receivers=set()
        self.handovers=[];self.drops=0;self.violations={};self.events=[];self.sample_count=0

    def event(self,t,kind,**data):self.events.append(dict(time_s=t,event=kind,**data))

    def violation(self,t,kind):
        if kind not in HARD_FAILURES:raise ValueError('Undeclared invalidating event: '+kind)
        if kind not in self.violations:self.event(t,'violation',violation=kind)
        self.violations[kind]=self.violations.get(kind,0)+1

    def collision(self,t,bodies,depth):
        self.violation(t,'inter_robot_collision')
        if self.violations['inter_robot_collision']==1:self.event(t,'first_inter_robot_collision',bodies=bodies,depth_m=depth)

    def feed(self,s):
        t=s['time_s'];previous=self.last;self.last=deepcopy(s);self.sample_count+=1
        if previous and (t<=previous['time_s'] or t-previous['time_s']>self.c['max_sample_gap_sim_s']):
            self.violation(t,'incomplete_process_sampling')
        grip=set(s['gripping_robots']);minimum=self.p['minimum_grippers']
        table=s['table_contact'];floor=s['floor_contact'];flying=not(table or floor) and s['bottom_z_m']>self.task['table_height_m']+self.p['airborne_clearance_m']
        if s.get('unexpected_robot_contact'):self.violation(t,'payload_nonfinger_contact')
        if not self.airborne:
            if grip-self.custody:self.violation(t,'pickup_by_non_sender')
            if flying:
                if len(grip&self.custody)>=minimum:
                    self.airborne=True;self.event(t,'initial_airborne_support',custody=sorted(self.custody))
                else:self.violation(t,'unsupported_initial_lift')
        elif self.recovering:
            receivers=grip-self.custody
            if receivers-self.recovery_receivers or (flying and receivers):
                self.violation(t,'recovery_by_non_sender')
            self.recovery_receivers&=receivers
            if flying and not receivers and len(grip&self.custody)>=minimum:
                self.recovering=False;self.gap_since=None;self.ready={};self.overlap={}
                self.event(t,'sender_recovery_lift',custody=sorted(self.custody))
        else:
            newcomers=grip-self.custody
            # Each subset must already have co-supported with a sender before
            # release. No assumption about adjacent station pairs or grasp pose.
            combinations={k for k in itertools.combinations(sorted(grip),minimum) if set(k)-self.custody}
            if flying:
                if grip&self.custody:
                    for pair in combinations:self.overlap.setdefault(pair,t)
                    self.overlap={k:v for k,v in self.overlap.items() if k in combinations}
                    self.ready={k:v for k,v in self.overlap.items() if t-v>=self.p['handover_overlap_s']-1e-9}
                # A valid relay can replace one member while another keeps
                # supporting; do not hard-code two new receivers or pairs.
                retired=self.custody-set(s.get('contacting_robots',grip))
                if retired:
                    valid=[k for k in self.ready if set(k)<=grip]
                    if valid:
                        old=sorted(self.custody);self.custody=set().union(*(set(k) for k in valid))
                        h=dict(time_s=t,sender=old,receiver=sorted(self.custody));self.handovers.append(h)
                        self.event(t,'handover',sender=old,receiver=sorted(self.custody));self.ready={};self.overlap={}
                    elif newcomers and len(grip)>=minimum:self.violation(t,'unverified_handover')
            else:
                self.ready={};self.overlap={}
            if len(grip)<minimum and not (table and s['inside_goal']):
                if self.gap_since is None:self.gap_since=t
            else:self.gap_since=None
            dropped=floor or (table and not s['inside_goal']) or (self.gap_since is not None and t-self.gap_since>self.p['support_gap_tolerance_s'])
            if dropped:
                self.drops+=1;self.recovering=True;self.ready={};self.overlap={}
                self.event(t,'drop_attempt',custody=sorted(self.custody),attempt=self.drops)
                # Already-attached receivers may let go after a failed handoff;
                # that is not a new pickup. They cannot acquire a new hold or
                # help lift the dropped payload during sender recovery.
                self.recovery_receivers=(set(previous['gripping_robots'])&grip)-self.custody if previous else set()
                if (grip-self.custody)-self.recovery_receivers:self.violation(t,'recovery_by_non_sender')
            # Landing at the finish is exempt only if support was maintained
            # until near the surface, not merely by declaring a final phase.
            if table and s['inside_goal'] and previous and not previous['table_contact']:
                if len(set(previous['gripping_robots'])&self.custody)<minimum:
                    self.violation(t,'unsupported_final_landing')
        stable=(self.airborne and not self.recovering and s['inside_goal'] and table and not floor
                and not grip and s['released'] and s['linear_speed_m_s']<=self.c['max_linear_speed_m_s']
                and s['angular_speed_rad_s']<=self.c['max_angular_speed_rad_s'])
        if stable:
            if self.stable_since is None:self.stable_since=t
        else:self.stable_since=None

    def result(self):
        t=self.last['time_s'] if self.last else 0
        duration=0 if self.stable_since is None else t-self.stable_since
        done=bool(self.last and duration>=self.c['stable_duration_sim_s']-1e-9 and not self.violations
                  and len(self.handovers)>=self.c['minimum_handovers'])
        return dict(all_goals_met=done,task_success=done,sim_time_s=t,stable_duration_s=duration,
                    airborne_handovers=deepcopy(self.handovers),failed_transfer_attempts=self.drops,
                    recovering=self.recovering,custody=sorted(self.custody),violations=dict(self.violations),
                    samples=self.sample_count,events=deepcopy(self.events),last_sample=deepcopy(self.last),
                    scope='Private process evaluation; contact support proxy, not true load attribution. Collisions sampled each physics step; support/pose at control ticks.')


class BeamSampler:
    """MuJoCo-only sensor adapter; no policy interface exposes these values."""
    def __init__(self,world):
        import numpy as np
        self.world=world;self.evaluator=BeamEvaluator(world.task)
        output=Path(world.directory)/'private/outputs/beam-process.jsonl'
        output.parent.mkdir(parents=True,exist_ok=True)
        self.log=output.open('x');self.next_record=world.time
        m=world.model;self.geom=m.geom(world.task['beam']['id']+'_geom').id
        self.body=m.geom_bodyid[self.geom];self.owners=np.full(m.ngeom,-1,dtype=int);self.names=list(world.arms)
        from .motion_check import RobotGeometry
        geometry=RobotGeometry(m,world.arms)
        for i,a in enumerate(self.names):self.owners[geometry.geoms[a]]=i

    def check_collisions(self):
        import numpy as np
        w=self.world;c=w.data.contact
        if not len(c):return
        a,b=self.owners[c.geom1],self.owners[c.geom2]
        hits=np.flatnonzero((a>=0)&(b>=0)&(a!=b)&(c.dist < -self.evaluator.p['collision_penetration_tolerance_m']))
        if len(hits):
            hit=c[int(hits[0])]
            self.evaluator.collision(w.time,[w.model.body(w.model.geom_bodyid[g]).name for g in (hit.geom1,hit.geom2)],float(-hit.dist))

    def sample(self):
        import mujoco
        import numpy as np
        w=self.world;m,d=w.model,w.data;p=self.evaluator.p
        body=d.body(self.body);r=body.xmat.reshape(3,3);half=m.geom_size[self.geom]
        extent=np.abs(r)@half;pos=body.xpos;bottom=float(pos[2]-extent[2]);velocity=np.zeros(6)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,self.body,velocity,0)
        fingers={a:set() for a in self.names};table=False;floor=False;released=True;unexpected=False;contacting=set()
        for i,c in enumerate(d.contact):
            if self.geom not in (c.geom1,c.geom2) or c.dist>p['contact_tolerance_m']:continue
            other=int(c.geom2 if c.geom1==self.geom else c.geom1);name=m.geom(other).name or ''
            table|=name.startswith('table_') or name=='table';floor|=name=='floor'
            index=int(self.owners[other])
            if index>=0:
                released=False;robot=self.names[index];contacting.add(robot);bname=m.body(m.geom_bodyid[other]).name
                if bname in (robot+'_left_finger',robot+'_right_finger'):
                    force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                    if force[0]>p['minimum_finger_normal_force_n']:fingers[robot].add(bname)
                elif c.dist < -p['contact_tolerance_m']:unexpected=True
        goal=w.task['goal'];inside=bool(np.all(np.abs(pos[:2]-goal['center_xy_m'])+extent[:2]
                         <=np.array([goal['zone_length_x_m'],goal['zone_width_y_m']])/2+self.evaluator.c['containment_tolerance_m']))
        s=dict(time_s=w.time,position_m=pos.tolist(),quaternion_wxyz=body.xquat.tolist(),bottom_z_m=bottom,
               table_contact=bool(table),floor_contact=bool(floor),released=released,inside_goal=inside,
               gripping_robots=[a for a,v in fingers.items() if len(v)==2],unexpected_robot_contact=unexpected,
               contacting_robots=sorted(contacting),
               linear_speed_m_s=float(np.linalg.norm(velocity[3:])),angular_speed_rad_s=float(np.linalg.norm(velocity[:3])))
        self.evaluator.feed(s)
        if w.time+1e-9>=self.next_record:
            self.log.write(json.dumps(s,allow_nan=False)+'\n');self.log.flush()
            self.next_record=w.time+.1
        return s

    def close(self):
        self.log.close()
