"""Private geometry-driven v2 acceptance; no policy observations or fixed pairs."""
import math
import numpy as np
import mujoco
from .evaluate import geom_extent


def mosaic_candidates(mon,positions,stable):
    """Same private predicates, evaluated once per object or in array batches.

    Geometry/contact thresholds, requirement equality and inclusive boundaries
    match the scalar reference. No physics state or task metadata is cached or
    changed; mutable test metadata is observed on every call.
    """
    task=mon.task;m,d=mon.m,mon.d;goals=task['goals'];n=len(mon.names)
    out=np.zeros((len(goals),n),bool)
    if not len(goals) or not n or not np.any(stable):return out
    out[:]=True
    keys=set().union(*(g['requires'] for g in goals))
    for key in keys:
        expected=np.fromiter((g['requires'].get(key) for g in goals),dtype=object,count=len(goals))
        actual=np.fromiter((o.get(key) for o in mon.objects),dtype=object,count=n)
        required=np.fromiter((key in g['requires'] for g in goals),dtype=bool,count=len(goals))
        out&=(~required[:,None])|(expected[:,None]==actual[None,:])
    rotations=d.xmat[np.asarray(mon.bodies,dtype=int)].reshape(n,3,3)
    aligned=np.min(np.max(abs(rotations),axis=1),axis=1)
    out&=(np.asarray(stable,dtype=bool)&~(aligned<math.cos(math.radians(8))))[None,:]
    targets=np.asarray([g['position_m'] for g in goals]);delta=np.asarray(positions)[None,:,:]-targets[:,None,:]
    if task['level']!=1:
        out&=~(np.linalg.norm(delta[:,:,:2],axis=2)>.005)
        out&=~(abs(delta[:,:,2])>.003)
        return out
    half=task['cube_size_m']/2*np.sum(abs(rotations),axis=2)
    out&=~np.any(abs(delta[:,:,:2])+half[None,:,:2]>task['grid']['pocket_opening_m']/2+.0003,axis=2)
    out&=~(abs(np.asarray(positions)[:,2]-half[:,2]-task['grid']['floor_z_m'])>.0008)[None,:]
    candidates=np.flatnonzero(out.any(axis=0))
    if not len(candidates):return out
    floor=m.geom('pocket_bottom').id;pairs,dist=mon.contacts()
    touches=set(pairs[(pairs[:,0]==floor)&(dist<=.0002),1].tolist())
    touches.update(pairs[(pairs[:,1]==floor)&(dist<=.0002),0].tolist())
    seated=np.zeros(n,bool)
    for j in candidates:
        root=m.geom(mon.objects[int(j)]['id']+'_geom').id
        seated[j]=root in touches or not(mujoco.mj_geomDistance(m,d,root,floor,.00021,None)>.0002)
    return out&seated[None,:]


def placement_candidates(mon,positions,stable):
    if mon.task['family']=='mosaic':return mosaic_candidates(mon,positions,stable)
    task=mon.task;m,d=mon.m,mon.d;goals=task['goals'];out=np.zeros((len(goals),len(mon.names)),bool)
    side=task['cube_size_m'];tol=.0003
    for i,g in enumerate(goals):
        target=np.asarray(g['position_m'])
        if task['family']=='transport':
            board=d.body(g['board']);br=board.xmat.reshape(3,3);bv=mon.velocity(board.id)
            if np.linalg.norm(bv[3:])>.01 or np.linalg.norm(bv[:3])>.1:continue
        for j,o in enumerate(mon.objects):
            if not stable[j] or any(o.get(k)!=v for k,v in g['requires'].items()):continue
            body=d.body(o['id']);r=body.xmat.reshape(3,3)
            p=br.T@(positions[j]-board.xpos);rr=br.T@r;local=np.asarray(g['local_position_m'])
            if abs(p[2]-local[2])>.0015 or rr[2,2]<math.cos(math.radians(5)):continue
            if task['level']==1:
                root=m.geom(o['id']+'_geom').id;gr=d.geom_xmat[root].reshape(3,3);typ=o['shape']
                extent=(side/2*np.sum(abs(br.T@gr),axis=1) if typ=='cube' else side/2*(abs((br.T@gr)[:,2])+np.sqrt(np.maximum(0,1-(br.T@gr)[:,2]**2))))
                if np.any(abs(p[:2]-local[:2])+extent[:2]>(side+.010)/2+tol):continue
            else:
                mechanism=task['mechanism'];spacing=mechanism['blade_spacing_m'];opening=np.asarray(mechanism['slot_opening_m'])/2;depth=mechanism['slot_depth_m'];valid=True
                for blade in (0,1):
                    gid=m.geom(o['id']+f'_blade{blade}').id;center=br.T@(d.geom_xpos[gid]-board.xpos);rot=br.T@d.geom_xmat[gid].reshape(3,3);extent=abs(rot)@m.geom_size[gid]
                    expected=np.array([(-1 if blade==0 else 1)*spacing/2,local[1]])
                    # 180-degree placement swaps the two identical blades.
                    nearest=min((np.array([-spacing/2,local[1]]),np.array([spacing/2,local[1]])),key=lambda q:np.linalg.norm(center[:2]-q))
                    if np.any(abs(center[:2]-nearest)+extent[:2]>opening+tol) or center[2]+extent[2]>.0005 or center[2]-extent[2]<-depth-tol:valid=False;break
                if not valid:continue
            out[i,j]=True
    return out


class Relay:
    def __init__(self,monitor):
        self.mon=monitor;self.table=monitor.task['table_height_m'];self.route=monitor.task['route'];self.states={}
        for item in monitor.task['mechanism']['payload_assemblies']:
            ident=item['id'];all_geoms=monitor.descendant_geoms(monitor.m.body(ident).id)
            for name in (item.get('weight_body'),item.get('tray_body')):
                if name:all_geoms+=monitor.descendant_geoms(monitor.m.body(name).id)
            # Articulated L3 has graspable timber on both sides of the hinge.
            # Suspension rings and the weight are never valid beam grasps.
            beam_ids={monitor.m.body(n).id for n in item.get('beam_bodies',[ident])}
            rod=[g for g in all_geoms if monitor.m.geom_bodyid[g] in beam_ids
                 and ('_beam' in (monitor.m.geom(g).name or ''))]
            self.states[ident]=dict(assembly=item,all_geoms=all_geoms,rod_geoms=rod,airborne=False,custody=set(),candidate=set(),candidate_since={},overlap_since=None,qualified=set(),gap_since=None,dropped=False,drops=0,transitions=[],goal_since=None,landed=False,spill=False)
        self.payload_components={};self.geom_component={}
        for ident,state in self.states.items():
            names=[ident]+[state['assembly'][key] for key in ('weight_body','tray_body') if state['assembly'].get(key)]
            self.payload_components[ident]=set()
            for name in names:
                geoms=monitor.descendant_geoms(monitor.m.body(name).id)
                if not geoms:continue
                self.payload_components[ident].add(name)
                for gid in geoms:self.geom_component[gid]=name
    def environment_contact(self,geoms):
        mon=self.mon;own=set(geoms)
        return any(c.dist<=0 and any(int(a) in own and (mon.m.geom(int(b)).name or '').startswith(('table_','floor','pocket_')) for a,b in ((c.geom1,c.geom2),(c.geom2,c.geom1))) for c in mon.d.contact)
    def inside(self,state):
        mon=self.mon;region=self.route[-1];yaw=region['nominal_yaw_rad'];c,s=math.cos(yaw),math.sin(yaw);r=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        center=np.asarray(region['center_m']);half=np.asarray(region.get('half_size_m',[.24,.65]))
        for gid in state['all_geoms']:
            rotation=mon.d.geom_xmat[gid].reshape(3,3);aabb=mon.m.geom_aabb[gid];p=mon.d.geom_xpos[gid]+rotation@aabb[:3];extent=abs(r.T@rotation)@aabb[3:];delta=r.T@(p-center)
            if np.any(abs(delta[:2])+extent[:2]>half+.002):return False
        return True
    def supported_payloads(self,goal_only=True,require_all=True,inside_ids=None):
        """Actual cargo contacts/structural components connected to tabletop.

        Robot contacts never ground a component. Independent free weights
        require their own contact path; grouping an assembly does not weld it.
        """
        eligible=set(self.states)
        if goal_only:
            eligible&=set(inside_ids) if inside_ids is not None else {name for name,state in self.states.items() if self.inside(state)}
        components={part for name in eligible for part in self.payload_components[name]}
        edges={part:set() for part in components};ground=set();mon=self.mon
        for contact in mon.d.contact:
            if contact.dist>0:continue
            a,b=map(int,contact.geom);left=self.geom_component.get(a);right=self.geom_component.get(b)
            if left in components and right in components and left!=right:
                edges[left].add(right);edges[right].add(left)
            for part,other in ((left,b),(right,a)):
                if part in components and (mon.m.geom(other).name or '').startswith('table_') and contact.pos[2]>=self.table-.01:ground.add(part)
        pending=list(ground)
        while pending:
            part=pending.pop()
            for neighbor in edges[part]-ground:ground.add(neighbor);pending.append(neighbor)
        return {name for name in eligible if (self.payload_components[name]<=ground if require_all else bool(self.payload_components[name]&ground))}
    def sample(self):
        mon=self.mon;t=float(mon.d.time);completed=0;reports={}
        inside_flags={name:self.inside(state) for name,state in self.states.items()}
        stacking=bool(mon.task['completion'].get('support_via_goal_payload'))
        global_ground=self.supported_payloads(False,False) if stacking else set()
        goal_ground=self.supported_payloads(True,True,{name for name,inside in inside_flags.items() if inside}) if stacking else set()
        for ident,state in self.states.items():
            a=state['assembly'];support=mon.bilateral(state['rod_geoms'],positive_force=True);all_g=state['all_geoms'];external=self.environment_contact(all_g) or ident in global_ground;on_table=ident in goal_ground if stacking else external;inside=inside_flags[ident]
            clear=mon.bottom(all_g)>self.table+.002 and ident not in global_ground
            if not state['airborne'] and mon.bottom(state['rod_geoms'])>self.table+.025:
                if len(support)<2:mon.violation('unsupported_initial_lift',key=ident)
                elif clear:state.update(airborne=True,custody=set(support))
            if state['airborne']:
                if len(support)<2 and not state['landed']:
                    if state['gap_since'] is None:state['gap_since']=t
                else:state['gap_since']=None
                if not state['dropped']:
                    fresh=support-state['custody']
                    # Qualify each arriving hand while the established team
                    # still supplies at least two supports. A contact loss
                    # invalidates that receiver's earlier overlap evidence.
                    state['qualified'].intersection_update(fresh)
                    state['candidate_since']={a:s for a,s in state['candidate_since'].items() if a in fresh}
                    if len(support&state['custody'])>=2:
                        for robot in fresh:
                            since=state['candidate_since'].setdefault(robot,t)
                            if t-since>=.20-1e-9:state['qualified'].add(robot)
                    state['candidate']=set(fresh)
                    state['overlap_since']=min(state['candidate_since'].values(),default=None)
                    if not state['custody']<=support and len(support)>=2:
                        if fresh<=state['qualified']:
                            old=set(state['custody'])
                            state['transitions'].append(dict(sim_time_s=t,senders=sorted(old),receivers=sorted(support),
                                retained=sorted(old&support),released=sorted(old-support),joined=sorted(fresh),protocol='qualified-members-v2'))
                            state['custody']=set(support);state['qualified']=set();state['candidate']=set();state['candidate_since']={};state['overlap_since']=None
                        else:mon.violation('unverified_handover',key=ident)
                if on_table and inside and len(support&state['custody'])>=2:state['landed']=True
                bad_contact=external and not(state['landed'] and inside)
                gap=state['gap_since'] is not None and t-state['gap_since']>.20
                if (bad_contact or gap) and not state['dropped']:state['dropped']=True;state['drops']+=1
                if state['dropped']:
                    if support-state['custody']:mon.violation('receiver_recovered_drop',key=ident)
                    if len(support&state['custody'])>=2 and clear:state['dropped']=False;state['gap_since']=None
            if a.get('tray_body'):
                tray=mon.d.body(a['tray_body']);weight=mon.d.body(a['weight_body']);delta=tray.xmat.reshape(3,3).T@(weight.xpos-tray.xpos);spec=mon.task['mechanism']['tray']
                if np.linalg.norm(delta[:2])>(spec['outer_diameter_m']-spec['weight_diameter_m'])/2+.002 or delta[2]<spec['weight_height_m']/2-.005 or delta[2]>.075:
                    state['spill']=True;mon.violation('tray_weight_spilled',key=ident)
            moving_names=list(dict.fromkeys(a.get('beam_bodies',[ident])+[name for name in (a.get('weight_body'),a.get('tray_body')) if name]))
            velocities=[mon.velocity(mon.m.body(name).id) for name in moving_names]
            stable=all(np.linalg.norm(v[3:])<.01 and np.linalg.norm(v[:3])<.1 for v in velocities)
            handover_ok=not mon.task['completion'].get('required_airborne_handover',False) or any(set(item['receivers'])-set(item['senders']) for item in state['transitions'])
            done=state['landed'] and inside and on_table and mon.released(all_g) and stable and not state['dropped'] and not state['spill'] and handover_ok
            state['goal_since']=(t if state['goal_since'] is None else state['goal_since']) if done else None
            qualified=state['goal_since'] is not None and t-state['goal_since']>=mon.task['completion']['stable_duration_sim_s']
            completed+=int(qualified);reports[ident]=dict(completed=qualified,airborne=state['airborne'],drops=state['drops'],custody=sorted(state['custody']),support=sorted(support),goal_table_support=on_table,handovers=state['transitions'],spill=state['spill'])
        complete=completed==len(self.states)
        mon.result_cache=dict(all_goals_met=complete,task_success=complete and not mon.violations,completed_payloads=completed,total_payloads=len(self.states),payloads=reports,drops=sum(s['drops'] for s in self.states.values()))
