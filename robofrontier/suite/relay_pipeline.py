"""Private rolling Relay frontier with jointly screened finite carry chunks.

One Trial remains the sole physics owner. Planning and all FK state changes
are scratch-only. Independent payloads may share an actuator submission only
after simultaneous full-arm/full-payload screening; per-payload milestones
advance independently. This is an engineering Oracle, not a policy tool.
"""
from copy import copy,deepcopy
from dataclasses import dataclass, field
from itertools import combinations
import math
import time

import mujoco
import numpy as np

from ..robot_control import matrix_quat,rotation_vector
from ..run_setup import write_json
from . import relay_oracle_v2 as ref
from .oracle import ReferenceStopped
from .scenes import rz


@dataclass
class Frontier:
    ident: str
    number: int
    index: int
    phase: str='pickup'
    team: list=field(default_factory=list)
    offset: object=.44
    target: object=None
    yaw: float=0.
    following: str=''
    future: dict=field(default_factory=dict)
    receiver_offset: object=None
    blocked: str=''
    blocked_at: object=None
    milestone: int=0
    chunks: int=0
    extra_prefix: bool=False
    recheck_since: object=None


def snapshot(t,frontiers):
    actual=t.monitor.result().get('payloads',{})
    write_json(t.directory/'frontier.json',dict(sim_time_s=t.time,wall_elapsed_s=time.monotonic()-t.started,
        scopes=[dict(payload=f.ident,phase=f.phase,assigned_robots=f.team,holders=actual.get(f.ident,{}).get('support',[]),route_index=f.index,
            target_m=None if f.target is None else np.asarray(f.target).tolist(),yaw_rad=f.yaw,following=f.following,
            blocked=f.blocked,milestone=f.milestone,carry_chunks=f.chunks) for f in frontiers],
        payload_motion=getattr(t,'relay_motion_metrics',{})))


def finalize_frontier(t):
    frontiers=getattr(t,'relay_frontiers',None)
    if frontiers is None:return
    actual=t.monitor.result().get('payloads',{})
    for f in frontiers:
        if actual.get(f.ident,{}).get('completed'):
            f.phase='done';f.blocked='';f.blocked_at=None
    snapshot(t,frontiers)


def route_progress(route,point):
    points=np.array([r['center_m'][:2] for r in route]);point=np.asarray(point)[:2]
    best=None;length=0.
    for first,second in zip(points[:-1],points[1:]):
        delta=second-first;span=float(np.linalg.norm(delta))
        fraction=float(np.clip(np.dot(point-first,delta)/max(span*span,1e-12),0,1))
        candidate=(float(np.linalg.norm(point-first-fraction*delta)),length+fraction*span)
        if best is None or candidate[0]<best[0]:best=candidate
        length+=span
    return best[1] if best else 0.


def extra_prefix_plan(t,frontier,budget):
    """Add a private computational anchor when actual custody is upstream.

    The copied task is used only for FK candidate generation. The live task,
    monitor, public anchors and final GOAL are never mutated or regraded.
    """
    f=frontier;root=t.data.body(f.ident).xpos.copy();goal=np.asarray(t.task['route'][-1]['center_m'])
    free=[a for a in t.arms if a not in f.team and a not in t.held]
    bases={a:np.asarray(t.task['arms'][a]['base_position_m']) for a in t.arms}
    current=np.mean([bases[a] for a in f.team],axis=0)
    anchors=np.array([ref.receiving_anchor(t,r['center_m']) for r in t.task['route']])
    spacing=float(np.median(np.linalg.norm(np.diff(anchors[:,:2],axis=0),axis=1)))
    candidates=[]
    for first,second in combinations(free,2):
        anchor=(bases[first]+bases[second])/2;anchor[2]=t.task['table_height_m']
        if np.linalg.norm(anchor[:2]-current[:2])>1.5*spacing+.02:continue
        if route_progress(t.task['route'],anchor)<=route_progress(t.task['route'],root)+.02:continue
        if not any(np.linalg.norm(anchor[:2]-old[:2])<.02 for old in candidates):candidates.append(anchor)
    candidates.sort(key=lambda anchor:np.linalg.norm(anchor[:2]-root[:2]))
    started=time.monotonic();last=None
    # Bind the manufactured heading offset once; a computational intermediate
    # anchor preserves the observed load yaw instead of forcing a reversal.
    ref.payload_route_yaw(t,f.ident,t.task['route'][f.index])
    observed=t.data.body(f.ident).xmat.reshape(3,3);yaw=math.atan2(observed[1,0],observed[0,0])-t.relay_payload_heading_offsets[f.ident]
    for anchor in candidates[:4]:
        remaining=budget-(time.monotonic()-started)
        if remaining<=0:break
        probe=copy(t);probe.task=deepcopy(t.task);index=f.index+1
        virtual=dict(region_id='private-extra-prefix',center_m=anchor.tolist(),nominal_yaw_rad=yaw,painted=False)
        probe.task['route']=probe.task['route'][:index]+[virtual]+probe.task['route'][index:]
        depth=2 if t.task['level']>1 or t.task['mechanism']['layout']!='straight' else 1
        try:return ref.handover_plan(probe,f.ident,f.team,f.offset,index,search_budget_s=min(remaining,90. if depth==2 else 45.),lookahead_depth=depth)
        except ReferenceStopped:raise
        except RuntimeError as error:last=error
    raise RuntimeError('No bounded extra receiving prefix '+f.ident+': '+str(last))


def plan_transfer(t,f,frontiers,flexible):
    route=t.task['route'];nominal=ref.receiving_anchor(t,route[f.index]['center_m'])
    current=np.mean([np.asarray(t.task['arms'][a]['base_position_m']) for a in f.team],axis=0)
    anchors=np.array([ref.receiving_anchor(t,r['center_m'])[:2] for r in route])
    spacing=float(np.median(np.linalg.norm(np.diff(anchors,axis=0),axis=1)))
    upstream=route_progress(route,nominal)-route_progress(route,current)>.5*spacing+.02
    if upstream:
        try:return extra_prefix_plan(t,f,ref.STAGE_SEARCH_WALL_S),True
        except ReferenceStopped:raise
        except RuntimeError:pass
    planner=ref.flexible_handover_plan if flexible else ref.handover_plan
    shift=packing_shift(t,f,frontiers) if f.index+1==len(route)-1 else 0.
    completed_peer=any(t.monitor.result().get('payloads',{}).get(other.ident,{}).get('completed') for other in frontiers if other is not f)
    complex_stage=t.task['level']>1 or t.task['mechanism']['layout']!='straight' or completed_peer
    budget=ref.STAGE_SEARCH_WALL_S if complex_stage else min(40.,ref.STAGE_SEARCH_WALL_S)
    try:return planner(t,f.ident,f.team,f.offset,f.index+1,search_budget_s=budget,landing_shift_m=shift),False
    except ReferenceStopped:raise
    except RuntimeError:
        if upstream:raise
        return extra_prefix_plan(t,f,ref.STAGE_SEARCH_WALL_S),True


def direct_terminal_plan(t,f,frontiers):
    """A qualified current team may finish without another nominal relay."""
    if not ref.goal_contact_allowed(t):return None
    state=t.monitor.relay_v2.states[f.ident]
    if state['dropped'] or not state['transitions']:return None
    if not set(f.team)<=t.monitor.bilateral(state['rod_geoms'],positive_force=True):return None
    body=t.data.body(f.ident);actual=body.xpos.copy();r=body.xmat.reshape(3,3)
    if t.monitor.relay_v2.inside(state):return actual,math.atan2(r[1,0],r[0,0])
    region=t.task['route'][-1];frame=rz(region['nominal_yaw_rad']);yaw=ref.payload_route_yaw(t,f.ident,region)
    nominal=(f.number-(len(frontiers)-1)/2)*.12-.045+packing_shift(t,f,frontiers)
    for shift in dict.fromkeys([nominal,0.,-.12,.12,-.24,.24]):
        for height in dict.fromkeys([float(actual[2]),t.task['table_height_m']+.45,t.task['table_height_m']+.55]):
            point=np.asarray(region['center_m'])+frame@np.array([shift,0,0]);point[2]=height
            if not ref.carry_clear(t,f.ident,f.team,point,yaw):continue
            probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
            probe.data.qpos[:]=t.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
            if f.ident not in ref.terminal_contact_names(probe,f.ident,probe.data):continue
            landing=ref.support_approach_point(probe,f.ident,point,yaw)
            if ref.carry_clear(probe,f.ident,f.team,landing,yaw,landing=True):return point,yaw
    return None


def simultaneous_carry_clear(t,requests):
    """Sample all proposed payload/tool motions together, never live data."""
    robots=[a for request in requests for a in request['team']]
    if len(set(robots))!=len(robots):return False
    ends=[]
    for request in requests:
        if not ref.carry_clear(t,request['ident'],request['team'],request['point'],request['yaw'],target_rotation=request['rotation']):return False
        ends.append(t.relay_carry_endpoint)
    scratch=mujoco.MjData(t.model);moving={list(t.arms).index(a) for a in robots}
    owned={g:(request['ident'],set(request['team'])) for request in requests
           for g in t.monitor.relay_v2.states[request['ident']]['all_geoms']}
    # Per-load screens already check the nonlinear TCP interpolation. This
    # adds a joint sweep of every simultaneous arm and predicted payload.
    for fraction in np.linspace(0,1,11)[1:]:
        scratch.qpos[:]=t.data.qpos
        for request,end in zip(requests,ends):
            for robot in request['team']:
                arm=t.arms[robot];scratch.qpos[arm.qids]=(1-fraction)*t.data.qpos[arm.qids]+fraction*end['q'][robot]
            state=t.monitor.relay_v2.states[request['ident']]
            for name in (request['ident'],state['assembly'].get('weight_body'),state['assembly'].get('tray_body')):
                if not name:continue
                joint=scratch.joint(name+'_free');old=t.data.joint(name+'_free').qpos;new=end['qpos'][t.model.jnt_qposadr[t.model.joint(name+'_free').id]:][:7]
                joint.qpos[:3]=(1-fraction)*old[:3]+fraction*new[:3]
                start_q=old[3:];end_q=new[3:].copy();dot=float(np.dot(start_q,end_q))
                if dot<0:end_q=-end_q;dot=-dot
                angle=math.acos(np.clip(dot,-1.,1.))
                quat=(1-fraction)*start_q+fraction*end_q if angle<1e-8 else (math.sin((1-fraction)*angle)*start_q+math.sin(fraction*angle)*end_q)/math.sin(angle)
                joint.qpos[3:]=quat/np.linalg.norm(quat)
        mujoco.mj_forward(t.model,scratch)
        for contact in scratch.contact:
            if contact.dist>=-.0003:continue
            a,b=map(int,contact.geom);oa,ob=t.monitor.geom_owner[a],t.monitor.geom_owner[b]
            if a in owned or b in owned:
                if a in owned and b in owned and owned[a][0]==owned[b][0]:continue
                payload,team=owned[a] if a in owned else owned[b]
                other=b if a in owned else a;name=t.model.body(int(t.model.geom_bodyid[other])).name
                if any(name in (robot+'_left_finger',robot+'_right_finger') for robot in team):continue
                if ref.allowed_terminal_payload_contact(t,payload,other,scratch):continue
                return False
            if oa>=0 and ob>=0:
                if oa!=ob and (oa in moving or ob in moving):return False
                if oa==ob and oa in moving and contact.dist<-.001:
                    names=[t.model.body(int(t.model.geom_bodyid[g])).name for g in (a,b)]
                    if not all('finger' in name for name in names):return False
            for geom,other,owner,other_owner in ((a,b,oa,ob),(b,a,ob,oa)):
                if owner in moving and other_owner<0:
                    name=t.model.body(int(t.model.geom_bodyid[geom])).name
                    if name.endswith('_link0'):continue
                    permitted=False
                    for request in requests:
                        if any(name in (robot+'_left_finger',robot+'_right_finger') for robot in request['team']):
                            permitted=ref.allowed_terminal_payload_contact(t,request['ident'],other,scratch)
                            if permitted:break
                    if not permitted:return False
    return True


def packing_shift(t,frontier,frontiers):
    """Rank finite in-region landings away from already delivered loads."""
    result=t.monitor.result().get('payloads',{});region=t.task['route'][-1];frame=rz(region['nominal_yaw_rad'])
    peers=[float((frame.T@(t.data.body(other.ident).xpos-region['center_m']))[0]) for other in frontiers
           if other is not frontier and result.get(other.ident,{}).get('completed')]
    if not peers:return 0.
    nominal=(frontier.number-(len(frontiers)-1)/2)*.12-.045
    body=t.data.body(frontier.ident);relative=rz(ref.payload_route_yaw(t,frontier.ident,region))@body.xmat.reshape(3,3).T
    half=np.asarray(region['half_size_m'])
    valid=[]
    for shift in [0.,-.06,-.12,.06,.12]:
        anchor=np.asarray(region['center_m'])+frame@np.array([nominal+shift,0,0])
        inside=True
        for gid in t.monitor.relay_v2.states[frontier.ident]['all_geoms']:
            rotation=t.data.geom_xmat[gid].reshape(3,3);bounds=t.model.geom_aabb[gid]
            center=anchor+relative@(t.data.geom_xpos[gid]+rotation@bounds[:3]-body.xpos)
            extent=abs(frame.T@relative@rotation)@bounds[3:]
            if np.any(abs((frame.T@(center-region['center_m']))[:2])+extent[:2]>half+.002):inside=False;break
        if inside:valid.append((min(abs(nominal+shift-peer) for peer in peers),-abs(shift),shift))
    return max(valid)[2] if valid else 0.


def terminal_exit_preview(t,ident,team,endpoint):
    """A lowering forecast must leave room for real opening and withdrawal."""
    if t.task.get('completion',{}).get('allow_goal_payload_contact'):return True
    probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
    probe.data.qpos[:]=endpoint['qpos'];probe.arms={a:copy(v) for a,v in t.arms.items()}
    mujoco.mj_forward(t.model,probe.data)
    for robot in team:probe.arms[robot].qref=probe.data.qpos[t.arms[robot].qids].copy()
    return ref.donor_exit_routes(probe,ident,[],team,opening_width=.08) is not None


def carry_chunk(t,frontier):
    if frontier.following=='place':
        relay=t.monitor.relay_v2;state=relay.states[frontier.ident]
        support=t.monitor.bilateral(state['rod_geoms'],positive_force=True)
        if not state['dropped'] and set(frontier.team)<=support and relay.inside(state):
            # The painted GOAL is the task condition. A private packing-center
            # estimate is not an extra precision gate. Still preserve an
            # actual descent corridor past already delivered payloads.
            body=t.data.body(frontier.ident);rotation=body.xmat.reshape(3,3).copy()
            gap=t.monitor.bottom(state['all_geoms'])-t.task['table_height_m']
            landing=body.xpos.copy();landing[2]-=max(0.,gap-.01)
            yaw=math.atan2(rotation[1,0],rotation[0,0])
            if ref.carry_clear(t,frontier.ident,frontier.team,landing,yaw,target_rotation=rotation):
                if terminal_exit_preview(t,frontier.ident,frontier.team,t.relay_carry_endpoint):return None
            plan=ref.terminal_reposition_plan(t,frontier.ident,frontier.team)
            if plan is not None:
                probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
                if ref.carry_clear(t,frontier.ident,frontier.team,plan['target_m'],plan['yaw'],target_rotation=plan['rotation']):
                    probe.data.qpos[:]=t.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
                    if ref.carry_clear(probe,frontier.ident,frontier.team,plan['predicted_descent_target_m'],plan['yaw'],target_rotation=plan['rotation']):
                        if terminal_exit_preview(probe,frontier.ident,frontier.team,probe.relay_carry_endpoint):return None
    body=t.data.body(frontier.ident);point=body.xpos.copy();rotation=body.xmat.reshape(3,3).copy()
    desired=rz(frontier.yaw);delta=rotation_vector(desired,rotation);angle=float(np.linalg.norm(delta))
    distance=float(np.linalg.norm(frontier.target-point))
    if distance<.003 and angle<.01:return None
    fraction=min(1.,.025/max(distance,1e-9),.06/max(angle,1e-9))
    from scripts.verify_floating_dial_oracle import axis_rotation
    frame=axis_rotation(delta/max(angle,1e-9),angle*fraction)@rotation if angle>1e-9 else rotation
    # Carry yaw interpolation follows the same level-target reference. Small
    # measured pitch/roll are corrected by the ordinary Cartesian controller.
    heading=math.atan2(frame[1,0],frame[0,0]);target=point+fraction*(frontier.target-point)
    return dict(ident=frontier.ident,team=frontier.team,point=target,yaw=heading,rotation=frame)


def execute_carry_chunks(t,requests):
    """Choose a compatible concurrent set, deferring only conflicting work."""
    selected=[]
    for request in requests:
        if simultaneous_carry_clear(t,selected+[request]):selected.append(request)
    if not selected:return []
    commands={}
    for request in selected:
        body=t.data.body(request['ident']);p=body.xpos.copy();r=body.xmat.reshape(3,3)
        frame=request['rotation']
        for robot in request['team']:
            arm=t.arms[robot];offset=r.T@(t.data.site_xpos[arm.site]-p);relative=r.T@t.data.site_xmat[arm.site].reshape(3,3)
            commands[robot]=t.pose_command(robot,request['point']+frame@offset,frame@relative,.035,.25)
    t.event('relay_frontier_carry',payloads=[r['ident'] for r in selected],robots=list(commands))
    t.execute('relay-frontier-carry',commands)
    for request in selected:
        state=t.monitor.relay_v2.states[request['ident']]
        if state['dropped']:raise RuntimeError('Physical support lost in concurrent carry '+request['ident'])
    return [request['ident'] for request in selected]


def install_motion_metrics(t):
    """Count observed supported movement of different loads at20ms windows."""
    if hasattr(t,'relay_motion_metrics'):return
    original=t.step;last=t.time
    positions={ident:t.data.body(ident).xpos.copy() for ident in t.monitor.relay_v2.states}
    metrics=dict(multiple_payload_motion_sim_s=0.,any_payload_motion_sim_s=0.,payload_motion_sim_s={},peak_moving_payloads=0)
    t.relay_motion_metrics=metrics
    def step():
        nonlocal last,positions
        original()
        if t.time-last<.02-1e-8:return
        dt=t.time-last;moving=[]
        result=t.monitor.result().get('payloads',{})
        for ident in positions:
            current=t.data.body(ident).xpos.copy();speed=np.linalg.norm(current-positions[ident])/dt
            state=result.get(ident,{})
            if speed>.005 and state.get('airborne') and len(state.get('support',[]))>=2 and not state.get('completed'):
                moving.append(ident);metrics['payload_motion_sim_s'][ident]=metrics['payload_motion_sim_s'].get(ident,0.)+dt
            positions[ident]=current
        if moving:metrics['any_payload_motion_sim_s']+=dt
        if len(moving)>1:metrics['multiple_payload_motion_sim_s']+=dt
        metrics['peak_moving_payloads']=max(metrics['peak_moving_payloads'],len(moving));last=t.time
    t.step=step


def solve(t,*,flexible=False,terminal_observer=None,handover_observer=None):
    """Advance independent scopes; no whole-team or whole-payload barrier."""
    assemblies=t.task['mechanism']['payload_assemblies'];route=t.task['route']
    frontiers=getattr(t,'relay_frontiers',None)
    if frontiers is None:
        frontiers=[Frontier(item['id'],i,next(j for j,r in enumerate(route) if r['region_id']==item['initial_route_stage_id'])) for i,item in enumerate(assemblies)]
        t.relay_frontiers=frontiers
    install_motion_metrics(t)
    def notify(f,phase):
        if terminal_observer is not None:terminal_observer(phase,f.ident,f.team)
    def ready(f):
        return tuple((other.ident,other.phase,other.milestone,tuple(other.team)) for other in frontiers if other is not f)
    while any(f.phase!='done' for f in frontiers) or not t.monitor.result().get('task_success'):
        t.check_stop();progress=False
        # Downstream terminal work frees scarce hands and goal access first.
        for f in sorted(frontiers,key=lambda item:(0 if item.phase in ('place','release') else 1,-item.index if item.phase=='plan' else 0)):
            if t.monitor.result().get('payloads',{}).get(f.ident,{}).get('completed'):
                if f.phase!='done':f.phase='done';f.milestone+=1;progress=True
                continue
            if f.phase=='done':
                f.phase='recheck';f.recheck_since=t.time;f.blocked='';f.milestone+=1
            if f.phase in ('done','carry'):continue
            if f.blocked and f.blocked_at==ready(f):continue
            prior_actions=len(t.actions)
            terminal_phase=f.phase in ('place','release')
            try:
                if f.phase=='pickup':
                    body=t.data.body(f.ident);point=body.xpos.copy();rotation=body.xmat.reshape(3,3).copy()
                    team=ref.choose_team(t,f.ident,point,rotation,f.offset,search_budget_s=20.)
                    if terminal_observer is not None:terminal_observer('preparation_started',f.ident,team)
                    f.team=team;ref.grasp(t,f.ident,team,f.offset);f.yaw=math.atan2(rotation[1,0],rotation[0,0])
                    for above in (.55,.50,.45,.40,.35):
                        target=point.copy();target[2]=t.task['table_height_m']+above
                        if ref.carry_clear(t,f.ident,f.team,target,f.yaw):break
                    else:raise RuntimeError('No frontier lift candidate '+f.ident)
                    f.target=target;f.phase='carry';f.following='plan'
                elif f.phase=='plan':
                    direct=direct_terminal_plan(t,f,frontiers)
                    if direct is not None:
                        f.target,f.yaw=direct;f.following='place';f.phase='carry'
                        t.event('relay_direct_terminal_prefix',payload=f.ident,holders=f.team,target_m=f.target.tolist())
                    elif f.index>=len(route)-1:
                        final=route[-1];f.target=np.asarray(f.future['landing_point']).copy() if f.future.get('landing_point') is not None else np.asarray(final['center_m'])+rz(final['nominal_yaw_rad'])@np.array([(f.number-(len(assemblies)-1)/2)*.12-.045,0,0]);f.target[2]=t.data.body(f.ident).xpos[2]
                        f.yaw=ref.payload_route_yaw(t,f.ident,final);f.following='place';f.phase='carry'
                    else:
                        planned,f.extra_prefix=plan_transfer(t,f,frontiers,flexible)
                        point,yaw,offset,future=planned
                        if f.extra_prefix:t.event('relay_extra_prefix_selected',payload=f.ident,holding_team=f.team,target_m=point.tolist(),receiver_offset_m=offset)
                        f.target=point;f.yaw=yaw;f.receiver_offset=offset;f.future=future;f.phase='carry';f.following='receive'
                elif f.phase=='receive':
                    body=t.data.body(f.ident);receiver=ref.choose_team(t,f.ident,body.xpos.copy(),body.xmat.reshape(3,3).copy(),f.receiver_offset,exclude=f.team,search_budget_s=40.,**f.future)
                    old=list(f.team);ref.grasp(t,f.ident,receiver,f.receiver_offset);t.wait(.25)
                    ref.release(t,f.ident,[a for a in old if a not in receiver]);f.team=receiver;f.offset=f.receiver_offset
                    if not f.extra_prefix:f.index+=1
                    f.future['previous_team']=old
                    f.target=np.asarray(f.future['future_region']).copy();f.target[2]=f.future['future_height'];f.yaw=f.future['future_yaw'];f.following='transit';f.phase='carry'
                elif f.phase=='transit':
                    f.target=np.asarray(f.future['future_waypoints'][0][0]);f.yaw=f.future['future_waypoints'][0][1];f.following='transit_done';f.phase='carry'
                elif f.phase=='transit_done':
                    if handover_observer is not None:handover_observer(f.ident,f.future['previous_team'],f.team,f.index)
                    f.phase='plan'
                elif f.phase=='place':
                    notify(f,'prepared');notify(f,'placement_started');ref.place(t,f.ident,f.team);notify(f,'table_supported');t.wait(.3);f.phase='release'
                elif f.phase=='release':
                    notify(f,'release_started');ref.release(t,f.ident,f.team,terminal=True);t.wait(1.1)
                    if not t.monitor.result()['payloads'][f.ident]['completed']:raise RuntimeError('Frontier final score not established '+f.ident)
                    notify(f,'terminal_complete');f.phase='done'
                elif f.phase=='recheck':
                    if t.time-f.recheck_since>=2.5:raise RuntimeError('Previously complete payload disturbed; inspect actual goal '+f.ident)
                    t.wait(.25)
                f.blocked='';f.blocked_at=None;f.milestone+=1;progress=True;snapshot(t,frontiers)
                # Finish enabled landing/release before an upstream search
                # for the very hands this scope is about to free.
                if terminal_phase:break
            except ReferenceStopped:raise
            except RuntimeError as error:
                if t.monitor.violations or any(a.job is not None for a in t.arms.values()):raise
                if len(t.actions)!=prior_actions:raise
                f.blocked=str(error);f.blocked_at=ready(f)
                t.event('relay_frontier_blocked',payload=f.ident,phase=f.phase,error=str(error));snapshot(t,frontiers)
        requests=[]
        for f in frontiers:
            if f.phase!='carry':continue
            request=carry_chunk(t,f)
            if request is None:
                f.phase=f.following;f.milestone+=1;progress=True
            else:requests.append(request)
        selected=execute_carry_chunks(t,requests) if requests else []
        for f in frontiers:
            if f.ident in selected:f.chunks+=1;f.blocked='';progress=True
        if selected:snapshot(t,frontiers)
        if not progress:
            snapshot(t,frontiers)
            raise RuntimeError('Relay frontier has no admitted independent prefix: '+str({f.ident:f.blocked or f.phase for f in frontiers if f.phase!='done'}))
    snapshot(t,frontiers)
