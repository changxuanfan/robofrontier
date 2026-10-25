"""Private rolling Relay frontier with jointly screened finite carry chunks.

One Trial remains the sole physics owner. Planning and all FK state changes
are scratch-only. Independent payloads may share an actuator submission only
after simultaneous full-arm/full-payload screening; per-payload milestones
advance independently. This is an engineering Oracle, not a policy tool.
"""
from copy import copy
from dataclasses import dataclass, field
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


def snapshot(t,frontiers):
    write_json(t.directory/'frontier.json',dict(sim_time_s=t.time,wall_elapsed_s=time.monotonic()-t.started,
        scopes=[dict(payload=f.ident,phase=f.phase,holders=f.team,route_index=f.index,
            target_m=None if f.target is None else np.asarray(f.target).tolist(),yaw_rad=f.yaw,following=f.following,
            blocked=f.blocked,milestone=f.milestone,carry_chunks=f.chunks) for f in frontiers],
        payload_motion=getattr(t,'relay_motion_metrics',{})))


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
                return False
            if oa>=0 and ob>=0:
                if oa!=ob and (oa in moving or ob in moving):return False
                if oa==ob and oa in moving and contact.dist<-.001:
                    names=[t.model.body(int(t.model.geom_bodyid[g])).name for g in (a,b)]
                    if not all('finger' in name for name in names):return False
            for geom,owner,other_owner in ((a,oa,ob),(b,ob,oa)):
                if owner in moving and other_owner<0 and not t.model.body(int(t.model.geom_bodyid[geom])).name.endswith('_link0'):return False
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
            if ref.carry_clear(t,frontier.ident,frontier.team,landing,yaw,target_rotation=rotation) or ref.terminal_reposition_plan(t,frontier.ident,frontier.team) is not None:
                return None
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
    while any(f.phase!='done' for f in frontiers):
        t.check_stop();progress=False
        # Downstream terminal work frees scarce hands and goal access first.
        for f in sorted(frontiers,key=lambda item:0 if item.phase in ('place','release') else 1):
            if t.monitor.result().get('payloads',{}).get(f.ident,{}).get('completed'):
                if f.phase!='done':f.phase='done';f.milestone+=1;progress=True
                continue
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
                    if f.index>=len(route)-1:
                        final=route[-1];f.target=np.asarray(f.future['landing_point']).copy() if f.future.get('landing_point') is not None else np.asarray(final['center_m'])+rz(final['nominal_yaw_rad'])@np.array([(f.number-(len(assemblies)-1)/2)*.12-.045,0,0]);f.target[2]=t.data.body(f.ident).xpos[2]
                        f.yaw=ref.payload_route_yaw(t,f.ident,final);f.following='place';f.phase='carry'
                    else:
                        planner=ref.flexible_handover_plan if flexible else ref.handover_plan
                        shift=packing_shift(t,f,frontiers) if f.index+1==len(route)-1 else 0.
                        point,yaw,offset,future=planner(t,f.ident,f.team,f.offset,f.index+1,search_budget_s=40.,landing_shift_m=shift)
                        f.target=point;f.yaw=yaw;f.receiver_offset=offset;f.future=future;f.phase='carry';f.following='receive'
                elif f.phase=='receive':
                    body=t.data.body(f.ident);receiver=ref.choose_team(t,f.ident,body.xpos.copy(),body.xmat.reshape(3,3).copy(),f.receiver_offset,exclude=f.team,search_budget_s=40.,**f.future)
                    old=list(f.team);ref.grasp(t,f.ident,receiver,f.receiver_offset);t.wait(.25)
                    ref.release(t,f.ident,[a for a in old if a not in receiver]);f.team=receiver;f.offset=f.receiver_offset;f.index+=1
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
