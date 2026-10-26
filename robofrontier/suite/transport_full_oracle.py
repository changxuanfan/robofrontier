"""Free-topology transport reference. All geometry is privileged Oracle input.

The private candidate graph and finite routing heuristics are not task rules.
Objects move only through ordinary PandaArm commands in the single live world.
"""
from collections import defaultdict, deque
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

from . import transport_oracle_v2 as ref
from .placement_oracle import clear_idle_arms, park
from .scenes import rz
from .transport_runtime import TransportRuntime, sampled_motion_admission
from .transport_socket_motion import place_socket, SocketPlanningError, payload_clear_of_board


def path_between(neighbors,start,end):
    queue=deque([[start]]);seen={start}
    while queue:
        path=queue.popleft()
        if path[-1]==end:return path
        for other in neighbors[path[-1]]:
            if other not in seen:seen.add(other);queue.append(path+[other])
    return None


def candidate_graph(t):
    robots=list(t.arms);base={r:np.asarray(t.stations[r]['base_position_m'])[:2] for r in robots}
    # Endpoint screens and real handovers decide feasibility later. The finite
    # 1.7 m pair envelope merely prunes obviously distant Panda pairs.
    return {r:sorted((s for s in robots if s!=r and np.linalg.norm(base[s]-base[r])<=1.7),
                     key=lambda s:np.linalg.norm(base[s]-base[r])) for r in robots}


def goal_pose(t,goal):
    board=t.data.body(goal['board']);r=board.xmat.reshape(3,3).copy()
    return board.xpos+r@np.asarray(goal['local_position_m']),r


def goal_coverage(t,goal):
    p,r=goal_pose(t,goal);result=[]
    for robot in sorted(t.arms,key=lambda a:np.linalg.norm(np.asarray(t.stations[a]['base_position_m'])[:2]-p[:2])):
        if np.linalg.norm(np.asarray(t.stations[robot]['base_position_m'])[:2]-p[:2])>.90:continue
        for axis in (0,1):
            y=r[:,axis];z=-r[:,2];tool=np.column_stack((np.cross(y,z),y,z))
            q=t.screen_ik(robot,p+r[:,2]*.02,tool,t.home_q[robot])
            if q is not None and t.screen_ik(robot,p+r[:,2]*.16,tool,q) is not None:
                result.append(robot);break
    # An owner is only a fallback candidate, never a physical feasibility claim.
    if not result:result=[goal['robot']]
    return result


def plan_jobs(t):
    neighbors=candidate_graph(t);pickups=ref._pickup_coverage(t)
    bases={r:np.asarray(t.stations[r]['base_position_m'])[:2] for r in t.arms}
    supply=defaultdict(list);demand=defaultdict(list);jobs=[]
    for obj in t.env['objects']:supply[obj['identity']].append(obj)
    for goal in t.task['goals']:demand[goal['requires']['identity']].append(goal)
    coverage={g['goal_id']:goal_coverage(t,g) for g in t.task['goals']}
    pins_up={o['id']:(t.task['level']>1 and t.data.body(o['id']).xmat.reshape(3,3)[2,2]<-.8) for o in t.env['objects']}
    rolling={o['id']:(t.task['level']>1 and o.get('shape')=='cylinder' and abs(t.data.body(o['id']).xmat.reshape(3,3)[2,2])<.8) for o in t.env['objects']}
    for identity,objects in supply.items():
        goals=demand[identity];cost=np.full((len(objects),len(goals)),1e6);options={}
        if len(objects)!=len(goals):raise ValueError('Transport identity inventory mismatch')
        for i,obj in enumerate(objects):
            for j,goal in enumerate(goals):
                p,_=goal_pose(t,goal);candidates=[]
                for source in pickups[obj['id']]:
                    for dest in coverage[goal['goal_id']]:
                        path=path_between(neighbors,source,dest)
                        if path is None:continue
                        pickup=float(np.linalg.norm(t.data.body(obj['id']).xpos[:2]-np.asarray(t.stations[source]['base_position_m'])[:2]))
                        placement=float(np.linalg.norm(p[:2]-np.asarray(t.stations[dest]['base_position_m'])[:2]))
                        posture=0.
                        if pins_up[obj['id']] and len(path)==1:
                            # A top pickup of pins-up stock may need a real
                            # grip change. Prefer assigning side stock to a
                            # direct job and using existing transfer work for
                            # the hard orientation, rather than extra regrips.
                            posture=2.25 if any(r!=source for r in coverage[goal['goal_id']]) else 4.25
                        route_length=sum(float(np.linalg.norm(bases[a]-bases[b])) for a,b in zip(path,path[1:]))
                        candidates.append((2.*(len(path)-1)+pickup+.4*placement+posture+.3*route_length,path,posture,route_length))
                if candidates:
                    score,path,posture,route_length=min(candidates,key=lambda c:c[0]);cost[i,j]=score;options[i,j]=(path,posture,route_length)
        rows,cols=linear_sum_assignment(cost)
        for i,j in zip(rows,cols):
            if cost[i,j]>=1e6:raise RuntimeError('No finite pickup/destination route '+objects[i]['id'])
            jobs.append(dict(id=goals[j]['goal_id'],goal=goals[j],object=objects[i]['id'],
                path=options[i,j][0],receiver_candidates=coverage[goals[j]['goal_id']],cost=float(cost[i,j]),posture_cost=options[i,j][1],route_length_m=options[i,j][2],rolling_stock=rolling[objects[i]['id']],attempts=0))
    # Long rolling-stock chains must not wait behind all stable local work.
    # This is a risk/critical-path heuristic, not a claimed optimal schedule.
    jobs.sort(key=lambda j:(0 if j['rolling_stock'] else 1,-len(j['path']) if j['rolling_stock'] else len(j['path']),j['cost']))
    t.event('transport_private_plan',jobs=[{k:j[k] for k in ('id','object','path','cost','posture_cost','route_length_m','rolling_stock')} for j in jobs],
        scope='Finite geometry/pose-aware matching, route-length cost and rolling-stock critical-path priority; not an optimality or dynamics certificate')
    return jobs


def handover_centers(t,donor,receivers):
    result={}
    for receiver in receivers:
        points=[]
        for station in (t.stations[donor],t.stations[receiver]):
            yaw=station['yaw_rad'];points.append(np.asarray(station['base_position_m'])+
                [.20*math.cos(yaw),.20*math.sin(yaw),0])
        p=(points[0]+points[1])/2;p[2]=t.task['table_height_m']+.30;result[receiver]=p
    return result


def air_transfer(t,robot,name,receivers,goal,next_receiver=None):
    centers=handover_centers(t,robot,receivers);last=None
    width=float(t.task['cube_size_m']);flat=t.task['level']==1
    # Compare low release first. Higher L1 release is ordinary free fall and
    # still needs the unchanged physical goal acceptance; never apply to pins.
    gaps=(.25*width,.5*width,5*width/6,4*width/3) if flat else (.015,)
    families=((0.,(.006,.015,.024)),(-60.,(-.012,.006,.020)),(60.,(-.012,.006,.020)))
    families+=tuple((angle,tuple(width*f for f in (13/30,1/3,-13/30,-1/3,-.2,.1))) for angle in (-90.,90.,-45.,45.,-75.,75.,-105.,105.,-120.,120.))
    for tilt,depths in families:
        for gap in gaps:
            t.check_stop()
            before=len(t.actions)
            try:
                future=dict(goal,check_socket_path=not flat,release_clearance_m=gap)
                if next_receiver is not None:
                    if len(receivers)!=1:raise ValueError('Intermediate transfer requires one selected receiver')
                    future=dict(next_handover_center_m=handover_centers(t,receivers[0],[next_receiver])[next_receiver].tolist())
                receiver=ref.regrasp_held_plug(t,robot,name,keep_receiver_held=True,
                    allowed_receivers=receivers,candidate_centers=centers,receiver_tilt_deg=tilt,receiver_depths=depths,
                    future_goal=future,planning_budget_s=3.)
                if flat and next_receiver is None:
                    if not hasattr(t,'transport_release_clearance'):t.transport_release_clearance={}
                    t.transport_release_clearance[receiver]=gap
                return receiver
            except RuntimeError as error:
                last=error
                if len(t.actions)!=before or not str(error).startswith('No bounded physical air-regrasp candidate'):raise
                t.event('transport_handover_candidate_family_rejected',robot=robot,object=name,tilt_deg=tilt,release_clearance_m=gap,reason=str(error))
    raise last


def place_flat(t,robot,point,rotation,pose_provider):
    """Preserve the selected receiver grasp's L1 release clearance.

    Trial.place's non-precise final command is 15 mm above its input anchor.
    Shift only that command anchor, never the goal or live object state.
    """
    if t.task['family']!='transport' or t.task['level']!=1:
        raise ValueError('Free-drop release applies only to Transport L1')
    gap=getattr(t,'transport_release_clearance',{}).get(robot,.015)
    def release_anchor():
        p,r=pose_provider()
        return p+r[:,2]*(gap-.015),r
    p,r=release_anchor()
    t.event('transport_flat_release_plan',robot=robot,goal_position_m=np.asarray(point).tolist(),release_clearance_m=gap)
    t.place(robot,p,r,precise=False,pose_provider=release_anchor,motion_precise=True)


def execute_job(t,runtime,job,resume=False):
    name=job['object'];goal=job['goal'];path=job['path'];robot=path[0]
    obj=next(o for o in t.env['objects'] if o['id']==name)
    def pose():return goal_pose(t,goal)
    accepted=t.monitor.result().get('assignments',{})
    if goal['goal_id'] in accepted:
        if t.monitor.released(t.monitor.geoms[name]):
            for owner in list(runtime._current()[1]['robots']):
                if t.held.get(owner)==name or t.arms[owner].grasp_force is not None:
                    result=t.execute('settle-completed-gripper',{owner:('gripper',dict(mode='move',width_m=min(t.arms[owner].grip_max,float(t.task['cube_size_m'])+.012),speed_m_s=.05))},allow_failure=True)[owner]
                    if result['status']=='uncertain':raise RuntimeError('Completed goal has unresolved gripper stop')
                    t.held.pop(owner,None);park(t,owner)
        return dict(goal=goal['goal_id'],object=accepted[goal['goal_id']],already_accepted=True)
    if resume:
        allowed=runtime._current()[1]['robots']
        previous_carriers=dict(t.held)
        for owner,held in list(t.held.items()):
            if owner in allowed and held==name and t.arms[owner].grasp_force is None and not ref._robot_payload_contacts(t,owner,name):
                t.held.pop(owner)
                t.event('transport_measured_carrier_cleared',robot=owner,object=name)
        holders=[r for r in allowed if t.arms[r].grasp_force is not None and t.finger_contacts(r,name)]
        if len(holders)>1:raise RuntimeError('Dual grasp frontier needs an explicit release/withdraw repair')
        if holders:
            robot=holders[0];t.held[robot]=name
            released_donors=[]
            for donor,held in previous_carriers.items():
                if donor not in allowed or donor==robot or held!=name or t.arms[donor].grasp_force is not None:continue
                restore=None
                if ref._robot_payload_contacts(t,donor,name):
                    restore=ref.unload_receiver_for_donor_release(t,donor,robot,name)
                ref.withdraw_released_donor(t,donor,robot,name,restore_force=restore)
                released_donors.append(donor)
            paths=[p for dest in goal_coverage(t,goal) if (p:=path_between({r:[s for s in ns if s in allowed] for r,ns in candidate_graph(t).items() if r in allowed},robot,dest))]
            if not paths:raise RuntimeError('Retained carrier has no finite remaining route')
            path=min(paths,key=len)
            for donor in released_donors:
                if donor not in path:runtime.release([donor])
        else:
            if name in accepted.values():
                candidates=[o for o in t.env['objects'] if o['identity']==obj['identity'] and o['id'] not in accepted.values()]
                if not candidates:raise RuntimeError('No remaining free matching object')
                obj=candidates[0];name=obj['id'];job=dict(job,object=name)
            for owner,held in list(t.held.items()):
                if held==name and t.arms[owner].grasp_force is None and not ref._robot_payload_contacts(t,owner,name):t.held.pop(owner)
            options=[];starts=ref._pickup_coverage(t,[obj])[name]
            for source in starts:
                if source not in allowed:continue
                for dest in goal_coverage(t,goal):
                    route=path_between({r:[s for s in ns if s in allowed] for r,ns in candidate_graph(t).items() if r in allowed},source,dest)
                    if route:options.append(route)
            if not options:raise RuntimeError('No current-state pickup and delivery route')
            path=min(options,key=len);robot=path[0]
        runtime._current()[1]['object']=name
    p,r=pose();rotation=t.data.body(name).xmat.reshape(3,3).copy()
    height=ref._pickup_height(t.task,rotation,.006,obj['shape'])
    if resume and t.held.get(robot)==name and t.finger_contacts(robot,name):
        t.event('transport_resume_retained_load',robot=robot,object=name,goal=goal['goal_id'])
    elif t.task['level']>1 and len(path)==1:
        from .transport_unit_motion import direct_pick,DirectGraspPlanningError
        try:direct_pick(t,robot,name,(p,r),goal['board'],budget_s=12.,check_seating=True)
        except DirectGraspPlanningError:
            if obj['shape']=='cylinder' and abs(rotation[2,2])<.8:ref._pick_side_cylinder(t,robot,name,height,70.)
            else:t.pick(robot,name,height=height,force=70.,placement_hint=(p,r))
    elif obj['shape']=='cylinder' and abs(rotation[2,2])<.8:
        ref._pick_side_cylinder(t,robot,name,height,70.)
    else:t.pick(robot,name,height=height,force=70.,placement_hint=(p,r))
    for index,receiver in enumerate(path[1:],1):
        following=path[index+1] if index+1<len(path) else None
        donor=robot;robot=air_transfer(t,robot,name,[receiver],goal,next_receiver=following)
        runtime.release([donor])
    for attempt in range(2):
        try:
            p,r=pose()
            if t.task['level']>1:place_socket(t,robot,p,r,goal['board'],pose_provider=pose)
            else:place_flat(t,robot,p,r,pose)
            break
        except (SocketPlanningError,RuntimeError) as error:
            allowed=(isinstance(error,SocketPlanningError) and error.stage in ('seat_release_descent','socket-descend')) or str(error).startswith('No common reachable transit height')
            if attempt or not allowed or not t.finger_contacts(robot,name) or not payload_clear_of_board(t,name,goal['board']):raise
            # Choose a free nearby receiver rather than waiting on an unrelated
            # job or treating a sent plan as an accepted shared commitment.
            ident=runtime._current()[0]
            candidates=sorted((s for s in t.arms if s!=robot and (s not in runtime.leases or runtime.leases[s]==ident)),
                key=lambda s:np.linalg.norm(np.asarray(t.stations[s]['base_position_m'])[:2]-p[:2]))
            candidates=[s for s in candidates if np.linalg.norm(np.asarray(t.stations[s]['base_position_m'])[:2]-p[:2])<1.0][:2]
            if not candidates or not runtime.acquire(candidates):raise
            donor=robot;robot=air_transfer(t,robot,name,candidates,goal)
            runtime.release([donor])
    park(t,robot);t.wait(1.2)
    accepted=t.monitor.result().get('assignments',{})
    if goal['goal_id'] not in accepted and t.task['level']==1:
        ref._repair_small_l1_placement(t,dict(goal,robot=robot),name)
        accepted=t.monitor.result().get('assignments',{})
    if goal['goal_id'] not in accepted:
        raise RuntimeError('Original goal not accepted after delivery '+goal['goal_id'])
    t.event('transport_goal_completed',goal=goal['goal_id'],object=name,operator=robot,
        progress=t.monitor.result().get('matched_goals',0))
    return dict(goal=goal['goal_id'],object=name,operator=robot)


def pickup_resources(t,runtime,job):
    """Keep a useful successor receiver available for unqualified socket grips.

    A screened direct grip needs only its own arm. Otherwise reserve a receiver
    that can reach the final board before filling the source gripper. This is a
    bounded Oracle allocation heuristic, not a task constraint or fixed arm cap.
    """
    robots=set(job['path'])
    if robots & set(runtime.leases):return None
    if t.task['level']==1 or len(robots)>1:return robots
    source=job['path'][0]
    monitor=getattr(t,'monitor',None)
    accepted=monitor.result().get('assignments',{}) if monitor is not None else {}
    # A failed readiness probe should not restart after every unrelated arm
    # action. Resource ownership or accepted placement changes justify review;
    # the actual pickup/insertion is still screened again when admitted.
    revision=(tuple(sorted(runtime.leases.items())),tuple(sorted(accepted.items())))
    if job.get('grasp_check_revision')!=revision:
        from .transport_unit_motion import direct_grasp_candidate
        plan,stats=direct_grasp_candidate(t,source,job['object'],goal_pose(t,job['goal']),job['goal']['board'],budget_s=3.,check_seating=True)
        job['grasp_check_revision']=revision;job['direct_grip_screened']=plan is not None
        t.event('transport_pickup_successor_capacity',job=job['id'],direct_candidate=plan is not None,diagnostics=stats,
                scope='Finite scratch candidate; actual pickup, insertion and concurrency still checked')
    if job['direct_grip_screened']:return robots
    receivers=job.get('receiver_candidates')
    if receivers is None:receivers=goal_coverage(t,job['goal'])
    for receiver in receivers:
        if receiver!=source and receiver not in runtime.leases and runtime.settled(receiver):
            return robots|{receiver}
    return None


def solve(t,on_blocked=None):
    if t.task['family']!='transport':raise ValueError('Transport-only reference')
    clear_idle_arms(t);jobs=plan_jobs(t)
    runtime=TransportRuntime(t,admission=sampled_motion_admission)
    previous_execute=t.execute
    objects={o['id']:o for o in t.env['objects']}
    def execute(label,commands,allow_failure=False):
        adjusted=ref.cylinder_commands(commands,t.held,objects)
        for r,(op,p) in adjusted.items():
            if op=='move' and (r in t.held or t.arms[r].grasp_force is not None):
                for key,limit in (('joint_speed_rad_s',.18),('speed_m_s',.025),('angular_speed_rad_s',.25)):
                    if key in p:p[key]=min(p[key],limit)
        return previous_execute(label,adjusted,allow_failure=allow_failure)
    t.execute=execute
    # Board resolution belongs to the currently running bounded job, rather
    # than mutating a global wrapper each time another arm takes a turn.
    if t.task['level']>1:
        from .transport_socket_motion import guard_contact_moves
        guard_contact_moves(t,lambda _robot:runtime._current()[1]['goal']['board'])
    pending=list(jobs)
    try:
        while True:
            t.check_stop()
            if on_blocked is not None and hasattr(on_blocked,'poll'):on_blocked.poll(t,runtime,jobs,pending)
            for job in list(pending):
                resources=pickup_resources(t,runtime,job)
                if resources is None:continue
                if runtime.add(job['id'],resources,lambda job=job:execute_job(t,runtime,job)):
                    runtime.jobs[job['id']].update(object=job['object'],goal=job['goal'])
                    pending.remove(job)
            if not runtime.active:
                unfinished=[g for g in t.task['goals'] if g['goal_id'] not in t.monitor.result().get('assignments',{})]
                if (unfinished or runtime.leases) and on_blocked is not None and on_blocked(t,runtime,jobs,pending):continue
                break
            runtime.cycle()
        accepted=t.monitor.result().get('assignments',{})
        history=[dict(job=i,goal=j.get('goal',{}).get('goal_id'),error=str(j.get('error')),status=j['status']) for i,j in runtime.jobs.items() if j['status'] in ('failed','prepared')]
        result=dict(completed=list(accepted),
            failed=[row for row in history if row['goal'] not in accepted],failures_history=history,
            pending=[j['id'] for j in pending],retained_resources=dict(runtime.leases),
            scope='One real world; no score stitching, state rewind or forced staging regions')
        t.event('transport_full_frontier',**result)
        return result
    finally:
        if not runtime.active and not runtime.running:runtime.close()
