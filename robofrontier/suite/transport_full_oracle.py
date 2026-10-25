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
    supply=defaultdict(list);demand=defaultdict(list);jobs=[]
    for obj in t.env['objects']:supply[obj['identity']].append(obj)
    for goal in t.task['goals']:demand[goal['requires']['identity']].append(goal)
    coverage={g['goal_id']:goal_coverage(t,g) for g in t.task['goals']}
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
                        candidates.append((2.*(len(path)-1)+pickup+.4*placement,path))
                if candidates:
                    score,path=min(candidates,key=lambda c:c[0]);cost[i,j]=score;options[i,j]=path
        rows,cols=linear_sum_assignment(cost)
        for i,j in zip(rows,cols):
            if cost[i,j]>=1e6:raise RuntimeError('No finite pickup/destination route '+objects[i]['id'])
            jobs.append(dict(id=goals[j]['goal_id'],goal=goals[j],object=objects[i]['id'],
                path=options[i,j],cost=float(cost[i,j]),attempts=0))
    # Cheap local work can use capacity while a longer relay is progressing.
    jobs.sort(key=lambda j:(len(j['path']),j['cost']))
    t.event('transport_private_plan',jobs=[dict(id=j['id'],object=j['object'],path=j['path'],cost=j['cost']) for j in jobs],
        scope='Finite geometric candidate graph and identity matching; not an optimality or dynamics certificate')
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


def air_transfer(t,robot,name,receivers,goal):
    centers=handover_centers(t,robot,receivers);last=None
    for tilt,depths in ((0.,(.006,.015,.024)),(-60.,(-.012,.006,.020)),(60.,(-.012,.006,.020))):
        before=len(t.actions)
        try:
            return ref.regrasp_held_plug(t,robot,name,keep_receiver_held=True,
                allowed_receivers=receivers,candidate_centers=centers,receiver_tilt_deg=tilt,receiver_depths=depths,
                future_goal=dict(goal,check_socket_path=t.task['level']>1,release_clearance_m=.015))
        except RuntimeError as error:
            last=error
            if len(t.actions)!=before or not str(error).startswith('No bounded physical air-regrasp candidate'):raise
            t.event('transport_handover_candidate_family_rejected',robot=robot,object=name,tilt_deg=tilt,reason=str(error))
    raise last


def execute_job(t,runtime,job):
    name=job['object'];goal=job['goal'];path=job['path'];robot=path[0]
    obj=next(o for o in t.env['objects'] if o['id']==name)
    def pose():return goal_pose(t,goal)
    p,r=pose();rotation=t.data.body(name).xmat.reshape(3,3).copy()
    height=ref._pickup_height(t.task,rotation,.006,obj['shape'])
    if t.task['level']>1 and len(path)==1:
        from .transport_unit_motion import direct_pick,DirectGraspPlanningError
        try:direct_pick(t,robot,name,(p,r),goal['board'],budget_s=12.,check_seating=True)
        except DirectGraspPlanningError:
            if obj['shape']=='cylinder' and abs(rotation[2,2])<.8:ref._pick_side_cylinder(t,robot,name,height,70.)
            else:t.pick(robot,name,height=height,force=70.,placement_hint=(p,r))
    elif obj['shape']=='cylinder' and abs(rotation[2,2])<.8:
        ref._pick_side_cylinder(t,robot,name,height,70.)
    else:t.pick(robot,name,height=height,force=70.,placement_hint=(p,r))
    for receiver in path[1:]:
        donor=robot;robot=air_transfer(t,robot,name,[receiver],goal)
        runtime.release([donor])
    for attempt in range(2):
        try:
            p,r=pose()
            if t.task['level']>1:place_socket(t,robot,p,r,goal['board'],pose_provider=pose)
            else:t.place(robot,p,r,precise=False,pose_provider=pose)
            break
        except (SocketPlanningError,RuntimeError) as error:
            allowed=(isinstance(error,SocketPlanningError) and error.stage in ('seat_release_descent','socket-descend')) or str(error).startswith('No common reachable transit height')
            if attempt or not allowed or not t.finger_contacts(robot,name) or not payload_clear_of_board(t,name,goal['board']):raise
            # Choose a free nearby receiver rather than waiting on an unrelated
            # job or treating a sent plan as an accepted shared commitment.
            candidates=sorted((s for s in t.arms if s!=robot and s not in runtime.leases),
                key=lambda s:np.linalg.norm(np.asarray(t.stations[s]['base_position_m'])[:2]-p[:2]))
            candidates=[s for s in candidates if np.linalg.norm(np.asarray(t.stations[s]['base_position_m'])[:2]-p[:2])<1.0][:2]
            if not candidates or not runtime.acquire(candidates):raise
            donor=robot;robot=air_transfer(t,robot,name,candidates,goal)
            runtime.release([donor])
    park(t,robot);t.wait(1.2)
    accepted=t.monitor.result().get('assignments',{})
    if accepted.get(goal['goal_id'])!=name and t.task['level']==1:
        ref._repair_small_l1_placement(t,dict(goal,robot=robot),name)
        accepted=t.monitor.result().get('assignments',{})
    if accepted.get(goal['goal_id'])!=name:
        raise RuntimeError('Original goal not accepted after delivery '+goal['goal_id'])
    t.event('transport_goal_completed',goal=goal['goal_id'],object=name,operator=robot,
        progress=t.monitor.result().get('matched_goals',0))
    return dict(goal=goal['goal_id'],object=name,operator=robot)


def solve(t):
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
        while pending or runtime.active:
            t.check_stop()
            for job in list(pending):
                if set(job['path']) & set(runtime.leases):continue
                if runtime.add(job['id'],job['path'],lambda job=job:execute_job(t,runtime,job)):
                    runtime.jobs[job['id']].update(object=job['object'],goal=job['goal'])
                    pending.remove(job)
            if not runtime.active:break
            runtime.cycle()
        result=dict(completed=[i for i,j in runtime.jobs.items() if j['status']=='done'],
            failed=[dict(job=i,error=str(j.get('error'))) for i,j in runtime.jobs.items() if j['status']=='failed'],
            pending=[j['id'] for j in pending],retained_resources=dict(runtime.leases),
            scope='One real world; no score stitching, state rewind or forced staging regions')
        t.event('transport_full_frontier',**result)
        return result
    finally:
        if not runtime.active and not runtime.running:runtime.close()
