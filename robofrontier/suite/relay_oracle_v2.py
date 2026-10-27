"""Finite candidate actuator reference for multiple rods and flexible teams.

Privileged engineering geometry only. A complete physical run is still required
before this can be called a successful reference. No live object-state writes.
"""
import math
import time
from copy import copy
from itertools import product
from collections import OrderedDict
import mujoco
import numpy as np
from .scenes import rz
from .oracle import ReferenceStopped,joint_edge_clear
from .joint_route import connect_route

STAGE_SEARCH_WALL_S=240.


def payload_route_yaw(t,ident,region):
 """A route rectangle is a spatial frame, not the timber's forward axis.

 Retain the manufactured initial heading relative to its initial route anchor.
 This reads immutable initial geometry; it never rewrites scoring regions.
 """
 from ..robot_control import quat_matrix
 offsets=getattr(t,'relay_payload_heading_offsets',None)
 if offsets is None:t.relay_payload_heading_offsets=offsets={}
 if ident not in offsets:
  obj=next((o for o in t.task.get('objects',[]) if o['id']==ident),None)
  item=next((a for a in t.task['mechanism'].get('payload_assemblies',[]) if a['id']==ident),None)
  if obj is None or item is None or 'initial_quaternion_wxyz' not in obj:offsets[ident]=0.
  else:
   initial=quat_matrix(np.asarray(obj['initial_quaternion_wxyz']))
   source=next(r for r in t.task['route'] if r['region_id']==item['initial_route_stage_id'])
   delta=math.atan2(initial[1,0],initial[0,0])-source['nominal_yaw_rad']
   offsets[ident]=math.atan2(math.sin(delta),math.cos(delta))
 heading=region['nominal_yaw_rad']+offsets[ident]
 return math.atan2(math.sin(heading),math.cos(heading))


def handover_yaws(t,ident,previous,following):
 """Try the actual heading first, then short route-relative rotations."""
 rotation=t.data.body(ident).xmat.reshape(3,3)
 actual=math.atan2(rotation[1,0],rotation[0,0])
 first=payload_route_yaw(t,ident,previous);last=payload_route_yaw(t,ident,following)
 delta=math.atan2(math.sin(last-first),math.cos(last-first));values=[]
 for heading in [actual]+[first+delta*f for f in (.5,.25,0.)]:
  heading=math.atan2(math.sin(heading),math.cos(heading))
  if not any(abs(math.atan2(math.sin(heading-old),math.cos(heading-old)))<1e-6 for old in values):values.append(heading)
 return values


def potentially_reachable(t,robot,point,rotation):
 """Conservative shoulder-to-last-joint bound from the actual fixed chain.

 The fixed tool offset is removed using its requested orientation. The sum
 of intervening link lengths is an upper bound, not an empirical reach ball.
 Passing this cheap test does not imply IK or collision feasibility.
 """
 cache=getattr(t,'relay_reach_bounds',None)
 if cache is None:t.relay_reach_bounds=cache={}
 if robot not in cache:
  arm=t.arms[robot];first,last=map(int,(arm.ik.joints[0],arm.ik.joints[-1]))
  body=int(t.model.jnt_bodyid[last]);shoulder_body=int(t.model.jnt_bodyid[first])
  radius=float(np.linalg.norm(t.model.jnt_pos[first])+np.linalg.norm(t.model.jnt_pos[last]))
  while body!=shoulder_body:
   radius+=float(np.linalg.norm(t.model.body_pos[body]));body=int(t.model.body_parentid[body])
  actual=t.data.site_xmat[arm.site].reshape(3,3)
  offset=actual.T@(t.data.site_xpos[arm.site]-t.data.xanchor[last])
  cache[robot]=(t.data.xanchor[first].copy(),radius+.004,offset)
 shoulder,radius,offset=cache[robot]
 return np.linalg.norm(np.asarray(point)-np.asarray(rotation)@offset-shoulder)<=radius


def screened_ik(t,robot,point,rotation,seed=None):
 """Bounded cache of pure fixed-chain IK, never of scene clearance.

 Alternative transport heights repeatedly ask identical contact/endpoint IK
 questions. Physical state is not part of this memo: seed, absolute goal and
 robot identity fully determine this fixed-chain query. Collision and support
 checks always run again against their current world.
 """
 t.check_stop();seed=t.arms[robot].qref if seed is None else seed
 stats=getattr(t,'relay_ik_stats',None)
 if stats is None:t.relay_ik_stats=stats={}
 stats['queries']=stats.get('queries',0)+1
 if not potentially_reachable(t,robot,point,rotation):
  stats['geometric_rejects']=stats.get('geometric_rejects',0)+1;return None
 key=(robot,*(np.asarray(v,dtype=np.float64).tobytes() for v in (point,rotation,seed)))
 cache=getattr(t,'relay_ik_cache',None)
 if cache is None:t.relay_ik_cache=cache=OrderedDict()
 if key in cache:
  stats['cache_hits']=stats.get('cache_hits',0)+1
  value=cache.pop(key);cache[key]=value
 else:
  stats['solver_calls']=stats.get('solver_calls',0)+1
  value=t.screen_ik(robot,point,rotation,seed=seed);cache[key]=None if value is None else value.copy()
  if len(cache)>4096:cache.popitem(last=False)
 return None if value is None else value.copy()


def cross_x(task,y):
 m=task['mechanism']
 if task['level']==1:return 0.
 if task['level']==2:return math.sqrt(max(0,m['radius_m']**2-y*y))-m['radius_m']*math.cos(math.radians(m.get('circular_angle_deg',90.))/2)
 return m['bend_rise_m']*(1-abs(y)/(.5*m['chord_m']))


def grasp_stations(offset):
 """Distances along the negative and positive timber halves, respectively."""
 values=(float(offset),float(offset)) if np.ndim(offset)==0 else tuple(map(float,offset))
 if len(values)!=2 or not all(np.isfinite(value) and value>0 for value in values):raise ValueError('Two positive grasp stations required')
 return values


def grasp_targets(t,ident,center,rotation,offset):
 points=[]
 if max(grasp_stations(offset))>=t.task['mechanism']['chord_m']/2:raise ValueError('Grasp station must lie inside the timber endpoint')
 for sign,station in zip((-1,1),grasp_stations(offset)):
  local=np.array([cross_x(t.task,sign*station),sign*station,.006])
  part_rotation=rotation
  if sign>0 and t.task['mechanism'].get('elastic_hinge'):
   child=t.data.body(ident+'_distal');part_rotation=child.xmat.reshape(3,3).copy()
   pivot=t.model.body_pos[t.model.body(ident+'_distal').id]
  # A dropped assembly can settle inverted. Approach its exposed face rather
  # than sending the nominal top tool beneath the tabletop. This changes only
  # candidate tool poses; contact, custody and real actuator checks still apply.
  surface=1. if part_rotation[2,2]>=0 else -1.
  local[2]*=surface
  p=(child.xpos+part_rotation@(local-pivot) if sign>0 and t.task['mechanism'].get('elastic_hinge')
     else center+rotation@local)
  # Parallel jaws admit antipodal grasps. Mirrored facing teams need opposed
  # wrist frames; a common frame previously drove the far wrist to its limit.
  if t.task['level']==2:
   theta=math.asin(sign*station/t.task['mechanism']['radius_m'])
  elif t.task['level']==3:
   theta=sign*math.atan2(t.task['mechanism']['bend_rise_m'],.5*t.task['mechanism']['chord_m'])
  else:theta=0.
  jaw_frame=part_rotation@rz(theta)
  y=-sign*jaw_frame[:,0];z=-surface*jaw_frame[:,2];r=np.column_stack((np.cross(y,z),y,z))
  points.append((p,r))
 return points


def receiving_anchor(t,region):
 """Reach-oriented sample anchor, not a binding of robots to a region.

 End destination paint extends past the last bases. Using that outboard
 landing centre as a handover centre unnecessarily exhausts sender reach.
 """
 center=np.asarray(region)
 nearby=sorted(t.arms,key=lambda a:np.linalg.norm(np.asarray(t.task['arms'][a]['base_position_m'])[:2]-center[:2]))[:2]
 return np.mean([np.asarray(t.task['arms'][a]['base_position_m']) for a in nearby],axis=0)


def receiving_routes(t,ident,selected):
 """Check arrival as well as grasp and departure, using one scratch world.

 A collision-clear grasp can lie in an IK branch inaccessible from home.
 Both selected arms are opened, then approached in the real execution order.
 Paths are retained so execution does not independently select another branch.
 """
 probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=t.data.qpos
 for item in selected:
  robot=item['robot']
  if item.get('retained'):continue
  for suffix in ('finger_joint1','finger_joint2'):probe.data.joint(robot+'_'+suffix).qpos[0]=(.065-t.arms[robot].grip_min)/2
 mujoco.mj_forward(t.model,probe.data)
 paths={}
 for item in selected:
  robot=item['robot'];arm=t.arms[robot];start=probe.data.qpos[arm.qids].copy();goal=item['seed']
  if item.get('retained'):
   paths[robot]=[start];continue
  def edge(a,b):
   steps=max(2,math.ceil(np.max(abs(b-a))/.025))
   return all(probe.pose_clear(robot,a+(b-a)*f) for f in np.linspace(0,1,steps+1)[1:])
  if not probe.pose_clear(robot,goal):return None
  path=[start,goal] if edge(start,goal) else connect_route(start,goal,arm.ik.limits,edge,np.random.default_rng(9901+list(t.arms).index(robot)),iterations=400)
  if path is None:return None
  # The second receiver must also avoid the first receiver at contact.
  near=goal.copy();contact=item['contact_q'];pre,_=arm.ik.pose(goal);end,_=arm.ik.pose(contact)
  for f in np.linspace(0,1,11)[1:]:
   near=screened_ik(probe,robot,pre*(1-f)+end*f,item['rotation'],seed=near)
   if near is None or not probe.pose_clear(robot,near,(ident,),.001):return None
  probe.data.qpos[arm.qids]=contact;mujoco.mj_forward(t.model,probe.data);paths[robot]=path
 return paths


def grasp_load_moment(t,ident,offset):
 """Gravity moment about the two-grasp line, used only to rank candidates.

 Curved timber grasped near its inner apex may be reachable yet roll out
 immediately after the donor lets go. This estimate uses the actual assembly
 mass/COM, including its independently suspended weight; it is not a grasp
 certificate, and the real support/drop monitor remains decisive.
 """
 body=t.data.body(ident);points=grasp_targets(t,ident,body.xpos,body.xmat.reshape(3,3),offset)
 line=points[1][0]-points[0][0];line/=max(np.linalg.norm(line),1e-9)
 ids={int(t.model.geom_bodyid[g]) for g in t.monitor.relay_v2.states[ident]['all_geoms']}
 moment=np.zeros(3)
 for bid in ids:
  moment+=np.cross(t.data.xipos[bid]-points[0][0],t.model.body_mass[bid]*t.model.opt.gravity)
 return abs(float(np.dot(moment,line)))


def grasp_horizon_score(t,ident,offset,alternatives):
 """Rank load now and at a subsequent, spatially separated handover.

 Ten centimetres is a palm-space ranking proxy, never a collision exemption
 or feasibility gate. Actual four-hand arrivals/openings/exits are screened
 separately. This avoids greedily using the best-balanced line now while
 leaving only an inner, high-moment line for the next team.
 """
 current=grasp_load_moment(t,ident,offset);stations=np.asarray(grasp_stations(offset))
 later=[grasp_load_moment(t,ident,value) for value in alternatives
        if np.all(abs(np.asarray(grasp_stations(value))-stations)>=.10-1e-9)]
 successor=min(later,default=current)
 return tuple(round(value,6) for value in (max(current,successor),current+successor,current))


def preview_released_home(t,data,robots):
 """Match release's 75 mm aperture as well as its eventual home joints."""
 for robot in robots:
  data.qpos[t.arms[robot].qids]=t.home_q[robot]
  for suffix in ('finger_joint1','finger_joint2'):data.joint(robot+'_'+suffix).qpos[0]=(.075-t.arms[robot].grip_min)/2


def received_transport_preview(t,ident,senders,receivers,future):
 """Predict the committed post-handover legs in an independent FK state."""
 probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=t.data.qpos
 probe.arms=dict(t.arms)
 departing=set(senders)-set(receivers)
 probe.held={a:payload for a,payload in getattr(t,'held',{}).items() if a not in departing}
 probe.held.update({a:ident for a in receivers})
 preview_released_home(t,probe.data,set(senders)-set(receivers))
 for robot in receivers:
  probe.data.qpos[t.arms[robot].qids]=t.relay_grasp_plan[robot]['contact_q']
  for suffix in ('finger_joint1','finger_joint2'):probe.data.joint(robot+'_'+suffix).qpos[0]=(t.task['mechanism']['cross_section_m'][0]-t.arms[robot].grip_min)/2
 mujoco.mj_forward(t.model,probe.data)
 lift=np.asarray(future['future_region']).copy();lift[2]=future['future_height']
 legs=[(lift,future['future_yaw'])]+list(future['future_waypoints'])
 for point,yaw in legs:
  if not carry_clear(probe,ident,receivers,np.asarray(point),yaw):return None
  probe.data.qpos[:]=probe.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
 for robot in receivers:probe.arms[robot]=copy(t.arms[robot]);probe.arms[robot].qref=probe.data.qpos[t.arms[robot].qids].copy()
 return probe


def donor_exit_routes(t,ident,selected,donors,opening_width=.075):
 """Screen the complete overlap/open/exit sequence before choosing a grasp.

 Arrival and later transport do not prove that the donor can leave. These
 predicted openings and joint routes exist only in a separate FK data object.
 No hand attachment, collision exemption, or live world state is introduced.
 """
 if not donors:return {}
 from .relay_donor_staging import parked_donor_routes
 parked=parked_donor_routes(t,ident,selected,donors,opening_width)
 if parked is not None:return parked
 probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=t.data.qpos
 probe.held={a:body for a,body in t.held.items() if a not in donors}
 finger_bodies={t.model.body(robot+'_'+s).id for robot in donors for s in ('left_finger','right_finger')}
 timber=set(t.monitor.relay_v2.states[ident]['rod_geoms']);baseline={}
 for contact in t.data.contact:
  a,b=map(int,contact.geom)
  if ((a in timber and int(t.model.geom_bodyid[b]) in finger_bodies)
      or (b in timber and int(t.model.geom_bodyid[a]) in finger_bodies)):
   key=tuple(sorted((a,b)));baseline[key]=min(baseline.get(key,0.),float(contact.dist))
 for item in selected:
  robot=item['robot'];probe.data.qpos[t.arms[robot].qids]=item['contact_q']
  for suffix in ('finger_joint1','finger_joint2'):probe.data.joint(robot+'_'+suffix).qpos[0]=(t.task['mechanism']['cross_section_m'][0]-t.arms[robot].grip_min)/2
 initial={robot:[float(probe.data.joint(robot+'_'+s).qpos[0]) for s in ('finger_joint1','finger_joint2')] for robot in donors}
 for f in np.linspace(0,1,11):
  for robot in donors:
   for i,suffix in enumerate(('finger_joint1','finger_joint2')):probe.data.joint(robot+'_'+suffix).qpos[0]=(1-f)*initial[robot][i]+f*(opening_width-t.arms[robot].grip_min)/2
  mujoco.mj_forward(t.model,probe.data)
  # A loaded grasp already has small compliant contact compression. Retreat
  # must not deepen that measured initial contact; it must not create new
  # penetration, and the open endpoint/exit must be fully collision clear.
  for contact in probe.data.contact:
   limit=min(-.0005,baseline.get(tuple(sorted(map(int,contact.geom))),0.)-.0001)
   if contact.dist<limit and any(int(t.model.geom_bodyid[g]) in finger_bodies for g in contact.geom):return None
 paths={}
 for robot in donors:
  arm=t.arms[robot];start=probe.data.qpos[arm.qids].copy();goal=t.home_q[robot]
  def edge(a,b):
   steps=max(2,math.ceil(np.max(abs(b-a))/.025))
   return all(probe.pose_clear(robot,a+(b-a)*f) for f in np.linspace(0,1,steps+1)[1:])
  if not probe.pose_clear(robot,goal):return None
  path=[start,goal] if edge(start,goal) else connect_route(start,goal,arm.ik.limits,edge,np.random.default_rng(7701+list(t.arms).index(robot)),iterations=400)
  if path is None:
   withdrawal=withdrawal_target(probe,robot,ident)
   q=None if withdrawal is None else screened_ik(probe,robot,*withdrawal,seed=start)
   if q is not None and edge(start,q):
    tail=[q,goal] if edge(q,goal) else connect_route(q,goal,arm.ik.limits,edge,np.random.default_rng(8801+list(t.arms).index(robot)),iterations=400)
    if tail is not None:path=[start]+tail
  if path is None:return None
  paths[robot]=path;probe.data.qpos[arm.qids]=goal;mujoco.mj_forward(t.model,probe.data)
 return paths


def choose_team(t,*args,search_budget_s=None,**kwargs):
 """Bound initial selection and actual-world revalidation, not just previews."""
 budget=STAGE_SEARCH_WALL_S if search_budget_s is None else max(0.,float(search_budget_s))
 started=time.monotonic();original_stop=t.stop_requested
 t.stop_requested=lambda:((original_stop is not None and original_stop()) or time.monotonic()-started>=budget)
 try:return _choose_team(t,*args,**kwargs)
 except ReferenceStopped:
  if time.monotonic()-started>=budget and not(original_stop is not None and original_stop()):
   raise RuntimeError('Receiving candidate search exhausted '+str(budget)+' wall seconds')
  raise
 finally:t.stop_requested=original_stop


def terminal_pair_order(pairs,rotation,weighted):
 """Top access is a class preference, not permission to ignore wrist room."""
 downward=lambda pair:all(abs(np.dot(v['rotation'][:,2],rotation[:,2]))>.8 for v in pair)
 top=[pair for pair in pairs if downward(pair)]
 side=[pair for pair in pairs if not downward(pair)]
 if weighted:
  top.sort(key=lambda pair:-min(v['margin'] for v in pair))
  side.sort(key=lambda pair:-min(v['margin'] for v in pair))
 preferred=top[:48]
 return preferred+side[:64-len(preferred)]


def _choose_team(t,ident,center,rotation,offset,exclude=(),future_region=None,future_yaw=None,future_fraction=.55,future_height=None,future_waypoints=(),landing_point=None,require_topdown=False,retained=None,departing=None):
 # Resolve finite jaw-symmetric IK branches and approach directions before
 # selecting a team. Endpoint reach alone can hide elbow interference.
 points=grasp_targets(t,ident,center,rotation,offset)
 retained={} if retained is None else dict(retained)
 if len(retained)>1 or any(side not in (0,1) or robot not in exclude for side,robot in retained.items()):raise ValueError('Retain at most one existing side holder')
 departing=set(exclude)-set(retained.values()) if departing is None else set(departing)
 if not departing<=set(exclude) or departing&set(retained.values()):raise ValueError('Departure must only contain excluded, non-retained holders')
 robots=[a for a in t.arms if a not in exclude and getattr(t,'held',{}).get(a,ident)==ident];options=[]
 for side,(point,base_r) in enumerate(points):
  if side in retained:
   robot=retained[side];arm=t.arms[robot];q=t.data.qpos[arm.qids].copy()
   r=t.data.site_xmat[arm.site].reshape(3,3).copy()
   options.append([dict(robot=robot,retained=True,rotation=r,point_delta=t.data.site_xpos[arm.site]-point,
       approach=np.zeros(3),seed=q,contact_q=q,
       margin=float(np.min(np.minimum(q-arm.ik.limits[:,0],arm.ik.limits[:,1]-q))))])
   continue
  choices=[]
  candidates=sorted(robots,key=lambda a:np.linalg.norm(np.asarray(t.task['arms'][a]['base_position_m'])[:2]-point[:2]))[:5]
  for a in candidates:
   arm=t.arms[a];seeds=[arm.qref,t.home_q[a]]
   for delta in (-1.5,-1.,-.5,.5,1.,1.5):
    q=t.home_q[a].copy();q[0]+=delta;q[2]-=delta;seeds.append(np.clip(q,*arm.ik.limits.T))
   seen=[]
   side_turn=np.array([[1.,0,0],[0,0,-1.],[0,1.,0]])
   orientations=[(base_r,np.zeros(3)),(base_r@side_turn,base_r[:,2]*.006),(base_r@side_turn.T,base_r[:,2]*.006)]
   for frame,point_delta in orientations:
    contact=point+point_delta
    for flip in (False,True):
     r=frame@np.diag([-1.,-1.,1.]) if flip else frame
     # This fixed-chain pose bound is independent of the IK seed. Reject it
     # once before enumerating eight joint seeds, rather than rebuilding the
     # same unreachable wrist query for every seed.
     t.check_stop()
     if not potentially_reachable(t,a,contact,r):
      stats=getattr(t,'relay_ik_stats',None)
      if stats is None:t.relay_ik_stats=stats={}
      stats['pose_bound_rejections']=stats.get('pose_bound_rejections',0)+1
      continue
     for seed in seeds:
      q=screened_ik(t,a,contact,r,seed=seed)
      if q is None or not t.pose_clear(a,q,(ident,),.001):continue
      if future_region is not None and future_fraction==1.:
       goal=np.asarray(future_region).copy();goal[2]=center[2] if future_height is None else future_height
       later=rotation if future_yaw is None else rz(future_yaw)
       if future_waypoints:goal=np.asarray(future_waypoints[-1][0]);later=rz(future_waypoints[-1][1])
       if landing_point is not None:goal=np.asarray(landing_point)
       future_q=screened_ik(t,a,goal+later@rotation.T@(contact-center),later@rotation.T@r,seed=q)
       if future_q is None:continue
      for length in (.10,.15):
       displacement=-r[:,2]*length;pre=contact+displacement
       pre_q=screened_ik(t,a,pre,r,seed=q)
       if pre_q is None or not t.pose_clear(a,pre_q,(),.001):continue
       near=pre_q;clear=True
       for f in np.linspace(0,1,11)[1:]:
        near=screened_ik(t,a,pre*(1-f)+contact*f,r,seed=near)
        if near is None or not t.pose_clear(a,near,(ident,),.001):clear=False;break
       if clear:
        if not any(np.linalg.norm(q-old)<.15 for old in seen):
         choices.append(dict(robot=a,rotation=r,point_delta=point_delta,approach=displacement,seed=pre_q,contact_q=q,margin=float(np.min(np.minimum(q-arm.ik.limits[:,0],arm.ik.limits[:,1]-q)))));seen.append(q)
        break
  options.append(choices)
 pairs=[p for p in product(*options) if p[0]['robot']!=p[1]['robot']]
 if require_topdown:pairs=[pair for pair in pairs if all(abs(np.dot(v['rotation'][:,2],rotation[:,2]))>.8 for v in pair)]
 if future_region is not None:
  if landing_point is not None or (t.task['level']==1 and t.task['mechanism']['layout']=='straight'):
   # A terminal side grasp can leave the workpiece resting on an open lower
   # finger. Prefer top approaches whose jaws open horizontally; retain side
   # grasps as bounded fallback. Preserve the simple L1 witness order, too.
   pairs=terminal_pair_order(pairs,rotation,t.task['level']>1)
  else:pairs=sorted(pairs,key=lambda p:-min(v['margin'] for v in p))[:64]
 for selected in pairs:
  if selected[0]['robot']!=selected[1]['robot']:
   if future_region is not None:
    # Predict only in a separate data object. Senders are assumed to complete
    # the explicit release/withdraw before this later carry; live contacts are
    # still audited in the actual run. Never move live objects or arm state.
    probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=t.data.qpos
    preview_released_home(t,probe.data,departing)
    team=[v['robot'] for v in selected]
    for v in selected:probe.data.qpos[t.arms[v['robot']].qids]=v['contact_q']
    mujoco.mj_forward(t.model,probe.data)
    bases=np.mean([np.asarray(t.task['arms'][a]['base_position_m']) for a in team],axis=0)
    future=(1-future_fraction)*bases+future_fraction*np.asarray(future_region);future[2]=center[2] if future_height is None else future_height
    clear=carry_clear(probe,ident,team,future,math.atan2(rotation[1,0],rotation[0,0]) if future_yaw is None else future_yaw)
    for later_point,later_yaw in future_waypoints:
     if not clear:break
     probe.data.qpos[:]=probe.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
     clear=carry_clear(probe,ident,team,np.asarray(later_point),later_yaw)
    if clear and landing_point is not None:
     probe.data.qpos[:]=probe.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
     clear=carry_clear(probe,ident,team,np.asarray(landing_point),future_waypoints[-1][1],landing=True)
    if not clear:continue
   paths=receiving_routes(t,ident,selected)
   if paths is None:continue
   exits=donor_exit_routes(t,ident,selected,departing)
   if exits is None:continue
   t.relay_grasp_plan={v['robot']:{**v,'arrival_route':paths[v['robot']]} for v in selected}
   t.relay_release_plan=exits
   return [v['robot'] for v in selected]
 raise RuntimeError('No sampled collision-clear two-arm receiving approach for '+ident)


def grasp(t,ident,team,offset):
 p=t.data.body(ident).xpos.copy();r=t.data.body(ident).xmat.reshape(3,3).copy();targets=grasp_targets(t,ident,p,r,offset)
 # Weighted apparatus needs more torsional holding margin, particularly
 # when a table-compatible grasp lies inward of the best-balanced station.
 # This is an ordinary allowed robot command, not a raised actuator limit.
 force=70. if t.monitor.relay_v2.states[ident]['assembly'].get('weight_body') else 50.
 continuing={a for a in team if t.relay_grasp_plan[a].get('retained')}
 if continuing and not continuing<=t.monitor.bilateral(t.monitor.relay_v2.states[ident]['rod_geoms'],positive_force=True):raise RuntimeError('Retained holder lost actual beam support '+ident)
 t.execute('relay-preshape',{a:('gripper',dict(mode='move',width_m=.065,speed_m_s=.06)) for a in team if a not in continuing})
 for a,(point,rotation) in zip(team,targets):
  if a in continuing:continue
  plan=t.relay_grasp_plan[a];rotation=plan['rotation'];point=point+plan['point_delta']
  t.event('relay_receiving_plan',robot=a,payload=ident,offset_m=offset,arrival_waypoints=len(plan['arrival_route']),approach_joints=plan['seed'].tolist(),tool_z_dot_payload_z=float(np.dot(plan['rotation'][:,2],r[:,2])))
  arm=t.arms[a]
  for q in plan['arrival_route'][1:]:
   start=t.data.qpos[arm.qids].copy();steps=max(2,math.ceil(np.max(abs(q-start))/.025))
   if not all(t.pose_clear(a,start+(q-start)*f) for f in np.linspace(0,1,steps+1)[1:]):raise RuntimeError('Receiving route changed before execution '+a)
   t.execute('receive-selected',{a:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.65))})
  t.move(a,point,rotation,'receive-contact',precise=True)
  t.execute('relay-grasp',{a:('gripper',dict(mode='grasp',width_m=t.task['mechanism']['cross_section_m'][0],force_n=force,speed_m_s=.025,epsilon_inner_m=.006,epsilon_outer_m=.006))})
 t.wait(.30)
 support=t.monitor.bilateral(t.monitor.relay_v2.states[ident]['rod_geoms'],positive_force=True)
 if not set(team)<=support:raise RuntimeError('Physical two-arm support absent '+ident)
 for a in team:t.held[a]=ident


def carry_rotation(yaw,target_rotation=None):
 """A reference may retain a measured load attitude instead of forcing level."""
 if target_rotation is None:return rz(yaw)
 rotation=np.asarray(target_rotation,dtype=float)
 if (rotation.shape!=(3,3) or not np.isfinite(rotation).all()
     or np.linalg.norm(rotation.T@rotation-np.eye(3))>1e-6
     or abs(np.linalg.det(rotation)-1.)>1e-6):
  raise ValueError('Carried target rotation must be a proper finite rotation')
 return rotation.copy()


def idle_posture_candidate(t,ident,team,idle,point,yaw,target_rotation=None):
 """Read-only finite clearance selection; a candidate never moves an arm."""
 arm=t.arms[idle]
 start=t.data.qpos[arm.qids].copy();candidates=[]
 for delta in (.3,-.3):
  q=start.copy();q[4]+=delta;candidates.append(q)
 candidates.append(t.home_q[idle].copy())
 following=getattr(t,'relay_committed_carry',None)
 for goal in candidates:
  if np.any(goal<arm.ik.limits[:,0]) or np.any(goal>arm.ik.limits[:,1]):continue
  if not joint_edge_clear(start,goal,lambda q:t.pose_clear(idle,q),step_rad=.005):continue
  probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
  probe.data.qpos[:]=t.data.qpos;probe.data.qpos[arm.qids]=goal
  probe.arms={robot:copy(value) for robot,value in t.arms.items()}
  mujoco.mj_forward(t.model,probe.data)
  legs=[(point,yaw,target_rotation)]
  if following is not None:legs.append((following[0],following[1],None))
  clear=True
  for target,heading,rotation in legs:
   if not carry_clear(probe,ident,team,np.asarray(target),heading,target_rotation=rotation):clear=False;break
   probe.data.qpos[:]=probe.relay_carry_endpoint['qpos'];mujoco.mj_forward(t.model,probe.data)
   for robot in team:probe.arms[robot].qref=probe.data.qpos[t.arms[robot].qids].copy()
  if clear:return goal
 return None


def clear_free_idle_blocker(t,ident,team,point,yaw,target_rotation=None):
 """One real L1 repair; never move a holding arm or planning scratch world."""
 if t.task.get('level')!=1 or t.data is not getattr(t,'_live_data',None):return False
 reason=getattr(t,'relay_screen_reason',{}) or {}
 if reason.get('kind')!='arm_contact':return False
 owner=lambda name:next((a for a in t.arms if str(name).startswith(a+'_')),None)
 a,b=owner(reason.get('a')),owner(reason.get('b'))
 idle=b if a in team and b not in team else a if b in team and a not in team else None
 if idle is None:return False
 arm=t.arms[idle]
 if arm.job is not None or arm.grasp_force is not None or idle in t.held:return False
 if any(idle in t.monitor.bilateral(state['rod_geoms'],positive_force=True)
        for state in t.monitor.relay_v2.states.values()):return False
 goal=idle_posture_candidate(t,ident,team,idle,point,yaw,target_rotation)
 if goal is None:return False
 t.event('reference_idle_posture_repair',robot=idle,payload=ident,
         original_reason=reason,joint_target=goal.tolist(),
         successor_checked=getattr(t,'relay_committed_carry',None) is not None)
 t.execute('relay-clear-free-idle',{idle:('move',dict(joint_rad=goal.tolist(),joint_speed_rad_s=.3))})
 t.wait(.1)
 return carry_clear(t,ident,team,point,yaw,target_rotation=target_rotation)


def carry(t,ident,team,point,yaw,landing=False,target_rotation=None):
 # The earlier candidate is not the actual post-contact world. In particular
 # released donors have open fingers and may settle into slightly different
 # joint poses. Recheck before issuing this committed segment's commands.
 if not carry_clear(t,ident,team,point,yaw,landing=landing,target_rotation=target_rotation):
  if landing or not clear_free_idle_blocker(t,ident,team,point,yaw,target_rotation):
   raise RuntimeError('Actual carry path no longer clears '+ident+': '+str(t.relay_screen_reason))
 body=t.data.body(ident);p0=body.xpos.copy();r0=body.xmat.reshape(3,3).copy();r1=carry_rotation(yaw,target_rotation)
 from scripts.verify_floating_dial_oracle import axis_rotation
 from ..robot_control import rotation_vector
 delta=rotation_vector(r1,r0);angle=float(np.linalg.norm(delta));axis=delta/angle if angle>1e-9 else np.array([0.,0,1.])
 offsets={a:(r0.T@(t.data.site_xpos[t.arms[a].site]-p0),r0.T@t.data.site_xmat[t.arms[a].site].reshape(3,3)) for a in team}
 steps=max(1,math.ceil(np.linalg.norm(np.asarray(point)-p0)/.025),math.ceil(angle/.06))
 for f in np.linspace(0,1,steps+1)[1:]:
  desired=p0*(1-f)+np.asarray(point)*f;rotation=axis_rotation(axis,angle*f)@r0
  commands={a:t.pose_command(a,desired+rotation@off,rotation@relative,.035,.25) for a,(off,relative) in offsets.items()}
  t.execute('relay-carry',commands)
  if t.monitor.violations:raise RuntimeError('Physical relay violation: '+str(t.monitor.violations[-1]))
  if t.monitor.relay_v2.states[ident]['dropped']:raise RuntimeError('Physical support lost during carry '+ident)


LANDING_STOP_REASON='Final-region support contact; controlled stop before TCP target'


def support_approach_point(t,ident,point,yaw):
 """A free-space approach above the lowest part of this articulated load.

 The final policy then seeks actual support; it does not require a bent root
 to reach the height of a flat beam's centre. This is a forecast, not contact
 truth, and neither live poses nor acceptance are changed.
 """
 root=t.data.body(ident);r0=root.xmat.reshape(3,3);relative=rz(yaw)@r0.T;lowest=float('inf')
 for gid in t.monitor.relay_v2.states[ident]['all_geoms']:
  rotation=t.data.geom_xmat[gid].reshape(3,3);aabb=t.model.geom_aabb[gid]
  center=relative@(t.data.geom_xpos[gid]+rotation@aabb[:3]-root.xpos)
  extent=abs(relative@rotation)@aabb[3:];lowest=min(lowest,float(center[2]-extent[2]))
 goal=np.asarray(point).copy();goal[2]=t.task['table_height_m']-lowest+.01
 return goal


def beam_table_contact(t,ident):
 state=t.monitor.relay_v2.states[ident]
 owned=set(state['rod_geoms'])
 return any(contact.dist<=0 and any(int(a) in owned and (t.model.geom(int(b)).name or '').startswith('table_') for a,b in ((contact.geom1,contact.geom2),(contact.geom2,contact.geom1))) for contact in t.data.contact)


def goal_contact_allowed(t):
 return bool(getattr(t,'task',{}).get('completion',{}).get('allow_goal_payload_contact',False))


def assembly_table_contact(t,ident):
 owned=set(t.monitor.relay_v2.states[ident]['all_geoms'])
 return any(contact.dist<=0 and any(int(a) in owned and (t.model.geom(int(b)).name or '').startswith('table_') for a,b in ((contact.geom1,contact.geom2),(contact.geom2,contact.geom1))) for contact in t.data.contact)


def grounded_timber(t,ident):
 """Wood supported by tabletop or an in-goal wood stack, excluding pendants.

 A hanging weight can stand on its base and transmit compression through the
 two eyes while the timber is still 30cm above the table. That instantaneous
 assembly contact graph is useful benchmark evidence, but is not a sound
 release policy. Keep this manipulation gate separate from task acceptance.
 """
 relay=t.monitor.relay_v2
 eligible={name for name,state in relay.states.items() if relay.inside(state)}
 owner={int(gid):name for name in eligible for gid in relay.states[name]['rod_geoms']}
 edges={name:set() for name in eligible};ground=set()
 for contact in t.data.contact:
  if contact.dist>0:continue
  a,b=int(contact.geom1),int(contact.geom2);left,right=owner.get(a),owner.get(b)
  if left is not None and right is not None and left!=right:
   edges[left].add(right);edges[right].add(left)
  for name,other in ((left,b),(right,a)):
   if name is not None and (t.model.geom(other).name or '').startswith('table_'):ground.add(name)
 pending=list(ground)
 while pending:
  for neighbor in edges[pending.pop()]-ground:ground.add(neighbor);pending.append(neighbor)
 return ident in ground


def final_support(t,ident,team):
 from .relay_contract import is_current
 if is_current(getattr(t,'task',{})):
  from .relay_user_goal_policy import final_support as current_support
  return current_support(t,ident,team)
 state=t.monitor.relay_v2.states[ident];relay=t.monitor.relay_v2
 # A pendant weight touching the table cannot certify that the timber no
 # longer depends on the open lower fingers. The user also accepts a short
 # final fall inside Goal: timber within20mm of the table may release once
 # native assembly Goal support has already been established. This avoids
 # demanding exact timber contact; it never permits the old30cm free fall.
 # This is a manipulation policy gate, not a change to benchmark acceptance.
 stacking=getattr(t,'task',{}).get('completion',{}).get('support_via_goal_payload')
 height=getattr(t,'task',{}).get('table_height_m')
 near_table=stacking and height is not None and -.002<=t.monitor.bottom(state['rod_geoms'])-height<=.020
 supported=(ident in relay.supported_payloads() and (near_table or grounded_timber(t,ident))) if stacking else beam_table_contact(t,ident)
 if getattr(t,'task',{}).get('completion',{}).get('allow_near_goal_release'):
  # User-authorized low release has no pre-existing table-contact condition.
  # The unchanged final predicate still needs actual stable grounded cargo.
  limit=t.task['completion']['max_timber_release_gap_m']
  supported=grounded_timber(t,ident) or (height is not None and -.002<=t.monitor.bottom(state['rod_geoms'])-height<=limit)
 return bool(relay.inside(state) and supported
             and set(team)<=t.monitor.bilateral(state['rod_geoms'],positive_force=True))


def terminal_contact_names(t,ident,data=None):
 """Declared final cargo contact is a task permission, not a physics edit."""
 from .relay_contract import is_current
 if is_current(getattr(t,'task',{})):
  from .relay_user_goal_policy import terminal_contact_names as current_contacts
  return current_contacts(t,ident,data)
 if not goal_contact_allowed(t):return []
 monitor=copy(t.monitor);monitor.d=t.data if data is None else data
 relay=copy(t.monitor.relay_v2);relay.mon=monitor
 if not relay.inside(relay.states[ident]):return []
 return [name for name,state in relay.states.items() if relay.inside(state)]


def allowed_terminal_payload_contact(t,ident,other,data):
 return any(name!=ident and other in t.monitor.relay_v2.states[name]['all_geoms']
            for name in terminal_contact_names(t,ident,data))


def execute_landing_increment(t,ident,team,commands):
 """Observe an actual terminal contact and use the existing bounded stop.

 This is a private privileged reference policy, not an automatic Agent queue.
 The normal actuator jobs/recording remain in use. A deliberately interrupted
 TCP command remains failed/stopped in its original result, never completed.
 """
 original_step=t.step;requested=False
 def observed_step():
  nonlocal requested
  original_step()
  if requested or not final_support(t,ident,team):return
  targets={}
  for robot in team:
   arm=t.arms[robot]
   if arm.job is not None and not arm.stopping:
    arm.request_stop(t.time,LANDING_STOP_REASON,dict(trigger='final_region_table_support',tcp_target_reached=False))
    targets[robot]=commands[robot][1]
  if targets:
   requested=True;t.event('landing_support_stop_requested',payload=ident,trigger='actual_final_region_table_contact_and_bilateral_support',robots=list(targets),original_targets=targets,tcp_target_reached=False)
 t.step=observed_step
 try:results=t.execute('relay-place-contact',commands,allow_failure=True)
 finally:t.step=original_step
 for robot,result in results.items():
  expected=result['status']=='failed' and result.get('reason')==LANDING_STOP_REASON
  if result['status']!='completed' and not expected:raise RuntimeError('Terminal placement action failed '+robot+': '+str(result))
 if any(t.arms[robot].job is not None for robot in team):raise RuntimeError('Terminal placement stop unresolved')
 if requested:t.event('landing_support_stop_settled',payload=ident,results=results,tcp_target_reached=False)
 return requested


def landing_increment_clear(t,ident,team,dz):
 """Whole-arm sweep for a small contact-seeking descent inside the goal.

 Workpiece/table contact is intentional here; robot/table and every unrelated
 collision remain forbidden. Real contact ends the motion with normal brakes.
 No collision masks, contact parameters or evaluation thresholds are changed.
 """
 probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
 # Payload motion is explicitly predicted below; pose_clear must screen the
 # arms without its free-space carried-object/table rejection in this one
 # contact-seeking terminal policy. Actual held/grasp control is unchanged.
 probe.held={}
 probe.data.qpos[:]=t.data.qpos;start={a:t.data.qpos[t.arms[a].qids].copy() for a in team};goals={};commands={}
 state=t.monitor.relay_v2.states[ident];owned=set(state['all_geoms'])
 for robot in team:
  arm=t.arms[robot];point=t.data.site_xpos[arm.site].copy()+np.array([0,0,-dz]);rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()
  q=screened_ik(t,robot,point,rotation,seed=start[robot])
  if q is None:return None
  goals[robot]=q;commands[robot]=t.pose_command(robot,point,rotation,.015,.15)
 for f in np.linspace(0,1,6)[1:]:
  probe.data.qpos[:]=t.data.qpos
  for robot in team:probe.data.qpos[t.arms[robot].qids]=start[robot]+f*(goals[robot]-start[robot])
  for name in (ident,state['assembly'].get('weight_body'),state['assembly'].get('tray_body')):
   if name:probe.data.joint(name+'_free').qpos[2]-=f*dz
  mujoco.mj_forward(t.model,probe.data)
  allowed=terminal_contact_names(t,ident,probe.data) or [ident]
  for robot in team:
   if not probe.pose_clear(robot,probe.data.qpos[t.arms[robot].qids],allowed,.001):return None
  for contact in probe.data.contact:
   if contact.dist>=-.0003:continue
   a,b=map(int,contact.geom)
   if (a in owned)==(b in owned):continue
   other=b if a in owned else a;name=t.model.body(int(t.model.geom_bodyid[other])).name or ''
   if name in {robot+'_'+suffix for robot in team for suffix in ('left_finger','right_finger')}:continue
   if allowed_terminal_payload_contact(t,ident,other,probe.data):continue
   if (t.model.geom(other).name or '').startswith('table_') and contact.dist>=-dz-.001:continue
   return None
 return commands


def terminal_reposition_plan(t,ident,team):
 """Find a small coupled L1 adjustment with a checked descent successor.

 A settled two-hand grasp can enter a local IK boundary despite a successful
 earlier preview. Compare finite actuator-reachable alternatives in scratch
 data, retaining measured load/tool transforms. This never relaxes limits or
 predicts suspended-weight articulation; other mechanisms keep their guard.
 """
 if t.task['level']!=1:return None
 body=t.data.body(ident);point=body.xpos.copy();rotation=body.xmat.reshape(3,3).copy()
 yaw=math.atan2(rotation[1,0],rotation[0,0]);table=t.task['table_height_m']
 gap=t.monitor.bottom(t.monitor.relay_v2.states[ident]['all_geoms'])-table
 if gap<=.01:return None
 for delta in ([0,-.02,0],[0,.02,0],[0,0,.03],[-.02,0,0],[.02,0,0]):
  target=point+delta
  if not carry_clear(t,ident,team,target,yaw,target_rotation=rotation):continue
  endpoint=t.relay_carry_endpoint
  probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model)
  probe.data.qpos[:]=endpoint['qpos'];probe.arms={a:copy(v) for a,v in t.arms.items()}
  mujoco.mj_forward(t.model,probe.data)
  for robot in team:probe.arms[robot].qref=probe.data.qpos[t.arms[robot].qids].copy()
  probe.monitor=copy(t.monitor);probe.monitor.d=probe.data;probe.monitor.w=probe
  probe.monitor.relay_v2=copy(t.monitor.relay_v2);probe.monitor.relay_v2.mon=probe.monitor
  if not probe.monitor.relay_v2.inside(probe.monitor.relay_v2.states[ident]):continue
  landing=target.copy();landing[2]-=gap+float(delta[2])-.01
  if not carry_clear(probe,ident,team,landing,yaw,target_rotation=rotation):continue
  if landing_increment_clear(probe,ident,team,.005) is None:continue
  return dict(target_m=target,rotation=rotation,yaw=yaw,delta_m=list(delta),
              predicted_descent_target_m=landing)
 return None


def place(t,ident,team):
 """Stop at supported placement, rather than pressing a flexed root to z."""
 state=t.monitor.relay_v2.states[ident]
 if not t.monitor.relay_v2.inside(state):raise RuntimeError('Terminal descent requires entire apparatus inside final region')
 limit=math.ceil(max(0.,float(t.data.body(ident).xpos[2])-t.task['table_height_m']+.05)/.005)+1
 repairs=0
 for _ in range(limit):
  t.check_stop()
  if not t.monitor.relay_v2.inside(state):raise RuntimeError('Apparatus left final region during terminal descent '+ident)
  if final_support(t,ident,team):
   t.wait(.15)
   if final_support(t,ident,team):return
  commands=landing_increment_clear(t,ident,team,.005)
  if commands is None:
   live=t.data is getattr(t,'_live_data',None)
   support=t.monitor.bilateral(state['rod_geoms'],positive_force=True)
   known=live and not t.monitor.violations and not state['dropped'] and set(team)<=support and not any(arm.job is not None for arm in t.arms.values())
   plan=terminal_reposition_plan(t,ident,team) if known and repairs<2 else None
   if plan is None:raise RuntimeError('No whole-arm-clear terminal descent '+ident)
   repairs+=1
   t.event('terminal_descent_forward_adjustment',payload=ident,robots=list(team),
           target_m=plan['target_m'].tolist(),delta_m=plan['delta_m'],
           predicted_descent_target_m=plan['predicted_descent_target_m'].tolist(),
           reason='Actual settled local IK or collision guard; checked coupled successor')
   carry(t,ident,team,plan['target_m'],plan['yaw'],target_rotation=plan['rotation'])
   continue
  execute_landing_increment(t,ident,team,commands)
  if t.monitor.violations:raise RuntimeError('Physical terminal placement violation: '+str(t.monitor.violations[-1]))
  if state['dropped']:raise RuntimeError('Physical support lost during terminal placement '+ident)
 raise RuntimeError('No final-region table support within bounded terminal descent '+ident)


def carry_clear(t,ident,team,point,yaw,landing=False,target_rotation=None):
 """Finite whole-payload/whole-arm prediction, never a live attachment.

 Predict the complete path with both supporting arms at each sample. A robot
 can reach a TCP while its shoulder blocks the timber; fingers alone do not
 establish clearance. Independent suspension is rigidly predicted for screening
 only; real swing/support remains subject to the live monitor. Landing checks
 retain robot/world and timber/table checks, while the free suspension's
 tabletop articulation is explicitly deferred to real contact execution.
 """
 from ..robot_control import matrix_quat
 from ..robot_control import rotation_vector
 from scripts.verify_floating_dial_oracle import axis_rotation
 t.relay_carry_endpoint=None;t.relay_screen_reason=None
 body=t.data.body(ident);p0=body.xpos.copy();r0=body.xmat.reshape(3,3).copy();r1=carry_rotation(yaw,target_rotation)
 delta=rotation_vector(r1,r0);angle=float(np.linalg.norm(delta));axis=delta/angle if angle>1e-9 else np.array([0.,0,1.])
 offsets={a:(r0.T@(t.data.site_xpos[t.arms[a].site]-p0),r0.T@t.data.site_xmat[t.arms[a].site].reshape(3,3)) for a in team}
 assembly=t.monitor.relay_v2.states[ident];owned=set(assembly['all_geoms'])
 suspended=set()
 if landing:
  for name in (assembly['assembly'].get('weight_body'),assembly['assembly'].get('tray_body')):
   if name:suspended.update(t.monitor.descendant_geoms(t.model.body(name).id))
 q={a:t.data.qpos[t.arms[a].qids].copy() for a in team};scratch=mujoco.MjData(t.model)
 n=max(1,math.ceil(np.linalg.norm(np.asarray(point)-p0)/.025),math.ceil(angle/.06))
 for f in np.linspace(0,1,n+1)[1:]:
  desired=p0*(1-f)+np.asarray(point)*f;rotation=axis_rotation(axis,angle*f)@r0
  scratch.qpos[:]=t.data.qpos
  for a,(off,relative) in offsets.items():
   q[a]=screened_ik(t,a,desired+rotation@off,rotation@relative,seed=q[a])
   if q[a] is None:
    t.relay_screen_reason=dict(kind='unreachable',robot=a,fraction=float(f),target=desired.tolist());return False
   scratch.qpos[t.arms[a].qids]=q[a]
  root=scratch.joint(ident+'_free').qpos;root[:3]=desired;root[3:]=matrix_quat(rotation)
  for name in (assembly['assembly'].get('weight_body'),assembly['assembly'].get('tray_body')):
   if name:
    old=t.data.body(name);free=scratch.joint(name+'_free').qpos
    free[:3]=desired+rotation@r0.T@(old.xpos-p0);free[3:]=matrix_quat(rotation@r0.T@old.xmat.reshape(3,3))
  mujoco.mj_forward(t.model,scratch)
  moving={list(t.arms).index(robot) for robot in team}
  permitted_names=terminal_contact_names(t,ident,scratch)
  permitted_geoms={gid for name in permitted_names for gid in t.monitor.relay_v2.states[name]['all_geoms']}
  for contact in scratch.contact:
   if contact.dist>=-.0003:continue
   a,b=map(int,contact.geom)
   if a not in owned and b not in owned:
    oa,ob=t.monitor.geom_owner[a],t.monitor.geom_owner[b]
    if oa!=ob and oa>=0 and ob>=0 and (oa in moving or ob in moving):
     t.relay_screen_reason=dict(kind='arm_contact',a=t.model.body(int(t.model.geom_bodyid[a])).name,b=t.model.body(int(t.model.geom_bodyid[b])).name,fraction=float(f));return False
    if oa==ob and oa in moving and contact.dist<-.001:
     names=[t.model.body(int(t.model.geom_bodyid[g])).name for g in (a,b)]
     if not all('finger' in name for name in names):
      t.relay_screen_reason=dict(kind='self_contact',a=names[0],b=names[1],fraction=float(f));return False
    for robot_geom,other,rid,other_owner in ((a,b,oa,ob),(b,a,ob,oa)):
     if rid in moving and other_owner<0:
      name=t.model.body(int(t.model.geom_bodyid[robot_geom])).name
      if name.endswith('_link0'):continue # Fixed bolted mounting contact.
      if other in permitted_geoms and any(name in (robot+'_left_finger',robot+'_right_finger') for robot in team):continue
      t.relay_screen_reason=dict(kind='robot_world_contact',a=name,b=t.model.geom(other).name,fraction=float(f));return False
    continue
   if a in owned and b in owned:continue
   other=b if a in owned else a;name=t.model.body(int(t.model.geom_bodyid[other])).name
   obj=a if a in owned else b
   # During landing, a free suspended weight must articulate against the
   # tabletop; rigidly transforming it with the beam would falsely bury it.
   # This one suspension/table interaction is left to live contact physics.
   # Timber/table and every robot/world contact remain checked throughout.
   if landing and obj in suspended and (t.model.geom(other).name or '').startswith('table_'):continue
   if any(name in (robot+'_left_finger',robot+'_right_finger') for robot in team):continue
   if allowed_terminal_payload_contact(t,ident,other,scratch):continue
   t.relay_screen_reason=dict(kind='payload_contact',a=t.model.body(int(t.model.geom_bodyid[a])).name,b=t.model.body(int(t.model.geom_bodyid[b])).name,fraction=float(f));return False
 t.relay_carry_endpoint=dict(q=q,qpos=scratch.qpos.copy(),position=desired,rotation=rotation)
 return True


def withdrawal_target(t,robot,ident,allowed_objects=()):
 arm=t.arms[robot];point=t.data.site_xpos[arm.site].copy();rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()
 outward=point-t.data.body(ident).xpos;outward[2]=0.;outward/=max(np.linalg.norm(outward),1e-8)
 for delta in (-rotation[:,2]*.10,[0,0,.06],outward*.20,[0,0,.10],outward*.24+np.array([0,0,.04])):
  q=t.data.qpos[arm.qids].copy();clear=True
  for f in np.linspace(0,1,21)[1:]:
   q=screened_ik(t,robot,point+f*np.asarray(delta),rotation,seed=q)
   # Once the gripper is open, a withdrawal may not sweep either finger
   # through the beam. Allowed grasp contacts would conceal a pushing motion.
   if q is None or not t.pose_clear(robot,q,allowed_objects,.001):clear=False;break
  if clear:return point+delta,rotation
 return None


def arm_payload_contact(t,ident,robot):
 owned=set(t.monitor.relay_v2.states[ident]['all_geoms'])
 return any(contact.dist<=.0002 and any(int(a) in owned and t.model.body(int(t.model.geom_bodyid[b])).name.startswith(robot+'_') for a,b in ((contact.geom1,contact.geom2),(contact.geom2,contact.geom1))) for contact in t.data.contact)


def move_empty_robot(t,ident,robot,joint_rad):
 """Finite normal joint motion for an observed separated, open donor.

 A stale held label is removed only after real action feedback and another
 separation check. Planning uses a private empty-hand view, never a live
 attachment/pose edit. A contacting or force-held hand cannot use this path.
 """
 arm=t.arms[robot];width=float(arm.observe()['gripper_width_m']);goal=np.asarray(joint_rad,dtype=float)
 if arm.job is not None or arm.grasp_force is not None:raise ValueError('Empty movement requires resolved force-free execution')
 if not math.isfinite(width) or width<t.task['mechanism']['cross_section_m'][0]+.012:raise ValueError('Empty movement requires measured open jaws')
 if t.held.get(robot) not in (None,ident):raise ValueError('Different cargo label requires its own owner review')
 from .relay_contract import is_current
 if is_current(t.task) and arm_payload_contact(t,ident,robot):
  from .relay_contact_escape import execute_open_contact_withdrawal
  return execute_open_contact_withdrawal(t,ident,robot,joint_rad,goal_names=terminal_contact_names)
 if any(arm_payload_contact(t,name,robot) for name in t.monitor.relay_v2.states):raise ValueError('Robot still contacts cargo; review separation first')
 state=t.monitor.relay_v2.states[ident]
 def supported():
  current=t.monitor.bilateral(state['rod_geoms'],positive_force=True)-{robot}
  return len(current&set(state['custody']))>=2 or t.monitor.relay_v2.environment_contact(state['all_geoms'])
 if not supported():raise ValueError('Established receivers or actual environment support required')
 if goal.shape!=(7,) or not np.isfinite(goal).all() or np.any(goal<arm.ik.limits[:,0]) or np.any(goal>arm.ik.limits[:,1]):raise ValueError('Finite in-limit seven-joint target required')
 probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=t.data.qpos;probe.screen_data=mujoco.MjData(t.screen_model);probe.held={a:p for a,p in t.held.items() if a!=robot}
 mujoco.mj_forward(t.model,probe.data);start=t.data.qpos[arm.qids].copy()
 edge=lambda a,b:joint_edge_clear(a,b,lambda q:probe.pose_clear(robot,q,(),.001),step_rad=.01)
 if not probe.pose_clear(robot,goal):raise RuntimeError('Empty-arm target collides with actual scene')
 path=[start,goal] if edge(start,goal) else connect_route(start,goal,arm.ik.limits,edge,np.random.default_rng(7103+list(t.arms).index(robot)),iterations=500)
 if path is None:raise RuntimeError('No screened empty-arm joint route')
 t.event('relay_empty_joint_route',robot=robot,payload=ident,joint_target=goal.tolist(),waypoints=len(path),scope='Measured open/separated donor; ordinary actuators, unchanged physics and cargo poses')
 for q in path[1:]:
  if not supported():raise RuntimeError('Independent support changed before empty-arm motion')
  if any(arm_payload_contact(t,name,robot) for name in t.monitor.relay_v2.states):raise RuntimeError('Empty-arm separation changed before motion')
  probe.data.qpos[:]=t.data.qpos;mujoco.mj_forward(t.model,probe.data)
  if not edge(t.data.qpos[arm.qids],q):raise RuntimeError('Actual scene changed before empty-arm motion')
  t.execute('recovery-empty-joint-move',{robot:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.3))})
 if any(arm_payload_contact(t,name,robot) for name in t.monitor.relay_v2.states):raise RuntimeError('Empty motion ended with cargo contact; retain original held cache')
 t.held.pop(robot,None)


def terminal_recovery_state(t,ident,team):
 """A known grounded world, not an exception mask or a reset permission."""
 live=getattr(t,'_live_data',None)
 if live is None or getattr(t,'data',None) is not live:return False
 active={a:arm.job for a,arm in t.arms.items() if arm.job is not None}
 if active:
  t.relay_recovery_pending=dict(reason='Execution must be resolved before shared-load recovery',active_jobs=active)
  return False
 state=t.monitor.relay_v2.states[ident]
 ok=bool(not t.monitor.violations and not state['dropped'] and state['landed']
         and set(team)<=state['custody'] and t.monitor.relay_v2.inside(state)
         and beam_table_contact(t,ident))
 if not ok:t.relay_recovery_pending=dict(reason='Actual beam/table/custody state does not admit this release recovery',payload=ident)
 return ok


def sequential_terminal_release(t,ident,team):
 """Known blocked paired opening may progress one actually admissible arm.

 A first release changes the supported object pose. Replan the remaining arm
 from that whole measured world; never replay its old pose or erase failures.
 The helper itself never closes the world or stops unrelated execution.
 """
 if len(team)<2 or not terminal_recovery_state(t,ident,team):return False
 pending=list(team)
 for round_index in range(len(team)+1):
  progress=False
  for robot in list(pending):
   if not terminal_recovery_state(t,ident,team):return False
   if donor_exit_routes(t,ident,[],[robot],opening_width=.08) is None:continue
   t.event('terminal_forward_release_attempt',payload=ident,robot=robot,pending=list(pending),round=round_index)
   try:release(t,ident,[robot],terminal=True)
   except (RuntimeError,ValueError) as error:
    t.event('terminal_forward_release_known_stop',payload=ident,robot=robot,reason=str(error))
    if t.monitor.violations or any(arm.job is not None for arm in t.arms.values()):raise
    # An exact-aperture failure stays failed. Physical separation and a known
    # force-mode exit can still admit an ordinary, freshly screened home move.
    if (str(error).startswith('Relay release aperture not verified')
        and t.arms[robot].grasp_force is None and not arm_payload_contact(t,ident,robot)):
     t.held.pop(robot,None)
     try:t.move_free(robot,*t.home[robot],label='terminal-known-release-home',target_joints=t.home_q[robot])
     except (RuntimeError,ValueError):continue
    else:continue
   if not arm_payload_contact(t,ident,robot):
    t.held.pop(robot,None);pending.remove(robot);progress=True
    t.event('terminal_forward_release_clear',payload=ident,robot=robot,pending=list(pending))
    break
  if not pending:
   for _ in range(60):
    t.wait(.05)
    if t.monitor.result()['payloads'][ident]['completed'] and beam_table_contact(t,ident):
     t.relay_recovery_pending=None;return True
   break
  t.wait(.25)
  if not progress and round_index>=1:break
 t.relay_recovery_pending=dict(reason='Bounded sequential release still pending from actual state',payload=ident,remaining=list(pending))
 return False


def observed_wider_release(t,ident,robot,feedback,width):
 """A failed aperture command can still leave a genuinely empty open hand.

 Preserve the failed motor feedback. Only the measured opening overshoot,
 ended force/job and actual cargo separation authorize ordinary withdrawal;
 an unknown failure or an undershot opening does not establish release.
 """
 actual=feedback.get('actual_width_m')
 return bool(feedback.get('status')=='failed'
     and feedback.get('reason')=='Requested aperture exceeded'
     and feedback.get('direction')=='opening'
     and isinstance(actual,(int,float)) and math.isfinite(actual) and actual>=width
     and t.arms[robot].grasp_force is None and t.arms[robot].job is None
     and not arm_payload_contact(t,ident,robot))


def release(t,ident,team,terminal=False):
 width=.080 if terminal else .075
 permitted=terminal and goal_contact_allowed(t)
 if permitted:width=min(.08,float(t.task['mechanism']['cross_section_m'][0])+.020)
 if permitted and not final_support(t,ident,team):raise RuntimeError('Terminal release requires actual supported in-goal landing '+ident)
 if terminal and not permitted:
  exits=donor_exit_routes(t,ident,[],team,opening_width=width)
  if exits is None:
   if len(team)>1 and sequential_terminal_release(t,ident,team):return
   raise RuntimeError('No safe terminal opening/exit plan '+ident)
  t.relay_release_plan=exits
 result=t.execute('relay-release',{a:('gripper',dict(mode='move',width_m=width,speed_m_s=.06)) for a in team},allow_failure=True)
 for a,r in result.items():
  if r['status']!='completed':
   if not observed_wider_release(t,ident,a,r,width):raise RuntimeError('Relay release aperture not verified '+a)
   t.event('relay_release_observed_wider',robot=a,payload=ident,
           requested_width_m=width,original_feedback=dict(r),
           reason='Opening is wider than requested, force/job ended and actual cargo contact absent; original call remains failed')
  t.held.pop(a,None)
 t.wait(.25)
 if t.monitor.relay_v2.states[ident]['dropped']:raise RuntimeError('Independent receiver support lost at release '+ident)
 for a in team:
  if permitted:
   allowed=terminal_contact_names(t,ident)
   withdrawal=withdrawal_target(t,a,ident,allowed_objects=allowed)
   if withdrawal is None:raise RuntimeError('No bounded terminal withdrawal from actual opened state '+a)
   t.execute('terminal-contact-withdraw',{a:t.pose_command(a,*withdrawal,.025,.25)})
   t.move_free(a,*t.home[a],label='terminal-contact-home',target_joints=t.home_q[a])
   continue
  planned=getattr(t,'relay_release_plan',{}).get(a)
  if planned is not None:
   arm=t.arms[a];valid=True
   for q in planned[1:]:
    start=t.data.qpos[arm.qids].copy();steps=max(2,math.ceil(np.max(abs(q-start))/.025))
    if not all(t.pose_clear(a,start+(q-start)*f,(),.001) for f in np.linspace(0,1,steps+1)[1:]):
     t.event('donor_exit_replanning',robot=a,payload=ident,reason='Predicted exit no longer clears actual settled world')
     valid=False;break
    t.execute('send-clear-selected',{a:('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.65))})
   if valid:continue
  withdrawal=withdrawal_target(t,a,ident)
  # A released arm can have no safe straight TCP withdrawal while a short
  # joint-space route still exists. The existing bounded whole-arm router
  # checks the opened fingers against the payload, without grasp exemptions.
  if withdrawal is not None:
   t.execute('send-withdraw',{a:t.pose_command(a,*withdrawal,.035,.25)})
  t.move_free(a,*t.home[a],label='send-clear',target_joints=t.home_q[a])


def handover_plan(t,ident,team,offset,index,search_budget_s=None,lookahead_depth=2,retained_side=None,landing_shift_m=0.):
 """One bounded search stage, including all nested IK and route checks."""
 started=time.monotonic();original_stop=t.stop_requested;search=copy(t);search.live_preview=False
 if not hasattr(t,'relay_ik_stats'):t.relay_ik_stats={}
 search.relay_ik_stats=t.relay_ik_stats;before=dict(t.relay_ik_stats)
 search.relay_handover_best=None;search.relay_handover_seen=set();search.relay_handover_score=None
 search.relay_handover_depth=max(1,min(2,int(lookahead_depth)));search.relay_handover_started=started
 search.relay_retained_side=retained_side
 search.relay_landing_shift=float(landing_shift_m)
 budget=STAGE_SEARCH_WALL_S if search_budget_s is None else max(0.,float(search_budget_s))
 search.stop_requested=lambda:((original_stop is not None and original_stop()) or time.monotonic()-started>=budget)
 live=t.data is getattr(t,'_live_data',None) and hasattr(t,'trace')
 if live:t.event('handover_search_start',payload=ident,stage=index,budget_wall_s=budget)
 try:return _handover_plan(search,ident,team,offset,index)
 except ReferenceStopped:
  if time.monotonic()-started>=budget and not(original_stop is not None and original_stop()):
   if search.relay_handover_best is not None:return search.relay_handover_best
   raise RuntimeError('Bounded handover search exhausted '+str(budget)+' wall seconds at stage '+str(index)+' '+ident)
  raise
 finally:
  if live:t.event('handover_search_end',payload=ident,stage=index,search_wall_s=time.monotonic()-started,ik_counts={k:v-before.get(k,0) for k,v in t.relay_ik_stats.items()},feasible_grasp_candidates=len(search.relay_handover_seen),selected_load_score=search.relay_handover_score,selected_score_fields=['unverified_successor']+(['terminal_side_approach'] if index==len(t.task['route'])-1 else [])+['peak_gravity_moment_Nm','sum_gravity_moment_Nm','current_gravity_moment_Nm'])


def _handover_plan(t,ident,team,offset,index):
 """Finite task-space candidates, checked before moving the real payload.

 Straight-line midpoints are poor samples on curved installations. Circular
 fits are only candidate generators; FK, full-arm contacts and receiving paths
 still decide feasibility, followed by a real physical run.
 """
 route=t.task['route'];prev,nxt=route[index-1:index+1]
 bases=np.mean([np.asarray(t.task['arms'][a]['base_position_m']) for a in team],axis=0)
 anchor=receiving_anchor(t,nxt['center_m']);height=t.data.body(ident).xpos[2]
 anchors=np.array([receiving_anchor(t,r['center_m'])[:2] for r in route])
 fit,_,rank,_=np.linalg.lstsq(np.c_[anchors*2,np.ones(len(anchors))],(anchors**2).sum(1),rcond=None)
 curved=rank==3;origin=fit[:2]
 heights=list(dict.fromkeys(([height] if not curved else [])+[t.task['table_height_m']+v for v in (.35,.40,.45,.30,.55)]))
 terminal=index==len(route)-1
 if terminal:
  # A tall transport posture can exhaust receiver wrist room at the goal.
  # Try the existing lower-clearance candidates first; retain every height
  # and the unchanged whole-arm/payload screens as fallbacks.
  heights=list(dict.fromkeys([t.task['table_height_m']+v for v in (.35,.40,.45,.30,.55)]+[height]))
 # Nearby but distinct grasp stations may be feasible with opposed/side
 # wrists. Full geometry, rather than an arbitrary large spacing rule,
 # decides whether the four hands fit during overlap.
 candidates=(.54,.56,.58,.44,.48,.40,.36,.38,.50,.20,.24,.28,.32) if curved else (.20,.24,.28,.32,.54,.56,.58,.40,.48,.50,.36,.38,.44)
 previous=np.asarray(grasp_stations(offset))
 if not curved and previous.mean()<=.35:candidates=(.44,.48,.40,.54,.56,.58,.50,.36,.38,.20,.24,.28,.32)
 half=t.task['mechanism']['chord_m']/2
 candidates=tuple(dict.fromkeys(v for v in (*candidates,round(half-.02,6)) if 0<v<half))
 offsets=[v for v in candidates if np.all(abs(np.asarray(grasp_stations(v))-previous)>=.035)]
 if t.task['level']>1 or curved:
  # A curved, end-weighted body need not balance on two symmetric stations.
  # Keep a small finite set; actual COM ranks it, geometry and physics judge it.
  asymmetric=[(.32,.54),(.54,.32),(.36,.54),(.54,.36),(.38,.56),(.56,.38),(.40,.56),(.56,.40),(.32,.50),(.50,.32),(.36,.50),(.50,.36),(.40,.54),(.54,.40)]
  offsets += [v for v in asymmetric if max(v)<half and np.all(abs(np.asarray(v)-previous)>=.035)]
 retained_side=getattr(t,'relay_retained_side',None)
 if retained_side is not None:
  if retained_side not in (0,1):raise ValueError('Unknown retained grasp side')
  changing=1-retained_side
  offsets=[tuple(previous[side] if side==retained_side else value for side in (0,1))
           for value in candidates if abs(value-previous[changing])>=.035]
 weighted=t.task['level']>1;lookahead=weighted or curved
 # Unverified asymmetric future contacts are not evidence of future room.
 # They remain available as current grasps, subject to full geometry checks.
 alternatives=list(candidates)
 def load_score(value):
  if weighted and index<len(route)-1 and t.relay_handover_depth>1:return grasp_horizon_score(t,ident,value,alternatives)
  current=round(grasp_load_moment(t,ident,value),6);return (current,current,current)
 scores={grasp_stations(value):load_score(value) for value in offsets}
 offsets=sorted(offsets,key=lambda value:scores[grasp_stations(value)])
 best_possible=min(scores.values())
 for h in heights:
  for fraction in ((.55,.5,.45,.35,.30,.25,.20) if not curved else (.5,.45,.55,.35,.30,.25,.20)):
   point=(1-fraction)*bases+fraction*anchor
   if curved:
    radius=(np.linalg.norm(bases[:2]-origin)+np.linalg.norm(anchor[:2]-origin))/2
    vector=point[:2]-origin;point[:2]=origin+vector*radius/max(np.linalg.norm(vector),1e-9)
   point[2]=h
   for yaw in handover_yaws(t,ident,prev,nxt):
    if not carry_clear(t,ident,team,point,yaw):continue
    end=t.relay_carry_endpoint;probe=copy(t);probe.live_preview=False;probe.data=mujoco.MjData(t.model);probe.data.qpos[:]=end['qpos'];mujoco.mj_forward(t.model,probe.data)
    for receiving_offset in offsets:
     if lookahead and grasp_stations(receiving_offset) in t.relay_handover_seen:continue
     later_heights=(h,t.task['table_height_m']+.45,t.task['table_height_m']+.55) if terminal else (t.task['table_height_m']+.55,t.task['table_height_m']+.45,h)
     for later_height in dict.fromkeys(later_heights):
      try:
       # Low receiving height and high transport clearance are different
       # constraints. Raise vertically before translating over the bases;
       # a diagonal shortcut can strike the shoulder before gaining height.
       transit=np.asarray(nxt['center_m']).copy();transit[2]=later_height
       if retained_side is not None and index<len(route)-1:
        # Intermediate anchors are search references, not compulsory stops.
        # A retained inner hand can make a shorter useful prefix while the
        # denser outer ring changes hands; a later stage replans the frontier.
        transit[:2]=.65*point[:2]+.35*transit[:2]
       future_kwargs=dict(future_region=point.tolist(),future_yaw=yaw,future_fraction=1.,future_height=later_height,future_waypoints=[(transit.tolist(),payload_route_yaw(t,ident,nxt))])
       if retained_side is not None:future_kwargs['retained']={retained_side:team[retained_side]}
       # Only require the legs actually executed before the next planning
       # boundary. An arbitrary extra reach toward a later station can
       # exclude a valid receiver that hands over earlier from this centre.
       if index==len(route)-1:
        assemblies=t.task['mechanism']['payload_assemblies'];number=next(i for i,item in enumerate(assemblies) if item['id']==ident)
        landing=np.asarray(nxt['center_m'])+rz(nxt['nominal_yaw_rad'])@np.array([(number-(len(assemblies)-1)/2)*.12-.045+getattr(t,'relay_landing_shift',0.),0,0])
        landing=support_approach_point(probe,ident,landing,payload_route_yaw(t,ident,nxt))
        prelanding=landing.copy();prelanding[2]=later_height
        future_kwargs['future_waypoints'].append((prelanding.tolist(),payload_route_yaw(t,ident,nxt)))
        future_kwargs['landing_point']=landing.tolist()
       receivers=choose_team(probe,ident,point,end['rotation'],receiving_offset,exclude=team,**future_kwargs)
       result=(point.copy(),yaw,receiving_offset,future_kwargs)
       # Straight L1 has neither a load tradeoff nor the reproduced curved
       # layout dead-end: preserve its verified lightweight first witness.
       if not lookahead:return result
       key=grasp_stations(receiving_offset);estimate=scores[key]
       terminal=index==len(route)-1
       if terminal:
        top=all(abs(np.dot(probe.relay_grasp_plan[robot]['rotation'][:,2],end['rotation'][:,2]))>.8 for robot in receivers)
        if top:future_kwargs['require_topdown']=True
        # This is an orientation preference, not a release certificate. The
        # real terminal opening/exit still has to pass its separate checks.
        estimate=(0 if top else 1,*estimate)
       score=(1,*estimate)
       # Preserve this fully screened current witness before spending any
       # remaining budget investigating its as-yet unverified successor.
       t.relay_handover_seen.add(key)
       if t.relay_handover_best is None or score<t.relay_handover_score:
        t.relay_handover_best=result;t.relay_handover_score=score
       # One bounded real successor screen, including the next four-hand
       # overlap, open/exit and committed transport legs. This is FK planning
       # only, not a weld or a live state change. A failed finite screen leaves
       # the conservative estimate explicitly unverified.
       if index==len(route)-1 or t.relay_handover_depth==1:
        score=(0,*estimate)
       else:
        following=received_transport_preview(probe,ident,team,receivers,future_kwargs)
        if following is not None:
         try:
          _,_,next_offset,_=handover_plan(following,ident,receivers,receiving_offset,index+1,search_budget_s=30.,lookahead_depth=1)
          current=grasp_load_moment(probe,ident,receiving_offset);later=grasp_load_moment(following,ident,next_offset)
          score=(0,round(max(current,later),6),round(current+later,6),round(current,6))
         except ReferenceStopped:raise
         except RuntimeError:pass
       if t.relay_handover_best is None or score<t.relay_handover_score:
        t.relay_handover_best=result;t.relay_handover_score=score
       if not weighted and score[0]==0 and (not terminal or score[1]==0):return result
       optimal_terminal=terminal and score[1]==0 and score[2]<=best_possible[0]
       if (t.relay_handover_depth==1 and not terminal) or optimal_terminal or len(t.relay_handover_seen)>=4:return t.relay_handover_best
       # One verified witness per grasp line is sufficient for this bounded
       # comparison; do not spend three transport-height variants on it.
       break
      except ReferenceStopped:raise
      except RuntimeError:continue
 if t.relay_handover_best is not None:return t.relay_handover_best
 raise RuntimeError('No bounded whole-arm carry/receiving candidate at stage '+str(index)+' '+ident)


def flexible_handover_plan(t,ident,team,offset,index,search_budget_s=None,landing_shift_m=0.):
 """Try a retained holder within the same finite stage search budget.

 Unequal rings prefer retaining an inner arm when both choices are feasible.
 This is a candidate order, never a fixed team or a clearance exemption.
 """
 budget=STAGE_SEARCH_WALL_S if search_budget_s is None else max(0.,float(search_budget_s))
 started=time.monotonic();mechanism=t.task['mechanism']
 inner=set(mechanism.get('inner_robot_ids',list(t.arms)[mechanism.get('outer_arms',len(t.arms)):]))
 balanced=mechanism.get('layout')=='straight' or mechanism.get('outer_arms')==mechanism.get('inner_arms')
 if balanced:
  try:return handover_plan(t,ident,team,offset,index,search_budget_s=budget*.60,landing_shift_m=landing_shift_m)
  except ReferenceStopped:raise
  except RuntimeError:pass
 order=sorted((0,1),key=lambda side:team[side] not in inner)
 for side in order:
  remaining=budget-(time.monotonic()-started)
  if remaining<=0:break
  try:return handover_plan(t,ident,team,offset,index,search_budget_s=min(remaining,budget*.20),retained_side=side,landing_shift_m=landing_shift_m)
  except ReferenceStopped:raise
  except RuntimeError:pass
 if balanced:raise RuntimeError('Bounded whole/retained receiving search exhausted '+ident)
 return handover_plan(t,ident,team,offset,index,search_budget_s=max(0.,budget-(time.monotonic()-started)),landing_shift_m=landing_shift_m)


def solve(t,payload_ids=None,terminal_observer=None,allow_partial_handover=False,handover_observer=None):
 if t.task.get('suite_version')!=2:raise ValueError('V2 relay reference requires v2 task')
 assemblies=t.task['mechanism']['payload_assemblies'];route=t.task['route']
 selected=None if payload_ids is None else set(payload_ids)
 if selected is not None and (not selected or selected-{item['id'] for item in assemblies}):raise ValueError('Unknown/empty commissioning payload selection')
 for number,item in enumerate(assemblies):
  if selected is not None and item['id'] not in selected:continue
  ident=item['id'];body=t.data.body(ident);rotation=body.xmat.reshape(3,3).copy();center=body.xpos.copy();offset=.44
  if terminal_observer is not None:terminal_observer('preparation_started',ident,[])
  team=choose_team(t,ident,center,rotation,offset);grasp(t,ident,team,offset)
  yaw0=math.atan2(rotation[1,0],rotation[0,0]);height=None
  stage=next(i for i,r in enumerate(route) if r['region_id']==item['initial_route_stage_id'])
  for above in (.55,.50,.45,.40,.35):
   point=center.copy();point[2]=t.task['table_height_m']+above
   if carry_clear(t,ident,team,point,yaw0):height=point[2];break
  if height is None:raise RuntimeError('No whole-arm/beam-clear sampled lift height '+ident)
  carry(t,ident,team,point,yaw0)
  for index in range(stage+1,len(route)):
   nxt=route[index]
   receiver=None;remaining_search=STAGE_SEARCH_WALL_S
   for correction in range(3):
    search_started=time.monotonic()
    # Retain a share of the same total stage budget to validate the actual
    # settled pose after the selected physical carry, including corrections.
    selection_budget=remaining_search*(.75 if t.task['level']>1 or t.task['mechanism']['layout']!='straight' else 1.)
    planner=flexible_handover_plan if allow_partial_handover else handover_plan
    point,yaw,receiver_offset,future_kwargs=planner(t,ident,team,offset,index,search_budget_s=selection_budget)
    remaining_search-=time.monotonic()-search_started
    height=point[2];carry(t,ident,team,point,yaw)
    # Compliance and tracking change a predicted pose slightly. A rejected
    # actual-world screen permits bounded replanning from this same world,
    # never a saved-state reset, unverified grasp, or blind action replay.
    try:
     search_started=time.monotonic()
     receiver=choose_team(t,ident,t.data.body(ident).xpos.copy(),t.data.body(ident).xmat.reshape(3,3).copy(),receiver_offset,exclude=team,search_budget_s=remaining_search,**future_kwargs)
     break
    except ReferenceStopped:raise
    except RuntimeError as error:
     t.event('handover_revalidation_rejected',payload=ident,stage=index,correction=correction,reason=str(error))
    finally:remaining_search-=time.monotonic()-search_started
   if receiver is None:raise RuntimeError('Actual receiving pose remained infeasible after bounded replanning '+ident)
   prior_team=list(team)
   grasp(t,ident,receiver,receiver_offset);t.wait(.25)
   release(t,ident,[robot for robot in team if robot not in receiver]);team=receiver;offset=receiver_offset
   height=future_kwargs['future_height']
   lift=np.asarray(future_kwargs['future_region']).copy();lift[2]=height
   point=np.asarray(future_kwargs['future_waypoints'][0][0]).copy()
   next_yaw=future_kwargs['future_waypoints'][0][1]
   t.relay_committed_carry=(point.copy(),next_yaw)
   try:
    if np.linalg.norm(lift-t.data.body(ident).xpos)>.003:carry(t,ident,team,lift,future_kwargs['future_yaw'])
   finally:t.relay_committed_carry=None
   carry(t,ident,team,point,next_yaw)
   if handover_observer is not None:handover_observer(ident,prior_team,list(team),index)
  final=route[-1];rotation=rz(final['nominal_yaw_rad']);point=np.asarray(final['center_m'])+rotation@np.array([(number-(len(assemblies)-1)/2)*.12-.045,0,0]);point[2]=height
  carry(t,ident,team,point,payload_route_yaw(t,ident,final))
  state=t.monitor.relay_v2.states[ident]
  supported=set(team)<=set(t.monitor.bilateral(state['rod_geoms'],positive_force=True))
  if not t.monitor.relay_v2.inside(state) or not supported or state['dropped']:
   raise RuntimeError('Terminal preparation did not establish supported in-region readiness '+ident)
  if terminal_observer is not None:terminal_observer('prepared',ident,team)
  if terminal_observer is not None:terminal_observer('placement_started',ident,team)
  place(t,ident,team)
  if terminal_observer is not None:terminal_observer('table_supported',ident,team)
  t.wait(.3)
  if terminal_observer is not None:terminal_observer('release_started',ident,team)
  release(t,ident,team,terminal=True);t.wait(1.1)
  if not t.monitor.result().get('payloads',{}).get(ident,{}).get('completed'):raise RuntimeError('Released payload did not satisfy final acceptance '+ident)
  if terminal_observer is not None:terminal_observer('terminal_complete',ident,team)
 if selected is not None:return dict(commissioning_only=True,selected_payloads=sorted(selected),selected_completed=all(t.monitor.result()['payloads'][ident]['completed'] for ident in selected),scope='Selected apparatus commissioning; never a complete family-case result')
