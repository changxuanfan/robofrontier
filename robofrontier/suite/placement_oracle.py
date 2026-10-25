"""Private finite-buffer logistics and mosaic reference; no task-policy recipes."""
from collections import deque
import math
import numpy as np
from .scenes import rz

def graph(env):
    edges={};neighbors={s['id']:[] for s in env['stations']}
    for e in env['edges']:
        a,b=e['stations'];neighbors[a].append(b);neighbors[b].append(a);edges[frozenset((a,b))]=np.asarray(e['position_m'])
    return neighbors,edges

def route(neighbors,start,end):
    queue=deque([[start]]);seen={start}
    while queue:
        p=queue.popleft()
        if p[-1]==end:return p
        for n in neighbors[p[-1]]:
            if n not in seen:seen.add(n);queue.append(p+[n])
    raise ValueError('No declared transport route '+start+'/'+end)

def park(t,robot):
    p,r=t.home[robot];p=p.copy();p[2]=t.task['table_height_m']+.68;t.move_free(robot,p,r,'park',target_joints=t.parking_q[robot])

def clear_idle_arms(t):
    t.parking_q={};commands={}
    for robot,arm in t.arms.items():
        p,r=t.home[robot];p=p.copy();p[2]=t.task['table_height_m']+.68;q=t.screen_ik(robot,p,r,seed=t.home_q[robot])
        if q is None:raise RuntimeError('No raised idle posture '+robot)
        t.parking_q[robot]=q;commands[robot]=('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.5))
    t.execute('clear-idle-arms',commands);t.wait(.2)

def pickup_coverage(t):
    """Physical stock has provenance, not an exclusive operating arm.

    This private endpoint map can remove an unnecessary relay when a later
    robot can already reach an item. Real routes and contact are checked at
    execution, so this is not a motion or grasp certificate.
    """
    result={}
    for item in t.env['objects']:
        body=t.data.body(item['id']);p=body.xpos.copy();r=body.xmat.reshape(3,3);up_index=int(np.argmax(abs(r[2])));up=r[:,up_index]*np.sign(r[2,up_index]);point=p+up*(.020 if t.task['family']=='mosaic' else .006);candidates=[]
        for robot in t.arms:
            if np.linalg.norm(p[:2]-np.asarray(t.task['arms'][robot]['base_position_m'][:2]))>.88:continue
            for axis in (i for i in range(3) if i!=up_index):
                y=r[:,axis];z=-up;rotation=np.column_stack((np.cross(y,z),y,z));q=t.screen_ik(robot,point,rotation,seed=t.home_q[robot])
                if q is not None and t.screen_ik(robot,point+np.array([0,0,.15]),rotation,seed=q) is not None:candidates.append(robot);break
        # Source identifies supply provenance, not a proven pickup capability.
        # Preserve an empty list when no robot passed even this endpoint screen.
        result[item['id']]=list(dict.fromkeys(candidates))
    return result

def shortest_pickup_route(t,neighbors,starts,destination,name):
    point=t.data.body(name).xpos[:2]
    return min((route(neighbors,source,destination) for source in starts),key=lambda path:(len(path),np.linalg.norm(point-np.asarray(t.task['arms'][path[0]]['base_position_m'][:2]))))

def deliver(t,name,path,position,rotation=None,provider=None):
    _,edges=graph(t.env)
    for i,robot in enumerate(path):
        final=i+1==len(path)
        stage_position,stage_rotation=(provider() if provider is not None else (position,np.eye(3) if rotation is None else rotation)) if final else (edges[frozenset(path[i:i+2])],np.eye(3))
        hint=(stage_position,stage_rotation)
        t.pick(robot,name,height=.020 if t.task['family']=='mosaic' else .006,force=50.,placement_hint=hint)
        if not final:t.place(robot,stage_position,stage_rotation,precise=False)
        else:t.place(robot,position,rotation,precise=True,pose_provider=provider)
        park(t,robot)

def transport(t):
    clear_idle_arms(t)
    neighbors,_=graph(t.env);pickups=pickup_coverage(t);unused={o['id']:o for o in t.objects} if hasattr(t,'objects') else {o['id']:o for o in t.env['objects']}
    jobs=[]
    for g in t.task['goals']:
        obj=next(o for o in unused.values() if o['identity']==g['requires']['identity']);unused.pop(obj['id']);jobs.append((g,obj,shortest_pickup_route(t,neighbors,pickups[obj['id']],g['robot'],obj['id'])))
    # Independent of slot indexing: longer transports first leave fewer late relays.
    jobs.sort(key=lambda item:-len(item[2]))
    for i,(g,obj,path) in enumerate(jobs):
        def pose(g=g):
            board=t.data.body(g['board']);r=board.xmat.reshape(3,3).copy();return board.xpos+r@np.asarray(g['local_position_m']),r
        p,r=pose();deliver(t,obj['id'],path,p,r,pose);t.wait(.7)
        t.event('goal_attempt',goal=g['goal_id'],object=obj['id'],path=path,progress=t.monitor.result().get('matched_goals',0))
        print(f"{t.task['id']} logistics {i+1}/{len(jobs)}",flush=True)
        if t.monitor.violations:raise RuntimeError('Process violation: '+str(t.monitor.violations[-1]))
        if g['goal_id'] not in t.monitor.result().get('assignments',{}):raise RuntimeError('Released object did not satisfy socket/board goal '+g['goal_id'])
    t.wait(1.2)

def goal_coverage(t):
    result={};base=np.array([t.task['arms'][a]['base_position_m'][:2] for a in t.arms]);robots=list(t.arms)
    rotations=[]
    for axis in (np.array([1.,0,0]),np.array([0.,1,0])):
        for sign in (1,-1):
            y=axis*sign;z=np.array([0,0,-1.]);rotations.append(np.column_stack((np.cross(y,z),y,z)))
    for g in t.task['goals']:
        p=np.asarray(g['position_m'])+np.array([0,0,.020]);distance=np.linalg.norm(base-p[:2],axis=1);valid=[]
        for index in np.argsort(distance):
            robot=robots[index]
            if distance[index]>.88:continue
            for r in rotations:
                q=t.screen_ik(robot,p,r)
                if q is not None and t.screen_ik(robot,p+np.array([0,0,.14]),r,seed=q) is not None:valid.append(robot);break
        result[g['goal_id']]=valid
        if not valid:raise RuntimeError('No candidate goal approach/seat/withdraw endpoints '+g['goal_id'])
    return result

def mosaic(t):
    clear_idle_arms(t)
    neighbors,_=graph(t.env);coverage=goal_coverage(t);pickups=pickup_coverage(t)
    from ..run_setup import write_json
    write_json(t.directory/'coverage.json',dict(scope='Endpoint candidates; full physical execution is the witness, not this map',goals=coverage))
    available={o['id']:o for o in t.env['objects']};goals=sorted(t.task['goals'],key=lambda g:np.linalg.norm(g['position_m'][:2]))
    for i,g in enumerate(goals):
        options=[]
        for dest in coverage[g['goal_id']]:
            local_distance=np.linalg.norm(np.asarray(t.task['arms'][dest]['base_position_m'][:2])-np.asarray(g['position_m'][:2]))
            for o in available.values():
                if o['color']!=g['requires']['color']:continue
                path=shortest_pickup_route(t,neighbors,pickups[o['id']],dest,o['id'])
                # A cell has no permanent owner. Avoid an unnecessary physical
                # relay when the source arm has verified goal approach/seat
                # endpoints; full paths and contacts are still checked later.
                pickup_distance=np.linalg.norm(t.data.body(o['id']).xpos[:2]-np.asarray(t.task['arms'][path[0]]['base_position_m'][:2]))
                score=local_distance+pickup_distance+2.*(len(path)-1)
                options.append((score,o['id'],path))
        if not options:raise RuntimeError('No supply/robot candidate '+g['goal_id'])
        _,name,path=min(options,key=lambda v:v[0]);available.pop(name)
        deliver(t,name,path,np.asarray(g['position_m']),np.eye(3));t.wait(.7)
        t.event('goal_attempt',goal=g['goal_id'],object=name,path=path,progress=t.monitor.result().get('matched_goals',0))
        if i%5==0:print(f"{t.task['id']} mosaic {i+1}/{len(goals)}",flush=True)
        if t.monitor.violations:raise RuntimeError('Process violation: '+str(t.monitor.violations[-1]))
        for retry in range(2):
            if g['goal_id'] in t.monitor.result().get('assignments',{}):break
            robot=path[-1];t.event('reference_placement_repair',goal=g['goal_id'],object=name,retry=retry+1,scope='Physical regrasp and replacement; no state writes or evaluator relaxation')
            t.pick(robot,name,height=.020,force=30.,placement_hint=(np.asarray(g['position_m']),np.eye(3)),aperture=.040)
            t.place(robot,np.asarray(g['position_m']),np.eye(3),precise=True);park(t,robot);t.wait(.7)
            if t.monitor.violations:raise RuntimeError('Process violation during repair')
        if g['goal_id'] not in t.monitor.result().get('assignments',{}):raise RuntimeError('Released cube did not satisfy assembly goal after bounded repair '+g['goal_id'])
    t.wait(1.2)
