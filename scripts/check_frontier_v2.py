"""Engineering initial-state/endpoint checks, never an Oracle task execution."""
import argparse,json,math,time
from pathlib import Path
import mujoco,numpy as np
from robofrontier.suite.v2 import cases,directory,validate_task,ROOT
from robofrontier.robot_control import PandaArm
from robofrontier.run_setup import write_json
from robofrontier.suite.geometry_checks import owner_map,clear_robot_pose,clear_pick_candidate

def collisions(m,d,owners):
    cross=[];obstacles=[]
    for c in d.contact:
        if c.dist>=-.0005:continue
        a,b=map(int,c.geom);x,y=owners[a],owners[b]
        names=[m.geom(a).name or m.body(int(m.geom_bodyid[a])).name,m.geom(b).name or m.body(int(m.geom_bodyid[b])).name]
        row=dict(geoms=names,penetration_mm=round(float(-c.dist*1000),3))
        if x and y and x!=y:cross.append(row)
        elif bool(x)!=bool(y):obstacles.append(row)
    return cross,obstacles

def endpoint_coverage(m,d,task,arms):
    if task['family'] not in ('transport','mosaic'):return None
    screen_geometry=task['family']=='mosaic'
    scratch=mujoco.MjData(m) if screen_geometry else None
    owners=owner_map(m,arms) if screen_geometry else None
    rotations=[]
    for axis in [np.array([1.,0,0]),np.array([0,1.,0])]:
        for sign in [-1,1]:
            y=axis*sign;z=np.array([0.,0,-1.]);rotations.append(np.column_stack((np.cross(y,z),y,z)))
    def reaches(point,candidates,source_id=None):
        for robot in candidates:
            arm=arms[robot]
            for rotation in rotations:
                if screen_geometry:
                    if clear_pick_candidate(m,d.qpos,scratch,robot,arm.ik,arm.qref,point,rotation,owners,source_id):return robot
                    continue
                q=arm.qref.copy()
                targets=[point+[0,0,.12],point+[0,0,.06],point] if screen_geometry else [point+[0,0,.12],point]
                for p in targets:
                    q,error=arm.ik.solve(p,rotation,q,140)
                    if error>.0025:break
                    if screen_geometry and not clear_robot_pose(m,d.qpos,scratch,robot,arm.qids,q,owners,source_id):break
                else:return robot
        return None
    def nearest(p):return sorted(arms,key=lambda r:np.linalg.norm(np.asarray(task['arms'][r]['base_position_m'])[:2]-p[:2]))[:3]
    source={};grasp_height=task['cube_size_m']/2-.018
    for obj in task['objects']:
        p=d.body(obj['id']).xpos.copy()+[0,0,grasp_height];source[obj['id']]=reaches(p,nearest(p),obj['id'])
    goals={}
    for goal in task['goals']:
        p=np.asarray(goal['position_m'])+[0,0,grasp_height];goals[goal['goal_id']]=reaches(p,nearest(p))
    return dict(scope='Downward IK endpoints; mosaic additionally screens two empty-wrist rotation samples and three approach/operation poses against the initial scene. Not continuous-path or dynamic grasp proof.',
                mosaic_open_hand_geometry_screened=screen_geometry,mosaic_candidate_gripper_width_m=.070 if screen_geometry else None,sources=source,goals=goals,
                passed=all(source.values()) and all(goals.values()))

def check(case,seconds=0.,coverage=False):
    folder=directory(case);task=json.loads((folder/'task.json').read_text());validate_task(task)
    m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    arms={r:PandaArm(m,d,r) for r in task['arms']};owners=owner_map(m,arms);cross,obstacles=collisions(m,d,owners)
    groups=m.geom_group.copy();m.geom_group[:]=5
    table_ids=[g for g in range(m.ngeom) if m.geom(g).name.startswith('table_') and (m.geom_contype[g] or m.geom_conaffinity[g]) and 'leg' not in m.geom(g).name]
    m.geom_group[table_ids]=0;mask=np.array([1,0,0,0,0,0],dtype=np.uint8);mount_support={}
    def supported_at(point):
        p=np.array([*point[:2],task['table_height_m']+.20],dtype=float)
        distance=mujoco.mj_ray(m,d,p,np.array([0.,0,-1.]),mask,True,-1,None)
        if distance>=0:return abs(distance-.20)<.001
        # A ray exactly on a shared mesh edge may miss both sectors. A
        # genuine gap must still fail on both sides of this numerical seam.
        near=[mujoco.mj_ray(m,d,p+[0,eps,0],np.array([0.,0,-1.]),mask,True,-1,None) for eps in (-1e-6,1e-6)]
        return all(abs(hit-.20)<.001 for hit in near)
    for robot in arms:
        base=d.body(robot+'_link0');rotation=base.xmat.reshape(3,3)
        supported=True
        for xy in [(0,0),(-.085,-.085),(-.085,.085),(.085,-.085),(.085,.085)]:
            p=base.xpos+rotation@np.array([*xy,0]);supported=supported and supported_at(p)
        mount_support[robot]=bool(supported)
    regions={};payload_support=None
    if task['family']=='relay':
        for region in task['route']:
            yaw=region['nominal_yaw_rad'];c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
            center=np.asarray(region['center_m'][:2]);half=region.get('half_size_m',[.24,.65])
            regions[region['region_id']]=all(supported_at(center+rotation@np.array([x,y])) for x in (-half[0]-.001,0,half[0]+.001) for y in (-half[1]-.001,0,half[1]+.001))
        for assembly in task['mechanism'].get('payload_assemblies',[]):
            yaw=assembly['pickup_yaw_rad'];c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
            center=np.asarray(assembly['pickup_center_m'][:2]);half=assembly['pickup_half_size_m']
            regions[assembly['source_region_id']]=all(supported_at(center+rotation@np.array([x,y])) for x in (-half[0]-.001,0,half[0]+.001) for y in (-half[1]-.001,0,half[1]+.001))
        payload_support={}
        for obj in task['objects']:
            ident=obj['id'];bid=m.body(ident).id;ok=True
            bodies={bid}
            for body in range(bid+1,m.nbody):
                if int(m.body_parentid[body]) in bodies:bodies.add(body)
            assembly=next(a for a in task['mechanism']['payload_assemblies'] if a['id']==ident)
            for name in (assembly.get('weight_body'),assembly.get('tray_body')):
                if name:bodies.add(m.body(name).id)
            for g in range(m.ngeom):
                if int(m.geom_bodyid[g]) not in bodies or not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
                rotation=d.geom_xmat[g].reshape(3,3);center=d.geom_xpos[g];aabb=m.geom_aabb[g]
                ok=ok and all(supported_at(center+rotation@(aabb[:3]+aabb[3:]*np.array([sx,sy,sz]))) for sx in (-1,1) for sy in (-1,1) for sz in (-1,1))
            payload_support[ident]=bool(ok)
    m.geom_group[:]=groups
    initial={obj['id']:d.body(obj['id']).xpos.copy() for obj in task['objects']};coverage_report=endpoint_coverage(m,d,task,arms) if coverage else None
    # Include separate free tray/weight bodies: watching only the beam would
    # miss an unstable suspended assembly before any manipulation starts.
    free_ids=[int(m.jnt_bodyid[j]) for j in range(m.njnt) if int(m.jnt_type[j])==mujoco.mjtJoint.mjJNT_FREE]
    free_initial={m.body(b).name:d.xpos[b].copy() for b in free_ids}
    eq_residual=float(np.max(np.abs(d.efc_pos[:d.ne]))) if d.ne else 0.
    max_cross=list(cross);max_obstacles=list(obstacles)
    for tick in range(round(seconds/m.opt.timestep)):
        if tick%round(.02/m.opt.timestep)==0:
            for arm in arms.values():arm.control(.02)
        mujoco.mj_step(m,d)
        if tick%10==0:
            a,b=collisions(m,d,owners)
            if a:max_cross=a
            if b:max_obstacles=b
    mujoco.mj_forward(m,d)
    drift={name:float(np.linalg.norm(d.body(name).xpos-p)) for name,p in initial.items()}
    free_drift={name:float(np.linalg.norm(d.body(name).xpos-p)) for name,p in free_initial.items()}
    on_table=all(d.body(name).xpos[2]>task['table_height_m']-.005 for name in free_initial)
    tray_check=None;inter_payload=[]
    if task['family']=='relay':
        assemblies=task['mechanism'].get('payload_assemblies',[dict(id='payload',weight_body='weight',tray_body='tray')])
        if any(a.get('tray_body') for a in assemblies):
            tray_check={}
            for assembly in assemblies:
                if not assembly.get('tray_body'):continue
                tray=d.body(assembly['tray_body']);weight=d.body(assembly['weight_body'])
                delta=tray.xmat.reshape(3,3).T@(weight.xpos-tray.xpos)
                tray_check[assembly['id']]=dict(weight_center_tray_m=delta.tolist(),weight_in_tray=bool(np.linalg.norm(delta[:2])<.008 and .024<delta[2]<.030))
        def assembly_of(g):
            body=m.body(int(m.geom_bodyid[g])).name
            return next((a['id'] for a in assemblies if body==a['id'] or body.startswith(a['id']+'_')),None)
        for contact in d.contact:
            if contact.dist>=-.0005:continue
            a,b=map(int,contact.geom);x,y=assembly_of(a),assembly_of(b)
            if x and y and x!=y:inter_payload.append(dict(payloads=[x,y],geoms=[m.geom(a).name,m.geom(b).name],penetration_mm=float(-contact.dist*1000)))
    report=dict(case_id=case['id'],scope='Static geometry, optional passive settle and endpoint IK only; no robot task commands',oracle_verified=False,
        static_robot_contacts=cross,static_robot_apparatus_contacts=obstacles,robot_mount_footprints_on_table=mount_support,
        route_region_footprints_on_table=regions,payload_initial_footprint_on_table=payload_support,inter_payload_penetrations=inter_payload,initial_equality_residual_m=eq_residual,
        settle_duration_sim_s=seconds,settle_robot_contacts=max_cross,settle_robot_apparatus_contacts=max_obstacles,
        max_main_object_drift_m=max(drift.values(),default=0),main_object_drift_m=drift,free_body_drift_m=free_drift,
        initial_table_supported_assembly_remains_above_table=on_table,tray_check=tray_check,endpoint_coverage=coverage_report,
        finite_state=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()))
    report['passed']=bool(report['finite_state'] and all(mount_support.values()) and all(regions.values()) and (payload_support is None or all(payload_support.values())) and not inter_payload and on_table and max(free_drift.values(),default=0)<.015 and
        (tray_check is None or all(v['weight_in_tray'] for v in tray_check.values())) and not cross and not obstacles and not max_cross and not max_obstacles and
        eq_residual<.001 and (not coverage_report or coverage_report['passed']))
    write_json(folder/'engineering-check.json',report)
    validation=json.loads((folder/'validation.json').read_text())
    validation.update(status='engineering_initial_state_checked' if report['passed'] else 'needs_attention',engineering_passed=report['passed'],
        scope=report['scope'],check_duration_sim_s=seconds,endpoint_coverage_checked=coverage_report is not None)
    write_json(folder/'validation.json',validation)
    print(json.dumps(report),flush=True);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--seconds',type=float,default=0);p.add_argument('--coverage',action='store_true');a=p.parse_args()
    reports=[]
    for c in cases():
        if not a.case or c['id'] in a.case:reports.append(check(c,a.seconds,a.coverage))
    out=ROOT/'docs/generated/frontier-v2';out.mkdir(parents=True,exist_ok=True);write_json(out/'engineering-checks.json',reports)
    if not all(r['passed'] for r in reports):raise SystemExit('Some engineering checks need attention; this is not a task pass/fail.')
