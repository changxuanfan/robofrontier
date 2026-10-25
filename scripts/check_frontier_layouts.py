"""Initial geometry/stability/performance checks, never a manipulation PASS."""
import argparse,json,time
from pathlib import Path
import mujoco,numpy as np
from robofrontier.suite.scenes import catalog,case_directory,update_case
from robofrontier.suite.tasks import make_task
from robofrontier.robot_control import PandaArm
from robofrontier.run_setup import write_json

def endpoint_coverage(task,model,data,arms):
    if task['family'] not in ('transport','mosaic'):return None
    rotations=[]
    for axis in (np.array([1.,0,0]),np.array([0,1.,0])):
        for sign in (1,-1):
            y=axis*sign;z=np.array([0,0,-1.]);rotations.append(np.column_stack((np.cross(y,z),y,z)))
    def reachable(robot,p):
        arm=arms[robot]
        for r in rotations:
            q=arm.qref.copy()
            for point in (p+np.array([0,0,.15]),p):
                q,e=arm.ik.solve(point,r,q,250)
                if e>.0018:break
            else:return True
        return False
    sources={o['id']:reachable(o['source'],data.body(o['id']).xpos+np.array([0,0,.006])) for o in task['objects']}
    goals={}
    for g in task['goals']:
        p=np.asarray(g['position_m'])+np.array([0,0,.051 if task['family']=='mosaic' else .006])
        candidates=[g['robot']] if task['family']=='transport' else sorted(arms,key=lambda a:np.linalg.norm(p[:2]-np.asarray(task['arms'][a]['base_position_m'][:2])))
        goals[g['goal_id']]=next((a for a in candidates if reachable(a,p)),None)
    return dict(scope='Robot-only endpoint IK, not full trajectory/contact proof',sources=sources,goals=goals,passed=all(sources.values()) and all(goals.values()))

def check(case,seconds=1.):
    folder=case_directory(case);task=make_task(folder);m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    arms={r:PandaArm(m,d,r) for r in task['arms']};owner=np.full(m.ngeom,-1,int)
    roots={m.body(r+'_link0').id:i for i,r in enumerate(arms)}
    for g in range(m.ngeom):
        b=int(m.geom_bodyid[g])
        while b and b not in roots:b=int(m.body_parentid[b])
        if b in roots:owner[g]=roots[b]
    initial={o['id']:d.body(o['id']).xpos.copy() for o in task['objects']};cross=set();fixture=set();start=time.monotonic();stride=round(.02/m.opt.timestep)
    for tick in range(round(seconds/m.opt.timestep)):
        if tick%stride==0:
            for a in arms.values():a.control(.02)
        mujoco.mj_step(m,d)
        if d.ncon:
            pairs=np.asarray(d.contact.geom);dist=np.asarray(d.contact.dist);who=owner[pairs];bad=(who[:,0]>=0)&(who[:,1]>=0)&(who[:,0]!=who[:,1])&(dist<-.0002)
            for a,b in pairs[bad]:cross.add(tuple(sorted((m.body(m.geom_bodyid[a]).name,m.body(m.geom_bodyid[b]).name))))
    drift=max((float(np.linalg.norm(d.body(n).xpos-p)) for n,p in initial.items()),default=0)
    home=max(float(np.linalg.norm(d.site(r+'_tcp').xpos-s['home_tcp_m'])) for r,s in ((s['id'],s) for s in json.loads((folder/'environment.json').read_text())['stations']))
    coverage=endpoint_coverage(task,m,d,arms)
    report=dict(passed=bool(np.isfinite(d.qpos).all() and not cross and home<.01 and drift<.01 and (coverage is None or coverage['passed'])),scope='Initial geometry, stationary dynamics and endpoint IK only; no full trajectory or manipulation proof.',robot_count=len(arms),inter_robot_contacts=[list(p) for p in cross],max_home_tcp_error_m=home,max_object_drift_m=drift,endpoint_coverage=coverage,sim_duration_s=seconds,wall_time_s=time.monotonic()-start,scene_sha256=task['scene_sha256'])
    write_json(folder/'layout-validation.json',report)
    current=next(c for c in catalog()['cases'] if c['id']==case['id']);status=current['status'] if current['status'].startswith('oracle_') or current['status']=='repeatability_checked' else 'layout_checked' if report['passed'] else 'layout_failed'
    update_case(case['id'],status=status,layout_report=str((folder/'layout-validation.json').relative_to(Path.cwd())))
    print(json.dumps({'id':case['id'],**report}),flush=True);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--seconds',type=float,default=1);a=p.parse_args()
    for c in catalog()['cases']:
        if a.case is None or c['id'] in a.case:check(c,a.seconds)
