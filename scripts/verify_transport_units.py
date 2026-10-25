"""Bounded real-actuator Transport R13 minimum units, never a full task run."""
import argparse
from contextlib import nullcontext
import json
import os
from pathlib import Path
import signal
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def specifications():
    result=[]
    for level in (1,2,3):
        for n,robot,slot in [(4,'B',1),(8,'A',1),(12,'F',2)]:
            indices=(1,) if level==1 or n!=4 else (1,2,3)
            for item in indices:
                result.append(dict(id=f'a{n:02}-l{level}-cube-{item}',case=f'transport-a{n:02d}-d{level}',
                    jobs=[dict(robot=robot,object=f'{robot}_item{item}',goal=f'{robot}_slot{slot}')]))
        for n,source in [(8,'G'),(12,'H')]:
            for item in ((1,) if level==1 else (1,2,3)):
                result.append(dict(id=f'a{n:02}-l{level}-cylinder-{item}',case=f'transport-a{n:02d}-d{level}',
                    jobs=[dict(robot='H',object=f'{source}_item{item}',goal='H_slot3')]))
    # Original live objects are placed by actuators. No fixture teleportation.
    # Some local colors differ from the order: separate mechanical seating
    # from original identity acceptance rather than modifying the evaluator.
    for n,robot in [(4,'B'),(12,'H')]:
        for last in (2,3):
            jobs=([dict(robot='B',object='B_item1',receiver='C',goal='B_slot1'),
                   dict(robot='D',object='D_item1',receiver='B',goal=f'B_slot{last}')] if n==4 else
                  [dict(robot='H',object='H_item1',receiver='I',goal='H_slot1'),
                   dict(robot='H',object=f'H_item{last}',goal=f'H_slot{last}')])
            if n==12 and last==2:jobs[-1]['grasp_policy']='side'
            result.append(dict(id=f'a{n:02}-l3-occupied-{last}',case=f'transport-a{n:02d}-d3',
                mechanical_only=n==12,jobs=jobs))
    for level in (1,2,3):
        for shape,source,donor,receiver,slot in [('cube','E','F','E',2),('cylinder','G','G','H',3)]:
            result.append(dict(id=f'air-l{level}-{shape}',case=f'transport-a08-d{level}',
                jobs=[dict(robot=donor,object=f'{source}_item1',receiver=receiver,goal=f'{receiver}_slot{slot}')]))
    for n,source,donor,receiver,slot,shape in [(4,'A','A','D',1,'cube'),(12,'F','F','E',2,'cube'),(12,'H','I','H',3,'cylinder')]:
        result.append(dict(id=f'air-a{n:02}-l1-{shape}',case=f'transport-a{n:02d}-d1',
            jobs=[dict(robot=donor,object=f'{source}_item1',receiver=receiver,goal=f'{receiver}_slot{slot}')]))
    result += [dict(id='air-a04-l2-cube',case='transport-a04-d2',jobs=[dict(robot='B',object='B_item1',receiver='C',goal='B_slot1')]),
               dict(id='air-a12-l2-cylinder',case='transport-a12-d2',jobs=[dict(robot='H',object='H_item1',receiver='G',goal='H_slot3')])]
    result.append(dict(id='parallel-a12-l2-cube-side',case='transport-a12-d2',jobs=[dict(robot='F',object='F_item2',goal='F_slot2')]))
    return result


def seat_geometry(t,name,goal):
    obj=t.data.body(name);board=t.data.body(goal['board']);br=board.xmat.reshape(3,3)
    p=br.T@(obj.xpos-board.xpos);rr=br.T@obj.xmat.reshape(3,3);local=np.asarray(goal['local_position_m'])
    out=dict(released=t.monitor.released(t.monitor.geoms[name]),height_error_m=float(abs(p[2]-local[2])),
        upright=bool(rr[2,2]>=np.cos(np.deg2rad(5))),xy_error_m=float(np.linalg.norm(p[:2]-local[:2])))
    if t.task['level']>1:
        opening=np.asarray(t.task['mechanism']['slot_opening_m'])/2;depth=t.task['mechanism']['slot_depth_m'];spacing=t.task['mechanism']['blade_spacing_m'];valid=True
        for i in (0,1):
            g=t.model.geom(name+f'_blade{i}').id;c=br.T@(t.data.geom_xpos[g]-board.xpos)
            extent=abs(br.T@t.data.geom_xmat[g].reshape(3,3))@t.model.geom_size[g]
            nearest=min([np.array([-spacing/2,local[1]]),np.array([spacing/2,local[1]])],key=lambda q:np.linalg.norm(c[:2]-q))
            valid&=bool(np.all(abs(c[:2]-nearest)+extent[:2]<=opening+.0003) and c[2]+extent[2]<=.0005 and c[2]-extent[2]>=-depth-.0003)
        out['both_blades_inside']=valid
    else:out['both_blades_inside']=True
    out['passed']=bool(out['released'] and out['height_error_m']<=.0015 and out['upright'] and out['both_blades_inside'] and (t.task['level']>1 or out['xy_error_m']<=.005))
    return out


def run(spec,output,harness,wall,grasp_policy,receiver_tilt,pickup_height,receiver_depth,low_release,successor_check,loaded_joint_speed,socket_planning=False,fallback_air=False):
    from robofrontier.suite.oracle import Trial,ReferenceStopped
    from robofrontier.suite import transport_oracle_v2 as ref
    from robofrontier.suite.placement_oracle import clear_idle_arms,park
    from robofrontier.run_setup import load_harness
    from robofrontier.suite import v2
    from robofrontier.suite.source_snapshot import MARKER
    case=next(c for c in v2.cases() if c['id']==spec['case']);frozen=Path(os.environ[MARKER])
    files=list((frozen/'robofrontier').rglob('*.py'))+list((frozen/'scripts').rglob('*.py'))+[frozen/'inputs/harness.json',frozen/'inputs/catalog.json']
    sources={str(p.relative_to(frozen)):p.read_bytes() for p in files}
    t=Trial(v2.directory(case),output,controller_cfg=load_harness(harness),sources=sources)
    t.live_preview=True;t.abort_on_process_violation=True
    t.stop_requested=lambda:time.monotonic()-t.started>wall or t.time>900 or len(t.actions)>1000 or (output.parent.parent/'STOP').exists()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(ReferenceStopped('External stop')))
    report=dict(case_id=case['id'],passed=False,commissioning_only=True,unit=spec,
        scope=__doc__,execution_mode='pure_frozen_program',oracle_privileged_geometry=True,
        fresh_canonical_initialization=True,grasp_policy=grasp_policy,socket_planning=socket_planning,fallback_air=fallback_air,receiver_tilt_deg=receiver_tilt,pickup_height_override_m=pickup_height,receiver_depth_m=receiver_depth,low_release=low_release,successor_check=successor_check,loaded_joint_speed=loaded_joint_speed,local_passed=False,jobs=[],budgets=dict(wall_s=wall,sim_s=900,action_batches=1000))
    original_execute=t.execute
    def execute(label,commands,allow_failure=False):
        carrying=dict(t.held)
        for r in commands:
            if t.arms[r].grasp_force is not None and r not in carrying:
                active=getattr(t,'unit_picking',None)
                if active:carrying[r]=active
        commands=ref.cylinder_commands(commands,carrying,{o['id']:o for o in t.env['objects']})
        if loaded_joint_speed is not None:
            for robot,(op,params) in commands.items():
                if op=='move' and robot in carrying:
                    for key,limit in [('joint_speed_rad_s',loaded_joint_speed),('speed_m_s',.025),('angular_speed_rad_s',.25)]:
                        if key in params:params[key]=min(params[key],limit)
        if label.startswith(('regrasp-donor-transit','regrasp-send-clear')):
            for op,params in commands.values():
                if op=='move':
                    for k,limit in [('joint_speed_rad_s',.15),('speed_m_s',.025),('angular_speed_rad_s',.25)]:
                        if k in params:params[k]=min(params[k],limit)
        return original_execute(label,commands,allow_failure=allow_failure)
    t.execute=execute
    try:
        t.wait(.6);clear_idle_arms(t)
        for job in spec['jobs']:
            robot,name=job['robot'],job['object'];goal=next(g for g in t.task['goals'] if g['goal_id']==job['goal'])
            obj=next(o for o in t.env['objects'] if o['id']==name)
            def pose():
                b=t.data.body(goal['board']);r=b.xmat.reshape(3,3).copy()
                return b.xpos+r@np.asarray(goal['local_position_m']),r
            entry=dict(job=job,identity_matches=obj['identity']==goal['requires']['identity'],start_sim_s=t.time)
            report['jobs'].append(entry);t.event('minimum_unit_job',spec=job,identity_matches=entry['identity_matches'])
            if socket_planning and case['level']>1:
                from robofrontier.suite.transport_socket_motion import guard_contact_moves
                # One guard per job; restore the speed-policy wrapper first.
                t.execute=execute
                guard_contact_moves(t,goal['board'])
            rotation=t.data.body(name).xmat.reshape(3,3).copy();height=ref._pickup_height(t.task,rotation,.006,obj['shape'])
            if pickup_height is not None:height=pickup_height
            t.unit_picking=name
            if job.get('grasp_policy',grasp_policy) in ('side','planned'):
                from robofrontier.suite.transport_unit_motion import direct_pick
                hint=pose()
                if job.get('receiver'):
                    bases=[t.stations[r] for r in (robot,job['receiver'])];points=[]
                    for station in bases:
                        yaw=station['yaw_rad'];points.append(np.asarray(station['base_position_m'])+np.array([.20*np.cos(yaw),.20*np.sin(yaw),0]))
                    target=(points[0]+points[1])/2;target[2]=t.task['table_height_m']+.38
                    hint=(target,rotation)
                try:
                    direct_pick(t,robot,name,hint,goal['board'],check_seating=socket_planning and not job.get('receiver'))
                except RuntimeError as error:
                    if not fallback_air or not str(error).startswith('No finite direct pickup/insertion candidate') or any(a.job is not None for a in t.arms.values()) or t.monitor.violations:raise
                    t.event('unit_direct_strategy_unavailable',robot=robot,object=name,reason=str(error),
                        scope='No direct-grasp candidate executed; try pickup for subsequent air regrasp')
                    if obj['shape']=='cylinder' and abs(rotation[2,2])<.8:
                        ref._pick_side_cylinder(t,robot,name,height,70.)
                    else:t.pick(robot,name,height=height,force=70.,placement_hint=pose())
            elif obj['shape']=='cylinder' and abs(rotation[2,2])<.8:ref._pick_side_cylinder(t,robot,name,height,70.)
            else:t.pick(robot,name,height=height,force=70.,placement_hint=pose())
            t.unit_picking=None;entry['picked_sim_s']=t.time
            if job.get('receiver'):
                receiver=job['receiver'];a=t.stations[robot];b=t.stations[receiver]
                centers=[]
                for station in (a,b):
                    yaw=station['yaw_rad'];centers.append(np.asarray(station['base_position_m'])+np.array([.20*np.cos(yaw),.20*np.sin(yaw),0]))
                center=(centers[0]+centers[1])/2;center[2]=t.task['table_height_m']+.40
                t.event('unit_private_handover_candidate',donor=robot,receiver=receiver,center_m=center.tolist(),scope='Planner-proposed center; not a scene region, permission boundary or feasible-motion certificate')
                robot=ref.regrasp_held_plug(t,robot,name,keep_receiver_held=True,
                    allowed_receivers=[receiver],candidate_centers={receiver:center},receiver_tilt_deg=receiver_tilt,
                    receiver_depths=None if receiver_depth is None else [receiver_depth],
                    future_goal=dict(goal,release_clearance_m=.015 if low_release else 0.,check_socket_path=socket_planning) if successor_check or socket_planning else None)
                entry['actual_receiver']=robot;entry['handover_sim_s']=t.time
            p,r=pose()
            if socket_planning and case['level']>1:
                from robofrontier.suite.transport_socket_motion import place_socket,SocketPlanningError,may_replan_in_air
                try:
                    place_socket(t,robot,p,r,goal['board'],pose_provider=pose)
                except SocketPlanningError as error:
                    # Never pull a partially inserted plug out implicitly or
                    # replay an unresolved action. Free-air guarded rejection
                    # can replan in the same world after a known transit.
                    if not fallback_air or not may_replan_in_air(t,robot,name,goal['board'],error.stage):raise
                    entry['forward_recovery']=dict(stage=error.stage,reason=error.diagnostics,donor=robot,world_reset=False)
                    t.event('unit_forward_replan',object=name,donor=robot,reason=error.diagnostics,
                        scope='Same physical world; choose a receiving grasp for the unfinished goal')
                    receivers=sorted((other for other in t.arms if other!=robot),
                        key=lambda other:np.linalg.norm(np.asarray(t.stations[other]['base_position_m'])[:2]-p[:2]))[:3]
                    centers={}
                    for other in receivers:
                        points=[]
                        for station in (t.stations[robot],t.stations[other]):
                            yaw=station['yaw_rad'];points.append(np.asarray(station['base_position_m'])+np.array([.20*np.cos(yaw),.20*np.sin(yaw),0]))
                        center=(points[0]+points[1])/2;center[2]=t.task['table_height_m']+.40;centers[other]=center
                    for tilt in (0.,-60.,60.):
                        before_actions=len(t.actions)
                        try:
                            next_robot=ref.regrasp_held_plug(t,robot,name,keep_receiver_held=True,
                                allowed_receivers=receivers,candidate_centers=centers,
                                receiver_tilt_deg=tilt,receiver_depths=(.006,-.012,.020),
                                future_goal=dict(goal,check_socket_path=True))
                            robot=next_robot;entry['forward_recovery']['receiver_tilt_deg']=tilt;break
                        except RuntimeError as rejected:
                            # Try another private candidate family only when
                            # none of this family's motions were executed.
                            if tilt==60. or len(t.actions)!=before_actions or not str(rejected).startswith('No bounded physical air-regrasp candidate'):raise
                            t.event('unit_handover_family_rejected',robot=robot,object=name,tilt_deg=tilt,reason=str(rejected))
                    entry['forward_recovery']['receiver']=robot
                    p,r=pose();place_socket(t,robot,p,r,goal['board'],pose_provider=pose)
            else:
                with ref.socket_contact_clearance(t,name,goal['board']) if case['level']>1 else nullcontext():
                    t.place(robot,p,r,precise=not(low_release and case['level']==1),pose_provider=pose)
            park(t,robot);t.wait(2.)
            entry['geometry']=seat_geometry(t,name,goal)
            entry['original_order_accepted']=t.monitor.result().get('assignments',{}).get(goal['goal_id'])==name
            entry['end_sim_s']=t.time
            # Observe real drift for a full second after gripper withdrawal.
            initial=t.data.body(name).xpos.copy();t.wait(1.);entry['stability_drift_m']=float(np.linalg.norm(t.data.body(name).xpos-initial))
            if not entry['geometry']['passed'] or entry['stability_drift_m']>.001:
                raise RuntimeError('Physical seating or stability did not pass')
            if entry['identity_matches'] and not entry['original_order_accepted']:
                raise RuntimeError('Original task did not accept the matching-identity placement')
        for entry in report['jobs']:
            goal=next(g for g in t.task['goals'] if g['goal_id']==entry['job']['goal'])
            entry['final_geometry']=seat_geometry(t,entry['job']['object'],goal)
        accepted=t.monitor.result().get('assignments',{})
        report['local_passed']=all(e['final_geometry']['passed'] and (not e['identity_matches'] or accepted.get(e['job']['goal'])==e['job']['object']) for e in report['jobs']) and not t.monitor.violations
    except (Exception,KeyboardInterrupt) as error:
        report['error']=type(error).__name__+': '+str(error)
        if hasattr(error,'diagnostics'):report['planning_failure']=dict(stage=error.stage,diagnostics=error.diagnostics)
    finally:
        try:t.snapshot('final')
        except Exception as error:report['snapshot_error']=str(error)
        report['evaluation']=t.monitor.result();report['unresolved_robot_actions']=[r for r,a in t.arms.items() if a.job is not None]
        t.close(report)
    print(json.dumps(dict(unit=spec['id'],local_passed=report['local_passed'],error=report.get('error'),report=str(output/'report.json'))),flush=True)
    return report['local_passed']


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--unit');p.add_argument('--output',type=Path);p.add_argument('--wall',type=float,default=180.);p.add_argument('--grasp-policy',choices=['top','side','planned'],default='top');p.add_argument('--receiver-tilt',type=float,default=0.);p.add_argument('--pickup-height',type=float);p.add_argument('--receiver-depth',type=float);p.add_argument('--l1-low-release',action='store_true');p.add_argument('--successor-check',action='store_true');p.add_argument('--loaded-joint-speed',type=float);p.add_argument('--socket-planning',action='store_true');p.add_argument('--fallback-air',action='store_true');p.add_argument('--list',action='store_true');a=p.parse_args()
    if a.list:print(json.dumps(specifications(),indent=2));raise SystemExit
    if not a.unit or not a.output:p.error('--unit and --output required')
    batch=ROOT/'runs/oracle/transport-r13-units-20261025'
    from robofrontier.suite.source_snapshot import reexec_entry,MARKER
    if not os.environ.get(MARKER):reexec_entry(ROOT,a.output,Path(__file__),__import__('sys').argv[1:],batch/'harness.json',batch/'catalog.json')
    frozen=Path(os.environ[MARKER]);spec=next(s for s in specifications() if s['id']==a.unit)
    ok=run(spec,a.output,frozen/'inputs/harness.json',a.wall,a.grasp_policy,a.receiver_tilt,a.pickup_height,a.receiver_depth,a.l1_low_release,a.successor_check,a.loaded_joint_speed,a.socket_planning,a.fallback_air)
    raise SystemExit(0 if ok else 2)
