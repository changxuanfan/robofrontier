"""Independent real-dynamics pickup -> arbitrary central placement units.

Each unit starts a separately declared canonical world. Failures are sealed,
never rolled back within a unit. This is an engineering Oracle, not a policy
run or a complete artwork benchmark. Existing control/physics stay unchanged.
"""
import argparse
import os
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from robofrontier.suite.oracle import Trial
from robofrontier.suite.v2 import ROOT
from robofrontier.run_setup import write_json,load_harness

DEFAULT_OUT=ROOT/'runs/engineering/mosaic-r13-physical-pick-place-20261025'


def freeze_sources(output):
    paths=[ROOT/'robofrontier'/n for n in ['robot_control.py','sim.py','motion_check.py','task_config.py','pipeline.py']]
    paths += [ROOT/'robofrontier/suite'/n for n in ['oracle.py','joint_route.py','evaluate.py','evaluate_v2.py','provenance.py']]
    paths += [ROOT/'scripts/probe_mosaic_pick_place.py',ROOT/'scripts/dispatch_mosaic_units.py',ROOT/'configs/harness-frontier-v2-servo2.json']
    sources={str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
    folder=output/'source';folder.mkdir(parents=True,exist_ok=True)
    for name,payload in sources.items():
        dest=folder/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists() and dest.read_bytes()!=payload:raise ValueError('Frozen unit source changed; choose a new batch output')
        if not dest.exists():
            temp=dest.with_name(dest.name+f'.{os.getpid()}.tmp');temp.write_bytes(payload);os.replace(temp,dest)
    return sources


def body_state(t,name):
    velocity=np.empty(6);body=t.model.body(name).id
    mujoco.mj_objectVelocity(t.model,t.data,mujoco.mjtObj.mjOBJ_BODY,body,velocity,0)
    return dict(sim_s=t.time,position_m=t.data.body(name).xpos.copy().tolist(),
        linear_speed_m_s=float(np.linalg.norm(velocity[3:])),angular_speed_rad_s=float(np.linalg.norm(velocity[:3])))


def unit(case,name,output,sources,coverage):
    item=next(x for x in coverage['sources'] if x['id']==name);w=item['witness'];robot=w['robot']
    directory=output/case['id']/name/'attempt-001'
    if directory.exists():raise FileExistsError('Unit already exists; do not silently replay '+str(directory))
    cfg=load_harness(ROOT/'configs/harness-frontier-v2-servo2.json')
    t=Trial(ROOT/case['directory'],directory,frames=True,sources=sources,controller_cfg=cfg)
    t.abort_on_process_violation=True
    # This explicitly authorized unit test ignores the artwork. Keep the
    # original per-physics-step collision auditor and real contact helpers;
    # avoid matching hundreds of colors/cells that do not enter unit success.
    def unit_goal_sample():
        t.monitor.result_cache=dict(task_success=False,all_goals_met=False,
            artwork_scoring_skipped=True,scope='Independent pick/place unit; not artwork completion')
    t.monitor.sample_goals=unit_goal_sample
    t.stop_requested=lambda:t.time>180 or time.monotonic()-t.started>180
    report=dict(case_id=case['id'],object=name,robot=robot,passed=False,commissioning_only=True,
        unit_passed=False,scope='Fresh independent real-dynamics unit: original supply -> arbitrary central support; no color/pattern requirement. Not a complete task pass.',
        initial_scene_reset='New independent episode before this unit only; no in-unit state writes/reset',
        physics_timestep_s=float(t.model.opt.timestep),controller_profile=cfg['gripper_position_servo'],
        budgets=dict(wall_s=180,sim_s=180),criteria=dict(lift_m=.08,lift_hold_s=.4,
            stable_window_s=.5,stable_linear_speed_m_s=.01,stable_angular_speed_rad_s=.1,
            central_full_horizontal_projection=True,released_and_robot_clear=True),
        expected_color_ignored=True,lift_samples=[],retention_samples=[])
    report['unit_evaluator']=dict(artwork_scoring_skipped=True,original_collision_audit_preserved=True,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),physics_and_control_unchanged=True)
    report['motion_recipe']=dict(loaded_joint_speed_rad_s=.20,central_boundary_margin_m=.10,grip_translation_drift_is_diagnostic=True)
    origin={o['id']:t.data.body(o['id']).xpos.copy() for o in t.task['objects']}
    watch=dict(carrying=False,released=False);original_step=t.step;original_execute=t.execute
    def step():
        original_step()
        if t.tick%t.stride==0 and watch['carrying'] and not watch['released']:
            state=body_state(t,name);state['bilateral']=bool(t.finger_contacts(robot,name))
            tool=t.data.site_xmat[t.arms[robot].site].reshape(3,3)
            state['object_tool_offset_m']=(tool.T@(t.data.body(name).xpos-t.data.site_xpos[t.arms[robot].site])).tolist()
            report['retention_samples'].append(state)
    def execute(label,commands,allow_failure=False):
        if label=='place-relax':
            watch['released']=True;t.event('unit_intended_release',object=name,robot=robot)
        return original_execute(label,commands,allow_failure)
    t.step=step;t.execute=execute
    original_free=t.move_free
    def move_free(r,*args,**kwargs):
        if r in t.held:kwargs.setdefault('joint_speed_rad_s',.20)
        return original_free(r,*args,**kwargs)
    t.move_free=move_free
    try:
        t.wait(.2)
        aperture=w['approach_width_m'];rotation=np.asarray(w['tool_rotation']);point=np.asarray(w['tcp_m'])
        t.execute('unit-open',{robot:('gripper',dict(mode='move',width_m=aperture,speed_m_s=.1))})
        t.execute('unit-approach',{robot:('move',dict(joint_rad=w['hover_joint_rad'],joint_speed_rad_s=.8))})
        t.move(robot,point,rotation,'unit-contact',precise=True,speed_m_s=.10)
        t.execute('unit-grasp',{robot:('gripper',dict(mode='grasp',width_m=.06,force_n=50.,speed_m_s=.05,
            epsilon_inner_m=.008,epsilon_outer_m=.008))})
        t.wait(.12)
        if not t.finger_contacts(robot,name):raise RuntimeError('No bilateral contact after force grasp')
        t.held[robot]=name;watch['carrying']=True
        t.move(robot,t.data.site_xpos[t.arms[robot].site]+[0,0,.14],rotation,'unit-lift',precise=True,speed_m_s=.06)
        for _ in range(20):
            t.wait(.02);state=body_state(t,name);state['bilateral']=bool(t.finger_contacts(robot,name));report['lift_samples'].append(state)
        samples=report['lift_samples'];positions=np.asarray([x['position_m'] for x in samples])
        lift_ok=bool(all(x['bilateral'] for x in samples) and positions[:,2].min()>origin[name][2]+.08 and
            np.max(np.linalg.norm(positions-positions[0],axis=1))<.003)
        report['lift_passed']=lift_ok
        if not lift_ok:raise RuntimeError('Lift retention/stability unit failed')
        grid=t.task['grid'];hx=grid['columns']*grid['pitch_m']/2;hy=grid['rows']*grid['pitch_m']/2
        interior=[g for g in coverage['goals'] if hx-abs(g['position_m'][0])>=.10 and hy-abs(g['position_m'][1])>=.10]
        choices=[g for g in interior if g['witness']['robot']==robot]
        if not choices:choices=interior
        if not choices:raise RuntimeError('No interior central test target')
        chosen=min(choices,key=lambda g:np.linalg.norm(np.asarray(g['position_m'])[:2]-origin[name][:2]))
        report['chosen_central_cell']=chosen['id'];report['requested_position_m']=chosen['position_m']
        t.place(robot,np.asarray(chosen['position_m']),np.eye(3),precise=False)
        settled=[]
        for _ in range(25):t.wait(.02);settled.append(body_state(t,name))
        body=t.data.body(name);half=abs(body.xmat.reshape(3,3))@np.array([.03,.03,.03]);grid=t.task['grid']
        central=bool(abs(body.xpos[0])+half[0]<grid['columns']*grid['pitch_m']/2 and abs(body.xpos[1])+half[1]<grid['rows']*grid['pitch_m']/2)
        released=bool(t.monitor.released(t.monitor.geoms[name]))
        rows=np.asarray([x['position_m'] for x in settled]);stable=bool(np.max(np.linalg.norm(rows-rows[0],axis=1))<.002 and
            max(x['linear_speed_m_s'] for x in settled)<.01 and max(x['angular_speed_rad_s'] for x in settled)<.1)
        direct_support=any((int(t.model.geom_bodyid[int(c.geom1)])==t.model.body(name).id and int(t.model.geom_bodyid[int(c.geom2)])==0 or
            int(t.model.geom_bodyid[int(c.geom2)])==t.model.body(name).id and int(t.model.geom_bodyid[int(c.geom1)])==0) and c.dist<.001 for c in t.data.contact)
        # Early samples are the commanded lift-off itself, not a dropped
        # payload. Test support throughout; measure slip after lift stabilizes.
        carried=[x for x in report['retention_samples'] if x['sim_s']>=samples[-1]['sim_s']]
        offsets=np.asarray([x['object_tool_offset_m'] for x in carried])
        drift=float(np.max(np.linalg.norm(offsets-offsets[0],axis=1))) if len(offsets) else float('inf')
        # The requested test allows any central placement; relative grip
        # displacement is diagnostic, not an unsolicited precision goal.
        retained=all(x['bilateral'] for x in report['retention_samples'])
        report.update(final_position_m=body.xpos.copy().tolist(),central_region=central,released=released,
            directly_supported=bool(direct_support),settled_samples=settled,stable=stable,retained_during_carry=retained,grip_translation_drift_m=drift,
            unit_passed=bool(lift_ok and central and released and direct_support and stable and retained and not t.monitor.violations))
        if not report['unit_passed']:raise RuntimeError('Final unit acceptance failed')
    except Exception as error:
        report.update(error=type(error).__name__+': '+str(error),unresolved_robot_actions=[r for r,a in t.arms.items() if a.job is not None])
        try:t.snapshot('unit-failure')
        except Exception as e:report['snapshot_error']=str(e)
    finally:
        report.update(evaluation=t.monitor.result(),max_other_object_translation_m=max(float(np.linalg.norm(t.data.body(o).xpos-p)) for o,p in origin.items() if o!=name))
        t.close(report)
    return report


def main(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True);sources=freeze_sources(output)
    cases=json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases']
    case=next(c for c in cases if c['arms']==args.arms and c['level']==args.level)
    coverage=json.loads((ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/case['id']/'report.json').read_text())
    assert coverage['complete'] and coverage['passed']
    assert hashlib.sha256((ROOT/case['directory']/'scene.xml').read_bytes()).hexdigest()==coverage['source_scene_sha256']
    names=args.items or [x['id'] for x in coverage['sources']]
    names=names[args.shard_index::args.shards]
    summary=dict(case_id=case['id'],planned=len(names),unit_results=[],pattern_ignored=True,independent_new_episode_per_item=True)
    destination=output/f'a{args.arms:02}-d{args.level}-shard{args.shard_index:02}.json'
    for name in names:
        result=unit(case,name,output,sources,coverage)
        summary['unit_results'].append(dict(object=name,unit_passed=result['unit_passed'],error=result.get('error'),
            sim_s=result['evaluation']['sim_time_s'],wall_s=result['wall_time_s'],report=str(output/case['id']/name/'attempt-001/report.json')))
        write_json(destination,summary)
        print('UNIT',case['id'],name,'pass',result['unit_passed'],'wall',round(result['wall_time_s'],2),'sim',round(result['evaluation']['sim_time_s'],2),'error',result.get('error'),flush=True)
    summary['complete']=True;write_json(destination,summary)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arms',type=int,choices=[4,6,8],required=True);p.add_argument('--level',type=int,choices=[1,2,3],default=1)
    p.add_argument('--items',nargs='+');p.add_argument('--shards',type=int,default=1);p.add_argument('--shard-index',type=int,default=0);p.add_argument('--output',type=Path,default=DEFAULT_OUT)
    main(p.parse_args())
