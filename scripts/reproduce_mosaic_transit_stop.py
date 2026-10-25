"""A declared diagnostic fixture, never a resumed/canonical artwork run.

Initialize the complete scene at the recorded pre-transit state BEFORE time
starts, physically establish the existing force grasp, execute one real move.
No runtime object state restoration, welds or evaluator qualification.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.suite import v2
from robofrontier.suite.oracle import Trial
from robofrontier.run_setup import load_harness,write_json


def fixture(source,folder):
    events=[json.loads(line) for line in (source/'trace.jsonl').read_text().splitlines()]
    failure=next(e for e in reversed(events) if e['event']=='action_start' and e.get('label','').startswith('mosaic-feedback-transit'))
    state=np.load(source/'event-states.npz');q=state['qpos'][failure['state_index']];vel=state['qvel'][failure['state_index']]
    if folder.exists():return failure
    folder.mkdir(parents=True)
    from scripts.setup_worktree_data import clone
    clone(source/'assets',folder/'assets')
    root=ET.parse(source/'scene.xml').getroot();old=mujoco.MjModel.from_xml_path(str(source/'scene.xml'))
    ctrl=old.key_ctrl[old.key('initial').id].copy()
    for r in json.loads((source/'task.json').read_text())['arms']:
        for i in range(1,8):ctrl[old.actuator(f'{r}_actuator{i}').id]=q[old.jnt_qposadr[old.joint(f'{r}_joint{i}').id]]
        width=sum(q[old.jnt_qposadr[old.joint(r+'_'+s).id]] for s in ('finger_joint1','finger_joint2'))
        ctrl[old.actuator(r+'_actuator8').id]=255*width/.08
    key=root.find("keyframe/key[@name='initial']");key.set('qpos',v2.vector(q));key.set('qvel',v2.vector(vel));key.set('ctrl',v2.vector(ctrl))
    ET.indent(root);ET.ElementTree(root).write(folder/'scene.xml',encoding='unicode')
    m=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    env=json.loads((source/'environment.json').read_text());task=json.loads((source/'task.json').read_text())
    for obj in env['objects']:obj['initial_position_m']=d.body(obj['id']).xpos.copy().tolist()
    task['objects']=deepcopy(env['objects']);task['scene_sha256']=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest()
    task['diagnostic_fixture_only']=True;env['diagnostic_fixture_only']=True
    write_json(folder/'environment.json',env);write_json(folder/'task.json',task)
    case=deepcopy(task['scale']);case['diagnostic_fixture_only']=True;case['directory']=str(folder)
    v2.manifest(m,d,case,folder,env)
    write_json(folder/'fixture-provenance.json',dict(source_attempt=str(source),event_index=failure['state_index'],
        initialization='Recorded qpos/qvel and matching encoder holds BEFORE first physics step; real force grasp established anew',
        not_a_physics_checkpoint_resume=True,not_benchmark_evidence=True,original_scene_geometry_material_controls_unchanged=True))
    return failure


def main(a):
    source=a.source.resolve();output=a.output.resolve();failure=fixture(source,output.parent/'fixture-source')
    cfg=load_harness(a.harness);sources={str(p.relative_to(v2.ROOT)):p.read_bytes() for base in ('robofrontier','scripts') for p in (v2.ROOT/base).rglob('*.py')}
    t=Trial(output.parent/'fixture-source',output,controller_cfg=cfg,sources=sources);t.abort_on_process_violation=True
    t.stop_requested=lambda:t.time>120
    robot=next(iter(failure['commands']));joints=np.asarray(failure['commands'][robot][1]['joint_rad']);target,rotation=t.arms[robot].ik.pose(joints)
    report=dict(case_id=t.task['id'],passed=False,commissioning_only=True,diagnostic_fixture_only=True,diagnostic_passed=False)
    try:
        t.execute('fixture-real-force-grasp',{robot:('gripper',dict(mode='grasp',width_m=.06,force_n=50.,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008))})
        if not t.finger_contacts(robot,'A_stock09'):raise RuntimeError('Diagnostic fixture failed to establish real support')
        t.held[robot]='A_stock09';t.mosaic_motion_profile={} if a.joint_speed is None else {'loaded_joint_speed_rad_s':a.joint_speed};t.move_free_original=t.move_free
        def fine(*args,**kwargs):kwargs.setdefault('route_joint_step_rad',.005);return t.move_free_original(*args,**kwargs)
        t.move_free=fine
        from robofrontier.suite.mosaic_oracle_v2 import execute_loaded_transit
        execute_loaded_transit(t,robot,target,rotation,joints)
        report['diagnostic_passed']=bool(t.finger_contacts(robot,'A_stock09') and np.max(abs(t.data.qpos[t.arms[robot].qids]-joints))<.004 and not t.monitor.violations)
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if a.observe_after_failure and not any(x.job is not None for x in t.arms.values()) and not t.monitor.violations:
            observations=[]
            for _ in range(20):
                t.wait(.02);observations.append(dict(joint_error_rad=float(np.max(abs(t.data.qpos[t.arms[robot].qids]-joints))),joint_speed_rad_s=float(np.max(abs(t.data.qvel[t.arms[robot].dofs]))),bilateral=bool(t.finger_contacts(robot,'A_stock09'))))
            report['post_failure_observations']=observations
    finally:
        report.update(evaluation=t.monitor.result(),unresolved_robot_actions=[r for r,x in t.arms.items() if x.job is not None]);t.close(report)
    print(json.dumps(dict(diagnostic_passed=report['diagnostic_passed'],error=report.get('error'),wall_s=report['wall_time_s'],report=str(output/'report.json'))),flush=True)
    return 0 if report['diagnostic_passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--harness',type=Path,default=v2.ROOT/'configs/harness-frontier-v2-servo2.json');p.add_argument('--joint-speed',type=float);p.add_argument('--observe-after-failure',action='store_true');a=p.parse_args();raise SystemExit(main(a))
