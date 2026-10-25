"""Declared noncanonical diagnostic: real timeout -> same-world repair -> seat.

Uses the archived pre-transit fixture only BEFORE the first physics step.
Neither its nine initialized cells nor the repaired tenth cell are a full-task
Oracle score. No live state restoration, controller tuning or hot source load.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

from scripts.reproduce_mosaic_transit_stop import fixture
from robofrontier.run_setup import load_harness,write_json
from robofrontier.suite.v2 import ROOT
from robofrontier.suite.oracle import Trial
from robofrontier.suite.mosaic_run import MosaicReview,run_reference
from robofrontier.suite.source_snapshot import reexec_entry,MARKER


def run(a):
    source=a.source.resolve();output=a.output.resolve()
    failure=fixture(source,output.parent/'fixture-source')
    sources={str(p.relative_to(ROOT)):p.read_bytes() for base in ('robofrontier','scripts') for p in (ROOT/base).rglob('*.py')}
    t=Trial(output.parent/'fixture-source',output,controller_cfg=load_harness(a.harness),sources=sources)
    t.abort_on_process_violation=True;t.stop_requested=lambda:t.time>180 or time.monotonic()-t.started>180
    # Planning metadata comes from the original initial scene, never writes live data.
    original=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));initial=mujoco.MjData(original)
    mujoco.mj_resetDataKeyframe(original,initial,original.key('initial').id);mujoco.mj_forward(original,initial)
    for robot,arm in t.arms.items():
        site=original.site(robot+'_grip_site').id if mujoco.mj_name2id(original,mujoco.mjtObj.mjOBJ_SITE,robot+'_grip_site')>=0 else arm.site
        t.home[robot]=(initial.site_xpos[site].copy(),initial.site_xmat[site].reshape(3,3).copy())
        t.home_q[robot]=initial.qpos[arm.qids].copy()
    first=json.loads((source/'actions.json').read_text())[0]
    t.parking_q={r:np.asarray(command[1]['joint_rad']) for r,command in first['commands'].items()}
    plan=json.loads((source/'supply-plan.json').read_text())
    job=next(j for j in plan['jobs'] if j['object']=='A_stock09')
    t.mosaic_current_job=dict(goal_id=job['goal'],object=job['object'],path=job['path'])
    robot=next(iter(failure['commands']));joints=np.asarray(failure['commands'][robot][1]['joint_rad'])
    target,rotation=t.arms[robot].ik.pose(joints)
    report=dict(passed=False,diagnostic_fixture_only=True,diagnostic_passed=False,commissioning_only=True,
        scope='Independent physical recovery fixture; never resumed original attempt or full artwork qualification')
    def enter(review):
        # This is a declared frozen engineering decision, not a live model answer.
        t.wait(.4)
        if not t.finger_contacts(robot,'A_stock09'):raise RuntimeError('Lost payload while reviewing')
        requests=[dict(id='retry-slow',operation='commands',batches=[{robot:['move',dict(joint_rad=joints.tolist(),joint_speed_rad_s=.2)]}]),
            dict(id='finish-held',operation='finish_held',robot=robot),dict(id='resume',operation='resume')]
        for i,request in enumerate(requests):
            request.update(world_id=review.world_id,review_id=review.epoch)
            # Order filenames deterministically; use the normal durable queue.
            write_json(review.directory/'requests'/f'{i:02d}.json',request)
    review=MosaicReview(t,review_wall_s=45,on_enter=enter)
    calls=0
    def segment(world,**kwargs):
        nonlocal calls
        calls+=1
        if calls==1:
            t.move_free(robot,target,rotation,'diagnostic-original-transit',target_joints=joints,joint_speed_rad_s=.45,route_joint_step_rad=.005)
            raise ValueError('Original timeout did not reproduce; no repair result claimed')
        t.wait(1.2)
        return dict(recovered_segment=True)
    try:
        t.execute('fixture-real-force-grasp',{robot:('gripper',dict(mode='grasp',width_m=.06,force_n=50.,speed_m_s=.04,epsilon_inner_m=.008,epsilon_outer_m=.008))})
        if not t.finger_contacts(robot,'A_stock09'):raise RuntimeError('Fixture did not establish real support')
        t.held[robot]='A_stock09';t.mosaic_motion_profile={'loaded_joint_speed_rad_s':.2}
        t.mosaic_protected_assignments=dict(t.monitor.result().get('assignments',{}))
        protected=dict(t.mosaic_protected_assignments)
        run_reference(t,review,solver=segment,max_reviews=2)
        evaluation=t.monitor.result()
        report['diagnostic_passed']=bool(calls==2 and len(review.history)==1 and
            evaluation['assignments'].get(job['goal'])=='A_stock09' and
            all(evaluation['assignments'].get(k)==v for k,v in protected.items()) and
            not t.held and not t.monitor.violations and all(r['status']=='completed' for r in review.requests.values()) and
            any(r['status']=='failed' for action in t.actions for r in action['results'].values()))
    except Exception as error:report['error']=f'{type(error).__name__}: {error}'
    finally:
        report.update(evaluation=t.monitor.result(),unresolved_robot_actions=[r for r,arm in t.arms.items() if arm.job is not None],
            recovery_reviews=review.history,recovery_requests=list(review.requests.values()))
        if report['unresolved_robot_actions']:report['diagnostic_passed']=False
        t.close(report)
    print(json.dumps(dict(diagnostic_passed=report['diagnostic_passed'],error=report.get('error'),
        goals=report['evaluation']['matched_goals'],failed_actions=report['failed_robot_actions'],report=str(output/'report.json'))),flush=True)
    return 0 if report['diagnostic_passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'runs/oracle/mosaic-r13-worktree-20261025/pilot/mosaic-a04-6x12-d1/seed-0/attempt-001')
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--harness',type=Path,default=ROOT/'configs/harness-frontier-v2-servo2.json')
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-mosaic-r13.json');a=p.parse_args()
    if os.environ.get(MARKER)!=str(ROOT):
        reexec_entry(ROOT,a.output,Path(__file__),sys.argv[1:],a.harness,a.catalog,cli_config=True)
    raise SystemExit(run(a))
