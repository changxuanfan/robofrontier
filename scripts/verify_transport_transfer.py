"""One complete original-order transfer from a fresh standard initial state."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,os.environ.get('TRANSPORT_ORACLE_DEPS',str(ROOT/'.oracle-deps')))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--donor',required=True);p.add_argument('--receiver',required=True)
    p.add_argument('--object',required=True);p.add_argument('--goal',required=True)
    p.add_argument('--case',default='transport-a04-d1')
    p.add_argument('--wall-s',type=float,default=600)
    p.add_argument('--harness',type=Path,default=ROOT/'runs/oracle/transport-r13-units-20261025/harness.json')
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-transport-r13.json')
    a=p.parse_args()
    from robofrontier.suite.source_snapshot import reexec_entry,MARKER
    os.environ.setdefault('TRANSPORT_ORACLE_DEPS',str(ROOT/'.oracle-deps'))
    reexec_entry(ROOT,a.output,Path(__file__),sys.argv[1:],a.harness,a.catalog,cli_config=True)
    from robofrontier.suite.oracle import Trial,ReferenceStopped
    from robofrontier.suite.transport_full_oracle import air_transfer,place_flat,goal_pose
    from robofrontier.suite.placement_oracle import clear_idle_arms,park
    from robofrontier.suite.transport_oracle_v2 import _repair_small_l1_placement
    from robofrontier.run_setup import load_harness
    case=next(c for c in json.loads(a.catalog.read_text())['cases'] if c['id']==a.case)
    sources={str(f.relative_to(ROOT)):f.read_bytes() for package in ('robofrontier','scripts') for f in (ROOT/package).rglob('*.py')}
    t=Trial(ROOT/case['directory'],a.output,controller_cfg=load_harness(a.harness),sources=sources)
    t.live_preview=True;t.abort_on_process_violation=True;t.reference_inter_robot_clearance_m=.006;stop={'requested':False}
    signal.signal(signal.SIGTERM,lambda *_:stop.update(requested=True))
    t.stop_requested=lambda:stop['requested'] or time.monotonic()-t.started>a.wall_s or t.time>400
    report=dict(case_id=case['id'],passed=False,commissioning_only=True,full_task_attempt=False,
        source_snapshot=os.environ[MARKER],goal=a.goal,object=a.object,donor=a.donor,receiver=a.receiver,
        scope='Actual original-order identity; full transfer/release/withdraw/placement in one fresh world')
    try:
        goal=next(g for g in t.task['goals'] if g['goal_id']==a.goal)
        obj=next(o for o in t.env['objects'] if o['id']==a.object)
        if t.task['level']!=1 or obj['identity']!=goal['requires']['identity']:
            raise ValueError('This check requires a matching original L1 order')
        t.wait(.6);t.snapshot('initial');clear_idle_arms(t)
        pose=lambda:goal_pose(t,goal)
        t.pick(a.donor,a.object,height=.006,force=70.,placement_hint=pose())
        receiver=air_transfer(t,a.donor,a.object,[a.receiver],goal)
        place_flat(t,receiver,*pose(),pose_provider=pose);park(t,receiver);t.wait(1.2)
        result=t.monitor.result()
        report['accepted_without_repair']=a.goal in result.get('assignments',{})
        if not report['accepted_without_repair'] and not result['violations']:
            result=_repair_small_l1_placement(t,dict(goal,robot=receiver),a.object)
            t.wait(1.2);result=t.monitor.result()
        report['passed']=bool(a.goal in result.get('assignments',{}) and not result['violations'] and
                              not any(arm.job is not None or arm.uncertainty_reported for arm in t.arms.values()))
        if not report['passed']:report['error']='Original target not accepted cleanly after complete release'
    except (Exception,KeyboardInterrupt) as error:
        report['error']=type(error).__name__+': '+str(error)
        if isinstance(error,(ReferenceStopped,KeyboardInterrupt)):report['interrupted']=True
    finally:
        report['evaluation']=t.monitor.result()
        report['unresolved_robot_actions']=[r for r,arm in t.arms.items() if arm.job is not None or arm.uncertainty_reported]
        t.snapshot('final');t.close(report)
    print(json.dumps({k:report.get(k) for k in ('passed','goal','error','wall_time_s','robot_action_count')}),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
