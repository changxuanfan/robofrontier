"""Frozen R13 full-order transport Oracle, with exclusive physics ownership."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
DEPS=Path(os.environ.get('TRANSPORT_ORACLE_DEPS',ROOT/'.oracle-deps')).resolve()
sys.path.insert(0,str(DEPS))


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-transport-r13.json')
    p.add_argument('--harness',type=Path,default=ROOT/'runs/oracle/transport-r13-units-20261025/harness.json')
    p.add_argument('--wall-s',type=float,help='Optional explicit episode wall cap; default is unlimited')
    p.add_argument('--sim-s',type=float,help='Optional explicit episode simulation cap; default is unlimited')
    p.add_argument('--pause-on-handback',action='store_true',help='Pause idle handbacks and known grasp-fault reviews; preserve other in-flight controllers until owner resumes')
    return p.parse_args(argv)


def main():
    a=parse_args();out=a.output.resolve()
    from robofrontier.suite.source_snapshot import MARKER,freeze_executor
    if not os.environ.get(MARKER):
        if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
            raise RuntimeError('Commit the Transport run source before starting')
        env=os.environ.copy();env['TRANSPORT_ORACLE_WORKTREE']=str(ROOT)
        env['TRANSPORT_ORACLE_COMMIT']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        launch=out.parent/'_executor_launches'/('transport-'+uuid.uuid4().hex[:12])
        frozen=freeze_executor(ROOT,launch,a.harness.resolve(),a.catalog.resolve())
        env[MARKER]=str(frozen);env['TRANSPORT_ORACLE_DEPS']=str(DEPS);env['PYTHONPATH']=str(frozen)
        args=[sys.executable,str(frozen/'scripts/run_transport_full_oracle.py'),'--case',a.case,'--output',str(out),
            '--harness',str(frozen/'inputs/harness.json'),'--catalog',str(frozen/'inputs/catalog.json')]
        if a.wall_s is not None:args.extend(['--wall-s',str(a.wall_s)])
        if a.sim_s is not None:args.extend(['--sim-s',str(a.sim_s)])
        if a.pause_on_handback:args.append('--pause-on-handback')
        os.execve(sys.executable,args,env)
    from robofrontier.suite.oracle import Trial,ReferenceStopped
    from robofrontier.suite.transport_full_oracle import solve
    from robofrontier.suite.transport_recovery import EngineeringRecovery
    from robofrontier.suite.oracle_budget import ExplicitPauseBudget
    from robofrontier.run_setup import load_harness,write_json
    cases=json.loads(a.catalog.read_text())['cases'];case=next(c for c in cases if c['id']==a.case)
    if case['family']!='transport':raise ValueError('Transport-only runner')
    frozen=Path(os.environ[MARKER]);files=list((frozen/'robofrontier').rglob('*.py'))+list((frozen/'scripts').rglob('*.py'))+[a.harness,a.catalog]
    sources={str(f.relative_to(frozen)):f.read_bytes() for f in files}
    t=Trial(frozen/case['directory'],out,controller_cfg=load_harness(a.harness),sources=sources)
    t.live_preview=True;t.abort_on_process_violation=True;t.reference_inter_robot_clearance_m=.006;stopped={'requested':False}
    budget=ExplicitPauseBudget(out)
    from robofrontier.suite.transport_inventory import unreachable_required_inventory
    def pause_idle(trial,runtime):
        if runtime.active or runtime.running or any(arm.job is not None or arm.uncertainty_reported for arm in trial.arms.values()):
            raise RuntimeError('Engineering handback has unresolved robot actions; cannot adopt or replay them')
        unseen=lambda:any(file.name not in recovery.seen for file in (recovery.path/'inbox').glob('*.json'))
        if unseen():return
        if trial.motion is not None:
            trial.motion.capture(trial.data,force=True);trial.motion.flush()
        trial.event('transport_engineering_paused',reason='Known-idle diagnosis boundary; preserve passive stock and controller state',
            scope='Acknowledged engineering pause, not a real-time policy performance result')
        request=dict(paused=True,reason='Known-idle Oracle engineering diagnosis; no physics or state changes while waiting')
        write_json(out/'control.json',request);budget.update(request)
        # An input published during the pause acknowledgement must not lose
        # its accompanying wake request.
        if unseen():
            request=dict(paused=False,reason='Owner input arrived during pause acknowledgement')
            write_json(out/'control.json',request);budget.update(request)
    def pause_review(trial,runtime,notice):
        # Affected actions are known terminal; unrelated in-flight actions
        # remain in memory and resume their original commands after the gate.
        # This is an engineering freeze, never an exact disk checkpoint.
        if trial.motion is not None:
            trial.motion.capture(trial.data,force=True);trial.motion.flush()
        trial.event('transport_engineering_review_paused',**notice,
            scope='Known affected actions; independent controller jobs preserved, never adopted or replayed')
        request=dict(paused=True,reason='Measured grasp/preparation review: '+str(notice['reason'])[:300])
        write_json(out/'control.json',request);budget.update(request)
        if any(file.name not in recovery.seen for file in (recovery.path/'inbox').glob('*.json')):
            request=dict(paused=False,reason='Owner recovery input already queued at review acknowledgement')
            write_json(out/'control.json',request);budget.update(request)
        trial.check_stop()
    recovery=EngineeringRecovery(out,on_idle=pause_idle if a.pause_on_handback else None,
        inventory_check=unreachable_required_inventory,on_review=pause_review if a.pause_on_handback else None)
    signal.signal(signal.SIGTERM,lambda *_:stopped.update(requested=True))
    stop=lambda:stopped['requested'] or (a.wall_s is not None and budget.times()['active_wall_s']>a.wall_s) or (a.sim_s is not None and t.time>a.sim_s) or (out.parent.parent/'STOP').exists()
    t.stop_requested=lambda:budget.gate(stop)
    dependency_files={str(f.relative_to(DEPS)):hashlib.sha256(f.read_bytes()).hexdigest() for f in DEPS.rglob('*.so')}
    report=dict(case_id=case['id'],passed=False,full_task_attempt=True,commissioning_only=False,
        git_commit=os.environ['TRANSPORT_ORACLE_COMMIT'],worktree=os.environ['TRANSPORT_ORACLE_WORKTREE'],
        dependencies=dict(greenlet=importlib.metadata.version('greenlet'),binary_sha256=dependency_files),
        scope='Privileged geometry engineering Oracle; one real world, original full-order acceptance',
        engineering_protocol=dict(pause_on_handback=a.pause_on_handback,inventory_guard=True,
            known_action_fault_review=a.pause_on_handback,prepared_reservations=True,
            full_state_observation='Coherent qpos/qvel for engineering; not exact checkpoint restoration'),
        oracle_grasp_policy=dict(preferred_force_n=50.,epsilon_inner_m=.008,pickup_probe_m=.02,
            physical_force_bounds_unchanged=True),
        budgets=dict(wall_s=a.wall_s,simulation_s=a.sim_s,action_batches=None,
            wall_protocol='No default episode cap; all time recorded; only explicitly supplied caps apply'))
    try:
        t.wait(.6);t.snapshot('initial');report['frontier']=solve(t,on_blocked=recovery)
        t.wait(2.);result=t.monitor.result()
        unresolved=[r for r,arm in t.arms.items() if arm.job is not None or arm.uncertainty_reported]
        report.update(evaluation=result,unresolved_robot_actions=unresolved)
        report['passed']=bool(result.get('task_success') and not result['violations'] and not unresolved and not report['frontier']['pending'] and not report['frontier']['failed'] and not report['frontier']['retained_resources'])
        if not report['passed']:report['error']='Full original order not completed; inspect preserved frontier'
    except (Exception,KeyboardInterrupt) as error:
        report['error']=type(error).__name__+': '+str(error)
        if isinstance(error,(ReferenceStopped,KeyboardInterrupt)):report['interrupted']=True
    finally:
        report['timing']=budget.times()
        report['engineering_paused']=report['timing']['paused_wall_s']>0
        report['evaluation']=t.monitor.result()
        report['unresolved_robot_actions']=[r for r,arm in t.arms.items() if arm.job is not None or arm.uncertainty_reported]
        report['final_robot_states']={r:arm.observe() for r,arm in t.arms.items()}
        try:t.snapshot('final')
        except Exception as error:report['snapshot_error']=str(error)
        if recovery.external_inputs:
            report.update(llm_used=True,engineering_assisted=True,engineering_inputs=recovery.external_inputs)
        elif report['engineering_paused']:report['engineering_assisted']=True
        t.close(report)
        if recovery.external_inputs or report['engineering_paused']:
            # Trial.close describes the frozen executor by default. Explicit
            # owner-authored live repairs must not be mislabeled as pure code.
            report.update(llm_used=bool(recovery.external_inputs),engineering_assisted=True,engineering_inputs=recovery.external_inputs)
            write_json(out/'report.json',report)
        write_json(out/'status.json',dict(phase='passed' if report['passed'] else 'failed',report=str(out/'report.json'),updated_at_unix_s=time.time()))
    print(json.dumps(dict(case_id=case['id'],passed=report['passed'],matched_goals=report['evaluation'].get('matched_goals'),
        error=report.get('error'),report=str(out/'report.json'))),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
