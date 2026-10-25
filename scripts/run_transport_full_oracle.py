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


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-transport-r13.json')
    p.add_argument('--harness',type=Path,default=ROOT/'runs/oracle/transport-r13-units-20261025/harness.json')
    p.add_argument('--wall-s',type=float,default=1200);p.add_argument('--sim-s',type=float,default=3600)
    a=p.parse_args();out=a.output.resolve()
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
            '--harness',str(frozen/'inputs/harness.json'),'--catalog',str(frozen/'inputs/catalog.json'),
            '--wall-s',str(a.wall_s),'--sim-s',str(a.sim_s)]
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
    t.live_preview=True;t.abort_on_process_violation=True;stopped={'requested':False}
    recovery=EngineeringRecovery(out)
    budget=ExplicitPauseBudget(out)
    signal.signal(signal.SIGTERM,lambda *_:stopped.update(requested=True))
    stop=lambda:stopped['requested'] or budget.times()['active_wall_s']>a.wall_s or t.time>a.sim_s or len(t.actions)>10000 or (out.parent.parent/'STOP').exists()
    t.stop_requested=lambda:budget.gate(stop)
    dependency_files={str(f.relative_to(DEPS)):hashlib.sha256(f.read_bytes()).hexdigest() for f in DEPS.rglob('*.so')}
    report=dict(case_id=case['id'],passed=False,full_task_attempt=True,commissioning_only=False,
        git_commit=os.environ['TRANSPORT_ORACLE_COMMIT'],worktree=os.environ['TRANSPORT_ORACLE_WORKTREE'],
        dependencies=dict(greenlet=importlib.metadata.version('greenlet'),binary_sha256=dependency_files),
        scope='Privileged geometry engineering Oracle; one real world, original full-order acceptance',
        budgets=dict(wall_s=a.wall_s,simulation_s=a.sim_s,action_batches=10000,
            wall_protocol='Planning/recovery time counts; only acknowledged engineering pauses excluded'))
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
        report['evaluation']=t.monitor.result()
        report['unresolved_robot_actions']=[r for r,arm in t.arms.items() if arm.job is not None or arm.uncertainty_reported]
        report['final_robot_states']={r:arm.observe() for r,arm in t.arms.items()}
        try:t.snapshot('final')
        except Exception as error:report['snapshot_error']=str(error)
        if recovery.external_inputs:
            report.update(llm_used=True,engineering_assisted=True,engineering_inputs=recovery.external_inputs)
        t.close(report)
        if recovery.external_inputs:
            # Trial.close describes the frozen executor by default. Explicit
            # owner-authored live repairs must not be mislabeled as pure code.
            report.update(llm_used=True,engineering_assisted=True,engineering_inputs=recovery.external_inputs)
            write_json(out/'report.json',report)
        write_json(out/'status.json',dict(phase='passed' if report['passed'] else 'failed',report=str(out/'report.json'),updated_at_unix_s=time.time()))
    print(json.dumps(dict(case_id=case['id'],passed=report['passed'],matched_goals=report['evaluation'].get('matched_goals'),
        error=report.get('error'),report=str(out/'report.json'))),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
