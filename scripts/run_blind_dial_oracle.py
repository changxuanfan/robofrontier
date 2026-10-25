"""One frozen blind-code physical Oracle; use separate processes for nine cases."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time


def final_success(report):
    """A prior win cannot survive a late violation or unknown action."""
    evaluation=report.get('evaluation',{})
    return bool(report.get('passed') and not report.get('error')
                and evaluation.get('task_success') and not evaluation.get('violations')
                and not report.get('unresolved_robot_actions'))


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--codebook',type=Path,required=True);p.add_argument('--wall-s',type=float,default=1800)
    p.add_argument('--best5',action='store_true',help='Use R11 five-symbol BEST; --codebook is an evaluator-only per-case puzzle JSON')
    p.add_argument('--harness',type=Path,help='Frozen controller configuration for this batch')
    p.add_argument('--sim-s',type=float,default=2400);p.add_argument('--actions',type=int,default=12000)
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    from robofrontier.suite.source_snapshot import reexec_entry
    harness=a.harness or root/'runs/oracle/suite-v2-r10-closure-20261025/inputs/harness.json'
    catalog=root/'configs/suites/dial-best5-r11.json' if a.best5 else root/'runs/oracle/suite-v2-r10-closure-20261025/inputs/catalog.json'
    reexec_entry(root,a.output,Path(__file__),sys.argv[1:],harness,catalog)
    from robofrontier.suite.oracle import Trial
    if a.best5:
        from robofrontier.suite.dial_best_oracle import BestMonitor,solve
    else:
        from robofrontier.suite.dial_hold_oracle import HoldMonitor,solve
    from robofrontier.run_setup import write_json
    cat=json.loads((root/'inputs/catalog.json').read_text())
    case=next(c for c in cat['cases'] if c['id']==a.case and c['family']=='dial')
    # These files are evaluator inputs, never passed to the solver process.
    cfg=json.loads((root/'inputs/harness.json').read_text())
    sources={name:(root/name).read_bytes() for name in json.loads((root/'snapshot.json').read_text())['files']}
    scene=Path(case['directory']);scene=scene if scene.is_absolute() else root/scene
    trial=Trial(scene,a.output,frames=True,sources=sources,controller_cfg=cfg,
        monitor_factory=(lambda w:BestMonitor(w,a.codebook.resolve())) if a.best5 else None)
    if not a.best5:
        trial.monitor.close();trial.monitor=HoldMonitor(trial,a.codebook.resolve())
        (trial.directory/'private-code.json').rename(trial.directory/'unused-r10-initializer-code.json')
    protocol=dict(version='dial-best5-continuous-grasp-v1' if a.best5 else 'dial-continuous-grasp-v1',geometry_revision='R11' if a.best5 else 'R10',
        feedback='All nonzero detents within4deg for250ms, real red bilateral force-held contact and separate airborne blue support forL2/L3',
        solver=('Separate -I -S process; N,K,current inputs and timestamped observed inputs/BEST only; no current score or private evaluator history' if a.best5 else 'Separate -I -S process; N and current/guess/visible-count only; private code loaded in different evaluator process'),
        mechanical_warmup_in_episode=False,physics_timestep_s=float(trial.model.opt.timestep))
    write_json(trial.directory/'protocol.json',protocol)
    trial.task['rules'].append('Feedback also requires sustained real red-handle grasp on every device; release invalidates the current combination until re-established. No mandatory release between attempts.')
    write_json(trial.directory/'task.json',trial.task)
    trial.abort_on_process_violation=True;stop=[False];began=time.monotonic()
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    trial.stop_requested=lambda:stop[0] or (a.output.parent/'STOP').exists() or time.monotonic()-began>a.wall_s or trial.time>a.sim_s or len(trial.actions)>a.actions
    report=dict(case_id=a.case,passed=False,scope='Full blind BEST-only Oracle; no mechanical warmup or historical command prefix' if a.best5 else 'Full blind count-only Oracle, new continuous-grasp protocol; no historical command prefix',protocol=protocol,
        budgets=dict(wall_s=a.wall_s,sim_s=a.sim_s,action_batches=a.actions),
        new_physics_integration=True,solver_answer_input=False,mechanical_oracle_geometry_privileged=True,
        new_evaluator_sha256=hashlib.sha256(b''.join((root/'robofrontier/suite'/name).read_bytes() for name in (('dial_best_worker.py','dial_feedback.py','dial_best_oracle.py') if a.best5 else ('dial_blind_worker.py','dial_hold_oracle.py')))).hexdigest())
    try:
        trial.event('blind_protocol_start',protocol=protocol['version']);solve(trial)
        report['evaluation']=trial.monitor.result();report['passed']=bool(report['evaluation']['task_success'])
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['evaluation']=trial.monitor.result()
    finally:
        # Settle original known actions; unresolved outcomes stay explicit.
        stopping={r:arm.job for r,arm in trial.arms.items() if arm.job is not None}
        cleanup=[]
        for r in stopping:trial.arms[r].request_stop(trial.time,reason='oracle_episode_closing')
        for _ in range(round(16./trial.model.opt.timestep)):
            busy=[arm for arm in trial.arms.values() if arm.job is not None]
            if not busy:break
            try:
                trial.stop_requested=None;trial.abort_on_process_violation=False;trial.step()
                for arm in busy:
                    feedback=arm.poll(trial.time,15.)
                    if feedback:cleanup.append(feedback)
            except Exception:break
        report['terminal_cleanup_feedback']=cleanup
        report['unresolved_robot_actions']=[r for r,arm in trial.arms.items() if arm.job is not None]
        report['evaluation']=trial.monitor.result()
        report['passed']=final_success(report)
        try:trial.close(report)
        finally:trial.monitor.game.close()
    print(json.dumps(dict(case_id=a.case,passed=report['passed'],error=report.get('error'),report=str(trial.directory/'report.json'))),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
