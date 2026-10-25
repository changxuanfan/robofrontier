"""One frozen blind-code physical Oracle; use separate processes for nine cases."""
import argparse
import hashlib
import json
import math
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


def make_stop_check(trial,stop,output,began,wall_s=0,sim_s=0,actions=0):
    """Zero disables a user-selected budget; explicit STOP always applies."""
    return lambda:(stop[0] or (Path(output).parent/'STOP').exists()
        or (wall_s>0 and time.monotonic()-began>wall_s)
        or (sim_s>0 and trial.time>sim_s)
        or (actions>0 and len(trial.actions)>actions))


def close_trial(trial,report):
    """Only the current execution owner may drain its submitted actions."""
    adapter=getattr(trial,'_oracle_recovery_attachment',None)
    cleanup=[]
    trial.stop_requested=None;trial.abort_on_process_violation=False
    try:
        if adapter is not None and adapter.attached:
            adapter.pause(adapter.owner)
            for _ in range(800):
                if not adapter.running:break
                adapter.advance(.02)
            cleanup=[feedback for request in adapter.requests.values() for feedback in request['feedback']]
        else:
            for arm in trial.arms.values():
                if arm.job is not None:arm.request_stop(trial.time,reason='oracle_episode_closing')
            for _ in range(round(16./trial.model.opt.timestep)):
                busy=[arm for arm in trial.arms.values() if arm.job is not None]
                if not busy:break
                trial.step()
                for arm in busy:
                    feedback=arm.poll(trial.time,15.)
                    if feedback:cleanup.append(feedback)
    except Exception as error:
        report['cleanup_error']=type(error).__name__+': '+str(error)
        report['passed']=False
    report['terminal_cleanup_feedback']=cleanup
    report['unresolved_robot_actions']=sorted(
        {r for r,arm in trial.arms.items() if arm.job is not None} |
        (set(adapter.unknown) if adapter is not None else set()))
    report['evaluation']=trial.monitor.result()
    report['goal_reached']=bool(report['evaluation'].get('solved'))
    report['online_repair_interventions']=getattr(trial,'oracle_repair_interventions',0)
    report['continued_after_violation']=getattr(trial,'continued_after_violation',False)
    report['passed']=final_success(report)
    if adapter is not None:adapter.close(report)
    else:trial.close(report)


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--codebook',type=Path,required=True);p.add_argument('--wall-s',type=float,default=0,help='Optional wall budget; 0 means no limit')
    p.add_argument('--best5',action='store_true',help='Use R11 five-symbol BEST; --codebook is an evaluator-only per-case puzzle JSON')
    p.add_argument('--harness',type=Path,help='Frozen controller configuration for this batch')
    p.add_argument('--sim-s',type=float,default=0);p.add_argument('--actions',type=int,default=0)
    p.add_argument('--guesses',type=int,default=0,help='Optional BEST5 query budget; 0 means no limit')
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    if any(not math.isfinite(x) or x<0 for x in (a.wall_s,a.sim_s,a.actions,a.guesses)):p.error('Budgets must be nonnegative; zero disables the budget')
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
    protocol=dict(version='dial-best5-continuous-grasp-repair-v2' if a.best5 else 'dial-continuous-grasp-v1',geometry_revision='R11' if a.best5 else 'R10',
        feedback='All nonzero detents within4deg for250ms, real red bilateral force-held contact and separate airborne blue support forL2/L3',
        solver=('Separate -I -S process; N,K,current inputs and timestamped observed inputs/BEST only; no current score or private evaluator history' if a.best5 else 'Separate -I -S process; N and current/guess/visible-count only; private code loaded in different evaluator process'),
        mechanical_warmup_in_episode=False,physics_timestep_s=float(trial.model.opt.timestep))
    if a.best5:protocol['failure_handling']='Retain world/solver for forward actuator repair; keep all failures and violations; no checkpoint splicing'
    write_json(trial.directory/'protocol.json',protocol)
    trial.task['rules'].append('Feedback also requires sustained real red-handle grasp on every device; release invalidates the current combination until re-established. No mandatory release between attempts.')
    write_json(trial.directory/'task.json',trial.task)
    trial.abort_on_process_violation=True;stop=[False];began=time.monotonic()
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    trial.stop_requested=make_stop_check(trial,stop,a.output,began,a.wall_s,a.sim_s,a.actions)
    report=dict(case_id=a.case,passed=False,scope='Full blind BEST-only Oracle; no mechanical warmup or historical command prefix' if a.best5 else 'Full blind count-only Oracle, new continuous-grasp protocol; no historical command prefix',protocol=protocol,
        budgets=dict(wall_s=a.wall_s or None,sim_s=a.sim_s or None,action_batches=a.actions or None,explicit_guesses=a.guesses or None),
        new_physics_integration=True,solver_answer_input=False,mechanical_oracle_geometry_privileged=True,
        new_evaluator_sha256=hashlib.sha256(b''.join((root/'robofrontier/suite'/name).read_bytes() for name in (('dial_best_worker.py','dial_feedback.py','dial_best_oracle.py') if a.best5 else ('dial_blind_worker.py','dial_hold_oracle.py')))).hexdigest())
    try:
        trial.event('blind_protocol_start',protocol=protocol['version'])
        if a.best5:solve(trial,max_guesses=a.guesses or None)
        else:solve(trial)
        report['evaluation']=trial.monitor.result();report['passed']=bool(report['evaluation']['task_success'])
        report['goal_reached']=bool(report['evaluation'].get('solved'))
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['evaluation']=trial.monitor.result()
    finally:
        try:close_trial(trial,report)
        finally:trial.monitor.game.close()
    print(json.dumps(dict(case_id=a.case,passed=report['passed'],error=report.get('error'),report=str(trial.directory/'report.json'))),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
