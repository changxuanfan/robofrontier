"""One frozen R11 BEST5 physical Oracle; explicit launch only."""
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
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True,choices=[f'dial-best5-n{n:02d}-d{level}' for n in (4,6,8) for level in (1,2,3)]);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--codebook',type=Path,required=True);p.add_argument('--wall-s',type=float,default=0,help='Optional wall budget; 0 means no limit')
    p.add_argument('--harness',type=Path,help='Frozen controller configuration for this batch')
    p.add_argument('--sim-s',type=float,default=0);p.add_argument('--actions',type=int,default=0)
    p.add_argument('--guesses',type=int,default=0,help='Optional BEST5 query budget; 0 means no limit')
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    if any(not math.isfinite(x) or x<0 for x in (a.wall_s,a.sim_s,a.actions,a.guesses)):p.error('Budgets must be nonnegative; zero disables the budget')
    from robofrontier.suite.source_snapshot import reexec_entry
    harness=a.harness or root/'configs/harness-frontier-v2-servo2.json'
    catalog=root/'configs/suites/dial-best5-r11.json'
    reexec_entry(root,a.output,Path(__file__),sys.argv[1:],harness,catalog)
    from robofrontier.suite.oracle import Trial
    from robofrontier.suite.dial_best_oracle import BestMonitor,solve
    from robofrontier.run_setup import write_json
    cat=json.loads((root/'inputs/catalog.json').read_text())
    case=next(c for c in cat['cases'] if c['id']==a.case and c['family']=='dial')
    # These files are evaluator inputs, never passed to the solver process.
    cfg=json.loads((root/'inputs/harness.json').read_text())
    sources={name:(root/name).read_bytes() for name in json.loads((root/'snapshot.json').read_text())['files']}
    scene=Path(case['directory']);scene=scene if scene.is_absolute() else root/scene
    trial=Trial(scene,a.output,frames=True,sources=sources,controller_cfg=cfg,
        monitor_factory=lambda w:BestMonitor(w,a.codebook.resolve()))
    protocol=dict(version='dial-best5-continuous-grasp-repair-v2',geometry_revision='R11',
        feedback='All nonzero detents within4deg for250ms, real red bilateral force-held contact and separate airborne blue support forL2/L3',
        solver='Separate -I -S process; N,K,current inputs and timestamped observed inputs/BEST only; no current score or private evaluator history',
        mechanical_warmup_in_episode=False,physics_timestep_s=float(trial.model.opt.timestep))
    protocol['failure_handling']='Retain world/solver for forward actuator repair; keep all failures and violations; no checkpoint splicing'
    write_json(trial.directory/'protocol.json',protocol)
    trial.abort_on_process_violation=True;stop=[False];began=time.monotonic()
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    trial.stop_requested=make_stop_check(trial,stop,a.output,began,a.wall_s,a.sim_s,a.actions)
    report=dict(case_id=a.case,passed=False,scope='Full blind BEST-only Oracle; no mechanical warmup or historical command prefix',protocol=protocol,
        budgets=dict(wall_s=a.wall_s or None,sim_s=a.sim_s or None,action_batches=a.actions or None,explicit_guesses=a.guesses or None),
        new_physics_integration=True,solver_answer_input=False,mechanical_oracle_geometry_privileged=True,
        new_evaluator_sha256=hashlib.sha256(b''.join((root/'robofrontier/suite'/name).read_bytes() for name in ('dial_best_worker.py','dial_feedback.py','dial_best_oracle.py'))).hexdigest())
    write_json(trial.directory/'run-limits.json',report['budgets'])
    try:
        trial.event('blind_protocol_start',protocol=protocol['version'])
        solve(trial,max_guesses=a.guesses or None)
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
