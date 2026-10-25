"""Frozen Mosaic Oracle with bounded live review; no episode reset or hot code.

--max-goals and --review-after are declared commissioning only. Formal runs
never inject a fault. Recovery requests are private engineering input, not a
policy API, and external decisions are recorded as privileged assistance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from robofrontier.run_setup import load_harness,write_json
from robofrontier.suite.v2 import ROOT
from robofrontier.suite.oracle import Trial,ReferenceStopped
from robofrontier.suite.mosaic_run import MosaicReview,run_reference,require_known_idle
from robofrontier.suite.source_snapshot import reexec_entry,MARKER


def run(args):
    overall_started=time.monotonic()
    catalog=json.loads(args.catalog.read_text())
    case=next(c for c in catalog['cases'] if c['id']==args.case)
    if case['family']!='mosaic':raise ValueError('Mosaic cases only')
    concurrency=None
    if args.concurrency_source:
        from robofrontier.suite.mosaic_concurrency import build_plan
        concurrency=build_plan(args.concurrency_source)
        if concurrency['case_id']!=case['id']:
            from robofrontier.suite.mosaic_equivalence import check
            from robofrontier.suite.provenance import bundle_manifest
            equivalent=check(args.concurrency_source,ROOT/case['directory'])
            if not equivalent['equivalent_for_command_candidate']:
                raise ValueError('Cross-case concurrency requires exact L2/L3 physical equivalence')
            concurrency['source_case_id']=concurrency['case_id']
            concurrency['case_id']=case['id']
            concurrency['physical_equivalence']=equivalent
            concurrency['target_scene_bundle_sha256']=bundle_manifest(ROOT/case['directory'])['sha256']
    replay=None
    if args.equivalent_replay_source:
        from robofrontier.suite.mosaic_replay import prepare
        replay=prepare(args.equivalent_replay_source,ROOT/case['directory'])
    sources={str(p.relative_to(ROOT)):p.read_bytes()
        for folder in ('robofrontier','scripts') for p in (ROOT/folder).rglob('*.py')}
    sources.update({'inputs/harness.json':args.harness.read_bytes(),'inputs/catalog.json':args.catalog.read_bytes()})
    if replay is not None:
        sources.update({'private-reference/l2-actions.json':(args.equivalent_replay_source/'actions.json').read_bytes(),
            'private-reference/l2-report.json':(args.equivalent_replay_source/'report.json').read_bytes()})
    trial=Trial(ROOT/case['directory'],args.output,seed=0,record_hz=60,
        controller_cfg=load_harness(args.harness),sources=sources)
    trial.live_preview=True;trial.abort_on_process_violation=True
    process_identity=dict(pid=os.getpid(),case_id=case['id'],output=str(trial.directory),
        frozen_root=str(ROOT),git_commit=os.environ.get('ROBOFRONTIER_MOSAIC_COMMIT'),started_unix_s=time.time())
    write_json(trial.directory/'process.json',dict(process_identity,phase='running'))
    budget_hit=lambda:(time.monotonic()-overall_started>=args.wall_s or trial.time>=args.sim_s or
        len(trial.actions)>=args.action_batches)
    trial.stop_requested=lambda:budget_hit() or (trial.directory/'STOP').exists()
    def stop(signum,frame):raise ReferenceStopped('External owner stop requested')
    signal.signal(signal.SIGTERM,stop)
    report=dict(case_id=case['id'],passed=False,scope='R13 privileged physical Mosaic Oracle',
        commissioning_only=args.max_goals is not None,declared_review_after_goals=args.review_after,declared_goal_ids=args.goal_id,
        git_commit=os.environ.get('ROBOFRONTIER_MOSAIC_COMMIT'),
        budgets=dict(wall_s=args.wall_s,simulation_s=args.sim_s,action_batches=args.action_batches,
            review_wall_s=args.review_wall_s,max_reviews=args.max_reviews),
        motion_profile=dict(loaded_joint_speed_rad_s=args.loaded_speed))
    def on_enter(review):
        trial.snapshot(f'review-{review.epoch:03d}')
        if args.auto_resume_reviews and str(review.history[-1]['error']).endswith('Declared commissioning review boundary'):
            review.submit(dict(id=f'commissioning-resume-{review.epoch}',world_id=review.world_id,
                review_id=review.epoch,operation='resume'),assisted=False)
            # Use the same durable request route; submit is idempotent.
            write_json(review.directory/'requests'/f'commissioning-{review.epoch}.json',
                review.requests[f'commissioning-resume-{review.epoch}']['request'])
    review=MosaicReview(trial,review_wall_s=args.review_wall_s,on_enter=on_enter)
    pending=set(args.review_after)
    def after_goal(world):
        review._status('running')
        count=len(world.monitor.result().get('assignments',{}))
        if count in pending:
            pending.remove(count)
            raise RuntimeError('Declared commissioning review boundary')
    trial.mosaic_after_goal=after_goal
    trial.mosaic_motion_profile=dict(report['motion_profile'])
    solve_kwargs={}
    if concurrency is not None:
        write_json(trial.directory/'concurrency-plan.json',concurrency)
        report['concurrency_planning_wall_s']=concurrency['planning_wall_s']
        report['concurrency_source']=str(args.concurrency_source.resolve())
        report['concurrency_execution']=args.concurrency_execution
        def solve_concurrent(world,**kwargs):
            if kwargs['resume']:
                from robofrontier.suite.mosaic_oracle_v2 import solve
                world.event('mosaic_concurrency_replanned_from_actual_state',scope='Owner reviewed physical repair; remaining matching uses actual state, no failed command stream replay')
                return solve(world,**kwargs)
            if args.concurrency_execution=='async':
                from robofrontier.suite.mosaic_async import execute_plan
            else:
                from robofrontier.suite.mosaic_concurrency import execute_plan
            return execute_plan(world,concurrency)
        solve_kwargs['solver']=solve_concurrent
    elif replay is not None:
        write_json(trial.directory/'physical-equivalence.json',replay['equivalence'])
        report['equivalent_replay']=dict(source=replay['source'],actions_sha256=replay['actions_sha256'],scope=replay['scope'])
        def solve_replay(world,**kwargs):
            if kwargs['resume']:
                from robofrontier.suite.mosaic_oracle_v2 import solve
                return solve(world,**kwargs)
            from robofrontier.suite.mosaic_replay import execute
            return execute(world,replay)
        solve_kwargs['solver']=solve_replay
    try:
        trial.wait(.6);trial.snapshot('initial');review._status('running')
        result=run_reference(trial,review,max_goals=args.max_goals,motion_profile=report['motion_profile'],max_reviews=args.max_reviews,goal_ids=args.goal_id,**solve_kwargs)
        trial.wait(1.2);require_known_idle(trial)
        evaluation=trial.monitor.result()
        report.update(reference_result=result,evaluation=evaluation,
            passed=bool(args.max_goals is None and evaluation.get('task_success')),
            commissioning_passed=bool(args.max_goals is not None and evaluation.get('matched_goals',0)>=args.max_goals and not trial.monitor.violations))
        trial.snapshot('final')
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=f'{type(error).__name__}: {error}',evaluation=trial.monitor.result(),
            interrupted=isinstance(error,(ReferenceStopped,KeyboardInterrupt)),budget_exhausted=budget_hit())
        try:trial.snapshot('failure')
        except Exception as snapshot_error:report['snapshot_error']=str(snapshot_error)
    finally:
        unknown=[r for r,a in trial.arms.items() if a.job is not None]
        unknown+= [r for action in trial.actions for r,result in action.get('results',{}).items() if result.get('status')=='uncertain']
        report['unresolved_robot_actions']=sorted(set(unknown))
        if unknown or trial.monitor.violations:
            report['passed']=False;report['commissioning_passed']=False
        metadata=dict(llm_used=review.assisted,assisted=review.assisted,
            oracle_mode='privileged_assisted' if review.assisted else 'pure_program',
            recovery=dict(world_id=review.world_id,reviews=review.history,
                requests=list(review.requests.values()),same_world_forward_only=True,physical_restart_supported=False),
            recovered_motion_failures=getattr(trial,'mosaic_recovered_motion_failures',[]))
        report.update(metadata);trial.close(report);report.update(metadata)
        report['total_wall_s_including_planning']=time.monotonic()-overall_started
        write_json(trial.directory/'report.json',report)
        phase='passed' if report['passed'] else 'commissioning_complete' if report.get('commissioning_passed') else 'interrupted' if report.get('interrupted') else 'failed'
        review._status(phase)
        write_json(trial.directory/'process.json',dict(process_identity,phase=phase,finished_unix_s=time.time(),report=str(trial.directory/'report.json')))
    print(json.dumps(dict(case_id=case['id'],passed=report['passed'],commissioning_passed=report.get('commissioning_passed'),
        goals=report['evaluation'].get('matched_goals'),error=report.get('error'),report=str(trial.directory/'report.json'))),flush=True)
    return 0 if report['passed'] or report.get('commissioning_passed') else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',default='mosaic-a04-6x12-d1');p.add_argument('--output',required=True,type=Path)
    p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-mosaic-r13.json')
    p.add_argument('--harness',type=Path,default=ROOT/'configs/harness-frontier-v2-servo2.json')
    p.add_argument('--wall-s',type=float,default=1800.);p.add_argument('--sim-s',type=float,default=8000.)
    p.add_argument('--action-batches',type=int,default=20000);p.add_argument('--review-wall-s',type=float,default=300.)
    p.add_argument('--max-reviews',type=int,default=20);p.add_argument('--loaded-speed',type=float,default=.20)
    p.add_argument('--max-goals',type=int);p.add_argument('--review-after',nargs='*',type=int,default=[])
    p.add_argument('--goal-id',action='append')
    p.add_argument('--concurrency-source',type=Path)
    p.add_argument('--concurrency-execution',choices=('barrier','async'),default='barrier')
    p.add_argument('--equivalent-replay-source',type=Path)
    p.add_argument('--auto-resume-reviews',action='store_true');a=p.parse_args()
    if a.review_after and (a.max_goals is None or any(i<1 or i>=a.max_goals for i in a.review_after)):
        p.error('Injected reviews require --max-goals and boundaries below that limit')
    if a.auto_resume_reviews and not a.review_after:p.error('Auto resume is for declared commissioning boundaries only')
    if a.max_goals is not None and a.max_goals<1:p.error('Positive commissioning goal count required')
    if a.goal_id and (a.max_goals is None or a.max_goals>len(set(a.goal_id))):
        p.error('Explicit cells require a compatible --max-goals commissioning limit')
    if a.concurrency_source and (a.goal_id or a.max_goals or a.review_after):
        p.error('Full source-bound concurrency cannot be mixed with commissioning selectors')
    if a.equivalent_replay_source and (a.concurrency_source or a.goal_id or a.max_goals or a.review_after):
        p.error('Equivalent replay is a separate full canonical execution mode')
    if any(not math.isfinite(v) or v<=0 for v in (a.wall_s,a.sim_s,a.review_wall_s,a.action_batches,a.max_reviews,a.loaded_speed)):
        p.error('Positive finite budgets and speed required')
    a.output=a.output.resolve()
    if os.environ.get(MARKER)!=str(ROOT):
        if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
            p.error('Commit and freeze the complete executor before physical execution')
        os.environ['ROBOFRONTIER_MOSAIC_COMMIT']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        reexec_entry(ROOT,a.output,Path(__file__),sys.argv[1:],a.harness.resolve(),a.catalog.resolve(),cli_config=True)
    return run(a)


if __name__=='__main__':raise SystemExit(main())
