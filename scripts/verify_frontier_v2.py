"""V2 preflight by default. Full-task Oracle requires explicit --execute.

Designed for four external workers, one --family per worker; no nested pool.
"""
import argparse,json,time,signal
from pathlib import Path
import mujoco
from robofrontier.suite.v2 import ROOT,CATALOG
from robofrontier.suite.oracle import Trial,ReferenceStopped
from robofrontier.suite.evaluate import Monitor
from robofrontier.workstation import Workstation
from robofrontier.robot_control import PandaArm
from robofrontier.run_setup import write_json,load_harness


def preflight(case):
 p=ROOT/case['directory'];source=Workstation(p);task=json.loads((p/'task.json').read_text())
 from types import SimpleNamespace
 world=SimpleNamespace(model=source.model,data=source.data,task=task,arms={a:PandaArm(source.model,source.data,a) for a in task['arms']})
 secret=[1]*len(task['dials']) if task['family']=='dial' else None
 monitor=Monitor(world,secret=secret);monitor.physical_step();result=monitor.result();monitor.close()
 checked=json.loads((p/'engineering-check.json').read_text()) if (p/'engineering-check.json').exists() else {}
 return dict(case_id=case['id'],source_scene_sha256=source.manifest['scene_sha256'],schema=task['suite_version'],passed=bool(checked.get('passed') and not result.get('task_success')),initial_acceptance=result,
   source_valid=True,evaluator_constructed=True,static_layout_checked=checked.get('passed',False),oracle_started=False,scope='Input/evaluator preflight only; no physics time or actions; not a physical reference witness')


def run(case,seed=0,output_root=None,wall_budget_s=900.,sim_budget_s=3600.,action_budget=20000,max_goals=None,harness_path=None):
 # This entry point is called only after --execute. It intentionally never
 # changes the policy readiness gate or calls this a GPT run.
 parent=(Path(output_root) if output_root else ROOT/'runs/oracle/frontier-v2')/case['id']/f'seed-{seed}';parent.mkdir(parents=True,exist_ok=True);attempt=1
 while (parent/f'attempt-{attempt:03d}').exists():attempt+=1
 output=parent/f'attempt-{attempt:03d}'
 harness_path=Path(harness_path) if harness_path else ROOT/'configs/harness-frontier-v2.json'
 cfg=load_harness(harness_path)
 files=list((ROOT/'robofrontier').rglob('*.py'))+list((ROOT/'scripts').rglob('*.py'))+[Path(__file__),harness_path]
 files += [p for p in (ROOT/'inputs/catalog.json',) if p.exists()]
 trial=Trial(ROOT/case['directory'],output,seed=seed,record_hz=60,controller_cfg=cfg,sources={str(p.relative_to(ROOT)):p.read_bytes() for p in files})
 trial.live_preview=True
 trial.abort_on_process_violation=True
 stop_files=[output/'STOP',Path(output_root)/'STOP'] if output_root else [output/'STOP']
 budget_hit=lambda:time.monotonic()-trial.started>wall_budget_s or trial.time>sim_budget_s or len(trial.actions)>=action_budget
 trial.stop_requested=lambda:budget_hit() or any(p.exists() for p in stop_files)
 def controlled_stop(signum,frame):raise ReferenceStopped('External process stop requested')
 signal.signal(signal.SIGTERM,controlled_stop)
 report=dict(case_id=case['id'],suite_version=2,passed=False,seed=seed,scope='V2 privileged physical Oracle; not a model result',motion_format='robofrontier.motion.v1')
 try:
  trial.wait(.6);trial.snapshot('initial')
  import importlib
  solver=importlib.import_module('robofrontier.suite.'+case['family']+'_oracle_v2').solve
  partial=solver(trial,max_goals=max_goals) if case['family']=='mosaic' else solver(trial)
  if partial is not None:report['reference_result']=partial
  result=trial.monitor.result();report.update(evaluation=result,passed=bool(result.get('task_success')));trial.snapshot('final')
 except (Exception,KeyboardInterrupt) as error:
  report.update(error=f'{type(error).__name__}: {error}',evaluation=trial.monitor.result(),unresolved_robot_actions=[r for r,a in trial.arms.items() if a.job is not None])
  try:trial.snapshot('failure')
  except Exception as image_error:report['snapshot_error']=str(image_error)
  if budget_hit():report['budget_exhausted']=True
  if isinstance(error,(ReferenceStopped,KeyboardInterrupt)):report['interrupted']=True
 finally:
  report['budgets']=dict(wall_s=wall_budget_s,simulation_s=sim_budget_s,action_batches=action_budget)
  trial.close(report)
  partial_ok=bool(report.get('reference_result',{}).get('commissioning_only')) and not report.get('error')
  write_json(output/'status.json',dict(phase='passed' if report['passed'] else 'commissioning_complete' if partial_ok else 'interrupted' if report.get('interrupted') else 'failed',report=str(output/'report.json'),updated_at_unix_s=time.time()))
 print(json.dumps(dict(case_id=case['id'],passed=report['passed'],report=str(output/'report.json'))),flush=True)
 return report


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--family',choices=('transport','mosaic','relay','dial'));p.add_argument('--case',action='append');p.add_argument('--execute',action='store_true');p.add_argument('--seed',type=int,default=0)
 p.add_argument('--harness',type=Path);p.add_argument('--catalog',type=Path,default=CATALOG);p.add_argument('--output-root',type=Path);p.add_argument('--wall-budget-s',type=float,default=900);p.add_argument('--sim-budget-s',type=float,default=3600);p.add_argument('--action-budget',type=int,default=20000);p.add_argument('--max-goals',type=int);a=p.parse_args()
 data=json.loads(a.catalog.read_text())['cases'];known={c['id'] for c in data}
 if set(a.case or [])-known:p.error('Unknown case')
 selected=[c for c in data if (not a.family or c['family']==a.family) and (not a.case or c['id'] in a.case)]
 if a.execute:
  if not(a.family or a.case):p.error('Execution requires one family or explicit cases')
  from robofrontier.suite.source_snapshot import reexec_frozen
  reexec_frozen(ROOT,a)
  for case in selected:
   result=run(case,a.seed,a.output_root,a.wall_budget_s,a.sim_budget_s,a.action_budget,a.max_goals,harness_path=a.harness)
   if not result['passed']:break
 else:
  rows=[preflight(c) for c in selected];write_json(ROOT/'docs/generated/frontier-v2/oracle-preflight.json',dict(passed=all(r['passed'] for r in rows),oracle_started=False,cases=rows))
  print(json.dumps(dict(passed=all(r['passed'] for r in rows),cases=len(rows),oracle_started=False)))
  if not all(r['passed'] for r in rows):raise SystemExit(1)
