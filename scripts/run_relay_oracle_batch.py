"""Committed, worktree-local Relay Oracle: one pilot, then bounded worlds.

Inputs, code, motion and outputs are independently frozen in an external run
directory. No physical state is restored and no GPT policy is launched here.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def worker(base,cid):
    from robofrontier.run_setup import write_json,load_harness
    from robofrontier.suite.oracle import Trial,ReferenceStopped
    from robofrontier.suite import relay_oracle_v2 as ref
    frozen=Path(os.environ['ROBOFRONTIER_ORACLE_FROZEN_ROOT'])
    manifest=json.loads((base/'manifest.json').read_text());case=next(c for c in manifest['cases'] if c['id']==cid)
    for name,checksum in case['input_hashes'].items():
        if hashlib.sha256((Path(case['frozen_scene'])/name).read_bytes()).hexdigest()!=checksum:raise RuntimeError('Frozen input changed: '+name)
    snapshot=json.loads((frozen/'snapshot.json').read_text())
    sources={name:(frozen/name).read_bytes() for name in snapshot['files']}
    if any(hashlib.sha256(payload).hexdigest()!=snapshot['files'][name] for name,payload in sources.items()):raise RuntimeError('Frozen executor changed')
    out=base/cid/'attempt-001';out.parent.mkdir(exist_ok=True)
    trial=Trial(case['frozen_scene'],out,controller_cfg=load_harness(frozen/'inputs/harness.json'),sources=sources,record_hz=60.)
    trial.live_preview=True;trial.abort_on_process_violation=True
    wall=manifest['pilot_wall_s'] if cid==manifest['pilot'] else manifest['wall_s']
    trial.stop_requested=lambda:time.monotonic()-trial.started>wall or trial.time>manifest['simulation_s'] or len(trial.actions)>manifest['action_batches'] or (base/'STOP').exists()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(ReferenceStopped('Supervisor stop')))
    ref.STAGE_SEARCH_WALL_S=manifest['planning_s']
    report=dict(case_id=cid,passed=False,commissioning_only=False,scope='Complete Relay actuator Oracle; privileged geometry; not a GPT policy result',
        git_commit=manifest['git_commit'],source_worktree=manifest['worktree'],protocol='whole-pair-relay-v1',single_side_handover_enabled=False,
        wall_budget_s=wall,phases=[],execution_mode='pure_program')
    def observe(phase,ident,team):
        row=dict(phase=phase,payload=ident,team=list(team),simulation_time_s=trial.time)
        report['phases'].append(row);trial.event('relay_batch_phase',detail=row)
        try:trial.snapshot(ident+'-'+phase,('global','global_rear'))
        except Exception as error:report.setdefault('image_warnings',[]).append(repr(error));trial.live_preview=False
    try:
        trial.wait(.6);trial.snapshot('initial')
        ref.solve(trial,terminal_observer=observe)
        trial.wait(.5)
        result=trial.monitor.result();pending=[r for r,a in trial.arms.items() if a.job is not None]
        report.update(evaluation=result,passed=bool(result.get('task_success') and not result.get('violations') and not pending),unresolved_robot_actions=pending)
        if not report['passed']:raise RuntimeError('Complete task success was not sustained after final settling')
        trial.snapshot('final')
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=type(error).__name__+': '+str(error),passed=False,evaluation=trial.monitor.result(),unresolved_robot_actions=[r for r,a in trial.arms.items() if a.job is not None])
        try:trial.snapshot('stopped')
        except Exception:pass
    finally:trial.close(report)
    from robofrontier.motion_recording import MotionReader
    from robofrontier.workstation import Workstation
    import numpy as np
    reader=MotionReader(out/'motion');original=Workstation(case['frozen_scene'],verify_assets=False);count=0
    for stamp,qpos in reader.frames():
        if count==0:assert stamp==0 and np.array_equal(qpos,np.asarray(original.initial_qpos,dtype='<f4'))
        count+=1
    write_json(out/'recording-verification.json',dict(frames=count,complete_chunks_verified=True,canonical_initial_state=True,physics_timestep_s=reader.meta['physics_timestep_s'],status=reader.meta['status']))
    return 0 if report['passed'] else 1


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--workers',type=int,choices=[1,2],default=2);parser.add_argument('--wall-s',type=float,default=1800.)
    parser.add_argument('--pilot-wall-s',type=float,default=900.);parser.add_argument('--planning-s',type=float,default=120.)
    parser.add_argument('--cases',nargs='*');parser.add_argument('--pilot-only',action='store_true')
    parser.add_argument('--worker');args=parser.parse_args();base=args.output_root.resolve()
    if args.worker:return worker(base,args.worker)
    from robofrontier.run_setup import write_json
    from robofrontier.motion_recording import freeze_source
    from robofrontier.suite.source_snapshot import freeze_executor
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('Commit the run source before freezing')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    cases=[c for c in json.loads((ROOT/'configs/suites/frontier-v2.json').read_text())['cases'] if c['family']=='relay' and (not args.cases or c['id'] in args.cases)]
    if not cases or set(args.cases or [])-{c['id'] for c in cases}:raise ValueError('Unknown/empty Relay case selection')
    cases.sort(key=lambda c:(c['arms'],c['level']));pilot=cases[0]['id']
    base.mkdir(parents=True,exist_ok=False);(base/'inputs').mkdir()
    harness=ROOT/'runs/oracle/relay-key-units-20261025-213449/inputs/harness.json'
    (base/'inputs/harness.json').write_bytes(harness.read_bytes())
    for c in cases:
        destination=base/'scenes'/c['id'];destination.parent.mkdir(exist_ok=True)
        freeze_source(ROOT/c['directory'],destination,cache=base/'asset-cache')
        c['frozen_scene']=str(destination)
        c['input_hashes']={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.rglob('*') if p.is_file()}
    write_json(base/'inputs/catalog.json',dict(cases=cases))
    manifest=dict(git_commit=commit,worktree=str(ROOT),cases=cases,pilot=pilot,workers=args.workers,wall_s=args.wall_s,pilot_wall_s=args.pilot_wall_s,
        planning_s=args.planning_s,simulation_s=6000.,action_batches=20000,recording_hz=60.,profile='pd1000-v1',source_root_units='metres',
        scope='Complete sequential-custody Oracle before multi-rod performance optimization; GPT not started',single_side_handover_enabled=False,created_unix_s=time.time())
    write_json(base/'manifest.json',manifest)
    frozen=freeze_executor(ROOT,base/'executor',base/'inputs/harness.json',base/'inputs/catalog.json')
    results=[]
    def run(cid):
        out=base/cid/'attempt-001';log=base/(cid+'.log');env=os.environ.copy()
        env.update(PYTHONPATH=str(frozen),ROBOFRONTIER_ORACLE_FROZEN_ROOT=str(frozen),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
        wall=args.pilot_wall_s if cid==pilot else args.wall_s;started=time.monotonic();signalled=None
        with log.open('x') as stream:
            process=subprocess.Popen([sys.executable,str(frozen/'scripts/run_relay_oracle_batch.py'),'--output-root',str(base),'--worker',cid],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            write_json(base/(cid+'-process.json'),dict(pid=process.pid,phase='running',output=str(out),git_commit=commit))
            while process.poll() is None:
                elapsed=time.monotonic()-started
                if ((base/'STOP').exists() or elapsed>wall+45) and signalled is None:os.kill(process.pid,signal.SIGTERM);signalled=time.monotonic()
                if signalled is not None and time.monotonic()-signalled>25:os.killpg(process.pid,signal.SIGKILL);process.wait();break
                time.sleep(.5)
        r=json.loads((out/'report.json').read_text()) if (out/'report.json').exists() else {}
        passed=bool(process.returncode==0 and r.get('passed') and r.get('evaluation',{}).get('task_success') and not r.get('unresolved_robot_actions'))
        row=dict(case_id=cid,passed=passed,returncode=process.returncode,error=r.get('error'),report=str(out/'report.json'),wall_s=time.monotonic()-started,pid=process.pid)
        write_json(base/(cid+'-process.json'),dict(row,phase='passed' if passed else 'failed'))
        print(json.dumps(row),flush=True);return row
    def status(active):write_json(base/'status.json',dict(active=active,git_commit=commit,results=results,planned=len(cases),passed=sum(r['passed'] for r in results)))
    status(True);results.append(run(pilot));status(True)
    if not results[0]['passed'] or args.pilot_only:status(False);return 0 if results[0]['passed'] else 1
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        pending={pool.submit(run,c['id']):c['id'] for c in cases[1:]}
        for future in concurrent.futures.as_completed(pending):
            try:results.append(future.result())
            except Exception as error:results.append(dict(case_id=pending[future],passed=False,error=repr(error)))
            status(True)
    status(False);return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
