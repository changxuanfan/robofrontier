"""One continuous committed Relay Oracle with a live, typed recovery boundary."""
import argparse,hashlib,json,os
from pathlib import Path
import signal,subprocess,sys,time

ROOT=Path(__file__).resolve().parents[1]

class CommissionComplete(Exception):pass

def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scene',type=Path);p.add_argument('--harness',type=Path);p.add_argument('--wall-s',type=float,default=3600.);p.add_argument('--frozen',action='store_true')
    p.add_argument('--flexible',action='store_true');p.add_argument('--pipeline',action='store_true');p.add_argument('--commission-transfers',type=int,default=0)
    a=p.parse_args();out=a.output.resolve()
    if a.commission_transfers<0:raise ValueError('Commission transfer count must be nonnegative')
    if a.pipeline and a.commission_transfers:raise ValueError('Pipeline and bounded single-payload commissioning are separate runs')
    from robofrontier.run_setup import write_json,load_harness
    if not a.frozen:
        from robofrontier.suite.v2 import cases,directory
        from robofrontier.suite.source_snapshot import freeze_executor
        if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('Commit the run source first')
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        case=next(c for c in cases() if c['family']=='relay' and c['id']==a.case)
        scene=a.scene.resolve() if a.scene else directory(case)
        harness=a.harness.resolve() if a.harness else ROOT/'runs/oracle/relay-key-units-20261025-213449/inputs/harness.json'
        parent=out.parent;parent.mkdir(parents=True,exist_ok=True);setup=parent/(out.name+'-source');setup.mkdir(exist_ok=False)
        (setup/'harness.json').write_bytes(harness.read_bytes());write_json(setup/'catalog.json',dict(cases=[case]))
        frozen=freeze_executor(ROOT,setup/'executor',setup/'harness.json',setup/'catalog.json')
        manifest=dict(git_commit=commit,worktree=str(ROOT),scene=str(scene),case_id=a.case,wall_budget_s=a.wall_s,initialization='Fresh canonical initial state; real actuators execute all prefixes again',runtime_recovery='Typed privileged assisted; no physical rollback/state injection',flexible_handover=a.flexible,commission_transfers=a.commission_transfers,pipeline=a.pipeline)
        write_json(setup/'run.json',manifest)
        env=os.environ.copy();env.update(PYTHONPATH=str(frozen),ROBOFRONTIER_ORACLE_FROZEN_ROOT=str(frozen),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        command=[sys.executable,str(frozen/'scripts/run_relay_live_oracle.py'),'--case',a.case,'--output',str(out),'--scene',str(scene),'--harness',str(frozen/'inputs/harness.json'),'--wall-s',str(a.wall_s),'--frozen','--commission-transfers',str(a.commission_transfers)]
        if a.flexible:command.append('--flexible')
        if a.pipeline:command.append('--pipeline')
        os.execve(sys.executable,command,env)
    from robofrontier.suite.oracle import Trial,ReferenceStopped
    from robofrontier.suite import relay_oracle_v2 as ref
    from robofrontier.suite.relay_live_recovery import RelayLiveRecovery
    from robofrontier.suite import relay_pipeline
    frozen=Path(os.environ['ROBOFRONTIER_ORACLE_FROZEN_ROOT']);snap=json.loads((frozen/'snapshot.json').read_text());sources={n:(frozen/n).read_bytes() for n in snap['files']}
    assert all(hashlib.sha256(payload).hexdigest()==snap['files'][n] for n,payload in sources.items())
    manifest=json.loads((frozen.parent/'run.json').read_text())
    t=Trial(a.scene,out,controller_cfg=load_harness(a.harness),sources=sources,record_hz=60.)
    relay_pipeline.install_motion_metrics(t)
    t.live_preview=True;t.abort_on_process_violation=True
    t.stop_requested=lambda:time.monotonic()-t.started>a.wall_s or (out/'STOP').exists() or (out.parent/'STOP').exists()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(ReferenceStopped('Explicit process stop')))
    ref.STAGE_SEARCH_WALL_S=120.
    report=dict(case_id=a.case,passed=False,commissioning_only=bool(a.commission_transfers),git_commit=manifest['git_commit'],source_worktree=manifest['worktree'],scope='Bounded physical handover commissioning; not full task success' if a.commission_transfers else 'One complete canonical episode, including any retained forward recovery history',execution_mode='pure_program',phases=[],handovers=[],handover_protocol='qualified-members-v2' if a.flexible else 'whole-pair-v1')
    recovery=RelayLiveRecovery(t,report)
    def phase(name,ident,team):
        row=dict(phase=name,payload=ident,team=list(team),simulation_time_s=t.time);report['phases'].append(row);t.event('relay_live_phase',detail=row)
        t.snapshot(ident+'-'+name,('global','global_rear'))
    def handover(ident,old,new,index):
        row=dict(payload=ident,senders=old,receivers=new,retained=sorted(set(old)&set(new)),stage=index,sim_time_s=t.time)
        report['handovers'].append(row);t.event('relay_handover_commission',detail=row)
        if a.commission_transfers and len(report['handovers'])>=a.commission_transfers:raise CommissionComplete()
    try:
        t.wait(.6)
        while not t.monitor.result().get('task_success'):
            pending=[ident for ident,s in t.monitor.result()['payloads'].items() if not s['completed']]
            try:
                if a.pipeline:relay_pipeline.solve(t,flexible=a.flexible,terminal_observer=phase,handover_observer=handover)
                else:ref.solve(t,payload_ids=pending[:1] if a.commission_transfers else pending,terminal_observer=phase,allow_partial_handover=a.flexible,handover_observer=handover)
            except CommissionComplete:
                t.wait(.5);report['evaluation']=t.monitor.result();report['unresolved_robot_actions']=[r for r,v in t.arms.items() if v.job is not None]
                report['unit_passed']=bool(not t.monitor.violations and not report['unresolved_robot_actions'] and any(row['retained'] for row in report['handovers']))
                break
            except Exception as error:
                if t.monitor.violations:raise
                outcome=recovery.wait_for_repair(error)
                if outcome=='complete':break
        t.wait(.5);report['evaluation']=t.monitor.result();report['unresolved_robot_actions']=[r for r,v in t.arms.items() if v.job is not None]
        report['passed']=bool(report['evaluation']['task_success'] and not report['unresolved_robot_actions'])
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=repr(error),evaluation=t.monitor.result(),unresolved_robot_actions=[r for r,v in t.arms.items() if v.job is not None])
    finally:
        report['payload_motion']=t.relay_motion_metrics
        report['scheduling']='rolling-payload-frontier' if a.pipeline else 'sequential-payloads'
        t.close(report)
        if recovery.assisted:
            report.update(llm_used=True,execution_mode='privileged_assisted');write_json(out/'report.json',report)
    return 0 if report['passed'] or report.get('unit_passed') else 1

if __name__=='__main__':raise SystemExit(main())
