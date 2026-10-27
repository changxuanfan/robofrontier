"""Engineering fault injection through the real blind solver and live owner.

Rejected actuator requests never execute. The owner must supply ordinary
repair plans through the same channel used by real reference failures.
This result is explicitly excluded from unassisted Oracle certification.
"""
import argparse
import json
from pathlib import Path
import signal
import sys
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wall-s',type=float,default=900)
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    from robofrontier.suite.source_snapshot import reexec_entry
    harness=root/'configs/harness-frontier-v2-servo2.json'
    catalog=root/'configs/suites/dial-best5-r11.json'
    reexec_entry(root,a.output,Path(__file__),sys.argv[1:],harness,catalog)
    from robofrontier.suite.oracle import Trial
    from robofrontier.suite.dial_best_oracle import BestMonitor,BestManipulator,solve
    from scripts.run_blind_dial_oracle import close_trial
    case='dial-best5-n04-d1'
    scene=root/'configs/environments/spring_dial/frontier-v2-r11-dial-best5-n04-d1'
    sources={name:(root/name).read_bytes() for name in json.loads((root/'snapshot.json').read_text())['files']}
    trial=Trial(scene,a.output,frames=True,sources=sources,
        controller_cfg=json.loads((root/'inputs/harness.json').read_text()),
        monitor_factory=lambda t:BestMonitor(t,root/f'configs/evaluations/private/dial-best5-r11/{case}.json'))
    stop=[False];began=time.monotonic()
    signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True))
    trial.stop_requested=lambda:stop[0] or time.monotonic()-began>a.wall_s
    trial.abort_on_process_violation=True
    original=BestManipulator.set_symbols;injected=set();solver_pids=set()
    def with_rejection(self,symbols,finish_when_solved=False):
        solver_pids.add(self.decoder.channel.process.pid)
        if len(injected)<2 and tuple(symbols) not in injected:
            injected.add(tuple(symbols))
            trial.event('engineering_fault_injection',index=len(injected),
                fault='Out-of-range empty/held hand aperture rejected before execution',
                solver_pid=self.decoder.channel.process.pid,candidates=len(self.decoder.candidates))
            trial.execute('engineering-rejected-aperture',{'A':('gripper',dict(mode='move',width_m=.2,speed_m_s=.05))})
            raise AssertionError('Invalid aperture was unexpectedly admitted')
        return original(self,symbols,finish_when_solved)
    BestManipulator.set_symbols=with_rejection
    report=dict(case_id=case,passed=False,full_task_attempt=False,engineering_only=True,
        scope='Two declared actuator-admission faults during a real blind solve; externally supplied repair plans',
        solver_answer_input=False,new_physics_integration=True)
    try:
        solve(trial)
        report['passed']=bool(trial.monitor.result()['task_success'])
    except Exception as error:report['error']=type(error).__name__+': '+str(error)
    finally:
        report['injected_faults']=len(injected);report['solver_processes_observed']=sorted(solver_pids)
        close_trial(trial,report)
    print(json.dumps({k:report.get(k) for k in ('passed','error','goal_reached','injected_faults','solver_processes_observed','assisted','unresolved_robot_actions')}),flush=True)
    return 0 if report['passed'] and len(injected)==2 and len(solver_pids)==1 else 1


if __name__=='__main__':raise SystemExit(main())
