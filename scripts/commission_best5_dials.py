"""Bounded mechanical commissioning only; never a full code-solving witness."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time


def turn(m,entries,symbols):
    import numpy as np
    from scripts.verify_floating_dial_oracle import axis_rotation
    t=m.t;m.prepare_symbols(entries,symbols);recoveries={a['id']:0 for a in entries}
    for leg in range(100):
        m.wait_support(entries);commands={}
        for a in entries:
            name=a['id'];held=m.grips[name];robot=held['robot'];arm=t.arms[robot]
            engaged=robot in t.monitor.bilateral(m.red_geoms(a)) and arm.grasp_force is not None
            error=held['target_angle_rad']-float(t.data.joint(name+'_hinge').qpos[0])
            if not engaged:
                if recoveries[name]>=1:raise RuntimeError('Commissioning grasp lost '+name)
                recoveries[name]+=1;t.event('commissioning_recovery',dial=name,reason='Measured red grip lost')
                m.prepare_symbols([a],symbols);break
            if abs(error)<math.radians(1.4):continue
            jid=t.model.joint(name+'_hinge').id;anchor=t.data.xanchor[jid].copy()
            dr=axis_rotation(t.data.xaxis[jid],float(np.clip(error,-math.radians(6),math.radians(6))))
            point=anchor+dr@(t.data.site_xpos[arm.site]-anchor);rotation=dr@t.data.site_xmat[arm.site].reshape(3,3)
            q=t.screen_ik(robot,point,rotation)
            if q is None:
                # Alternate seed can separate local IK failure from an actual
                # need to change the current contact posture.
                q=t.screen_ik(robot,point,rotation,seed=t.home_q[robot])
            if q is None:
                if recoveries[name]>=1:raise RuntimeError('Commissioning next arc has no IK after bounded regrasp '+name)
                recoveries[name]+=1;previous=held['twist']
                t.event('commissioning_regrasp_admission',dial=name,reason='Both IK seeds rejected before submission; measured state retained',previous_twist_deg=previous)
                m.release([a]);m.grasp([a],symbols,avoid={name:previous});break
            if not m.arc_joint_clear(robot,t.data.qpos[arm.qids],q):raise RuntimeError('Commissioning sampled whole-arm conflict '+name)
            commands[robot]=t.pose_command(robot,point,rotation,.025,.25)
        else:
            if not commands:
                t.wait(.35);return
            m.execute_turn('commission-turn-'+str(leg),commands);t.wait(.10)
            continue
        continue
    raise RuntimeError('Commissioning turn budget exhausted')


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--dial',action='append');p.add_argument('--marks',default='1,2,3,4,5,1');p.add_argument('--wall-s',type=float,default=240)
    p.add_argument('--harness',type=Path);p.add_argument('--release',action='store_true');p.add_argument('--grasp-force',type=float,default=30.)
    args=p.parse_args();root=Path(__file__).resolve().parents[1]
    harness=args.harness or root/'runs/oracle/suite-v2-r10-closure-20261025/inputs/harness.json'
    catalog=root/'configs/suites/dial-best5-r11.json'
    from robofrontier.suite.source_snapshot import reexec_entry
    reexec_entry(root,args.output,Path(__file__),sys.argv[1:],harness,catalog)
    import numpy as np
    from robofrontier.suite.oracle import Trial
    from robofrontier.suite.dial_best_oracle import BestMonitor,BestManipulator
    from robofrontier.run_setup import write_json
    cat=json.loads((root/'inputs/catalog.json').read_text());case=next(c for c in cat['cases'] if c['id']==args.case)
    scene=root/case['directory'];puzzle=root/f"configs/evaluations/private/dial-best5-r11/{args.case}.json"
    cfg=json.loads((root/'inputs/harness.json').read_text())
    sources={name:(root/name).read_bytes() for name in json.loads((root/'snapshot.json').read_text())['files']}
    t=Trial(scene,args.output,frames=True,sources=sources,controller_cfg=cfg,monitor_factory=lambda w:BestMonitor(w,puzzle))
    stop=[False];signal.signal(signal.SIGTERM,lambda *_:stop.__setitem__(0,True));began=time.monotonic()
    t.stop_requested=lambda:stop[0] or time.monotonic()-began>args.wall_s or (args.output.parent/'STOP').exists()
    t.abort_on_process_violation=True
    report=dict(case_id=args.case,passed=False,commissioning_only=True,scope='Mechanical commissioning only; no solver or full puzzle attempt',marks=[],release_requested=args.release,
        full_task_attempt=False,solver_started=False,private_code_loaded_only_by_evaluator=True,budget_wall_s=args.wall_s,
        mechanical_geometry_privileged=True,grasp_force_n=args.grasp_force)
    try:
        if not cfg['allowed_grasp_force_n'][0]<=args.grasp_force<=cfg['allowed_grasp_force_n'][1]:raise ValueError('Force outside existing robot limits')
        m=BestManipulator(t);m.red_grasp_force_n=args.grasp_force
        entries=[a for a in m.dials if not args.dial or a['id'] in args.dial]
        if not entries or (args.dial and len(entries)!=len(set(args.dial))):raise ValueError('Unknown commissioning dial')
        report['devices']=[a['id'] for a in entries];m.lift();t.snapshot('supported')
        for mark in map(int,args.marks.split(',')):
            if not 1<=mark<=5:raise ValueError('Invalid mark')
            symbols=[mark]*len(m.dials);turn(m,entries,symbols)
            # A positive hold interval, not merely a reached setpoint.
            worst=np.zeros(len(entries));gaps=np.zeros(len(entries));maxgap=np.zeros(len(entries))
            for _ in range(50):
                t.wait(.02)
                for i,a in enumerate(entries):
                    theta=math.degrees(float(t.data.joint(a['id']+'_hinge').qpos[0]));worst[i]=max(worst[i],abs((theta-60*mark+180)%360-180))
                    ok=m.grips[a['id']]['robot'] in t.monitor.bilateral(m.red_geoms(a))
                    gaps[i]=0 if ok else gaps[i]+.02;maxgap[i]=max(maxgap[i],gaps[i])
            result=dict(symbol=mark,devices=[a['id'] for a in entries],worst_hold_error_deg=worst.tolist(),max_contact_gap_s=maxgap.tolist(),hold_sim_s=1.,
                        all_supported=all(t.monitor.dial_ready[a['index']] for a in entries),whole_input_ready=bool(t.monitor.game.ready))
            result['passed']=bool(max(worst)<=4 and max(maxgap)<=.02 and result['all_supported'] and not t.monitor.violations
                                  and (len(entries)<len(m.dials) or result['whole_input_ready']))
            report['marks'].append(result);t.event('mechanical_mark',**result);t.snapshot('mark-'+str(len(report['marks'])),cameras=('global','global_rear','board'))
            if not result['passed']:raise RuntimeError('Mechanical held-mark check failed '+str(mark))
        if args.release:
            m.release(entries);t.wait(4.)
            residual=[abs(math.degrees(float(t.data.joint(a['id']+'_hinge').qpos[0]))) for a in entries]
            released=[t.monitor.released(m.red_geoms(a)) for a in entries]
            report['release']=dict(residual_deg=residual,released=released,all_supported=all(t.monitor.dial_ready[a['index']] for a in entries))
            if max(residual)>4 or not all(released) or not report['release']['all_supported']:raise RuntimeError('Passive spring return not verified')
        report['passed']=True
    except Exception as error:report['error']=type(error).__name__+': '+str(error)
    finally:
        pending=[r for r,a in t.arms.items() if a.job is not None];cleanup=[]
        for r in pending:t.arms[r].request_stop(t.time,reason='commissioning_closed')
        t.stop_requested=None;t.abort_on_process_violation=False
        for _ in range(round(16/t.model.opt.timestep)):
            busy=[a for a in t.arms.values() if a.job is not None]
            if not busy:break
            try:
                t.step()
                for a in busy:
                    feedback=a.poll(t.time,15.)
                    if feedback:cleanup.append(feedback)
            except Exception:break
        report['cleanup_feedback']=cleanup;report['unresolved_robot_actions']=[r for r,a in t.arms.items() if a.job is not None]
        if report['unresolved_robot_actions']:report['passed']=False
        report['evaluation']=t.monitor.result();report['local_passed']=report['passed'];report['passed']=False
        try:t.snapshot('final')
        finally:t.close(report)
    print(json.dumps({k:report.get(k) for k in ('case_id','passed','local_passed','error','devices','marks','release')}),flush=True)
    return 0 if report['local_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
