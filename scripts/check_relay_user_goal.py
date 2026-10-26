"""Read-only acceptance for the user's final-in-Goal clarification.

Existing physical episodes continue unchanged. Legacy native reports/counters
are evidence and are never overwritten or relabelled as native passes.
No integration, state injection, velocity threshold or static-duration gate.
"""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scripts.diagnose_relay_descent import diagnostic_world
from robofrontier.run_setup import write_json
from robofrontier.suite.scenes import rz

RULE='relay-user-final-in-goal-20261026'
BOUNDARY_TOLERANCE_M=.005

def check_case(run):
    run=Path(run);t,o=diagnostic_world(run);relay=t.monitor.relay_v2
    stream=json.loads((run/'motion/stream.json').read_text())
    pose_source='Live native observation and latest complete recorded chunk'
    if stream.get('status')!='recording' and (run/'report.json').exists():
        # After the owned episode closes, judge its exact final recorded pose.
        # Do not replace it with an older recovery-boundary body observation.
        legacy=json.loads((run/'report.json').read_text())
        with np.load(run/'event-states.npz',allow_pickle=False) as events:
            t.data.qpos[:]=events['qpos'][-1]
        t.data.time=legacy['evaluation']['sim_time_s'];mujoco.mj_forward(t.model,t.data)
        pose_source='Exact final trial-end event state; original closed stream/task status retained'
    center=np.array(t.task['route'][-1]['center_m']);rotation=rz(t.task['route'][-1]['nominal_yaw_rad']);half=np.array(t.task['route'][-1]['half_size_m']);rows={}
    # A real final landing may be directly on the table or on another cargo.
    # It does not require every independent component to contact simultaneously.
    accepted=set()
    bounds={}
    for ident,state in relay.states.items():
        lo=np.full(2,np.inf);hi=np.full(2,-np.inf)
        for gid in state['all_geoms']:
            frame=t.data.geom_xmat[gid].reshape(3,3);aabb=t.model.geom_aabb[gid]
            p=rotation.T@(t.data.geom_xpos[gid]+frame@aabb[:3]-center);extent=(abs(rotation.T@frame)@aabb[3:])[:2]
            lo=np.minimum(lo,p[:2]-extent);hi=np.maximum(hi,p[:2]+extent)
        bounds[ident]=(lo,hi)
        if np.all(lo>=-half-BOUNDARY_TOLERANCE_M) and np.all(hi<=half+BOUNDARY_TOLERANCE_M):accepted.add(ident)
    grounded=relay.supported_payloads(goal_only=True,require_all=False,inside_ids=accepted)
    for ident,state in relay.states.items():
        source=o['payloads'][ident];lo,hi=bounds[ident]
        inside=ident in accepted
        released=not source['actual_support'] and t.monitor.released(state['all_geoms'])
        landed=ident in grounded or source['state']['goal_table_support']
        transfers=source['state']['handovers'];handover=any(set(v['receivers'])-set(v['senders']) for v in transfers)
        required=bool(t.task['completion'].get('required_airborne_handover'))
        done=inside and released and landed and (handover or not required)
        rows[ident]=dict(completed=bool(done),inside_goal=bool(inside),released=bool(released),actual_landing_contact=bool(landed),qualified_airborne_handover=bool(handover),actual_support=source['actual_support'],goal_local_min_m=lo.tolist(),goal_local_max_m=hi.tolist(),goal_half_m=half.tolist(),outside_distance_m=np.maximum(0,np.maximum(-half-lo,hi-half)).tolist(),legacy_native_completed=source['state']['completed'],legacy_drop_marks=source['state']['drops'])
    passed=not o['evaluation']['violations'] and not o['active_jobs'] and all(v['completed'] for v in rows.values())
    chunk=stream['chunks'][-1]
    return dict(rule_revision=RULE,boundary_tolerance_m=BOUNDARY_TOLERANCE_M,pose_source=pose_source,scope='User-authorized final placement check; no speed/static-duration requirement; original physics process/history unchanged',passed=bool(passed),completed_payloads=sum(v['completed'] for v in rows.values()),total_payloads=len(rows),payloads=rows,violations=o['evaluation']['violations'],world_id=o['world_id'],sim_time_s=float(t.data.time),motion_chunk=chunk,observation_sha256=hashlib.sha256((run/'recovery/observation.json').read_bytes()).hexdigest(),original_native_task_success=o['evaluation']['task_success'],original_task_contract_sha256=hashlib.sha256((run/'task.json').read_bytes()).hexdigest(),checked_unix_s=time.time())

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();status=json.loads((a.root/'batch-status.json').read_text());retained=[r for r in status['results'] if r.get('passed') and r['case_id'].endswith('d1')];rows=[]
    for run in sorted(a.root.glob('relay-*/attempt-001')):
        try:
            report=check_case(run);write_json(run/'user-goal-report.json',report);rows.append(dict(case_id=run.parent.name,run=str(run.resolve()),**{k:report[k] for k in ['passed','completed_payloads','total_payloads']}));print(run.parent.name,report['completed_payloads'],'/',report['total_payloads'],'passed',report['passed'],flush=True)
        except (RuntimeError,ValueError) as error:rows.append(dict(case_id=run.parent.name,passed=False,review_pending=str(error)));print(run.parent.name,'REVIEW_PENDING',str(error),flush=True)
    result=dict(rule_revision=RULE,completed_cases=len(retained)+sum(r['passed'] for r in rows),total_cases=9,unchanged_l1=retained,cases=rows,legacy_native_passed=status['passed'],note='Original native status remains unchanged; updated user completion is reported separately.')
    write_json(a.root/'user-goal-batch-status.json',result)

if __name__=='__main__':main()
