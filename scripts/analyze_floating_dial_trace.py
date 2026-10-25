"""Private engineering audit of recorded floating-module attempts; no world or LLM starts."""
import argparse
from bisect import bisect_right
from collections import Counter
import json
from pathlib import Path
import sqlite3


def analyze(run):
    run=Path(run);db=sqlite3.connect(run/'private/state/runtime.sqlite')
    actions=[json.loads(v) for v, in db.execute('select value from actions')]
    rows=[json.loads(line) for line in (run/'private/outputs/spring-dial-process.jsonl').open()]
    samples=[r for r in rows if r['event']=='sample'];times=[r['sim_time_s'] for r in samples]
    def before(t):return samples[max(0,bisect_right(times,t)-1)]
    airborne=[]
    for i,a in enumerate('ABCD'):
        episodes=[];start=None
        for s in samples:
            t=s['sim_time_s'];ok=s['apparatus']['airborne'][i]
            if ok and start is None:start=t
            if not ok and start is not None:episodes.append([start,t]);start=None
        if start is not None:episodes.append([start,times[-1]])
        airborne.append(dict(module=a,maximum_clearance_m=max(s['apparatus']['clearance_m'][i] for s in samples),airborne_episodes=episodes,longest_observed_airborne_s=max((b-a for a,b in episodes),default=0)))
    attempts=[]
    for a in actions:
        p=a['parameters']
        if a['type']=='gripper' and p.get('mode')=='grasp' and p.get('width_m',0)>=.045:
            i=('ABCDEFGH'.index(a['arm']))//2;s=before(a['started_sim_s'])
            attempts.append(dict(action_id=a['action_id'],robot=a['arm'],module='ABCD'[i],started_sim_s=a['started_sim_s'],status=a['status'],prior_clearance_m=s['apparatus']['clearance_m'][i],prior_airborne=s['apparatus']['airborne'][i],sample_age_s=a['started_sim_s']-s['sim_time_s']))
    lift=[]
    for a in actions:
        if 'lift' not in a['action_id'].lower():continue
        i='ABCDEFGH'.index(a['arm'])//2;end=a['finished_sim_s'];selected=[s for s in samples if a['started_sim_s']<=s['sim_time_s']<=end+2]
        if selected:lift.append(dict(action_id=a['action_id'],robot=a['arm'],move_status=a['status'],finish_clearance_m=before(end)['apparatus']['clearance_m'][i],max_during_and_2s_after_m=max(s['apparatus']['clearance_m'][i] for s in selected),min_2s_after_m=min((s['apparatus']['clearance_m'][i] for s in selected if s['sim_time_s']>=end),default=None)))
    forces=Counter(a['parameters'].get('force_n') for a in actions if a['type']=='gripper' and a['parameters'].get('mode')=='grasp')
    result=dict(action_count=len(actions),statuses=dict(Counter(a['status'] for a in actions)),failures=dict(Counter(a['reason'] for a in actions if a['status']=='failed')),grasp_force_setpoints_n=dict(forces),valid_trials=max(s['feedback']['valid_trials'] for s in samples),airborne=airborne,knob_grasp_attempts=attempts,lifts=lift,sampling_scope='10Hz private process samples; bounds and event timing have up to one sample interval uncertainty. Width >=45mm identifies candidate knob grasps from the command; it is not contact proof.')
    out=run/'private/outputs/grasp-audit.json';out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--check-stable-first-lift',action='store_true');a=p.parse_args();r=analyze(a.run)
    if a.check_stable_first_lift:
        lift=r['lifts'][0]
        if lift['min_2s_after_m'] is None or lift['min_2s_after_m']<.02:
            print('RED: a completed robot lift did not establish sustained 20mm whole-module clearance.');raise SystemExit(1)
