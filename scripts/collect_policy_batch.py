"""Read closed policy evidence and export completed recordings; no inference/motion."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from robofrontier.motion_recording import MotionReader
from robofrontier.run_setup import write_json
from robofrontier.runtime_store import RuntimeStore


def collect(base,output):
    base=Path(base).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    batch=json.loads((base/'batch.json').read_text());cases=[]
    for row in batch['cases']:
        run=Path(row['run_dir']);s=RuntimeStore(run/'private/state/runtime.sqlite')
        life=s.get('lifecycle');agents=s.agents();actions=s.actions();start=s.get('start_at');sim_start=s.get('start_sim_time');events=s.events();s.close()
        verification={}
        for actor,a in agents.items():
            tid=a.get('thread_id')
            if not tid or a['kind']=='manager':continue
            files=list((Path.home()/'.codex/sessions').glob('**/*'+tid+'*.jsonl'));turns=[];parent=None
            if len(files)==1:
                records=[];lines=files[0].read_text().splitlines()
                for i,line in enumerate(lines):
                    try:records.append(json.loads(line))
                    except json.JSONDecodeError:
                        if i!=len(lines)-1:raise
                meta=next(r['payload'] for r in records if r['type']=='session_meta')
                cutoff=meta.get('forked_from_ordinal_exclusive',0) or 0;parent=meta.get('forked_from_id')
                for record in records:
                    if record['type']=='turn_context' and record.get('ordinal',0)>=cutoff:
                        p=record['payload'];turns.append({k:p.get(k) for k in ('turn_id','model','effort','approval_policy','sandbox_policy')})
            verified=bool(turns) and all(t['model']=='gpt-6.1-sol' and t['effort']=='high' and t['sandbox_policy'].get('type')=='danger-full-access' for t in turns)
            verification[actor]=dict(thread_id=tid,own_turns=turns,actual_parent=parent,
                fresh_or_correct_parent=parent==(row['thread_id'] if a['kind']=='delegate' else None),verified=verified)
        if verification:write_json(run/'private/state/model-verification.json',verification)
        item={k:row[k] for k in ('case_id','family','level','arms','thread_id')};item.update(lifecycle=life,actions=len(actions),delegates=sum(x['kind']=='delegate' for x in agents.values()),model_verification=verification)
        lock=(run/'private/state/world.lock').open('a+');closed=False
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);closed=True
        except BlockingIOError:pass
        finally:lock.close()
        path=run/'private/outputs/evaluation.json'
        if not path.exists() and (run/'private/outputs/interruption-evaluation.json').exists():path=run/'private/outputs/interruption-evaluation.json'
        if closed and path.exists() and life in ('ended','interrupted'):
            runtime_evaluation=json.loads(path.read_text());evaluation=runtime_evaluation.get('evaluation',runtime_evaluation)
            cutoff=run/'private/outputs/budget-cutoff.json'
            if cutoff.exists():
                evaluation=json.loads(cutoff.read_text())['evaluation']
                item['cutoff_evaluation_ref']=str(cutoff)
            item.update(task_success=bool(runtime_evaluation.get('task_success',False)),
                budget_exhausted=bool(runtime_evaluation.get('budget_exhausted') or cutoff.exists()),
                matched_goals=evaluation.get('matched_goals'),total_goals=evaluation.get('total_goals'),
                completed_payloads=evaluation.get('completed_payloads'),total_payloads=evaluation.get('total_payloads'),
                valid_trials=evaluation.get('valid_trials'),process_compliant=evaluation.get('process_compliant'),
                failed_actions=sum(a['status']=='failed' for a in actions.values()),unknown_actions=[k for k,a in actions.items() if a['status'] in ('pending','running','uncertain')],
                simulation_elapsed_s=evaluation.get('sim_time_s',0)-(sim_start or 0),evaluation_ref=str(path))
            end=next((e['created_at'] for e in events if e['kind'] in ('POLICY_WALL_LIMIT','TEAM_FINISH_PROPOSED')),None)
            item['task_wall_s']=None if start is None or end is None else end-start
            video=run/'private/outputs/global-realtime.mp4';check=run/'private/outputs/full-video-decode.json'
            if video.exists():
                if not check.exists():
                    result=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(video),'-f','null','-'],capture_output=True,timeout=180)
                    write_json(check,dict(passed=result.returncode==0,bytes=video.stat().st_size,stderr=result.stderr.decode(errors='replace')[-1000:],scope='Complete video decoded; separate from real-time dropped-frame counts'))
                item['video_decode']=json.loads(check.read_text());target=output/(row['case_id']+'-global.mp4')
                if not target.exists():
                    try:os.link(video,target)
                    except OSError:shutil.copy2(video,target)
                item['video']=str(target)
            stream=run/'private/outputs/motion/stream.json'
            if stream.exists():
                reader=MotionReader(stream.parent);count=0
                for stamp,state in reader.frames():count+=1
                item['motion']=dict(frames=count,status=reader.meta['status'],sample_hz=reader.meta['sample_hz'],recording=str(stream.parent),index_matches=count==reader.meta['frames'],max_sample_gap_s=reader.meta['max_sample_gap_s'])
        cases.append(item)
    result=dict(schema='robofrontier.bounded-policy-results.v1',updated_unix_s=time.time(),
        scope='GPT policy, not Oracle; independent run contexts;1200 wall seconds after START; cutoff evaluation excludes later braking gains',cases=cases)
    write_json(output/'results.json',result)
    print(json.dumps(dict(closed_with_evaluation=sum('evaluation_ref' in r for r in cases),passed=sum(r.get('task_success',False) for r in cases),total=len(cases),output=str(output))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();collect(a.batch,a.output)
