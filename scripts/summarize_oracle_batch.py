"""Read-only summary of complete Oracle results, pilots and ongoing attempts."""
import argparse
import json
from pathlib import Path
import time


def summarize(root):
 root=Path(root);batch=json.loads((root/'batch.json').read_text())
 catalog=json.loads(Path(batch['catalog']).read_text());cases=[]
 for case in catalog['cases']:
  folder=root/case['family']/case['id'];attempts=sorted(folder.glob('seed-*/attempt-*'))
  records=[]
  for attempt in attempts:
   path=attempt/'report.json'
   if not path.exists():continue
   report=json.loads(path.read_text());evaluation=report.get('evaluation') or {}
   harness=attempt/'executor-source/effective-harness.json'
   driver=json.loads(harness.read_text()).get('gripper_position_servo','legacy') if harness.exists() else 'unknown'
   driver_matches=driver==batch.get('driver_profile','legacy')
   commissioning=bool(report.get('commissioning_only') or report.get('reference_result',{}).get('commissioning_only'))
   complete=report.get('passed') is True and evaluation.get('task_success') is True and not report.get('unresolved_robot_actions') and not commissioning and driver_matches
   records.append(dict(path=str(path),passed=complete,error=report.get('error'),wall_s=report.get('wall_time_s'),sim_s=evaluation.get('sim_time_s'),commissioning_only=commissioning,driver_profile=driver,driver_matches_batch=driver_matches,physical_control_sha256=report.get('physical_control_sha256'),failed_robot_actions=report.get('failed_robot_actions'),budget_exhausted=report.get('budget_exhausted',False)))
  status=None
  if attempts and (attempts[-1]/'status.json').exists():status=json.loads((attempts[-1]/'status.json').read_text())
  passed=[r for r in records if r['passed']]
  cases.append(dict(case_id=case['id'],family=case['family'],has_full_success=bool(passed),successful_reports=[r['path'] for r in passed],latest_result=records[-1] if records else None,latest_status=status,attempts=len(attempts)))
 families={f:dict(total=sum(c['family']==f for c in cases),fully_passed=sum(c['family']==f and c['has_full_success'] for c in cases)) for f in batch['families']}
 return dict(batch_id=batch['batch_id'],driver_profile=batch.get('driver_profile','legacy'),updated_unix_s=time.time(),scope='Privileged engineering Oracle only; partial commissioning is never full success',all_36_passed=all(c['has_full_success'] for c in cases),families=families,cases=cases)


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);args=p.parse_args();report=summarize(args.batch)
 output=args.batch/'summary.json';temporary=output.with_suffix('.tmp');temporary.write_text(json.dumps(report,indent=2)+'\n');temporary.replace(output)
 print(json.dumps(report['families']))
