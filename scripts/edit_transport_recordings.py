"""Analyze Transport recordings directly; never renders or starts physics."""
import argparse,json
from pathlib import Path
from robofrontier.suite.transport_policy_package import load_json,resolve,verify_oracle
from robofrontier.suite.transport_motion_edit import build_edit,TransportEdit
from robofrontier.run_setup import write_json
ROOT=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/task_families/transport/half-opening-r14.json');p.add_argument('--case');p.add_argument('--output',type=Path,required=True);a=p.parse_args();config=load_json(a.config)
    if a.case and a.case not in {r['case_id']for r in config['cases']}:p.error('Unknown Transport case')
    a.output.mkdir(parents=True,exist_ok=True);rows=[]
    for row in config['cases']:
        if a.case and a.case!=row['case_id']:continue
        verify_oracle(row,ROOT)
        target=a.output/row['case_id']
        if (target/'edit.json').exists():TransportEdit(target/'edit.json');value=load_json(target/'edit.json')
        else:value=build_edit(resolve(ROOT,row['oracle_trial']),target)
        rows.append(value);write_json(a.output/'progress.json',dict(status='analyzing',completed=len(rows),current=row['case_id']))
        print('EDIT_COMPLETE',row['case_id'],value['retained_frames'],value['removed_frames'],flush=True)
    write_json(a.output/'summary.json',dict(schema='robofrontier.transport-source-edits.v1',completed=len(rows),native_source_decisions=True,case_edits=[dict(case_id=r['case_id'],edit=str((a.output/r['case_id']/'edit.json').resolve()),output_duration_s=r['output_duration_s'])for r in rows]))
    write_json(a.output/'progress.json',dict(status='complete',completed=len(rows)))
