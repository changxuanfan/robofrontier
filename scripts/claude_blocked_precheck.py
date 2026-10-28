"""Paid, bounded Claude fork precheck at a blocked synthetic robot tool call.

No real world is started. Without --confirm-spend no model request is made.
CLI-reported costs are estimates, not a provider billing cap.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

from robofrontier.claude.provenance import fork_provenance, history, transcript_rows
from robofrontier.claude.profile_identity import fingerprint
from robofrontier.claude.runner import run_turn
from robofrontier.task_families import ROOT
from scripts.benchmark import load_env
from scripts.claude_benchmark import profile
from scripts.claude_precheck import MIN_TURN_BUDGET, PROMPTS, World, claude_version, setup_run


def _cost(record, allowance):
    cost = (record.get('summary') or {}).get('total_cost_usd')
    if isinstance(cost,bool) or not isinstance(cost,(int,float)) or not math.isfinite(cost) or not 0 < cost <= allowance:
        raise ValueError('Cost unknown or exceeds allocated allowance')
    return cost


def _ended(record, session):
    summary = record.get('summary') or {}
    return (record.get('process') == 'exited' and record.get('exit_code') == 0
            and summary.get('session_id') == session and summary.get('turn_status') == 'ended'
            and summary.get('malformed_lines') == 0)


def _pending_observe(rows):
    for row in reversed(rows):
        message = row.get('message') or {}
        for block in message.get('content', []) if isinstance(message.get('content'), list) else []:
            if (isinstance(block,dict) and block.get('type')=='tool_use' and block.get('name')=='mcp__robot__robot'
                    and isinstance(block.get('input'),dict) and block['input'].get('op')=='observe'
                    and isinstance(block['input'].get('views'),list) and 'top' in block['input']['views']):
                return True
    return False


def run_blocked_precheck(name, output, budget, base_env, *, binary='claude', wait_s=60):
    if isinstance(budget,bool) or not isinstance(budget,(int,float)) or not math.isfinite(budget) or budget < MIN_TURN_BUDGET*3:
        raise ValueError('Budget must cover three minimum turn allowances')
    prof = profile(name)
    output = Path(output)
    output.mkdir(parents=True,exist_ok=False)
    run,mcp,server=setup_run(output,base_env)
    session=str(uuid.uuid4()); child=str(uuid.uuid4())
    receipt={'schema':'robofrontier.claude-blocked-precheck.v1','profile':name,'requested_model':prof['model'],
             'profile_sha256':fingerprint(prof),
             'claude_version':claude_version(binary),'budget_usd':budget,'started_at':time.time(),
             'parent_session':session,'child_session':child,
             'passed':False,'checks':{},'reported_cost_usd':0,'cost_complete':False,'error_type':None}
    cancel=threading.Event(); parent_records=[]; parent_errors=[]; worker=None
    started=0.0
    try:
        first_allowance=budget/3
        initial=run_turn(prof,prof['model'],prof['effort'],mcp,PROMPTS[0][1],run,base_env,
                         session_id=session,timeout=180,max_budget_usd=first_allowance,binary=binary,events_name='initial.jsonl')
        started += _cost(initial,first_allowance)
        if (not _ended(initial,session) or initial['summary'].get('tool_calls') != 0
                or (initial['summary'].get('result_text') or '').strip().lower() != 'ready'):
            raise ValueError('Initial parent turn failed')
        config=run/'private/claude-config'
        baseline=history(transcript_rows(config,session),session)
        server.blocked_entered=threading.Event();server.blocked_release=threading.Event()
        remaining=budget-started
        parent_allowance=remaining/2; fork_allowance=remaining-parent_allowance
        if min(parent_allowance,fork_allowance)<MIN_TURN_BUDGET: raise ValueError('Insufficient remaining budget')
        def parent_turn():
            try:
                parent_records.append(run_turn(prof,prof['model'],prof['effort'],mcp,PROMPTS[1][1],run,base_env,
                    resume=session,timeout=180,max_budget_usd=parent_allowance,binary=binary,
                    events_name='blocked-parent.jsonl',cancel_event=cancel))
            except Exception as error: parent_errors.append(type(error).__name__)
        worker=threading.Thread(target=parent_turn,daemon=True);worker.start()
        deadline=time.monotonic()+wait_s
        while not server.blocked_entered.wait(.1):
            if not worker.is_alive(): raise ValueError('Parent ended before reaching the tool boundary')
            if time.monotonic() >= deadline: raise TimeoutError('Parent did not call the tool')
        source=None
        while time.monotonic()<deadline and worker.is_alive():
            try:
                rows=history(transcript_rows(config,session),session)
                if len(rows)>len(baseline) and rows[:len(baseline)]==baseline and _pending_observe(rows[len(baseline):]):
                    source=rows;break
            except ValueError: pass  # CLI may be appending the native transcript.
            time.sleep(.05)
        if source is None: raise ValueError('Pending parent call was not recorded')
        digest=hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest()
        record=run_turn(prof,prof['model'],prof['effort'],None,'This is a tool-free setup turn. Reply READY.',
                        run,base_env,session_id=child,fork_from=session,enable_robot=False,timeout=180,
                        max_budget_usd=fork_allowance,binary=binary,events_name='blocked-fork.jsonl')
        started+=_cost(record,fork_allowance)
        proof=fork_provenance(session,child,config)
        receipt['checks']={'parent_blocked':worker.is_alive() and not server.blocked_release.is_set(),
                           'fork_ended':_ended(record,child),
                           'fork_has_no_tools':record['summary'].get('tool_calls')==0,
                           'source_unchanged':proof['source_sha256']==digest,
                           'native_history_prefix':proof['source_thread_id']==session and child!=session}
        server.blocked_release.set();worker.join(180)
        if worker.is_alive() or parent_errors or len(parent_records)!=1:
            raise ValueError('Parent outcome unknown')
        parent=parent_records[0];started+=_cost(parent,parent_allowance)
        receipt['cost_complete']=True
        receipt['checks']['parent_ended']=_ended(parent,session)
        receipt['checks']['parent_used_tool']=parent['summary'].get('tool_calls',0)>=1
        receipt['checks']['parent_requested_view']=parent['summary'].get('observe_top_calls',0)>=1
        receipt['checks']['parent_identified_image']=(parent['summary'].get('result_text') or '').strip().lower()==server.challenge_color
        receipt['passed']=all(receipt['checks'].values()) and started<=budget
    except Exception as error:
        receipt['error_type']=type(error).__name__
    finally:
        if hasattr(server,'blocked_release'):server.blocked_release.set()
        if worker and worker.is_alive():
            cancel.set();worker.join(10)
            if worker.is_alive(): receipt['error_type']='ParentStillActive';receipt['passed']=False
        server.shutdown();server.server_close()
    receipt.update(reported_cost_usd=round(started,6),finished_at=time.time(),challenge_color=server.challenge_color)
    (output/'receipt.json').write_text(json.dumps(receipt,indent=2,ensure_ascii=False))
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True)
    parser.add_argument('--max-budget-usd',type=float,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--confirm-spend',action='store_true')
    args=parser.parse_args()
    load_env(ROOT/'.env'); selected=profile(args.profile)
    if not args.confirm_spend:
        print(json.dumps({'status':'not_run','profile':args.profile,'model':selected['model'],
                          'potential_paid_cli_turns':3,'model_calls':0},indent=2));return 2
    if not os.environ.get(selected['env_key']):raise SystemExit('Missing credential variable: '+selected['env_key'])
    output=args.output or ROOT/'runs/engineering/claude-blocked-precheck'/(args.profile+'-'+time.strftime('%Y%m%d-%H%M%S'))
    result=run_blocked_precheck(args.profile,output,args.max_budget_usd,dict(os.environ))
    print(json.dumps({'profile':args.profile,'passed':result['passed'],'checks':result['checks'],
                      'error_type':result['error_type'],'receipt':str(output/'receipt.json')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
