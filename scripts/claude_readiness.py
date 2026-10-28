"""Compare Claude precheck receipts with current configuration; no model calls.

A match is a local evidence check, not scored-run qualification, proof of model
identity, or an authentication mechanism against a user who can edit files.
"""
import argparse
import json
import math
import uuid
from pathlib import Path

from robofrontier.claude.profile_identity import fingerprint
from robofrontier.task_families import ROOT
from scripts.benchmark import load_env
from scripts.claude_benchmark import profile
from scripts.claude_precheck import claude_version

BASIC = {
    'text': {'process_exited','turn_ended','same_session','valid_events','reply_ready','no_new_robot_call'},
    'tool_image': {'process_exited','turn_ended','same_session','valid_events','tool_called','robot_request_made','image_view_requested','image_color_named'},
    'resume': {'process_exited','turn_ended','same_session','valid_events','color_recalled','no_new_robot_call'},
    'fork': {'process_exited','turn_ended','distinct_expected_session','no_tool_calls','color_recalled','valid_events','native_history_prefix'},
}
BLOCKED = {'parent_blocked','fork_ended','fork_has_no_tools','source_unchanged','native_history_prefix',
           'parent_ended','parent_used_tool','parent_requested_view','parent_identified_image'}


def _positive_number(value):
    return not isinstance(value, bool) and isinstance(value, (int,float)) and math.isfinite(value) and value>0


def _uuid(value):
    try: return isinstance(value,str) and str(uuid.UUID(value))==value
    except (ValueError,AttributeError): return False


def _read(path):
    try:
        data=json.loads(Path(path).read_text())
        return data if isinstance(data,dict) else None
    except (OSError,ValueError,TypeError):
        return None


def _identity(receipt, name, selected, version, schema):
    return (isinstance(receipt,dict) and receipt.get('schema')==schema and receipt.get('passed') is True
            and receipt.get('profile')==name and receipt.get('requested_model')==selected['model']
            and receipt.get('profile_sha256')==fingerprint(selected) and bool(version)
            and receipt.get('claude_version')==version and _positive_number(receipt.get('budget_usd')))


def check_receipts(name, basic_path, blocked_path, *, selected=None, version=None):
    selected=profile(name) if selected is None else selected
    version=claude_version() if version is None else version
    basic=_read(basic_path); blocked=_read(blocked_path)
    checks={'basic_identity':_identity(basic,name,selected,version,'robofrontier.claude-precheck.v1'),
            'blocked_identity':_identity(blocked,name,selected,version,'robofrontier.claude-blocked-precheck.v1')}
    entries=basic.get('checks',[]) if isinstance(basic,dict) else []
    items={item['name']:item for item in entries if isinstance(item,dict) and isinstance(item.get('name'),str)} if isinstance(entries,list) else {}
    checks['basic_complete']=(len(entries)==len(BASIC) and len(items)==len(BASIC)
                              and basic.get('fork_checked') is True) if isinstance(basic,dict) and isinstance(entries,list) else False
    for name_of_check, required in BASIC.items():
        item=items.get(name_of_check,{})
        facts=item.get('checks',{})
        checks['basic_'+name_of_check]=(item.get('passed') is True and isinstance(facts,dict)
                                        and all(facts.get(key) is True for key in required)
                                        and _positive_number(item.get('cost_usd')))
    total=basic.get('total_cost_usd') if isinstance(basic,dict) else None
    costs=[items.get(key,{}).get('cost_usd') for key in BASIC]
    checks['basic_cost']=(_positive_number(total) and _positive_number(basic.get('budget_usd'))
                          and total <= basic['budget_usd'] and all(_positive_number(x) for x in costs)
                          and abs(total-sum(costs)) <= .00001) if isinstance(basic,dict) else False
    sessions=[items.get(key,{}).get('session_id') for key in BASIC]
    checks['basic_sessions']=(_uuid(sessions[0]) and all(x==sessions[0] for x in sessions[1:3])
                              and _uuid(sessions[3]) and sessions[3]!=sessions[0])
    facts=blocked.get('checks',{}) if isinstance(blocked,dict) else {}
    checks['blocked_complete']=(isinstance(facts,dict) and all(facts.get(key) is True for key in BLOCKED)
                                and blocked.get('cost_complete') is True and blocked.get('error_type') is None) if isinstance(blocked,dict) else False
    cost=blocked.get('reported_cost_usd') if isinstance(blocked,dict) else None
    checks['blocked_cost']=(_positive_number(cost) and _positive_number(blocked.get('budget_usd'))
                            and cost <= blocked['budget_usd']) if isinstance(blocked,dict) else False
    parent=blocked.get('parent_session') if isinstance(blocked,dict) else None
    child=blocked.get('child_session') if isinstance(blocked,dict) else None
    checks['blocked_sessions']=_uuid(parent) and _uuid(child) and parent!=child
    return {'profile':name,'requested_model':selected['model'],'claude_version':version,
            'precheck_evidence_matches':all(checks.values()),'checks':checks,
            'formal_execution_available':False,'model_calls':0,
            'note':'Local receipt consistency only; not proof of provider identity, physical task success, billing, or scored qualification.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',required=True)
    parser.add_argument('--precheck',required=True,type=Path)
    parser.add_argument('--blocked-precheck',required=True,type=Path)
    args=parser.parse_args()
    load_env(ROOT/'.env')
    result=check_receipts(args.profile,args.precheck,args.blocked_precheck)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['precheck_evidence_matches'] else 1


if __name__=='__main__':raise SystemExit(main())
