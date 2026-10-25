"""Read-only literal input audit. Flags need review; no OS isolation claim."""
import argparse
import json
from pathlib import Path
from robofrontier.runtime_store import RuntimeStore


def audit(base):
    base=Path(base);checks=[]
    for case in json.loads((base/'batch.json').read_text())['cases']:
        s=RuntimeStore(Path(case['run_dir'])/'private/state/runtime.sqlite');agents=s.agents();s.close()
        for actor,a in agents.items():
            tid=a.get('thread_id')
            if not tid or a['kind']=='manager':continue
            paths=list((Path.home()/'.codex/sessions').glob('**/*'+tid+'*.jsonl'))
            if len(paths)!=1:continue
            records=[];lines=paths[0].read_text().splitlines()
            for i,line in enumerate(lines):
                try:records.append(json.loads(line))
                except json.JSONDecodeError:
                    if i!=len(lines)-1:raise
            meta=next(x['payload'] for x in records if x['type']=='session_meta');cutoff=meta.get('forked_from_ordinal_exclusive',0) or 0
            count=0;flags=[]
            for r in records:
                p=r.get('payload',{})
                if r.get('ordinal',0)<cutoff or r['type']!='response_item' or p.get('type') not in ('function_call','custom_tool_call'):continue
                count+=1;value=p.get('arguments',p.get('input',''));value=value if isinstance(value,str) else json.dumps(value)
                patterns=[x for x in ('runs/oracle','scene.xml','robofrontier.suite','private/outputs/evaluation','private/inputs/scene') if x in value]
                if patterns:flags.append(dict(ordinal=r.get('ordinal'),tool=p.get('name'),patterns=patterns,input_excerpt=value[:1200]))
            checks.append(dict(case_id=case['case_id'],actor=actor,thread_id=tid,own_tool_calls_reviewed=count,flags_for_review=flags))
    result=dict(scope='Literal patterns in recorded own named/freeform tool-call inputs only; not an OS sandbox or exhaustive proof',checks=checks)
    (base/'tool-input-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(actors=len(checks),tool_calls=sum(x['own_tool_calls_reviewed'] for x in checks),flagged_actors=[(x['case_id'],x['actor'],len(x['flags_for_review'])) for x in checks if x['flags_for_review']])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);a=p.parse_args();audit(a.batch)
