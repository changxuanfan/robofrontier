"""Bounded process pool for independent mosaic physics units, not agents."""
from concurrent.futures import ProcessPoolExecutor,wait,FIRST_COMPLETED
from collections import Counter
import argparse
import gc
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import time

from scripts.probe_mosaic_pick_place import unit,freeze_sources,DEFAULT_OUT
from robofrontier.suite.v2 import ROOT
from robofrontier.run_setup import write_json

STATE=None


def initialize(state):
    global STATE
    STATE=state


def work(job):
    case,name=job;s=STATE
    result=unit(case,name,s['output'],s['sources'],s['coverage'][case['id']])
    record=dict(case_id=case['id'],object=name,unit_passed=result['unit_passed'],error=result.get('error'),
        sim_s=result['evaluation']['sim_time_s'],wall_s=result['wall_time_s'],
        report=str(s['output']/case['id']/name/'attempt-001/report.json'))
    gc.collect()
    return record


def main(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    sources=freeze_sources(output)
    catalog=json.loads((ROOT/'configs/suites/frontier-mosaic-r13.json').read_text())['cases']
    coverage={}
    for c in catalog:
        p=ROOT/'runs/engineering/mosaic-r13-coverage-20261025'/c['id']/'report.json'
        coverage[c['id']]=json.loads(p.read_text())
        assert coverage[c['id']]['passed']
        assert hashlib.sha256((ROOT/c['directory']/'scene.xml').read_bytes()).hexdigest()==coverage[c['id']]['source_scene_sha256']
    groups=[[(c,x['id']) for x in coverage[c['id']]['sources']] for c in catalog if c['level'] in (1,2)]
    jobs=[group[i] for i in range(max(map(len,groups))) for group in groups if i<len(group)]
    for n,name in [(4,'B_stock06'),(6,'C_stock01'),(8,'A_stock01')]:
        jobs.insert(0,(next(c for c in catalog if c['arms']==n and c['level']==3),name))
    write_json(output/'plan.json',dict(scope='Independent new canonical world per item.1044 L1/L2 units plus3 L3 physical-equivalence spotchecks. No artwork goal.',
        workers=args.workers,jobs=[dict(case_id=c['id'],object=name) for c,name in jobs],
        l3_scope='One physical spotcheck per size; full L3 items require explicit model-equivalence reuse or further runs, never claim independently executed.',
        no_source_scene_edits=True,model='MuJoCo original2ms',recording_hz=60,source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in sources.items()}))
    state=dict(output=output,sources=sources,coverage=coverage)
    progress=dict(manager_pid=os.getpid(),started_unix_s=time.time(),planned=len(jobs),results=[],complete=False)
    def publish(active):
        counts={}
        for r in progress['results']:
            x=counts.setdefault(r['case_id'],dict(tested=0,passed=0,failed=0))
            x['tested']+=1;x['passed']+=bool(r['unit_passed']);x['failed']+=not r['unit_passed']
        progress.update(completed=len(progress['results']),counts=counts,active=[dict(case_id=c['id'],object=n) for c,n in active.values()],
            updated_unix_s=time.time(),wall_s=time.time()-progress['started_unix_s'])
        write_json(output/'progress.json',progress)
    pending=[]
    for case,name in jobs:
        report=output/case['id']/name/'attempt-001/report.json'
        if report.exists():
            if not args.resume:raise FileExistsError(report)
            r=json.loads(report.read_text());progress['results'].append(dict(case_id=case['id'],object=name,unit_passed=r['unit_passed'],error=r.get('error'),sim_s=r['evaluation']['sim_time_s'],wall_s=r['wall_time_s'],report=str(report)))
        elif report.parent.exists():raise ValueError('Unsealed previous unit; do not automatically replay '+str(report.parent))
        else:pending.append((case,name))
    queue=iter(pending);active={};publish(active)
    context=multiprocessing.get_context('spawn')
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=context,initializer=initialize,initargs=(state,)) as pool:
        exhausted=False
        while active or not exhausted:
            while len(active)<args.workers and not exhausted and not (output/'STOP').exists():
                job=next(queue,None)
                if job is None:exhausted=True;break
                active[pool.submit(work,job)]=job
            if (output/'STOP').exists():exhausted=True;progress['stopped_admitting_new_units']=True
            publish(active)
            if not active:break
            done,_=wait(active,timeout=1,return_when=FIRST_COMPLETED)
            for future in done:
                case,name=active.pop(future)
                try:record=future.result()
                except Exception as e:
                    record=dict(case_id=case['id'],object=name,unit_passed=False,error='WORKER: '+type(e).__name__+': '+str(e),unsealed=True)
                progress['results'].append(record)
                print('COMPLETE',len(progress['results']),'/',len(jobs),case['id'],name,record['unit_passed'],record.get('error'),flush=True)
            publish(active)
    progress['complete']=len(progress['results'])==len(jobs);publish(active)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--workers',type=int,default=12);p.add_argument('--output',type=Path,default=DEFAULT_OUT/'batch-v4');p.add_argument('--resume',action='store_true');a=p.parse_args();main(a)
