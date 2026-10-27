"""Read-only differential timing of a contact-loop prefilter in a scratch mirror.

The experiment retains the complete existing pose-clear predicate. It changes
only which irrelevant contact rows reach that predicate. No live owner imports
this script, no physics steps occur, and no request is emitted.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
import time

import numpy as np

from scripts.plan_mosaic_repair_pick import observed_trial
from robofrontier.suite.oracle import Trial
from robofrontier.run_setup import write_json


def relevant_contact_indices(scratch, owner, robot_index, carried_geoms, clearance_m):
    """Keep all rows the existing predicate can reject, in original order."""
    pairs=np.asarray(scratch.contact.geom)
    if not len(pairs):return np.empty(0,dtype=int)
    # Negated >= deliberately preserves the original behavior for NaN too.
    close=~(np.asarray(scratch.contact.dist)>=clearance_m)
    relevant=np.any(owner[pairs]==robot_index,axis=1)
    if carried_geoms:relevant|=np.any(np.isin(pairs,list(carried_geoms)),axis=1)
    return np.flatnonzero(close&relevant)


def experimental_predicate():
    source=textwrap.dedent(inspect.getsource(Trial.pose_clear))
    old='for c in scratch.contact:'
    if source.count(old)!=1:raise ValueError('Existing predicate changed; inspect before benchmarking')
    source=source.replace(old,'for index in relevant_contact_indices(scratch, owner, rid, carried_geoms, clearance_m):\n        c=scratch.contact[index]')
    namespace=dict(Trial.pose_clear.__globals__,relevant_contact_indices=relevant_contact_indices)
    exec(compile(source,'<readonly-contact-prefilter-experiment>','exec'),namespace)
    return namespace['pose_clear']


def benchmark(a):
    t,o,payload=observed_trial(a.trial);old=Trial.pose_clear;new=experimental_predicate();rng=np.random.default_rng(20261026)
    original=t.data.qpos.copy();initial_held=dict(t.held);items=[];bodies=[v['id'] for v in t.task['objects']]
    # Mix observed/local configurations and broad joint-limit samples. Include
    # target-contact permission and carried-object screening, not only empty arms.
    for index in range(a.samples):
        robot=list(t.arms)[index%len(t.arms)];arm=t.arms[robot];current=original[arm.qids]
        q=(np.clip(current+rng.normal(0,.15 if index%3==0 else .6,7),*arm.ik.limits.T)
           if index%3 else rng.uniform(*arm.ik.limits.T))
        allowed=(bodies[index%len(bodies)],) if index%2 else ()
        held={robot:bodies[(index*7)%len(bodies)]} if index%5==0 else initial_held
        items.append((robot,q,allowed,held))
    mismatches=[];outcomes=[]
    for index,(robot,q,allowed,held) in enumerate(items):
        t.held=dict(held);before=t.data.qpos.copy();left=old(t,robot,q,allowed);right=new(t,robot,q,allowed)
        if left!=right:mismatches.append(index)
        if not np.array_equal(before,t.data.qpos):raise RuntimeError('Scratch benchmark modified observed state')
        outcomes.append(bool(left))
    timings={'legacy':[],'prefilter':[]}
    for repeat in range(a.repeats):
        for label,fn in ([('legacy',old),('prefilter',new)] if repeat%2==0 else [('prefilter',new),('legacy',old)]):
            start=time.perf_counter()
            for robot,q,allowed,held in items:t.held=dict(held);fn(t,robot,q,allowed)
            timings[label].append(time.perf_counter()-start)
    t.held=initial_held
    result=dict(scope='Read-only endpoint screening microbenchmark. No live owner changes, no dynamics, no whole-task speedup claim.',
        trial=str(a.trial.resolve()),observation_sha256=hashlib.sha256(payload).hexdigest(),source_sim_s=o['sim_time_s'],
        original_predicate_sha256=hashlib.sha256(inspect.getsource(old).encode()).hexdigest(),samples=a.samples,repeats=a.repeats,
        mismatches=mismatches,all_predicates_equal=not mismatches,accepted=sum(outcomes),rejected=len(outcomes)-sum(outcomes),
        wall_s=timings,median_speedup=float(np.median(timings['legacy'])/np.median(timings['prefilter'])),
        sampled_state_preserved=bool(np.array_equal(original,t.data.qpos)))
    write_json(a.report,result);print(json.dumps(result),flush=True)
    if mismatches:raise RuntimeError('Experimental screen differed; do not use it')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--samples',type=int,default=96);p.add_argument('--repeats',type=int,default=5)
    a=p.parse_args()
    if a.samples<1 or a.repeats<1:raise ValueError('Use positive finite sample and repeat counts')
    benchmark(a)
