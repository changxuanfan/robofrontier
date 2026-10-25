"""BEST feedback across alphabet sizes, with exact posterior beliefs.

N<=8,K<=7. Query entropy uses up to8192 belief samples; true passwords are
sampled separately and cannot influence the agent's next-query choice.
"""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import math
import os
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/oracle/dial-feedback-alphabet-20261025'
PERCENTILES=(5,20,50,80,95)


def all_codes(n,k):
    values=np.arange(k**n,dtype=np.uint32);codes=np.empty((len(values),n),dtype=np.uint8)
    for j in range(n-1,-1,-1):codes[:,j]=values%k+1;values//=k
    return codes


def evaluate(n,k,sample_size=4096,belief_cap=8192):
    started=time.perf_counter();codes=all_codes(n,k);total=len(codes)
    rng=np.random.default_rng(20261025+100*n+k)
    sample_ids=np.arange(total,dtype=np.uint32) if total<=sample_size else np.sort(rng.choice(total,sample_size,replace=False).astype(np.uint32))
    queries=np.zeros(len(sample_ids),dtype=np.int32);moves=np.zeros(len(sample_ids),dtype=np.int32)
    nodes=0;approx_nodes=0
    def visit(ids,test_indices,current,best,incumbent,seed):
        nonlocal nodes,approx_nodes
        nodes+=1
        if len(ids)==1:
            guess=codes[ids[0]];queries[test_indices]+=1;moves[test_indices]+=np.sum(guess!=current)
            assert np.all(sample_ids[test_indices]==ids[0]);return
        count=min(96,len(ids));positions=np.asarray([round(i*(len(ids)-1)/(count-1)) for i in range(count)])
        guesses=codes[ids[positions]]
        if incumbent is not None:
            extras=[]
            for j in range(n):
                for symbol in range(1,k+1):
                    if symbol!=incumbent[j]:
                        q=incumbent.copy();q[j]=symbol;extras.append(q)
            guesses=np.vstack((guesses,extras));_,unique=np.unique(guesses,axis=0,return_index=True);guesses=guesses[np.sort(unique)]
        if len(ids)>belief_cap:
            sample_positions=np.random.default_rng(seed).choice(len(ids),belief_cap,replace=False)
            belief=codes[ids[sample_positions]];approx_nodes+=1
        else:belief=codes[ids]
        scores=np.zeros((len(guesses),len(belief)),dtype=np.uint8)
        for j in range(n):scores+=(guesses[:,j,None]==belief[None,:,j])
        visible=np.maximum(scores,best) if best>=0 else scores
        entropy=np.zeros(len(guesses))
        for value in range(n+1):
            prob=np.mean(visible==value,axis=1);mask=prob>0;entropy[mask]-=prob[mask]*np.log2(prob[mask])
        changes=np.sum(guesses!=current,axis=1);rank=np.round(entropy,12)-.015*changes
        if np.any(entropy>1e-12):
            rank[entropy<1e-12]=-np.inf;selected=int(np.argmax(rank))
        else:
            # Approximate entropy may miss a rare positive response. A query
            # from the exact candidate set still guarantees progress.
            selected=int(np.argmin(changes[:count]))
        guess=guesses[selected];changed=int(changes[selected])
        queries[test_indices]+=1;moves[test_indices]+=changed
        full_scores=np.zeros(len(ids),dtype=np.uint8)
        for j in range(n):full_scores+=(codes[ids,j]==guess[j])
        responses=np.maximum(full_scores,best) if best>=0 else full_scores
        actual=np.sum(codes[sample_ids[test_indices]]==guess,axis=1)
        observed=np.maximum(actual,best) if best>=0 else actual
        for response in np.unique(observed):
            test_child=test_indices[observed==response]
            if response==n:
                assert np.all(codes[sample_ids[test_child]]==guess);continue
            child=ids[responses==response]
            assert len(child)<len(ids) and len(child)>0
            new_incumbent=guess if int(response)>best else incumbent
            next_seed=(seed*1664525+1013904223+int(response)+int(np.sum(guess*np.arange(1,n+1))))&0xffffffff
            visit(child,test_child,guess,max(best,int(response)),new_incumbent,next_seed)
    visit(np.arange(total,dtype=np.uint32),np.arange(len(sample_ids)),np.zeros(n,dtype=np.uint8),-1,None,91723+n*31+k)
    quantile=lambda xs,p:int(np.sort(xs)[math.ceil(len(xs)*p/100)-1])
    result=dict(dials=n,alphabet=k,passwords=total,tested=len(sample_ids),exhaustive=len(sample_ids)==total,
        entropy_belief_cap=belief_cap,entropy_approximate_nodes=approx_nodes,
        mean=float(np.mean(queries)),mean_changed_dials=float(np.mean(moves)),
        percentiles={f'P{p}':quantile(queries,p) for p in PERCENTILES},sample_maximum=int(np.max(queries)),
        histogram={str(int(q)):int(c) for q,c in zip(*np.unique(queries,return_counts=True))},
        decision_nodes=nodes,wall_s=time.perf_counter()-started)
    if not result['exhaustive']:
        # Monte Carlo uncertainty of the mean, with finite-population correction.
        se=float(np.std(queries,ddof=1)/np.sqrt(len(queries))*np.sqrt((total-len(queries))/(total-1)))
        result['mean_approx_95ci']=[result['mean']-1.96*se,result['mean']+1.96*se]
        boot=np.random.default_rng(66181+n*100+k).choice(queries,size=(1000,len(queries)),replace=True)
        sorted_boot=np.sort(boot,axis=1)
        result['percentile_bootstrap_95ci']={f'P{p}':np.quantile(sorted_boot[:,math.ceil(len(queries)*p/100)-1],[.025,.975],method='inverted_cdf').astype(int).tolist() for p in PERCENTILES}
    return result,sample_ids,queries,moves


def coordinate_distribution(n,k):
    """Exact count distribution via integer polynomial convolution, no secrets."""
    prefix=np.array([1],dtype=np.int64);one=np.zeros(k,dtype=np.int64);one[1:]=1;one[-1]+=1
    last=np.zeros(k,dtype=np.int64);last[1:]=1
    hist=np.zeros(2+n*(k-1),dtype=np.int64);hist[1]=1
    for length in range(1,n+1):
        term=np.convolve(prefix,last);hist[1:len(term)+1]+=term
        prefix=np.convolve(prefix,one)
    assert int(hist.sum())==k**n
    cumulative=np.cumsum(hist)
    return dict(mean=float(np.dot(hist,np.arange(len(hist)))/hist.sum()),
        percentiles={f'P{p}':int(np.searchsorted(cumulative,math.ceil(hist.sum()*p/100))) for p in PERCENTILES},
        guaranteed_maximum=1+n*(k-1),exhaustive=True)


def case_run(pair):
    n,k=pair;result,ids,queries,moves=evaluate(n,k)
    result['coordinate_baseline_exact']=coordinate_distribution(n,k)
    stem=f'n{n:02}-k{k}';np.savez_compressed(OUT/(stem+'.npz'),sample_ids=ids,queries=queries,changed_dials=moves)
    (OUT/(stem+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    jobs=[(n,k) for n in (4,6,8) for k in (4,5,6,7)]
    jobs.sort(key=lambda p:p[1]**p[0],reverse=True)
    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as pool:
        for result in pool.map(case_run,jobs):print(json.dumps(result),flush=True)
    rows=[json.loads(p.read_text()) for p in sorted(OUT.glob('n*-k*.json'))]
    assumptions=dict(feedback='BEST-so-far',effective_symbols_exclude_zero=True,
        query_policy='Candidate entropy + incumbent single-position neighbors; pool96; movement penalty0.015',
        belief_filtering='Exact full posterior, even when entropy ranking uses8192belief samples',
        password_sample='All if total<=4096; otherwise4096uniform distinct passwords, deterministic seed',
        entropy_sampling='History-derived seed, independent of true-password test sample',
        percentile='Smallest integer count covering at least p% of test passwords',
        counts_include_initial_and_winning=True,discrete_trials=True,physical_or_GPT_simulation=False)
    (OUT/'summary.json').write_text(json.dumps(dict(assumptions=assumptions,results=rows),indent=2)+'\n')
