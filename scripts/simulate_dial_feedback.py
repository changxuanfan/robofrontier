"""Exhaustive discrete-logic comparison; no MuJoCo or private codebook input.

Each completed configuration is one explicit evaluation. Robot trajectories,
intermediate automatic evaluations, grasp failures and model latency are absent.
"""
from itertools import product
from pathlib import Path
import csv
import json
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/oracle/dial-feedback-comparison-20261025'


def entropy_tree(codes,mode,pool_limit=96,movement_weight=.015,incumbent_neighbors=False):
    n=codes.shape[1];size=len(codes);queries=np.zeros(size,int);moves=np.zeros(size,int)
    matching_work=np.zeros(size,np.int64);nodes=0;start=time.perf_counter()
    def visit(ids,current,best,incumbent=None):
        nonlocal nodes
        nodes+=1;k=min(pool_limit,len(ids))
        indices=np.asarray([round(i*(len(ids)-1)/max(1,k-1)) for i in range(k)])
        guesses=codes[ids[indices]]
        if incumbent_neighbors and incumbent is not None:
            neighbors=[]
            for index in range(n):
                for symbol in (1,2,3):
                    if symbol==incumbent[index]:continue
                    q=incumbent.copy();q[index]=symbol;neighbors.append(q)
            guesses=np.vstack([guesses,neighbors])
            _,unique=np.unique(guesses,axis=0,return_index=True);guesses=guesses[np.sort(unique)];k=len(guesses)
        scores=np.sum(guesses[:,None,:]==codes[ids][None,:,:],axis=2,dtype=np.uint8)
        feedback=np.maximum(scores,best) if mode=='record' and best>=0 else scores
        entropy=np.zeros(k)
        for value in range(n+1):
            prob=np.mean(feedback==value,axis=1);nonzero=prob>0
            entropy[nonzero]-=prob[nonzero]*np.log2(prob[nonzero])
        changed=np.sum(guesses!=current,axis=1)
        ranks=np.round(entropy,12)-movement_weight*changed
        if len(ids)>1:ranks[entropy<1e-12]=-np.inf
        else:ranks[~np.all(guesses==codes[ids[0]],axis=1)]=-np.inf
        selected=int(np.argmax(ranks))
        guess=guesses[selected];actual=scores[selected];visible=feedback[selected]
        queries[ids]+=1;moves[ids]+=changed[selected];matching_work[ids]+=k*len(ids)
        # All children use only the visible response, not the hidden score.
        for response in np.unique(visible):
            child=ids[visible==response]
            if response==n:
                assert len(child)==1 and np.all(codes[child[0]]==guess)
                continue
            new_best=max(best,int(response))
            assert len(child)<len(ids), 'A candidate query must eliminate itself on failure'
            best_input=guess if int(response)>best else incumbent
            visit(child,guess,new_best,best_input)
    visit(np.arange(size),np.zeros(n,dtype=np.uint8),-1)
    return dict(queries=queries,moves=moves,matching_work=matching_work,nodes=nodes,simulation_wall_s=time.perf_counter()-start)


def coordinate_record(codes):
    """Keep a known record configuration and test alternate single digits.

    No-progress tests give only <=best. Next submitted configurations restore
    all unchanged digits to the incumbent; simultaneous changed digits count.
    """
    n=codes.shape[1];queries=[];moves=[];start=time.perf_counter()
    for secret in codes:
        incumbent=np.ones(n,dtype=np.uint8);current=np.zeros(n,dtype=np.uint8);best=-1;count=0;motion=0
        def evaluate(guess):
            nonlocal current,best,count,motion
            count+=1;motion+=int(np.sum(guess!=current));current=guess.copy()
            # This is the environment; only max(score,best) is returned.
            best=max(best,int(np.sum(guess==secret)))
            return best
        evaluate(incumbent)
        if best!=n:
            for index in range(n):
                baseline=best
                for symbol in (1,2,3):
                    if symbol==incumbent[index]:continue
                    candidate=incumbent.copy();candidate[index]=symbol
                    if evaluate(candidate)>baseline:
                        incumbent=candidate;break
                if best==n:break
        assert best==n and np.array_equal(current,secret)
        assert count<=1+2*n
        queries.append(count);moves.append(motion)
    return dict(queries=np.asarray(queries),moves=np.asarray(moves),simulation_wall_s=time.perf_counter()-start)


def stats(values):
    return dict(mean=float(np.mean(values)),median=float(np.median(values)),p90=float(np.percentile(values,90)),
                p95=float(np.percentile(values,95)),maximum=int(np.max(values)),minimum=int(np.min(values)),
                histogram={str(int(k)):int(v) for k,v in zip(*np.unique(values,return_counts=True))})


def main():
    OUT.mkdir(parents=True,exist_ok=True);summary=[];raw=[]
    for n in (4,6,8):
        codes=np.asarray(list(product((1,2,3),repeat=n)),dtype=np.uint8)
        for name,method in [('current_entropy',lambda:entropy_tree(codes,'current')),
                            ('record_entropy',lambda:entropy_tree(codes,'record')),
                            ('record_coordinate',lambda:coordinate_record(codes)),
                            ('record_local_entropy',lambda:entropy_tree(codes,'record',incumbent_neighbors=True))]:
            result=method();row=dict(dials=n,passwords=len(codes),policy=name,queries=stats(result['queries']),
                changed_dials=stats(result['moves']),simulation_wall_s=result['simulation_wall_s'])
            if 'matching_work' in result:row.update(candidate_pairs=stats(result['matching_work']),decision_nodes=result['nodes'])
            summary.append(row)
            for index,code in enumerate(codes):raw.append(dict(dials=n,code=''.join(map(str,code)),policy=name,queries=int(result['queries'][index]),changed_dials=int(result['moves'][index])))
            (OUT/'progress.json').write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps({k:row[k] for k in ['dials','policy','queries','changed_dials','simulation_wall_s']}),flush=True)
    with (OUT/'per-password.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=['dials','code','policy','queries','changed_dials']);writer.writeheader();writer.writerows(raw)
    assumptions=dict(alphabet=[1,2,3],password_distribution='Uniform exhaustive enumeration, not the9fixed task answers',
        total_passwords_per_policy=81+729+6561,discrete_settlement=True,
        feedback_modes=dict(current='Exact score of this evaluated configuration',record='max(previous record, this score); no-progress means score<=record'),
        movement_metric='Number of dial digits changed between submitted configurations, including initial0->nonzero; not robot actions or trajectory time',
        entropy_policy='Same candidate-only pool<=96, entropy minus0.015changed digits; not an optimality proof',
        coordinate_policy='Known record-incumbent, one-position alternatives; return lower-score experiments to incumbent in the next submitted configuration',
        excluded=['Robot dynamics','Intermediate automatic feedback during motion','Grasp/recovery','Mechanical timing','LLM reasoning/cognitive error'],
        algorithm_compute_note='Decision-tree walltime shares prefixes across all passwords; not a per-episode Oracle or GPT runtime')
    (OUT/'summary.json').write_text(json.dumps(dict(assumptions=assumptions,results=summary),indent=2)+'\n')


if __name__=='__main__':main()
