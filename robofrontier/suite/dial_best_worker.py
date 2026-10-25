"""Blind BEST reference workers. Secrets exist only in the evaluator process.

Launch with -I -S. Load dependencies before restricting file/process/network
capabilities. This is trusted-program separation, not an OS security sandbox.
"""
import sys
import json
import math
import runpy
from itertools import product
from collections import Counter


def guard(event,args):
    if event in ('open','os.listdir','os.scandir','os.chdir','import','subprocess.Popen','os.system') or event.startswith(('socket.','ctypes.','os.exec','os.spawn')):
        raise PermissionError('Worker capability denied')


class RecordDecoder:
    def __init__(self,n,k):
        if n not in (4,6,8) or k!=5:raise ValueError('Expected declared BEST5 dial scale')
        self.n=n;self.k=k;self.candidates=list(product(range(1,k+1),repeat=n))
        self.best=0;self.incumbent=None;self.seen=set();self.known=set();self.last=-math.inf

    def symbols(self,x,zero=False):
        if not isinstance(x,list) or len(x)!=self.n or any(type(v)!=int or not (0 if zero else 1)<=v<=self.k for v in x):raise ValueError('Bad observed input')
        return tuple(x)

    def observe(self,t,inputs,best):
        if type(t) not in (int,float) or not math.isfinite(t) or t<self.last:raise ValueError('Observation time must be monotone')
        if type(best)!=int or not self.best<=best<=self.n:raise ValueError('BEST must be monotone')
        if not isinstance(inputs,list) or len(inputs)>64:raise ValueError('Unbounded attribution alternatives')
        guesses=tuple(sorted(set(self.symbols(g) for g in inputs)))
        key=(guesses,best);rising=best>self.best
        if key not in self.seen and guesses:
            # Alternatives are mutually exclusive interpretations, not a claim
            # that every possible combination physically occurred.
            def compatible(code):
                scores=(sum(a==b for a,b in zip(code,g)) for g in guesses)
                return any(s==best for s in scores) if rising else any(s<=best for s in scores)
            remaining=[c for c in self.candidates if compatible(c)]
            if not remaining:raise ValueError('Visible record/attribution contradicts posterior')
            self.candidates=remaining;self.seen.add(key)
            if len(guesses)==1:self.known.add(guesses[0])
        if rising:self.incumbent=guesses[0] if len(guesses)==1 else None
        self.best=best;self.last=t
        return dict(candidates=len(self.candidates),best=self.best)

    def choose(self,current):
        current=self.symbols(current,zero=True);codes=self.candidates;n=len(codes)
        if n==1:return dict(guess=list(codes[0]),candidates=1,best=self.best)
        sample=[codes[round(i*(n-1)/max(1,min(n,4096)-1))] for i in range(min(n,4096))]
        pool={codes[round(i*(n-1)/max(1,min(n,96)-1))] for i in range(min(n,96))}
        if self.incumbent:
            for i in range(self.n):
                for symbol in range(1,self.k+1):pool.add(self.incumbent[:i]+(symbol,)+self.incumbent[i+1:])
        pool-=self.known
        if not pool:pool={next((c for c in codes if c not in self.known),codes[0])}
        choice=None
        for guess in sorted(pool):
            hist=Counter(max(self.best,sum(a==b for a,b in zip(c,guess))) for c in sample)
            entropy=-sum((size/len(sample))*math.log2(size/len(sample)) for size in hist.values())
            score=entropy-.015*sum(a!=b for a,b in zip(guess,current))
            if choice is None or score>choice[0]:choice=(score,guess)
        return dict(guess=list(choice[1]),candidates=n,best=self.best)


def serve_solver(n,k):
    solver=RecordDecoder(n,k)
    for line in sys.stdin:
        r=json.loads(line);op=r.get('op')
        if op=='close':return
        if op=='choose' and set(r)=={'op','current'}:result=solver.choose(r['current'])
        elif op=='observe' and set(r)=={'op','t','inputs','best'}:result=solver.observe(r['t'],r['inputs'],r['best'])
        else:raise ValueError('Only observed inputs and historical BEST are solver inputs')
        print(json.dumps(result),flush=True)


def serve_evaluator(game):
    for line in sys.stdin:
        r=json.loads(line)
        if r.get('op')=='close':return
        if set(r)!={'op','t','angles','eligible'} or r['op']!='sample':raise ValueError('Bad evaluator sample')
        game.update(r['t'],r['angles'],[0.]*game.n,r['eligible'])
        # No current correct-count, per-dial score, candidate or trial history.
        print(json.dumps({name:getattr(game,name) for name in ('display','won','queries','since','ready','input_valid','indicator')}),flush=True)


def main():
    if sys.argv[1]=='solver':
        n,k=map(int,sys.argv[2:4]);sys.addaudithook(guard);serve_solver(n,k)
    elif sys.argv[1]=='evaluator':
        with open(sys.argv[2]) as f:private=json.load(f)
        if len(sys.argv)>5:
            # Evaluator-owned replay input. Parent/solver never parse it.
            with open(sys.argv[5],'x') as f:json.dump(private,f)
        with open(sys.argv[3]) as f:spec=json.load(f)['mechanism']['feedback']
        Feedback=runpy.run_path(sys.argv[4])['DialFeedback']
        game=Feedback(private['code_symbols'],symbol_count=spec['symbol_count'],mode=spec['mode'],
                      tolerance_deg=spec['angle_tolerance_deg'],dwell_s=spec['dwell_sim_s'])
        del private,Feedback,spec
        sys.addaudithook(guard);serve_evaluator(game)
    else:raise ValueError('Unknown worker role')


if __name__=='__main__':main()
