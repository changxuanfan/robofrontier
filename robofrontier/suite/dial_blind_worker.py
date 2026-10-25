"""Two separate stdlib-only processes: private evaluator or count-only solver.

Run with Python -I -S. Solver arguments contain N only; only evaluator opens
the codebook. Audit hooks forbid subsequent file/network/process access.
"""
import sys
import json
import math
from itertools import product
from collections import Counter


def guard(event,args):
    if event in ('open','os.listdir','os.scandir','os.chdir','import','subprocess.Popen','os.system') or event.startswith(('socket.','ctypes.','os.exec','os.spawn')):
        raise PermissionError('Worker capability denied')


def serve_solver(n):
    candidates=list(product((1,2,3),repeat=n));seen=set()
    for line in sys.stdin:
        request=json.loads(line);op=request.get('op')
        if op=='close':return
        if op=='choose':
            if set(request)!={'op','current'}:raise ValueError('Only current symbols are solver input')
            current=request['current']
            if len(current)!=n or any(type(x)!=int or x not in (0,1,2,3) for x in current):raise ValueError('Bad current symbols')
            size=min(96,len(candidates));indices=[round(i*(len(candidates)-1)/max(1,size-1)) for i in range(size)]
            best=None
            for i in indices:
                guess=candidates[i];counts=Counter(sum(a==b for a,b in zip(c,guess)) for c in candidates)
                entropy=-sum((count/len(candidates))*math.log2(count/len(candidates)) for count in counts.values())
                rank=entropy-.015*sum(a!=b for a,b in zip(guess,current))
                if best is None or rank>best[0]:best=(rank,guess)
            result={'guess':list(best[1]),'candidates':len(candidates)}
        elif op=='feedback':
            if set(request)!={'op','guess','count'}:raise ValueError('Only guess/count are feedback')
            guess=request['guess'];count=request['count']
            if len(guess)!=n or any(type(x)!=int or x not in (1,2,3) for x in guess) or type(count)!=int or not 0<=count<=n:raise ValueError('Bad visible feedback')
            key=(tuple(guess),count)
            if key not in seen:
                seen.add(key);candidates=[c for c in candidates if sum(a==b for a,b in zip(c,guess))==count]
            if not candidates:raise RuntimeError('Observed count contradicts candidate inputs')
            result={'candidates':len(candidates)}
        else:raise ValueError('Unsupported solver operation')
        print(json.dumps(result),flush=True)


def serve_evaluator(secret):
    candidate=None;since=None;submitted=False;won=False;queries=0;last=None;n=len(secret)
    for line in sys.stdin:
        r=json.loads(line)
        if r.get('op')=='close':return
        if set(r)!={'op','t','angles','eligible'} or r['op']!='sample':raise ValueError('Bad evaluator sample')
        t=r['t'];degrees=[math.degrees(x)%360 for x in r['angles']];marks=tuple(round(x/90)%4 for x in degrees)
        valid=r['eligible'] and len(marks)==n and all(x!=0 for x in marks) and all(abs((a-b*90+180)%360-180)<=4 for a,b in zip(degrees,marks))
        guess=marks if valid else None
        if guess!=candidate or not valid:candidate=guess;since=t if valid else None;submitted=False
        if last is not None and t-last>.030001:since=t if valid else None
        last=t;ready=valid and t-since>=.25-1e-8
        score=sum(a==b for a,b in zip(guess,secret)) if ready else None
        if ready and not submitted and not won:queries+=1;submitted=True
        if score==n:won=True
        print(json.dumps(dict(display=n if won else score,won=won,queries=queries,since=since)),flush=True)


if __name__=='__main__':
    mode=sys.argv[1]
    if mode=='solver':
        n=int(sys.argv[2]);sys.addaudithook(guard);serve_solver(n)
    elif mode=='evaluator':
        with open(sys.argv[2]) as f:book=json.load(f)
        case=next(c for c in book['cases'] if c['case_id']==sys.argv[3]);secret=tuple(case['symbols'])
        if not secret or any(x not in (1,2,3) for x in secret):raise ValueError('Invalid evaluator code')
        del book,case;sys.addaudithook(guard);serve_evaluator(secret)
    else:raise ValueError('Unknown worker role')
