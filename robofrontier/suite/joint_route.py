"""Finite private joint-route search; no live physics writes or task choices."""
import numpy as np

def connect_route(start,goal,limits,edge,rng,iterations=500):
    trees=[[np.asarray(start).copy()],[np.asarray(goal).copy()]];parents=[[-1],[-1]]
    def extend(which,target):
        nodes=trees[which];nearest=int(np.argmin([np.linalg.norm(target-q) for q in nodes]));delta=target-nodes[nearest];distance=float(np.linalg.norm(delta))
        if distance<1e-10:return nearest,True
        q=nodes[nearest]+delta*min(1.,.35/distance)
        if not edge(nodes[nearest],q):return None,False
        nodes.append(q);parents[which].append(nearest);return len(nodes)-1,distance<=.35
    def branch(which,index):
        path=[]
        while index>=0:path.append(trees[which][index]);index=parents[which][index]
        return list(reversed(path))
    for iteration in range(iterations):
        a=iteration%2;b=1-a;sample=trees[b][0] if iteration%5==0 else rng.uniform(*np.asarray(limits).T)
        at,_=extend(a,sample)
        if at is None:continue
        for _ in range(80):
            other,reached=extend(b,trees[a][at])
            if other is None:break
            if reached:
                left=branch(a,at);right=branch(b,other);path=left+list(reversed(right[:-1])) if a==0 else right+list(reversed(left[:-1]))
                # Remove unnecessary bends without inventing untested edges.
                smooth=[path[0]];index=0
                while index<len(path)-1:
                    following=next((j for j in range(len(path)-1,index,-1) if edge(path[index],path[j])),index+1)
                    smooth.append(path[following]);index=following
                return smooth
    return None
