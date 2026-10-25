"""Owner-selected same-world recovery via available, previously checked ports.

Choose a short route in the conservative normal-port graph. Angled shortcuts
remain legal task options, but this recovery avoids the known incompatible
current grasp. Every actual grasp/transport/release is checked by the live owner.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

from robofrontier.run_setup import write_json
from robofrontier.suite.placement_oracle import route


def choose(rows,start,destinations,available,blocked=()):
    graph={r:[] for r in available};points={}
    for row in rows:
        pair=row['robots'];key=frozenset(pair)
        if row.get('angled_grasp') or key in blocked or not set(pair)<=set(graph):continue
        a,b=pair;graph[a].append(b);graph[b].append(a);points[key]=row['position_m']
    paths=[]
    for destination in destinations:
        if destination not in graph:continue
        try:paths.append(route(graph,start,destination))
        except ValueError:pass
    if not paths:raise ValueError('No free normal-port recovery route; leave current world for owner review')
    return min(paths,key=lambda p:(len(p),p)),points


def run(a):
    directory=a.trial.resolve();review=directory/'review';o=json.loads((review/'observation.json').read_text())
    if o['evaluation']['violations']:raise ValueError('Retained task violation prevents canonical recovery')
    task=json.loads((directory/'task.json').read_text());target=next(g for g in task['goals'] if g['goal_id']==a.goal)
    obj=a.object
    if obj is None:obj=o['job']['object']
    carrier=next((r for r,name in o['held'].items() if name==obj),None)
    if carrier is None:raise ValueError('Recovery route requires the actual tracked held object')
    if a.goal in o['evaluation']['assignments']:raise ValueError('Select an uncompleted goal')
    if next(i['color'] for i in task['objects'] if i['id']==obj)!=target['requires']['color']:
        raise ValueError('Object and owner-selected goal colors differ')
    if o['objects'][obj]['position_m'][2]<task['table_height_m']+.08:
        raise ValueError('First physically establish vertical payload clearance; no lateral command emitted')
    source=a.buffers_source.resolve();source_task=json.loads((source.parent/'task.json').read_text())
    if source_task['scene_sha256']!=task['scene_sha256']:raise ValueError('Port witness must bind the same scene revision')
    rows=json.loads(source.read_text())['table_buffers'];blocked=[]
    for row in rows:
        point=np.asarray(row['position_m'])
        if any(name!=obj and np.linalg.norm(np.asarray(body['position_m'])[:2]-point[:2])<.08 and abs(body['position_m'][2]-point[2])<.15 for name,body in o['objects'].items()):
            blocked.append(frozenset(row['robots']))
    available=[r for r,arm in o['robots'].items() if r==carrier or (r not in o['held'] and arm['grasp_force_setpoint_n'] is None)]
    coverage=json.loads((directory/'coverage.json').read_text())['goals']
    path,points=choose(rows,carrier,coverage[a.goal],available,blocked)
    evidence=dict(scope='Finite normal-port route, not a global optimum or task permission restriction',
        path=path,object=obj,goal=a.goal,world_id=o['world_id'],review_id=o['review_id'],
        source_ports_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),protected_pairs=dict(o['evaluation']['assignments']))
    write_json(review/'engineering'/f'{a.id}-route.json',evidence)
    code=Path(__file__).read_bytes();code_path=review/'engineering/sources'/(hashlib.sha256(code).hexdigest()+'.py');code_path.parent.mkdir(parents=True,exist_ok=True)
    if not code_path.exists():code_path.write_bytes(code)
    print(json.dumps(evidence),flush=True)
    first=points[frozenset(path[:2])] if len(path)>1 else target['position_m']
    ident=a.id+'-stage';request=review/'requests'/f'route-{ident}.json'
    if request.exists():raise ValueError('Do not rerun a partially executed route; inspect its original request IDs')
    write_json(request,dict(id=ident,world_id=o['world_id'],review_id=o['review_id'],operation='finish_held',robot=carrier,target_m=first))
    while True:
        result=review/'results'/f'{ident}.json'
        if result.exists():
            r=json.loads(result.read_text())
            if r['status']!='running':
                if r['status']!='completed':raise RuntimeError('Known route stage did not complete: '+str(r))
                current=json.loads((review/'observation.json').read_text())
                if current['sim_time_s']>=r['finished_sim_s']-1e-8:break
        if json.loads((directory/'runner-status.json').read_text())['phase']!='review':raise RuntimeError('Owner review ended')
        time.sleep(.1)
    if any(current['evaluation']['assignments'].get(g)!=name for g,name in evidence['protected_pairs'].items()):
        raise RuntimeError('First stage disturbed a completed pairing; no further commands')
    for index,robot in enumerate(path[1:],1):
        command=[sys.executable,str(Path(__file__).with_name('execute_mosaic_repair_transfer.py')),'--trial',str(directory),
            '--robot',robot,'--object',obj,'--id',a.id+f'-leg{index}']
        if index==len(path)-1:command+=['--goal',a.goal]
        else:command+=['--target-m',*map(str,points[frozenset(path[index:index+2])])]
        subprocess.run(command,check=True)
    current=json.loads((review/'observation.json').read_text())
    if current['evaluation']['assignments'].get(a.goal)!=obj:raise RuntimeError('Original evaluator has not accepted the recovery goal')
    if any(current['evaluation']['assignments'].get(g)!=name for g,name in evidence['protected_pairs'].items()):
        raise RuntimeError('Recovery disturbed a completed pairing')
    write_json(review/'engineering'/f'{a.id}-result.json',dict(passed=True,matched=current['evaluation']['matched_goals'],**evidence))
    if a.resume:
        if current['held']:raise ValueError('Other payload still held; leave owner review open')
        write_json(review/'requests'/f'zz-{a.id}-resume.json',dict(id=a.id+'-resume',world_id=current['world_id'],review_id=current['review_id'],operation='resume'))
    print('RECOVERY COMPLETE',a.goal,current['evaluation']['matched_goals'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--buffers-source',required=True,type=Path);p.add_argument('--goal',required=True);p.add_argument('--object')
    p.add_argument('--id',required=True);p.add_argument('--resume',action='store_true');run(p.parse_args())
