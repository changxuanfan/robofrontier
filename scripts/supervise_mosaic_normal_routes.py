"""Continue known frozen NoHover errors, with opt-in small flat pose repairs.

This external owner helper never changes a live executor. Any other error,
failed action, partial route, violation or identity change requires new review.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def select(status, observation):
    if status['phase']!='review':raise ValueError('Require current owner review')
    if (status['world_id']!=observation['world_id'] or status['review_id']!=observation['review_id']):
        raise ValueError('Review observation identity differs')
    if observation['evaluation']['violations']:raise ValueError('Retained violation needs owner review')
    error=status['review_history'][-1]['error']
    match=re.fullmatch(r'RuntimeError: No reachable loaded mosaic hover ([A-Z])/([A-Z]_stock\d+)',error)
    if match is None:raise ValueError('New error class needs owner review: '+error)
    robot,obj=match.groups();job=observation['job']
    if not job or job['object']!=obj or observation['held'].get(robot)!=obj:
        raise ValueError('Actual custody and original job differ')
    if any(r['status']!='completed' for a in observation['last_actions'] for r in a['results'].values()):
        raise ValueError('Original native feedback needs owner review')
    return job['goal_id'],obj


def select_flat_pose(status,observation):
    if (status['phase']!='review' or status['world_id']!=observation['world_id'] or
            status['review_id']!=observation['review_id'] or observation['evaluation']['violations']):
        raise ValueError('Flat correction needs current compliant owner review')
    match=re.fullmatch(r'RuntimeError: No pickup pose ([A-Z])/([A-Z]_stock\d+)',status['review_history'][-1]['error'])
    if match is None:raise ValueError('Not the known flat-pose admission error')
    robot,obj=match.groups();job=observation['job']
    if not job or job['object']!=obj or observation['held'] or any(a['grasp_force_setpoint_n'] is not None for a in observation['robots'].values()):
        raise ValueError('Actual free grippers and original object job must agree')
    if any(r['status']!='completed' for a in observation['last_actions'] for r in a['results'].values()):
        raise ValueError('Native feedback requires owner review')
    # The invoked finite helper also checks actual flat support, correct color,
    # position within5mm and yaw only8..12deg before emitting any command.
    return job['goal_id'],obj,robot


def run(a):
    directory=a.trial.resolve();identity=json.loads((directory/'process.json').read_text())
    if not identity['git_commit'].startswith(a.expected_source_commit):raise ValueError('Unexpected frozen source')
    payload=Path(__file__).read_bytes();archive=directory/'review/engineering/sources'/(hashlib.sha256(payload).hexdigest()+'.py')
    archive.parent.mkdir(parents=True,exist_ok=True);archive.write_bytes(payload)
    world=None;handled={}
    print('SUPERVISING KNOWN NORMAL-PORT RECOVERY',directory,flush=True)
    while True:
        status=json.loads((directory/'runner-status.json').read_text())
        if world is None:world=status['world_id']
        if status['world_id']!=world:raise ValueError('Live physical owner identity changed')
        if status['phase'] not in ('running','review'):
            print('OWNER TERMINAL',status['phase'],flush=True);return
        os.kill(identity['pid'],0)
        if status['phase']=='running':time.sleep(2);continue
        o=json.loads((directory/'review/observation.json').read_text())
        if o['review_id']!=status['review_id']:time.sleep(.2);continue
        epoch=status['review_id']
        if epoch in handled:
            ident=handled[epoch]
            result=directory/'review/results'/f'{ident}-resume.json'
            if result.exists() and json.loads(result.read_text())['status'] not in ('running','completed'):
                raise ValueError('Resume needs owner review: '+result.read_text())
            time.sleep(.2);continue
        flat=bool(a.flat_pose_corrections and re.fullmatch(r'RuntimeError: No pickup pose ([A-Z])/([A-Z]_stock\d+)',status['review_history'][-1]['error']))
        if flat:goal,obj,robot=select_flat_pose(status,o)
        else:goal,obj=select(status,o)
        ident=f"known-{'pose' if flat else 'route'}-supervisor-{epoch:03d}"
        requests=directory/'review/requests'
        for f in requests.glob('*.json'):
            r=json.loads(f.read_text())
            if r.get('review_id')!=epoch:continue
            result=directory/'review/results'/f"{r['id']}.json"
            if not result.exists() or json.loads(result.read_text())['status']=='running':
                raise ValueError('Another request owns this review; no concurrent recovery submission')
        command=[sys.executable,str(Path(__file__).with_name('execute_mosaic_flat_pose_correction.py' if flat else 'execute_mosaic_repair_route.py')),
            '--trial',str(directory),'--goal',goal,'--object',obj,'--id',ident,'--resume']
        command+=['--robot',robot] if flat else ['--buffers-source',str(a.buffers_source.resolve())]
        subprocess.run(command,check=True)
        handled[epoch]=ident


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',required=True,type=Path)
    p.add_argument('--buffers-source',required=True,type=Path);p.add_argument('--expected-source-commit',required=True)
    p.add_argument('--flat-pose-corrections',action='store_true')
    run(p.parse_args())
