"""Small oracle engineering probe, not a policy or a mosaic solution.

Uses the existing robot action adapter in an isolated world. Object truth is
used to measure a grasp offset and outcomes here only, never supplied to policy.
The probe does not certify dense assembly or choose a benchmark placement order.
Developer regression only; not a required per-episode policy calibration. Its
oracle-derived offsets and results are not exported as policy experience.
"""
import argparse
import json
from pathlib import Path
import tempfile

import numpy as np

from robofrontier.pipeline import World
from robofrontier.run_setup import prepare_run,write_json
from robofrontier.task_config import load_harness


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();cfg=load_harness();checks=[];calls=[]
    with tempfile.TemporaryDirectory(prefix='contact-probe-') as temporary:
        run=prepare_run(Path(temporary)/'run',args.task)
        task=json.loads((run/'private/inputs/task.json').read_text());world=World(run,task)
        arm='A';side=task['grid']['cube_side_m'];table=task['grid']['center_world_m'][2]
        objects=[o for o in task['objects'] if o['source']==arm][:2]
        def settle(seconds=1):
            for _ in range(round(seconds/.02)):world.step()
        def position(name):return world.data.body(name).xpos.copy()
        def tcp():return world.arms[arm].observe()['tcp_m']
        def action(op,data):
            cid='contact-'+str(len(calls));command=world.validate_action(arm,op,data,cfg)
            world.start_action(arm,cid,command);start=world.time;result=None
            budget=command.get('timing',{}).get('timeout_sim_s',60)+5
            while world.time-start<budget:
                world.step()
                result=next((r for r in world.poll_actions(15) if r['call_id']==cid),None)
                if result:break
            if result is None:raise AssertionError('Missing terminal robot action')
            calls.append({'op':op,'result':result})
            if result['status']!='completed':raise AssertionError(str(result))
        def move(p,speed=.08):action('move',{'target_m':list(p),'speed_m_s':speed,'quaternion_wxyz':[0,1,0,0]})
        settle()
        for i,obj in enumerate(objects):
            source=np.array(obj['initial_position_m']);target=np.array([-.04+i*side,-.08,table+side/2])
            before=position(objects[0]['id']) if i else None
            action('gripper',{'width_m':side+.01})
            move([source[0],source[1],table+.16]);move([source[0],source[1],table+.8*side],.03)
            action('gripper',{'width_m':0});move([source[0],source[1],table+.16])
            lifted=position(obj['id']);held=bool(lifted[2]>table+side/2+.05)
            if not held:raise AssertionError('Engineering grasp did not retain its object')
            offset=np.array(tcp())-lifted
            move([target[0]+offset[0],target[1]+offset[1],table+.16])
            # Preserve full-finger clearance; this is a probe, not policy advice.
            release=target+offset;release[2]+=side*.9
            move(release,.03);action('gripper',{'width_m':side+.01})
            move([release[0],release[1],table+.20]);move([0,-.34,table+.25]);settle()
            final=position(obj['id']);record={'case':'isolated_release' if i==0 else 'adjacent_release',
                'lift_retained':held,'xy_error_mm':float(np.linalg.norm(final[:2]-target[:2])*1000),
                'z_error_mm':float(abs(final[2]-target[2])*1000)}
            if i:record['neighbor_displacement_mm']=float(np.linalg.norm(position(objects[0]['id'])-before)*1000)
            checks.append(record);print(json.dumps(record),flush=True)
        before=position(objects[0]['id'])
        action('gripper',{'width_m':0})
        start=[before[0],before[1]-1.3*side,table+.65*side]
        move([start[0],start[1],table+.16]);move(start,.02)
        move([start[0],start[1]+.025,start[2]],.01)
        move([start[0],start[1]+.025,table+.20]);move([0,-.34,table+.25]);settle()
        after=position(objects[0]['id']);checks.append({'case':'short_push',
            'object_displacement_mm':float(np.linalg.norm(after[:2]-before[:2])*1000),
            'height_after_m':float(after[2]),'note':'Motion observed, not an alignment or force-control certificate'})
        print(json.dumps(checks[-1]),flush=True)
    report={'scope':'Oracle engineering in an isolated world; no native policy, no full puzzle solution or policy truth feedback',
        'action_adapter_passed':all(c['result']['status']=='completed' for c in calls),
        'dense_precision_certified':False,'checks':checks,'terminal_actions':len(calls)}
    args.output.parent.mkdir(parents=True,exist_ok=True);write_json(args.output,report)


if __name__=='__main__':main()
