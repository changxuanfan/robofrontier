"""Replay closed BEST5 solver transcripts and verify stored motion, no secrets."""
import argparse
import json
from pathlib import Path
import runpy

import mujoco
import numpy as np
from robofrontier.motion_recording import MotionReader
from robofrontier.run_setup import write_json


def audit(directory):
    directory=Path(directory)
    report=json.loads((directory/'report.json').read_text())
    task=json.loads((directory/'task.json').read_text())
    worker=directory/'executor-source/robofrontier/suite/dial_best_worker.py'
    decoder=runpy.run_path(str(worker))['RecordDecoder'](len(task['dials']),5)
    guesses=0;observations=0
    for line in (directory/'solver-io.jsonl').open():
        call=json.loads(line);request=call['input']
        if request['op']=='choose':
            assert set(request)=={'op','current'}
            actual=decoder.choose(request['current']);guesses+=1
        else:
            assert request['op']=='observe' and set(request)=={'op','t','inputs','best'}
            actual=decoder.observe(request['t'],request['inputs'],request['best']);observations+=1
        assert actual==call['output'],'Frozen decoder transcript differs'
    reader=MotionReader(directory/'motion');count=0;last=None
    model=mujoco.MjModel.from_xml_path(str(reader.path/'source/scene.xml'))
    initial=np.asarray(model.key_qpos[model.key('initial').id],dtype='<f4')
    for stamp,qpos in reader.frames():
        if count==0:assert stamp==0 and np.array_equal(initial,qpos)
        assert last is None or stamp>last
        count+=1;last=stamp
    assert count==reader.meta['frames']
    assert reader.meta['physics_timestep_s']==.002
    evaluation=report['evaluation']
    if report['passed']:
        assert evaluation['task_success'] and not evaluation['violations']
        assert not report['unresolved_robot_actions'] and reader.meta['status']=='complete'
    return dict(case_id=report['case_id'],report=str(directory/'report.json'),passed=report['passed'],
        goal_reached=report.get('goal_reached',evaluation.get('solved',False)),
        assisted=report.get('assisted',report.get('llm_used',False)),
        solver_replay_equal=True,private_password_read=False,explicit_guesses=guesses,
        observed_combinations=observations,valid_trials=evaluation['valid_trials'],
        motion_frames_verified=count,last_sim_s=last,violations=evaluation['violations'],
        unresolved_actions=report['unresolved_robot_actions'],wall_s=report['wall_time_s'],
        peak_concurrent_actions=evaluation['peak_concurrent_actions'],
        motion_overlap_sim_s=evaluation['motion_overlap_sim_s'],
        any_motion_sim_s=evaluation['any_motion_sim_s'],
        average_moving_robots_during_motion=(evaluation['robot_motion_s']/evaluation['any_motion_sim_s']
            if evaluation['any_motion_sim_s'] else 0),
        useful_blue_support_robot_s=evaluation['useful_blue_support_robot_s'],
        error=report.get('error'),physical_control_sha256=report.get('physical_control_sha256'))


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
    if (a.directory/'report.json').exists():reports=[a.directory/'report.json']
    else:reports=sorted(a.directory.glob('dial-*/attempt-*/report.json'))
    if not reports:raise ValueError('No closed BEST5 solver runs')
    rows=[audit(report.parent) for report in reports]
    result=dict(scope='Closed BEST5 episodes; frozen solver replay and stored states; no private puzzle read',attempts=rows)
    write_json(a.directory/'audit.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
