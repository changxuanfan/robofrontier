"""Known-state engineering checks of robot actions; no mosaic placement or LLM."""
import argparse
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from robofrontier.run_setup import prepare_run
from robofrontier.pipeline import World
from robofrontier.robot_control import matrix_quat, rotation_vector
from robofrontier.task_config import ROOT, load_harness


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    cfg=load_harness();results=[]
    with tempfile.TemporaryDirectory(prefix='mosaic-actions-') as temporary:
        run=Path(temporary)/'run';prepare_run(run,ROOT/'configs/tasks/mosaic-four-arm-v1.json')
        task=json.loads((run/'private/inputs/task.json').read_text());world=World(run,task)
        def submit(arm,op,p):
            command=world.validate_action(arm,op,p,cfg);call='check-'+str(len(results))+'-'+arm
            world.start_action(arm,call,command);return call
        def until(ids,stop_after=None):
            found={};start=world.time;stopped=False
            while len(found)<len(ids) and world.time-start<20:
                world.step()
                if stop_after is not None and world.time-start>=stop_after and not stopped:
                    world.request_stop('A',ids[0]);stopped=True
                for result in world.poll_actions(15):found[result['call_id']]=result
            if set(found)!=set(ids):raise AssertionError('Missing terminal actions')
            return list(found.values())
        for width in (.050,.020,.080):
            ids=[submit('A','gripper',{'width_m':width,'speed_m_s':.04})]
            terminal=until(ids)[0];actual=world.arms['A'].observe()['gripper_width_m']
            value={'check':'aperture','target_width_m':width,'actual_width_m':actual,'terminal':terminal}
            value['passed']=terminal['status']=='completed' and abs(width-actual)<.002
            results.append(value);print(json.dumps(value),flush=True)
        arm=world.arms['A'];position=world.data.site_xpos[arm.site].copy();rotation=world.data.site_xmat[arm.site].reshape(3,3).copy()
        target_r=np.array([[0,-1,0],[1,0,0],[0,0,1]])@rotation
        ids=[submit('A','move',{'target_m':position.tolist(),'quaternion_wxyz':matrix_quat(target_r).tolist(),'angular_speed_rad_s':.5}),
             submit('B','move',{'delta_m':[0,0,.03],'speed_m_s':.04})]
        terminal=until(ids);angle=float(np.linalg.norm(rotation_vector(target_r,world.data.site_xmat[arm.site].reshape(3,3))))
        value={'check':'parallel_rotation_translation','terminal':terminal,'orientation_error_rad':angle,
               'passed':all(r['status']=='completed' for r in terminal) and angle<.02}
        results.append(value);print(json.dumps(value),flush=True)
        q=world.data.qpos[arm.qids].copy();q[[0,6]]+=[.03,-.03]
        terminal=until([submit('A','move',{'joint_rad':q.tolist(),'joint_speed_rad_s':.2})])[0]
        value={'check':'joint_targets','terminal':terminal,'max_error_rad':float(np.max(np.abs(world.data.qpos[arm.qids]-q)))}
        value['passed']=terminal['status']=='completed' and value['max_error_rad']<.004
        results.append(value);print(json.dumps(value),flush=True)
        terminal=until([submit('A','move',{'delta_m':[0,0,.15],'speed_m_s':.05})],stop_after=.4)[0]
        value={'check':'controlled_stop','terminal':terminal,'reference_speed_rad_s':float(np.max(np.abs(arm.velocity))),
               'measured_speed_rad_s':float(np.max(np.abs(world.data.qvel[arm.dofs])))}
        value['passed']=terminal['status']=='failed' and terminal['reason']=='stopped_by_agent' and value['measured_speed_rad_s']<.03
        results.append(value);print(json.dumps(value),flush=True)
        value={'check':'mosaic_not_completed_by_smoke','evaluation':world.evaluate()};value['passed']=not value['evaluation']['all_goals_met'];results.append(value)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'scope':'isolated actual MuJoCo robot action checks; no LLM or mosaic solving', 'checks':results,'passed':all(r['passed'] for r in results)},indent=2)+'\n')
    if not all(r['passed'] for r in results):raise SystemExit('Robot action checks failed')

if __name__=='__main__':main()
