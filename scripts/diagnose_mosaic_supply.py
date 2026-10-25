"""Read-only kinematics on an archived state; never steps a live world."""
import argparse
from copy import copy
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from robofrontier.robot_control import PandaArm
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite import placement_oracle as helpers
from robofrontier.suite.mosaic_oracle_v2 import pickup_endpoint_candidates,goal_endpoint_candidates,assign_supply,buffer_graph


def inspect(source,objects=(),nominal=False):
    source=Path(source).resolve();t=Trial.__new__(Trial)
    t.model=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));t.data=mujoco.MjData(t.model)
    if nominal:mujoco.mj_resetDataKeyframe(t.model,t.data,t.model.key('initial').id)
    else:
        states=np.load(source/'event-states.npz');t.data.qpos[:]=states['qpos'][-1]
    mujoco.mj_forward(t.model,t.data)
    t.task=json.loads((source/'task.json').read_text());t.env=json.loads((source/'environment.json').read_text())
    t.data.ctrl[:]=t.model.key_ctrl[t.model.key('initial').id]
    initial_actions=[] if nominal else json.loads((source/'actions.json').read_text())
    if initial_actions and initial_actions[0]['label']=='clear-idle-arms':
        for robot,command in initial_actions[0]['commands'].items():
            for index,value in enumerate(command[1]['joint_rad'],1):t.data.ctrl[t.model.actuator(f'{robot}_actuator{index}').id]=value
    t.arms={r:PandaArm(t.model,t.data,r) for r in t.task['arms']};t.held={};t.carry_geoms={};t.hand_geoms={}
    initial=t.model.key_qpos[t.model.key('initial').id]
    t.home_q={r:initial[a.qids].copy() for r,a in t.arms.items()}
    t.screen_model=copy(t.model);t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
    t.screen_data=mujoco.MjData(t.screen_model);t.monitor=Monitor(t)
    began=time.monotonic();t.stop_requested=None
    def forbidden(*args,**kwargs):raise RuntimeError('Read-only archived-state inspection cannot execute or step')
    t.step=t.execute=t.wait=forbidden
    t.event=lambda kind,**fields:print(json.dumps(dict(event=kind,**fields)),flush=True)
    if nominal:
        parking={}
        for robot,arm in t.arms.items():
            point=t.data.site_xpos[arm.site].copy();point[2]=t.task['table_height_m']+.68
            parking[robot]=t.screen_ik(robot,point,t.data.site_xmat[arm.site].reshape(3,3),seed=t.home_q[robot])
            if parking[robot] is None:raise ValueError('No nominal parking candidate '+robot)
        for robot,q in parking.items():
            t.data.qpos[t.arms[robot].qids]=q;t.arms[robot].qref=q.copy()
        mujoco.mj_forward(t.model,t.data)
    pickups=pickup_endpoint_candidates(t)
    coverage=goal_endpoint_candidates(t) if nominal else json.loads((source/'coverage.json').read_text())['goals']
    neighbors,edges=helpers.graph(t.env)
    print(json.dumps(dict(scope='Read-only proposed/archived-state endpoint diagnosis; no commands, physics integration or task success',
        empty_pickup_candidates=[name for name,robots in pickups.items() if not robots],
        neighbor_graph=neighbors,pickup_robot_counts={r:sum(r in a for a in pickups.values()) for r in t.arms},
        wall_s=time.monotonic()-began)),flush=True)
    matches=[]
    try:matches=assign_supply(t,t.task['goals'],coverage,pickups,neighbors);print('DIRECT MATCH EXISTS')
    except ValueError as error:
        print('DIRECT',str(error),flush=True)
        neighbors,edges=buffer_graph(t,neighbors,edges)
        try:matches=assign_supply(t,t.task['goals'],coverage,pickups,neighbors);print('BUFFER MATCH EXISTS')
        except ValueError as error:print('BUFFER',str(error),flush=True)
    print('FINAL NEIGHBORS',json.dumps(neighbors),flush=True)
    print('SELECTED',json.dumps([dict(goal=g['goal_id'],object=name,path=path) for g,name,path in matches if name in objects]),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--object',action='append',default=[])
    p.add_argument('--nominal',action='store_true')
    a=p.parse_args();inspect(a.source,a.object,a.nominal)
