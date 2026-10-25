"""Generate the four-arm shared-table mosaic environment; no robot task run."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from robofrontier.mosaic import ROOT, scene_directory, specification, create_scene, target_preview, runtime_task
from robofrontier.cube_transport import initialize, add_initial_keyframe
from robofrontier.cube_transport_presentation import annotate, startup_context
from robofrontier.sim import RobotIK


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--reference',type=Path)
    parser.add_argument('--size',type=int,choices=(6,7,8,10),default=10)
    parser.add_argument('--recessed',action='store_true',help='Separate variant with 50 mm physical pockets, 1 mm walls and depth one third of a cube')
    args=parser.parse_args()
    directory=scene_directory(args.size,args.recessed);directory.mkdir(parents=True,exist_ok=True)
    env,task=specification(args.size,args.recessed);root=create_scene(directory,env)
    if args.reference:shutil.copy2(args.reference,directory/'assets/reference-original.png')
    scene=directory/'scene.xml';ET.indent(root);ET.ElementTree(root).write(scene,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(scene));data=initialize(model,env)
    add_initial_keyframe(root,model,data);ET.indent(root);ET.ElementTree(root).write(scene,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    initial=np.array([data.body(o['id']).xpos.copy() for o in env['objects']])
    penetration=[{'geom1':model.geom(c.geom1).name,'geom2':model.geom(c.geom2).name,'depth_m':float(-c.dist)} for c in data.contact if c.dist < -.001]
    report={'scope':'environment initialization and point IK only; no grasp, dense insertion, trajectory or multiarm collision-free proof',
            'mujoco_version':mujoco.__version__,'robot_count':4,'free_cubes':len(env['objects']),
            'actuators':model.nu,'cameras':model.ncam,'initial_penetrations_over_1mm':penetration}
    print('Checking 1 s passive physics',flush=True)
    for _ in range(500):mujoco.mj_step(model,data)
    mujoco.mj_forward(model,data)
    drift=float(np.max(np.linalg.norm(np.array([data.body(o['id']).xpos for o in env['objects']])-initial,axis=1)))
    report['max_cube_drift_after_1s_m']=drift
    report['max_home_tcp_error_m']=max(float(np.linalg.norm(data.site(s['id']+'_tcp').xpos-s['home_tcp_m'])) for s in env['stations'])
    reach={};coverage={f"{g['row']},{g['column']}":[] for g in task['goal_cells']}
    for station in env['stations']:
        arm=station['id'];ik=RobotIK(model,arm+'_');seed=data.qpos[ik.qids].copy();_,rotation=ik.pose(seed)
        reachable=[];source_failures=[]
        for goal in task['goal_cells']:
            q,err=ik.solve(np.array(goal['cube_center_world_m']),rotation,seed,160)
            _,r=ik.pose(q);angle=float(np.arccos(np.clip((np.trace(rotation.T@r)-1)/2,-1,1)))
            if err<.002 and angle<.02:
                key=f"{goal['row']},{goal['column']}";reachable.append(key);coverage[key].append(arm)
        for position in station['source_positions_m']:
            q,err=ik.solve(np.array(position),rotation,seed,180)
            _,r=ik.pose(q);angle=float(np.arccos(np.clip((np.trace(rotation.T@r)-1)/2,-1,1)))
            if err>.002 or angle>.02:source_failures.append({'position_m':position,'error_m':err,'rotation_error_rad':angle})
        reach[arm]={'point_reachable_grid_cells':len(reachable),'supply_point_ik_failures':source_failures}
        print(arm,reach[arm]['point_reachable_grid_cells'],'grid cells;',len(source_failures),'supply IK failures',flush=True)
    report.update(point_reachability=reach,uncovered_grid_cells=[k for k,v in coverage.items() if not v],
                  point_ik_scope='fixed initial down orientation, center endpoint only; not approach/retreat or collision clearance')
    report['passed']=bool(not penetration and drift<.003 and report['max_home_tcp_error_m']<.01 and not report['uncovered_grid_cells'] and not any(v['supply_point_ik_failures'] for v in reach.values()) and np.isfinite(data.qpos).all())
    env.update(scene='scene.xml',initial_keyframe='initial');write(directory/'environment.json',env);write(directory/'task-draft.json',task)
    write(directory/'validation.json',report);target_preview(task,directory/'target-pattern-draft.png')
    context=startup_context(env,model,data)
    for item,station in zip(context['initial_inventory'],env['stations']):item['count']=len(station['source_positions_m'])
    context.update(schema='robofrontier.mosaic.startup.v1',initial_top_image='initial-top.png',adapter_status='shared runtime supports four-arm actions and private evaluation; no complete dense manipulation validation',grid=env['grid'],task_file='task-draft.json')
    write(directory/'startup.json',context)
    with mujoco.Renderer(model,height=900,width=1280) as renderer:
        for camera in ('global','global_side','top','board_top'):
            renderer.update_scene(data,camera=camera);raw=Image.fromarray(renderer.render())
            raw.save(directory/f'initial-{camera}-raw.png');image,metadata=annotate(raw,model,data,env,camera)
            image.save(directory/f'initial-{camera}.png');write(directory/f'initial-{camera}.json',metadata)
    task_name=('mosaic-four-arm-v1.json' if args.size==10 else f'mosaic-four-arm-{args.size}x{args.size}-v1.json') if not args.recessed else f'mosaic-four-arm-{args.size}x{args.size}-recessed-v1.json'
    write(ROOT/'configs/tasks'/task_name,runtime_task(env,task,hashlib.sha256(scene.read_bytes()).hexdigest()))
    write(directory/'manifest.json',{'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'task_sha256':hashlib.sha256((directory/'task-draft.json').read_bytes()).hexdigest(),'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ('robofrontier/mosaic.py','scripts/build_mosaic.py','robofrontier/cube_transport.py')}})
    print(json.dumps(report,ensure_ascii=False),flush=True)
    if not report['passed']:raise SystemExit('Environment checks failed; see validation.json')

if __name__=='__main__':main()
