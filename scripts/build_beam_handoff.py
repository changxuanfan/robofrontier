"""Build and inspect the eight-arm environment only; no handoff execution."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from robofrontier.beam_handoff import DIRECTORY, specification, create_scene
from robofrontier.cube_transport import initialize, add_initial_keyframe
from robofrontier.cube_transport_presentation import annotate


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=DIRECTORY)
    args = parser.parse_args(); directory = args.directory.resolve()
    parameters = json.loads((directory/'parameters.json').read_text())
    env, task = specification(parameters)
    root = create_scene(directory,parameters,env); path = directory/'scene.xml'
    ET.indent(root); ET.ElementTree(root).write(path,encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(path)); data = initialize(model,env)
    # Passive settling is an initialization check, never a grasp trial.
    initial = data.body('timber').xpos.copy()
    for _ in range(round(2/model.opt.timestep)):
        mujoco.mj_step(model,data)
    mujoco.mj_forward(model,data)
    penetrations = [dict(geom1=model.geom(c.geom1).name,geom2=model.geom(c.geom2).name,depth_m=float(-c.dist))
                    for c in data.contact if c.dist < -.001]
    arm_ids = {s['id'] for s in env['stations']}
    def robot_for_geom(geom):
        # Upstream collision geoms can be unnamed; bodies carry the namespace.
        body_name = model.body(model.geom_bodyid[geom]).name
        prefix = body_name.partition('_')[0]
        return prefix if prefix in arm_ids else None
    robot_contacts = [dict(body1=model.body(model.geom_bodyid[c.geom1]).name,
                           body2=model.body(model.geom_bodyid[c.geom2]).name,distance_m=float(c.dist))
                      for c in data.contact
                      if robot_for_geom(c.geom1) and robot_for_geom(c.geom2)
                      and robot_for_geom(c.geom1) != robot_for_geom(c.geom2)]
    drift = float(np.linalg.norm(data.body('timber').xpos-initial))
    home_error = max(float(np.linalg.norm(data.site(s['id']+'_tcp').xpos-s['home_tcp_m'])) for s in env['stations'])
    data.time = 0
    add_initial_keyframe(root,model,data); ET.indent(root); ET.ElementTree(root).write(path,encoding='unicode')
    report = dict(scope='Static initial geometry, home IK, two-second passive stability and rendering only. No lift, transfer or collision-free trajectory has been demonstrated.',
                  mujoco_version=mujoco.__version__,robot_count=8,actuators=model.nu,
                  cameras=model.ncam,free_bodies=1,beam_mass_kg=env['beam']['mass_kg'],
                  timber_drift_during_2s_settle_m=drift,max_home_tcp_error_m=home_error,
                  penetrations_over_1mm=penetrations,inter_robot_initial_contacts=robot_contacts,
                  finite_state=bool(np.isfinite(data.qpos).all()),
                  transport_feasibility_tested=False,llm_used=False)
    report['passed'] = bool(not penetrations and not robot_contacts and drift < .003
                            and home_error < .01 and report['finite_state'])
    write(directory/'environment.json',env); write(directory/'task-draft.json',task)
    write(directory/'validation.json',report)
    write(directory/'startup.json',dict(environment=env['id'],pairs=env['pairs'],
        coordinate_frame='+X right / +Y north / +Z up; top view north-up',
        cameras=[dict(id=model.camera(i).name,position_world_m=data.cam_xpos[i].tolist(),
                      rotation_camera_to_world_row_major=data.cam_xmat[i].tolist(),
                      fovy_deg=float(model.cam_fovy[i])) for i in range(model.ncam)],
        policy_status='Preview only; no sessions or robot action bridge created.'))
    with mujoco.Renderer(model,height=900,width=1280) as renderer:
        for camera in ('global','top','global_side','pair_1'):
            renderer.update_scene(data,camera=camera)
            raw = Image.fromarray(renderer.render()); raw.save(directory/f'preview-{camera}-raw.png')
            image, labels = annotate(raw,model,data,env,camera)
            image.save(directory/f'preview-{camera}.png'); write(directory/f'preview-{camera}.json',labels)
    write(directory/'manifest.json',dict(scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        parameters_sha256=hashlib.sha256((directory/'parameters.json').read_bytes()).hexdigest(),
        scope='Generated environment preview, not an experiment result'))
    from robofrontier.beam_task import make_beam_task
    write(directory/'task.json',make_beam_task(directory))
    print(json.dumps(report,ensure_ascii=False),flush=True)
    if not report['passed']:
        raise SystemExit('Initial environment check failed; see validation.json')


if __name__ == '__main__':
    main()
