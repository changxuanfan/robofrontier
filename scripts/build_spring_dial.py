"""Generate four passive spring dials and four Panda arms; no policy run."""
import json
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from robofrontier.cube_transport import initialize, add_initial_keyframe
from robofrontier.scene import ROOT
from robofrontier.spring_dial import DIRECTORY, PARAMETERS, DESIGNS, specification, create_scene, DialFeedback, update_display


def build(directory, design=None):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    p = json.loads(json.dumps(PARAMETERS))
    p['appearance']['design'] = design or PARAMETERS['appearance']['design']
    env = specification(p); root = create_scene(directory, p, env)
    scene = directory/'scene.xml'
    def save_xml():
        ET.indent(root); ET.ElementTree(root).write(scene, encoding='unicode')
    def write(name, value):
        (directory/name).write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')
    save_xml()
    model = mujoco.MjModel.from_xml_path(str(scene)); data = initialize(model, env)
    for _ in range(round(2/model.opt.timestep)):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    bad_contacts = [dict(a=model.body(model.geom_bodyid[c.geom1]).name,
                         b=model.body(model.geom_bodyid[c.geom2]).name, depth_m=float(-c.dist))
                    for c in data.contact if c.dist < -.001]
    data.time = 0
    add_initial_keyframe(root, model, data); save_xml()
    apparatus = DialFeedback(p['feedback'])
    update_display(model, apparatus.update(0, [0]*4, [0]*4))
    with mujoco.Renderer(model, height=900, width=1280) as renderer:
        for camera in ('board', 'global', 'top', 'dial_A'):
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(directory/f'preview-{camera}.png')
    report = dict(robot_count=4, passive_dial_count=4, actuators=model.nu,
                  dial_actuators=0, initial_penetrations_over_1mm=bad_contacts,
                  finite_state=bool(np.isfinite(data.qpos).all()),
                  initial_dial_angle_rad=[float(data.joint('dial_'+a+'_hinge').qpos[0]) for a in 'ABCD'],
                  scope='Initial geometry and passive settling only; manipulation checked separately.',
                  policy_adapter_available=True, policy_trial_started=False, llm_used=False)
    report['passed'] = not bad_contacts and report['finite_state']
    write('parameters.json', p); write('environment.json', env); write('validation.json', report)
    write('task-draft.json', dict(id=p['id'], status='design draft; task.json is the canonical policy task when built under configs/environments',
          goal='Find and simultaneously hold the four hidden nonzero dial angles.',
          dial_ids=list('ABCD'), allowed_code_angles_deg=[90, 180, 270], candidate_codes=81,
          angle_convention='Top view: 0 north, 90 east, 180 south, 270 west. Clockwise positive; -90 displays 270. Mechanical range -180..+180, with either endpoint displaying 180.',
          feedback=p['feedback'],
          rules=['All dials must be nonzero, within angular tolerance and stable before feedback.',
                 'Only total correct count is displayed, never per-dial correctness.',
                 'Invalid or moving inputs display a dash, distinct from a valid zero score.',
                 'Dial return is passive spring physics; no latching at guessed angles.',
                 'A completed arm command continues holding pose/grasp; models need not repeat HOLD.',
                 'Secret is sampled privately when apparatus starts, absent from scene and public task.',
                 'Each new stable entry counts as a trial; unchanged stable holding does not repeatedly submit.'],
          observation_contract=['RGB including pointers and center score', 'robot proprioception', 'action status'],
          metrics=['success', 'valid code trials', 'robot action calls', 'total absolute dial travel',
                   'wall and simulation time', 'slip/release/reset events'],
          engineering_boundary='No dial-angle command exists. Gripper contact and arm pose/joint commands must produce rotation. Engineering joint perturbation checks are not policy actions or experience.'))
    print(json.dumps(report), flush=True)
    if not report['passed']:
        raise SystemExit('Initial environment failed validation')
    if directory.is_relative_to(ROOT/'configs/environments'):
        from robofrontier.spring_dial_task import make_spring_task
        write('task.json',make_spring_task(directory))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=DIRECTORY)
    parser.add_argument('--design', choices=DESIGNS, default=PARAMETERS['appearance']['design'])
    args = parser.parse_args()
    build(args.directory, args.design)


if __name__ == '__main__':
    main()
