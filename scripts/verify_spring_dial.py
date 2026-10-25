"""Engineering contact witness, not an LLM trial or general puzzle solver.

Commands only PandaArm through the existing finite pose/gripper controller.
Dial truth is used privately for engineering measurement, never to drive dials.
"""
import json
import math
import argparse
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from robofrontier.robot_control import PandaArm, matrix_quat
from robofrontier.spring_dial import DIRECTORY, DialFeedback, update_display
from robofrontier.task_config import load_harness
from robofrontier.scene import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'runs/spring-dial-engineering-20261023/attempt-02')
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = json.loads((DIRECTORY/'environment.json').read_text())
    settings = json.loads((DIRECTORY/'parameters.json').read_text())['feedback']
    model = mujoco.MjModel.from_xml_path(str(DIRECTORY/'scene.xml')); data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id); mujoco.mj_forward(model, data)
    arms = {a: PandaArm(model, data, a) for a in 'ABCD'}
    cfg = load_harness(); records = []; contact_violations = []; step_no = 0
    apparatus = DialFeedback(settings, secret=[90, 180, 270, 90])
    qids = [model.jnt_qposadr[model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
    dofs = [model.jnt_dofadr[model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
    def measure():
        return dict(time_s=float(data.time), dial_degrees=np.degrees(data.qpos[qids]).tolist(),
                    widths={a: arm.observe()['gripper_width_m'] for a, arm in arms.items()})
    def step():
        nonlocal step_no
        if step_no % 10 == 0:
            for arm in arms.values(): arm.control(.02)
        mujoco.mj_step(model, data); step_no += 1
        for c in data.contact:
            if c.dist >= -.0005: continue
            b = [model.body(model.geom_bodyid[g]).name for g in (c.geom1, c.geom2)]
            owners = [name.partition('_')[0] if name.partition('_')[0] in arms else None for name in b]
            if all(owners) and owners[0] != owners[1]:
                if len(contact_violations) < 20:
                    contact_violations.append(dict(time_s=float(data.time), bodies=b, depth_m=float(-c.dist)))
        feedback = apparatus.update(data.time, data.qpos[qids], data.qvel[dofs])
        update_display(model, feedback)
        return feedback
    def wait(duration):
        for _ in range(round(duration/model.opt.timestep)): step()
    def execute(label, commands):
        pending = set()
        record = dict(label=label, before=measure(), commands=commands, results={})
        for a, (op, params) in commands.items():
            command = arms[a].validate(op, params, cfg, np.array([-2, -2, .42]), np.array([2, 2, 1.6]))
            arms[a].start(label+'-'+a, command, data.time); pending.add(a)
        for _ in range(round(65/model.opt.timestep)):
            step()
            if step_no % 10: continue
            for a in list(pending):
                result = arms[a].poll(data.time, 45.)
                if result:
                    record['results'][a] = result; pending.remove(a)
            if not pending: break
        record['after'] = measure(); records.append(record)
        (output/'contact-witness.json').write_text(json.dumps(dict(scope=__doc__, actions=records,
            inter_robot_penetrations=contact_violations), indent=2)+'\n')
        print(json.dumps(record), flush=True)
        if pending or any(r['status'] != 'completed' for r in record['results'].values()):
            raise RuntimeError('Engineering action failed: '+label)
    positions = {d['id']: d['center_m'] for d in env['dials']}
    prealign = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.]])
    initial_rotations = {a: prealign@arm.rotation.copy() for a, arm in arms.items()}
    checks = []
    def rotate(label, degrees):
        commands = {}
        for a, value in zip(arms, degrees):
            t = -math.radians(value)
            r = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
            commands[a] = ('move', dict(quaternion_wxyz=matrix_quat(r@initial_rotations[a]).tolist(), angular_speed_rad_s=.4))
        execute(label, commands)
    for desired in (90, -90, 180):
        prefix = str(desired)
        execute(prefix+'-approach', {a: ('move', dict(target_m=[*p[:2], p[2]+.10], speed_m_s=.08,
            quaternion_wxyz=matrix_quat(initial_rotations[a]).tolist())) for a, p in positions.items()})
        execute(prefix+'-descend', {a: ('move', dict(target_m=p, speed_m_s=.04)) for a, p in positions.items()})
        execute(prefix+'-grasp', {a: ('gripper', dict(mode='grasp', width_m=.054, force_n=40, speed_m_s=.03,
                                             epsilon_inner_m=.008, epsilon_outer_m=.008)) for a in arms})
        # Small segments preserve direction. Private measured angle correction
        # demonstrates mechanical feasibility ONLY; it is not policy experience.
        for target in range(45 if desired > 0 else -45, desired+(1 if desired > 0 else -1), 45 if desired > 0 else -45):
            rotate(prefix+'-rotate-'+str(target), [target]*4)
        commanded = np.full(4, float(desired))
        for correction in range(4):
            wait(.5)
            error = desired-np.degrees(data.qpos[qids])
            if np.max(abs(error)) <= 3.: break
            commanded += np.clip(error, -10., 10.)
            rotate(prefix+'-engineering-correction-'+str(correction), commanded)
        wait(.5); held = measure()
        with mujoco.Renderer(model, height=900, width=1280) as renderer:
            for camera in ('board', 'global', 'dial_A'):
                renderer.update_scene(data, camera=camera)
                Image.fromarray(renderer.render()).save(output/f'held-{desired}-{camera}.png')
        execute(prefix+'-release', {a: ('gripper', dict(width_m=.08, speed_m_s=.04)) for a in arms})
        wait(2.); released = measure()
        checks.append(dict(target_degrees=desired, held=held, released=released,
            hold_passed=bool(np.max(abs(np.array(held['dial_degrees'])-desired)) < 4),
            return_passed=bool(np.max(abs(np.asarray(released['dial_degrees']))) < 2)))
        execute(prefix+'-withdraw', {a: ('move', dict(target_m=[*p[:2], p[2]+.10], speed_m_s=.06)) for a, p in positions.items()})
    report = dict(scope=__doc__, llm_used=False, direct_dial_actuation=False,
                  checks=checks, inter_robot_penetrations=contact_violations,
                  passed=bool(all(c['hold_passed'] and c['return_passed'] for c in checks) and not contact_violations),
                  all_angle_transition_feasibility_tested=False,
                  coverage='Four arms together: reset -> grasp -> each allowed angle -> release -> return. Arbitrary held-angle switching not exhaustively tested.',
                  corrections_use_private_dial_measurements=True,
                  policy_must_choose_corrections_from_permitted_observations=True)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
