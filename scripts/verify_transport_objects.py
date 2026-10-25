"""Engineering-only known-pose grasp/place fixtures; no policy or live service.

Uses real Panda actuators/contact and the shared robot-only IK. Object truth
is read solely for verification. This is not a model benchmark result.
"""
import json
import math
import argparse
import subprocess

import mujoco
import numpy as np

from robofrontier.cube_transport import ROOT, COLLECTION, layout

from robofrontier.sim import RobotIK


def check(count, arm, record=False):
    env = layout(count)
    model = mujoco.MjModel.from_xml_path(str(ROOT/f"configs/environments/{COLLECTION}/{env['id']}/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id)
    ik = RobotIK(model, arm+'_')
    actuators = [model.actuator(f'{arm}_actuator{i}').id for i in range(1, 8)]
    gripper = model.actuator(arm+'_actuator8').id
    station = next(s for s in env['stations'] if s['id'] == arm)
    obj = next(o for o in env['objects'] if o['id'] == arm+'_item2')
    _, rotation = ik.pose(data.qpos[ik.qids])
    renderer, encoder = None, None
    frames = 0
    if record:
        output = ROOT/f'benchmarks/{COLLECTION}/validation/{env["id"]}-{arm}-pick-place.mp4'
        renderer = mujoco.Renderer(model, height=720, width=960)
        encoder = subprocess.Popen(['/opt/homebrew/bin/ffmpeg', '-y', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '960x720', '-r', '20', '-i', '-',
            '-an', '-c:v', 'libx264', '-crf', '22', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(output)], stdin=subprocess.PIPE)
    def step():
        nonlocal frames
        mujoco.mj_step(model, data)
        if renderer is not None and data.time+1e-9 >= frames/20:
            renderer.update_scene(data, camera=arm+'_side')
            encoder.stdin.write(renderer.render().tobytes())
            frames += 1
    def hold(seconds):
        for _ in range(round(seconds/model.opt.timestep)):
            step()
        mujoco.mj_forward(model, data)
    def move(position):
        start = data.ctrl[actuators].copy()
        q, error = ik.solve(np.asarray(position), rotation, start, 300)
        if error > .002:
            raise RuntimeError('Fixture target unreachable')
        largest = float(np.max(np.abs(q-start)))
        duration = max(1.2, largest*1.875/.7, math.sqrt(6*largest/1.5))
        steps = math.ceil(duration/model.opt.timestep)
        for tick in range(1, steps+1):
            t = tick/steps
            blend = 10*t**3-15*t**4+6*t**5
            data.ctrl[actuators] = start+(q-start)*blend
            step()
        hold(.35)
    def grip(value):
        start = data.ctrl[gripper]
        for tick in range(1, 501):
            data.ctrl[gripper] = start+(value-start)*tick/500
            step()
        hold(.5)
    def fingers_touch():
        object_body = model.body(obj['id']).id
        fingers = [model.body(arm+'_'+name).id for name in ('left_finger', 'right_finger')]
        touched = set()
        for contact in data.contact:
            bodies = [model.geom_bodyid[g] for g in (contact.geom1, contact.geom2)]
            if object_body in bodies:
                touched.update(b for b in bodies if b in fingers)
        return [f in touched for f in fingers]
    hold(1)
    source = np.asarray(obj['initial_position_m']).copy()
    destination = np.asarray(station['goal_positions_m'][1]).copy()
    move([*source[:2], .64]); move(source); grip(0)
    move([*source[:2], .60]); hold(1)
    lifted = data.body(obj['id']).xpos.copy()
    finger_contact = fingers_touch()
    move([*destination[:2], .60]); move(destination); grip(255)
    move([*destination[:2], .64]); move(station['home_tcp_m'])
    samples = []
    for _ in range(100):
        hold(.02)
        body = model.body(obj['id']).id
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        samples.append({'time_s': float(data.time), 'position_m': data.body(body).xpos.copy().tolist(),
                        'linear_speed_m_s': float(np.linalg.norm(velocity[3:])),
                        'up_axis_world': data.body(body).xmat.reshape(3, 3)[:, 2].copy().tolist()})
    final = samples[-1]['position_m']
    # Require the whole final second to satisfy tolerance and velocity, not one frame.
    stable = all(np.linalg.norm(np.asarray(s['position_m'])[:2]-destination[:2]) < .02
                 and abs(s['position_m'][2]-.42) < .01 and s['linear_speed_m_s'] < .01
                 and (obj['shape'] != 'cylinder' or s['up_axis_world'][2] > math.cos(math.radians(15))) for s in samples[-51:])
    obj_id = model.body(obj['id']).id
    table_contact, robot_contact = False, False
    for contact in data.contact:
        geoms = (contact.geom1, contact.geom2)
        if obj_id not in [model.geom_bodyid[g] for g in geoms]:
            continue
        for g in geoms:
            table_contact |= (model.geom(g).name or '').startswith('table_')
            robot_contact |= any((model.body(model.geom_bodyid[g]).name or '').startswith(s['id']+'_link')
                                 or (model.body(model.geom_bodyid[g]).name or '') in (s['id']+'_hand', s['id']+'_left_finger', s['id']+'_right_finger')
                                 for s in env['stations'])
    result = {'environment': env['id'], 'robot': arm, 'shape': obj['shape'],
              'scope': 'known-pose engineering fixture; no policy, no multiarm transport',
              'lifted_position_m': lifted.tolist(), 'bilateral_finger_contact_after_lift': finger_contact,
              'final_position_m': final, 'target_m': station['goal_positions_m'][1], 'tcp_grip_offset_m': 0,
              'final_up_axis_world': samples[-1]['up_axis_world'],
              'final_xy_error_m': float(np.linalg.norm(np.array(final[:2])-destination[:2])),
              'final_second_stable': stable, 'table_supported': bool(table_contact),
              'released': not robot_contact, 'sim_time_s': float(data.time),
              'tail_samples': samples[-51:]}
    result['passed'] = bool(lifted[2] > .50 and all(finger_contact) and stable and table_contact and not robot_contact)
    if encoder is not None:
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError('Fixture video encoding failed')
        renderer.close()
        result['video'] = str(output.relative_to(ROOT))
    print(json.dumps({k: v for k, v in result.items() if k != 'tail_samples'}), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    results = [check(3, 'A', args.record), check(7, 'G', args.record), check(12, 'L', args.record)]
    path = ROOT/f'benchmarks/{COLLECTION}/validation/object-fixtures.json'
    path.write_text(json.dumps(results, indent=2)+'\n')
    if not all(r['passed'] for r in results):
        raise SystemExit('Some fixtures failed; see object-fixtures.json')
