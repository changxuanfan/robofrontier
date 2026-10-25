"""Native physical preview; F6 changes camera, F7 resets the entire preview.

No arm/dial automation, policy sessions or secret display. Mouse perturbation is
an engineering preview aid, never a benchmark action.
"""
import json
import argparse
from pathlib import Path
from queue import SimpleQueue
import time

import glfw
import mujoco
import mujoco.viewer

from robofrontier.spring_dial import DIRECTORY, CAMERAS, DialFeedback, update_display


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=DIRECTORY)
    args = parser.parse_args(); directory = args.directory.resolve()
    model = mujoco.MjModel.from_xml_path(str(directory/'scene.xml'))
    data = mujoco.MjData(model); key = model.key('initial').id
    settings = json.loads((directory/'parameters.json').read_text())['feedback']
    apparatus = DialFeedback(settings)
    mujoco.mj_resetDataKeyframe(model, data, key); mujoco.mj_forward(model, data)
    joints = [model.joint('dial_'+a+'_hinge').id for a in 'ABCD']
    qids = model.jnt_qposadr[joints]; dofs = model.jnt_dofadr[joints]
    events = SimpleQueue(); camera = 0; previous = None
    print('SPRING DIAL PREVIEW | 4 arms / 4 passive dials | F6 camera / F7 full reset | no Agent', flush=True)
    with mujoco.viewer.launch_passive(model, data, key_callback=events.put) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera(CAMERAS[camera]).id
        while viewer.is_running():
            started = time.monotonic()
            while not events.empty():
                event = events.get()
                if event == glfw.KEY_F6:
                    camera = (camera+1) % len(CAMERAS)
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = model.camera(CAMERAS[camera]).id
                elif event == glfw.KEY_F7:
                    mujoco.mj_resetDataKeyframe(model, data, key); mujoco.mj_forward(model, data)
                    apparatus = DialFeedback(settings); previous = None
                    print('Full preview reset; new private code sampled.', flush=True)
            for _ in range(10):
                mujoco.mj_step(model, data)
                feedback = apparatus.update(data.time, data.qpos[qids], data.qvel[dofs])
            update_display(model, feedback)
            if feedback != previous:
                print(json.dumps(feedback), flush=True); previous = feedback
            viewer.sync()
            time.sleep(max(0, 10*model.opt.timestep-(time.monotonic()-started)))


if __name__ == '__main__':
    main()
