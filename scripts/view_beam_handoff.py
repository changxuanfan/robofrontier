"""Native MuJoCo scene preview. F6 camera; F7 reset. No task execution."""
import argparse
from queue import SimpleQueue
import time

import glfw
import mujoco
import mujoco.viewer

from robofrontier.beam_handoff import DIRECTORY, PREVIEW_CAMERAS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera',choices=PREVIEW_CAMERAS,default='global')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(DIRECTORY/'scene.xml')); data = mujoco.MjData(model)
    key = model.key('initial').id; index = PREVIEW_CAMERAS.index(args.camera)
    mujoco.mj_resetDataKeyframe(model,data,key); mujoco.mj_forward(model,data)
    events = SimpleQueue()
    print('BEAM HANDOFF PREVIEW | 8 arms, 4 facing pairs | F6 camera, F7 reset | no Agent or motion commands',flush=True)
    with mujoco.viewer.launch_passive(model,data,key_callback=events.put) as viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera(PREVIEW_CAMERAS[index]).id
        while viewer.is_running():
            tick = time.monotonic()
            while not events.empty():
                pressed = events.get()
                if pressed == glfw.KEY_F7:
                    with viewer.lock():
                        mujoco.mj_resetDataKeyframe(model,data,key); mujoco.mj_forward(model,data)
                    print('Initial scene restored.',flush=True)
                elif pressed == glfw.KEY_F6:
                    index = (index+1)%len(PREVIEW_CAMERAS)
                    with viewer.lock():
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = model.camera(PREVIEW_CAMERAS[index]).id
                        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
                    print('Camera: '+PREVIEW_CAMERAS[index],flush=True)
            for _ in range(10):
                mujoco.mj_step(model,data)
            viewer.sync()
            time.sleep(max(0,10*model.opt.timestep-(time.monotonic()-tick)))


if __name__ == '__main__':
    main()
