"""Open the four-arm mosaic in native MuJoCo. F6: camera; F7: exact reset.

.venv/bin/mjpython -m scripts.view_mosaic
Viewing only: no Agent, autonomous placement, action bridge or keyboard jogging.
"""
import argparse
from queue import SimpleQueue
import time

import glfw
import mujoco
import mujoco.viewer

from robofrontier.mosaic import DIRECTORY


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    cameras=('global','top','board_top','global_side')
    parser.add_argument('--camera',choices=cameras,default='global')
    args=parser.parse_args()
    model=mujoco.MjModel.from_xml_path(str(DIRECTORY/'scene.xml'));data=mujoco.MjData(model)
    reset_id=model.key('initial').id
    mujoco.mj_resetDataKeyframe(model,data,reset_id);mujoco.mj_forward(model,data)
    events=SimpleQueue();camera_index=cameras.index(args.camera)
    print('MOSAIC: four arms / 100 cubes / 10x10 grid. F6 cycles global/top/board/side; F7 resets. No task execution.',flush=True)
    with mujoco.viewer.launch_passive(model,data,key_callback=events.put) as viewer:
        with viewer.lock():
            viewer.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid=model.camera(cameras[camera_index]).id
        while viewer.is_running():
            started=time.monotonic()
            while not events.empty():
                key=events.get()
                if key == glfw.KEY_F7:
                    with viewer.lock():
                        mujoco.mj_resetDataKeyframe(model,data,reset_id);mujoco.mj_forward(model,data)
                        viewer.opt.label=mujoco.mjtLabel.mjLABEL_NONE
                    print('Initial keyframe restored.',flush=True)
                elif key == glfw.KEY_F6:
                    camera_index=(camera_index+1)%len(cameras)
                    with viewer.lock():
                        viewer.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid=model.camera(cameras[camera_index]).id
                        # F6 is also MuJoCo's frame toggle; retain a clean view.
                        viewer.opt.frame=mujoco.mjtFrame.mjFRAME_NONE
                    print('Camera: '+cameras[camera_index],flush=True)
            for _ in range(10):mujoco.mj_step(model,data)
            viewer.sync()
            time.sleep(max(0,10*model.opt.timestep-(time.monotonic()-started)))

if __name__=='__main__':main()
