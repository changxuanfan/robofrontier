"""Open one generated world at its saved initial keyframe. No policy/tools.

macOS: .venv/bin/mjpython -m scripts.view_cube_transport --count 8
Close the viewer to exit; no trial, recorder or sessions are created.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from robofrontier.cube_transport import ROOT, COLLECTION, layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, choices=range(3, 13), default=8)
    parser.add_argument('--camera', choices=('global', 'global_side', 'top'), default='global')
    args = parser.parse_args()
    env = layout(args.count)
    model = mujoco.MjModel.from_xml_path(str(ROOT/f"configs/environments/{COLLECTION}/{env['id']}/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id)
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera(args.camera).id
        while viewer.is_running():
            started = time.monotonic()
            for _ in range(10):
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, 10*model.opt.timestep-(time.monotonic()-started)))


if __name__ == '__main__':
    main()
