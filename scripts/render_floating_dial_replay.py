"""Render saved Oracle states in two views; never advance the physics world."""
import argparse
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.build_floating_spring_dial import DEFAULT_DIRECTORY


def render(directory):
    directory = Path(directory).resolve()
    states = np.load(directory / 'frame-states.npz')
    actions = json.loads((directory / 'actions.json').read_text())
    knob_approach_time = next(a['before']['sim_time_s'] for a in actions
                              if a['label'] == 'A-knob-approach')
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_DIRECTORY / 'scene.xml'))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = .85
    camera.azimuth = -55
    camera.elevation = -12
    model.vis.global_.fovy = 35
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 23, index=1)
    except OSError:
        font = ImageFont.load_default(size=23)
    output = directory / 'oracle-two-arm-turn-detail.mp4'
    process = subprocess.Popen([
        'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
        '-s', '1600x620', '-framerate', '8', '-i', '-', '-an', '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output),
    ], stdin=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=560, width=800) as renderer:
            for i, (qpos, time_s) in enumerate(zip(states['qpos'], states['time_s'])):
                data.qpos[:] = qpos
                mujoco.mj_forward(model, data)
                camera.lookat[:] = data.body('dial_module').xpos
                # Show handle support during pickup, then the pointer during
                # rotation. This camera change affects only saved-state replay.
                camera.azimuth = -55 if time_s < knob_approach_time else 125
                renderer.update_scene(data, camera=camera)
                close = Image.fromarray(renderer.render())
                with Image.open(directory / 'frames' / f'frame_{i:05d}.png') as full:
                    frame = Image.new('RGB', (1600, 620), '#f3f6fa')
                    frame.paste(full, (0, 60))
                frame.paste(close, (800, 60))
                angle = float(np.degrees(data.joint('dial_hinge').qpos[0]))
                draw = ImageDraw.Draw(frame)
                draw.text((20, 16), 'ORACLE  |  FULL SCENE', font=font, fill='#182638')
                detail = 'HANDLE' if time_s < knob_approach_time else 'DIAL'
                draw.text((820, 16), f'{detail} DETAIL  |  t={time_s:.1f}s  |  hinge={angle:+.1f} deg',
                          font=font, fill='#182638')
                process.stdin.write(frame.tobytes())
    finally:
        process.stdin.close()
        code = process.wait()
    if code:
        raise RuntimeError(f'ffmpeg failed with exit code {code}')
    print(json.dumps({'video': str(output), 'frames': len(states['qpos']),
                      'physics_advanced_during_replay': False}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    render(parser.parse_args().directory)
