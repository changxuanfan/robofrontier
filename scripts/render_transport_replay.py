"""Film recorded real-physics qpos samples from a successful program attempt.

This is a rendering-only replay, not a new simulation or policy. No poses are
invented/interpolated: every video frame selects a saved trajectory sample.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.cube_transport import ROOT, _font
from robofrontier.cube_transport_presentation import annotate
from robofrontier.recording import Video


def render(attempt, playback, allow_failed=False, camera='global', start=None, end=None, label=None):
    report = json.loads((attempt/'report.json').read_text())
    if not report['passed'] and not allow_failed:
        raise ValueError('Refusing to render a failed attempt as a success replay')
    env = json.loads((attempt/'environment.json').read_text())
    collection = report.get("collection", "cube_transport")
    scene = ROOT/f"configs/environments/{collection}/{env['id']}/scene.xml"
    if hashlib.sha256(scene.read_bytes()).hexdigest() != report['scene_sha256']:
        raise ValueError('Scene changed since physical execution')
    states = np.load(attempt/'trajectory.npz')
    times, poses = states['sim_time_s'], states['qpos']
    model = mujoco.MjModel.from_xml_path(str(scene)); data = mujoco.MjData(model)
    model.camera(camera)  # Validate the requested existing camera.
    prefix = 'replay' if report['passed'] else 'failure-replay'
    selection = '' if camera == 'global' and start is None and end is None else f'-{camera}-t{start or 0:g}-{end if end is not None else times[-1]:g}'
    destination = attempt/f'{prefix}{selection}-{playback:g}x.mp4'
    if destination.exists():
        raise ValueError('Replay file already exists')
    video = Video(destination, 1536, 1152, 20)
    with mujoco.Renderer(model, height=900, width=1280) as renderer:
        first = max(times[0], start if start is not None else times[0])
        last = min(times[-1], end if end is not None else times[-1])
        if last <= first:
            raise ValueError('Empty replay interval')
        for t in np.arange(first, last+1e-9, playback/20):
            index = min(int(np.searchsorted(times, t)), len(times)-1)
            data.qpos[:] = poses[index]  # Rendering replay only; execution never writes live qpos.
            data.time = float(times[index])
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            image, _ = annotate(Image.fromarray(renderer.render()), model, data, env, camera)
            draw = ImageDraw.Draw(image); draw.rectangle((0, 0, image.width, 45), fill=(23, 48, 75))
            status = 'PASS' if report['passed'] else ('FAILED RUN' if 'evaluation' not in report else f"FULL RUN {report['evaluation']['matched_goals']}/{len(env['objects'])}")
            heading = label or f'{status} / RECORDED PHYSICS'
            draw.text((16, 8), f'{heading} | {camera} | sim {data.time:.1f}s | {playback:g}x',
                      font=_font(22), fill='white')
            if video.error:
                raise RuntimeError(video.error)
            video.queue.put(np.asarray(image).tobytes(), timeout=30)
    result = video.close()
    result.update(source_attempt_passed=report['passed'], camera=camera,
                  source_start_sim_s=float(first), source_end_sim_s=float(last), playback_speed=playback,
                  source_trajectory_sha256=hashlib.sha256((attempt/'trajectory.npz').read_bytes()).hexdigest())
    destination.with_suffix('.encoding.json').write_text(json.dumps(result, indent=2)+'\n')
    if result['error']:
        raise RuntimeError(result['error'])
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('attempt', type=Path)
    parser.add_argument('--playback', type=float, default=12)
    parser.add_argument('--allow-failed', action='store_true', help='Label a failed attempt explicitly as FAILURE REPLAY')
    parser.add_argument('--camera', default='global')
    parser.add_argument('--start-s', type=float)
    parser.add_argument('--end-s', type=float)
    parser.add_argument('--label')
    args = parser.parse_args()
    render(args.attempt.resolve(), args.playback, args.allow_failed, args.camera, args.start_s, args.end_s, args.label)
