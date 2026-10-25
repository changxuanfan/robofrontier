"""Fixed-camera regression for the observed distant shutter discontinuity."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates

from robofrontier.factory_scene import ROOT
from robofrontier.factory_view import FactoryRenderer


def sample_shutter(renderer, output, label):
    for _ in range(12):
        renderer.render()
    pixels = renderer.render().copy()
    Image.fromarray(pixels).save(output / (label + '.png'))
    camera = renderer.camera
    origin = np.array(camera['position_m'])
    forward = np.array(camera['target_m']) - origin
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    # Sample broad bands between grooves, below the door's windows. Horizontal
    # speckle in these otherwise flat bands was the specific observed defect.
    xx, zz = np.meshgrid(np.linspace(-2, 2, 200), np.arange(.185, 2.40, .13))
    door_y = renderer.manifest['dimensions_m'][1] / 2 - .129
    points = np.column_stack([xx.ravel(), np.full(xx.size, door_y), zz.ravel()]) - origin
    focal = renderer.height / 2 / np.tan(np.deg2rad(camera['vertical_fov_deg'] / 2))
    depth = points @ forward
    u = renderer.width / 2 + focal * (points @ right) / depth
    v = renderer.height / 2 - focal * (points @ up) / depth
    values = map_coordinates(pixels.mean(2), [v, u], order=1).reshape(xx.shape)
    return float(values.std(axis=1).mean())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', type=Path, default=ROOT / 'outputs/robo-factory/scene/scene.xml')
    p.add_argument('--output', type=Path, default=ROOT / 'outputs/robo-factory/reports/render-check')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    renderer = FactoryRenderer(args.scene, width=1600, height=1000)
    try:
        renderer.select('interior_diagonal')
        far = sample_shutter(renderer, args.output, 'far')
        depth = renderer.manifest['dimensions_m'][1]
        renderer.set_camera(dict(position_m=[0, depth / 2 - 7, 2.38],
                                 target_m=[0, depth / 2, 2.38], vertical_fov_deg=45))
        near = sample_shutter(renderer, args.output, 'near')
        report = dict(passed=far < 4 and near < 4, scene_sha256=renderer.scene_hash,
                      row_noise_8bit=dict(far=far, near=near), threshold_8bit=4,
                      scope='Visual proxy for this shutter artifact; not a general realism or aliasing score')
        (args.output / 'render-check.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
        if not report['passed']:
            raise SystemExit(1)
    finally:
        renderer.close()


if __name__ == '__main__':
    main()
