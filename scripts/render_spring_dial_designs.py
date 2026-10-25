"""Three cosmetic designs rendered by MuJoCo; no manipulation or policy trial."""
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.build_spring_dial import build
from robofrontier.spring_dial import DESIGNS, DIRECTORY
from robofrontier.scene import ROOT


def physical_signature(model):
    """Name-based comparison ignores noncontact visual additions and materials."""
    counters = {}; colliders = {}
    for i in range(model.ngeom):
        if not (model.geom_contype[i] or model.geom_conaffinity[i]): continue
        body = model.body(model.geom_bodyid[i]).name
        ordinal = counters.get(body, 0); counters[body] = ordinal+1
        key = model.geom(i).name or f'{body}:collision-{ordinal}'
        colliders[key] = dict(body=body, type=int(model.geom_type[i]),
            pos=model.geom_pos[i].tolist(), quat=model.geom_quat[i].tolist(),
            size=model.geom_size[i].tolist(), friction=model.geom_friction[i].tolist())
    return {
        'body_names': [model.body(i).name for i in range(model.nbody)],
        **{name: getattr(model, name).tolist() for name in (
            'body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_iquat',
            'jnt_type', 'jnt_axis', 'jnt_range', 'jnt_stiffness', 'dof_damping',
            'actuator_gainprm', 'actuator_biasprm', 'actuator_ctrlrange', 'actuator_forcerange',
            'key_qpos', 'key_ctrl')},
        'collision_geometry': colliders,
        'gravity': model.opt.gravity.tolist(),
        'timestep': float(model.opt.timestep), 'noslip_iterations': int(model.opt.noslip_iterations),
    }


def main():
    output = ROOT/'runs/spring-dial-designs-20261023'
    output.mkdir(parents=True, exist_ok=True)
    signatures = []; thumbnails = []
    font_path = '/System/Library/Fonts/STHeiti Medium.ttc'
    font = ImageFont.truetype(font_path, 36)
    for index, (style, details) in enumerate(DESIGNS.items(), 1):
        directory = output/style
        build(directory, style)
        model = mujoco.MjModel.from_xml_path(str(directory/'scene.xml'))
        signatures.append(physical_signature(model))
        # The overview shows table/arms; the board view preserves mark legibility.
        card = Image.new('RGB', (1280, 1012), (249, 247, 242))
        card.paste(Image.open(directory/'preview-board.png'), (0, 112))
        draw = ImageDraw.Draw(card)
        draw.text((48, 28), f'{index:02d}  {details["title"]}', font=font, fill=(75, 57, 43))
        draw.text((48, 77), '棕色桌面 · 红色旋钮 · 橙色指针',
                  font=ImageFont.truetype(font_path, 22), fill=(130, 113, 97))
        card.save(output/f'design-{index:02d}.png')
        thumbnails.append(card.resize((640, 506), Image.Resampling.LANCZOS))
    equal = all(s == signatures[0] for s in signatures[1:])
    comparison = Image.new('RGB', (1920, 506), (249, 247, 242))
    for i, thumb in enumerate(thumbnails): comparison.paste(thumb, (i*640, 0))
    comparison.save(output/'comparison.png')
    (output/'validation.json').write_text(json.dumps(dict(
        cosmetic_designs=list(DESIGNS), identical_physical_models=equal,
        no_llm=True, no_manipulation=True, scope='Rendering and initial-state verification; physical model comparison excludes noncontact decoration.'), indent=2)+'\n')
    if not equal: raise RuntimeError('Designs unexpectedly changed physical models')
    # Preserve the selected default rather than silently switching its design.
    build(DIRECTORY)
    print(str(output/'comparison.png'), flush=True)


if __name__ == '__main__':
    main()
