"""Passive spring-dial scene and count-only apparatus; no dial actuator or solver."""
import math
from pathlib import Path
import secrets
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .cube_transport import ROOT, _camera, _font, build_xml, xyz
from .scene import vector

DIRECTORY = ROOT / 'configs/environments/spring_dial/facing-04'
CAMERAS = ('board', 'global', 'top', 'global_side', 'dial_A', 'dial_B', 'dial_C', 'dial_D')
DESIGNS = {
    'warm': dict(title='暖白圆盘', paper=(248, 244, 234), rim=(184, 166, 146),
                 ink=(66, 57, 49), tick_step=30, case=[.71, .67, .61, 1]),
    'silver': dict(title='浅银仪表', paper=(231, 236, 239), rim=(157, 171, 182),
                   ink=(51, 65, 76), tick_step=15, case=[.70, .75, .79, 1]),
    'panel': dict(title='象牙白面板', paper=(249, 247, 240), rim=(217, 212, 200),
                  ink=(0, 0, 0), tick_step=90, case=[.84, .82, .76, 1]),
}
PARAMETERS = {
    'id': 'spring-dial-facing-04',
    'table': {'length_m': 1.90, 'width_m': 2.06, 'height_m': .40, 'thickness_m': .05},
    'base_column_spacing_m': .86, 'opposed_base_spacing_m': 1.72,
    'dial': {'center_y_m': .32, 'face_radius_m': .115,
             'knob_core_radius_m': .024, 'knob_height_m': .044,
             'knob_center_above_table_m': .061, 'mass_kg': .12,
             'spring_nm_per_rad': .03, 'damping_nm_s_per_rad': .003,
             'range_rad': [-math.pi, math.pi], 'friction': [.8, .005, .0001]},
    'physics': {'timestep_s': .002, 'gravity_m_s2': [0, 0, -9.81],
                'contact_friction': [1., .005, .0001], 'noslip_iterations': 0},
    'feedback': {'angle_tolerance_deg': 4., 'max_speed_deg_s': 5.,
                 'stable_duration_s': .25, 'max_sample_gap_s': .03},
    'appearance': {'design': 'panel', 'face_radius_m': .165,
                   'pointer_length_m': .154, 'letter_half_size_m': .04},
}


def specification(p):
    stations, dials = [], []
    height = p['table']['height_m']
    for column in range(2):
        x = (column-.5)*p['base_column_spacing_m']
        for side, sign in (('south', -1), ('north', 1)):
            arm = chr(65+len(stations))
            s = dict(id=arm, side=side, pair=column+1,
                     base_position_m=[x, sign*p['opposed_base_spacing_m']/2, height],
                     yaw_rad=-sign*math.pi/2,
                     source_positions_m=[], goal_positions_m=[], parking_positions_m=[])
            s['home_tcp_m'] = xyz(s, .25, 0, .82)
            stations.append(s)
            dials.append(dict(id=arm, nominal_robot=arm,
                              center_m=[x, sign*p['dial']['center_y_m'],
                                        height+p['dial']['knob_center_above_table_m']]))
    table = p['table']
    return dict(schema='robofrontier.spring_dial.environment.v1', id=p['id'],
                collection='spring_dial', arm_count=4, topology='two_facing_pairs',
                stations=stations, dials=dials, physics=p['physics'],
                tables=[dict(center_m=[0, 0, height-table['thickness_m']/2],
                             half_size_m=[table['length_m']/2, table['width_m']/2,
                                          table['thickness_m']/2])],
                floor_rgba=[.84, .89, .94, 1], table_rgba=[.64, .47, .35, 1],
                edges=[], objects=[], initial_keyframe='initial', scene='scene.xml')


def _texture(asset, directory, name, image):
    image.save(directory/'assets'/f'{name}.png')
    ET.SubElement(asset, 'texture', name=name, type='2d', file=f'assets/{name}.png')
    ET.SubElement(asset, 'material', name=name, texture=name, specular='0', emission='.15')


def _bold_font(size):
    try:
        return ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', size, index=1)
    except OSError:
        return _font(size)


def create_scene(directory, p, env):
    directory = Path(directory)
    root = build_xml(env, directory)
    root.set('model', 'RoboFrontier | Spring Dial Codebreaker | environment preview')
    world, asset = root.find('worldbody'), root.find('asset')
    height = p['table']['height_m']
    appearance = p.get('appearance', PARAMETERS['appearance'])
    style = appearance.get('design', 'panel')
    design = DESIGNS[style]
    for g in list(world.findall('geom')):
        if g.get('name', '').startswith('table_support_'):
            world.remove(g)
    leg_h = height-p['table']['thickness_m']
    for x in (-.82, .82):
        for y in (-.89, .89):
            ET.SubElement(world, 'geom', type='box', pos=vector([x, y, leg_h/2]),
                          size=vector([.04, .04, leg_h/2]), rgba='.25 .31 .36 1')
    ET.SubElement(asset, 'material', name='knob_metal', rgba='.78 .035 .025 1',
                  specular='.28', shininess='.35')
    ET.SubElement(asset, 'mesh', name='dial_pointer_arrow',
                  vertex='-.014 -.012 -.0005 .014 -.012 -.0005 0 .012 -.0005 -.014 -.012 .0005 .014 -.012 .0005 0 .012 .0005',
                  face='0 2 1 3 4 5 0 1 4 0 4 3 1 2 5 1 5 4 2 0 3 2 3 5')
    if style == 'panel':
        # Cosmetic mounting panel only: no contact, no change to the apparatus.
        ET.SubElement(asset, 'material', name='ivory_panel', rgba='.78 .76 .71 1', specular='0')
        # Print identifiers and angle numerals directly on the mounting panel,
        # outside the white dial faces, without adding sixteen separate badges.
        panel_image = Image.new('RGB', (2220, 1830), (199, 194, 181))
        pd = ImageDraw.Draw(panel_image)
        def pixel(x, y):
            return ((x+.74)*1500, (.61-y)*1500)
        radius = p.get('appearance', PARAMETERS['appearance']).get('face_radius_m', .145)
        for dial in env['dials']:
            x, y, _ = dial['center_m']
            for text, dx, dy, anchor in (
                ('0', 0, radius+.017, 'mb'),
                ('1', radius+.020, 0, 'lm'),
                ('2', 0, -radius-.017, 'mt'),
                ('3', -radius-.020, 0, 'rm'),
            ):
                pd.text(pixel(x+dx, y+dy), text, anchor=anchor, font=_bold_font(76), fill=(0, 0, 0))
            pd.text(pixel(x-.20, y+.21), dial['id'], anchor='mm', font=_bold_font(138), fill=(0, 0, 0))
        _texture(asset, directory, 'panel_print', panel_image)
        asset.find("material[@name='panel_print']").set('emission', '0')
        for name, pos, size in (
            ('panel_main', [0, 0, height+.0007], [.74, .61, .0005]),
            ('panel_cross', [0, 0, height+.00065], [.81, .54, .0005]),
        ):
            ET.SubElement(world, 'geom', name=name, type='box', pos=vector(pos), size=vector(size),
                          material='panel_print' if name == 'panel_main' else 'ivory_panel',
                          contype='0', conaffinity='0')
        for i, (x, y) in enumerate(((-.74, -.54), (-.74, .54), (.74, -.54), (.74, .54))):
            ET.SubElement(world, 'geom', name=f'panel_corner_{i}', type='cylinder',
                          pos=vector([x, y, height+.0005]), size='.07 .0005',
                          material='ivory_panel', contype='0', conaffinity='0')
    d = p['dial']
    face_radius = appearance.get('face_radius_m', d['face_radius_m'])
    letter_half = appearance.get('letter_half_size_m', .023)
    pointer_length = appearance.get('pointer_length_m', .054)
    for dial in env['dials']:
        arm = dial['id']; x, y, z = dial['center_m']
        image = Image.new('RGB', (768, 768), design['rim'])
        draw = ImageDraw.Draw(image); cx = cy = 384
        draw.ellipse((8, 8, 760, 760), fill=design['rim'])
        draw.ellipse((23, 23, 745, 745), fill=design['paper'])
        for angle in range(0, 360, design['tick_step']):
            t = math.radians(angle)
            a = 319 if angle % 90 else 297
            draw.line((cx+a*math.sin(t), cy-a*math.cos(t),
                       cx+343*math.sin(t), cy-343*math.cos(t)),
                      fill=design['ink'], width=6 if angle % 90 else 17)
        if style != 'panel':
            label_positions = {0: (480, 134), 90: (634, 464),
                               180: (269, 634), 270: (164, 304)}
            for angle, position in label_positions.items():
                draw.text(position, str(angle//90), anchor='mm', font=_bold_font(104), fill=design['ink'])
        # Cylinder end-cap UVs invert the vertical coordinate relative to boxes.
        _texture(asset, directory, 'face_'+arm, image.transpose(Image.Transpose.FLIP_TOP_BOTTOM))
        ET.SubElement(world, 'geom', name='dial_'+arm+'_base', type='cylinder',
                      pos=vector([x, y, height+.012]), size=vector([d['face_radius_m'], .012]),
                      rgba=vector(design['case']), friction=vector(d['friction']))
        ET.SubElement(world, 'geom', name='dial_'+arm+'_face', type='cylinder',
                      pos=vector([x, y, height+.0243]),
                      size=vector([face_radius*.99, .00025]),
                      material='face_'+arm, contype='0', conaffinity='0')
        badge = Image.new('RGB', (128, 128), design['paper'])
        ImageDraw.Draw(badge).text((64, 65), arm, anchor='mm', font=_bold_font(110), fill=design['ink'])
        _texture(asset, directory, 'station_'+arm, badge)
        if style != 'panel':
            ET.SubElement(world, 'geom', name='station_'+arm, type='box',
                          pos=vector([x, y-face_radius-.055, height+.0006]),
                          size=vector([letter_half, letter_half, .0003]), material='station_'+arm,
                          contype='0', conaffinity='0')
        mount = root.find(f".//geom[@name='{arm}_label']")
        mount.set('material', 'station_'+arm)
        mount.set('size', '.052 .052 .0003')
        body = ET.SubElement(world, 'body', name='dial_'+arm, pos=vector([x, y, z]))
        ET.SubElement(body, 'joint', name='dial_'+arm+'_hinge', type='hinge', axis='0 0 -1',
                      limited='true', range=vector(d['range_rad']), springref='0',
                      stiffness=str(d['spring_nm_per_rad']), damping=str(d['damping_nm_s_per_rad']))
        ET.SubElement(body, 'geom', name='dial_'+arm+'_knob', type='cylinder',
                      size=vector([d['knob_core_radius_m'], d['knob_height_m']/2]),
                      mass=str(d['mass_kg']*.8), material='knob_metal',
                      friction=vector(d['friction']), condim='4')
        for i in range(12):
            t = i*math.pi/6
            ET.SubElement(body, 'geom', name=f'dial_{arm}_tooth_{i}', type='box',
                          pos=vector([.024*math.cos(t), .024*math.sin(t), 0]),
                          size='.003 .004 .022', euler=vector([0, 0, t]),
                          mass=str(d['mass_kg']*.2/12), material='knob_metal',
                          friction=vector(d['friction']), condim='4')
        # The indicator is an optical marking, not a protruding collision blade.
        shaft_length=pointer_length-.024
        ET.SubElement(body, 'geom', name='dial_'+arm+'_pointer', type='box',
                      pos=vector([0, shaft_length/2, d['knob_height_m']/2+.0006]),
                      size=vector([.0045, shaft_length/2, .0005]), rgba='1 .30 .035 1', mass='0',
                      contype='0', conaffinity='0')
        ET.SubElement(body, 'geom', name='dial_'+arm+'_arrowhead', type='mesh', mesh='dial_pointer_arrow',
                      pos=vector([0,pointer_length-.012,d['knob_height_m']/2+.0006]),
                      rgba='1 .30 .035 1',mass='0',
                      contype='0', conaffinity='0')
        ET.SubElement(body, 'site', name='dial_'+arm+'_axis', pos='0 0 0', size='.002', rgba='0 0 0 0')
        _camera(world, 'dial_'+arm, [x+.28, y-.30, 1.04], np.array([x, y, z]), 43)
    # World-space seven-segment score; color changes are visual only.
    panel = [v/255 for v in design['paper']]+[1]
    ET.SubElement(world, 'geom', name='score_base', type='box', pos=vector([0, 0, height+.018]),
                  size='.17 .18 .018', rgba=vector(panel))
    title = Image.new('RGB', (768, 160), design['paper'])
    ImageDraw.Draw(title).text((384, 80), 'CORRECT / 4', anchor='mm', font=_bold_font(104), fill=design['ink'])
    _texture(asset, directory, 'score_title', title)
    ET.SubElement(world, 'geom', name='score_title', type='box',
                  pos=vector([0, .131, height+.0364]), size='.153 .032 .0003',
                  material='score_title', contype='0', conaffinity='0')
    positions = [(0, .055, .036, .005), (.041, .028, .005, .023),
                 (.041, -.028, .005, .023), (0, -.055, .036, .005),
                 (-.041, -.028, .005, .023), (-.041, .028, .005, .023), (0, 0, .036, .005)]
    for i, (x, y, hx, hy) in enumerate(positions):
        ET.SubElement(world, 'geom', name=f'score_segment_{i}', type='box',
                      pos=vector([x*1.2, y*1.2, height+.037]), size=vector([hx*1.2, hy*1.2, .0004]),
                      rgba=vector(panel), contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='score_ready', type='sphere',
                  pos=vector([-.10, -.11, height+.038]), size='.009',
                  rgba='0 0 0 0', contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='score_win', type='sphere',
                  pos=vector([.10, -.11, height+.038]), size='.009',
                  rgba='0 0 0 0', contype='0', conaffinity='0')
    for c in list(world.findall('camera')):
        if c.get('name') in ('global', 'global_side', 'top'):
            world.remove(c)
    _camera(world, 'global', [.9, -1.7, 3.65], np.array([0, 0, .48]), 43)
    _camera(world, 'global_side', [-2.3, -2.6, 2.4], np.array([0, 0, .49]), 46)
    ET.SubElement(world, 'camera', name='top', pos='0 0 3.4', xyaxes='1 0 0 0 1 0', fovy='47')
    ET.SubElement(world, 'camera', name='board', pos='0 0 2.13', xyaxes='1 0 0 0 1 0', fovy='44')
    root.find('statistic').attrib.update(center='0 0 .5', extent='3')
    return root


class DialFeedback:
    """Private apparatus: only aggregate count is public, never individual matches.

    Clockwise angles are positive. -90 deg displays 270; either +/-180 displays
    180. Valid stable entry counts once; changing/leaving and re-entering counts
    another physical trial. No query is generated at zero or during transit.
    """
    def __init__(self, settings, secret=None):
        self.settings = settings
        self._secret = tuple(secret if secret is not None else
                             (secrets.choice((90, 180, 270)) for _ in range(4)))
        if len(self._secret) != 4 or any(a not in (90, 180, 270) for a in self._secret):
            raise ValueError('Four nonzero code angles required')
        self._candidate = None; self._since = None; self._submitted = False; self._last_time = None
        self.queries = 0; self.won = False
        self.first_win_time_s = None; self.sampling_gaps = 0; self.new_trial = None

    def update(self, time_s, angles_rad, speeds_rad_s, *, eligible=True):
        angles = np.asarray(angles_rad, dtype=float)
        speeds = np.asarray(speeds_rad_s, dtype=float)
        if angles.shape != (4,) or speeds.shape != (4,) or not np.isfinite(np.r_[angles, speeds, time_s]).all():
            raise ValueError('Four finite measured angles and velocities required')
        if self._last_time is not None and time_s < self._last_time:
            raise ValueError('Feedback clock must be monotonic; reset apparatus with the world')
        gap = self._last_time is not None and time_s-self._last_time > self.settings['max_sample_gap_s']
        self._last_time = time_s
        self.new_trial = None
        if gap: self.sampling_gaps += 1
        degrees = np.degrees(angles) % 360
        marks = (np.rint(degrees/90).astype(int) % 4)*90
        error = abs((degrees-marks+180) % 360-180)
        valid = bool(eligible and np.all(marks != 0) and np.all(error <= self.settings['angle_tolerance_deg'])
                     and np.all(abs(np.degrees(speeds)) <= self.settings['max_speed_deg_s']))
        candidate = tuple(int(x) for x in marks) if valid else None
        if candidate != self._candidate or not valid:
            self._candidate = candidate; self._since = time_s if valid else None; self._submitted = False
        elif gap:
            # Re-establish stability, without inventing an unseen new attempt.
            self._since = time_s
        ready = valid and time_s-self._since >= self.settings['stable_duration_s']-1e-9
        score = None
        if ready:
            score = sum(a == b for a, b in zip(candidate, self._secret))
            if not self._submitted and not self.won:
                self.queries += 1; self._submitted = True
                self.new_trial = dict(index=self.queries, time_s=float(time_s),
                                      input_degrees=list(candidate), input_symbols=[x//90 for x in candidate],
                                      correct_count=score)
            if score == 4 and not self.won:
                self.first_win_time_s = float(time_s)
            self.won = self.won or score == 4
        return dict(ready=bool(ready), correct_count=score, won=self.won, valid_trials=self.queries)


def update_display(model, feedback):
    digits = {0: 'abcdef', 1: 'bc', 2: 'abged', 3: 'abgcd', 4: 'fgbc'}
    # Latch visible game success, independently of the still-passive mechanism.
    active = digits[4] if feedback['won'] else digits[feedback['correct_count']] if feedback['ready'] else 'g'
    background = model.geom_rgba[model.geom('score_base').id].copy()
    for i, segment in enumerate('abcdefg'):
        model.geom_rgba[model.geom(f'score_segment_{i}').id] = (
            [.61, .10, .055, 1] if segment in active else [*background[:3], 0])
    # Success is already the visible score 4; avoid redundant decorative lamps.
    model.geom_rgba[model.geom('score_ready').id] = [0, 0, 0, 0]
    model.geom_rgba[model.geom('score_win').id] = [0, 0, 0, 0]
