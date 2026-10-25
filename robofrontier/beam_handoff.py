"""Eight-arm beam relay assets. No task solver, grasp attachment or runtime API."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

from .cube_transport import ROOT, _camera, _font, build_xml, xyz
from .scene import vector

DIRECTORY = ROOT / 'configs/environments/beam_handoff/facing-08'
PREVIEW_CAMERAS = ('global', 'top', 'global_side', 'pair_1', 'pair_2', 'pair_3', 'pair_4')


def specification(parameters):
    """Parameters describe geometry; they do not prescribe a handover plan."""
    p = parameters
    if p['pair_count'] != 4 or p['beam']['long_axis_world'] != '+Y':
        raise ValueError('This version is four facing pairs with a transverse beam')
    if min(p['pair_spacing_m'], p['opposed_base_spacing_m'],
           *(p['beam'][k] for k in ('length_m', 'width_m', 'height_m', 'density_kg_m3'))) <= 0:
        raise ValueError('Geometry and density must be positive')
    table, beam = p['table'], p['beam']
    height = table['height_m']
    stations, pairs = [], []
    for column in range(p['pair_count']):
        x = (column - (p['pair_count'] - 1) / 2) * p['pair_spacing_m']
        members = []
        for side, sign in (('south', -1), ('north', 1)):
            arm = chr(65 + len(stations))
            station = dict(id=arm, side=side, pair=column + 1,
                           base_position_m=[x, sign * p['opposed_base_spacing_m'] / 2, height],
                           yaw_rad=-sign * math.pi / 2)
            station.update(home_tcp_m=xyz(station, .27, 0, .78),
                           source_positions_m=[], goal_positions_m=[], parking_positions_m=[])
            stations.append(station)
            members.append(arm)
        pairs.append(dict(id=column+1, robots=members, center_x_m=x))
    mass = beam['density_kg_m3'] * beam['length_m'] * beam['width_m'] * beam['height_m']
    env = dict(schema='robofrontier.beam_handoff.environment.v1', id=p['id'],
               collection='beam_handoff', arm_count=8, topology='four_facing_pairs',
               physics=p['physics'], stations=stations, pairs=pairs,
               tables=[dict(center_m=[0, 0, height-table['thickness_m']/2],
                            half_size_m=[table['length_m']/2, table['width_m']/2, table['thickness_m']/2])],
               table_rgba=table['rgba'], floor_rgba=p['appearance']['floor_rgba'], edges=[], objects=[],
               beam=dict(beam, id='timber', mass_kg=mass,
                         initial_center_world_m=[*beam['initial_center_xy_m'], height+beam['height_m']/2]),
               goal=p['goal'], initial_keyframe='initial', scene='scene.xml',
               observation_contract=dict(initial_geometry_public=True,
                   future_policy_runtime=['RGB', 'robot_proprioception', 'action_status'],
                   live_object_truth_for_policy=False))
    task = dict(schema='robofrontier.beam_handoff.task.v1', id=p['id']+'-draft',
                status='design draft; frozen policy task and evaluator are provided through task.json; engineering references are separate',
                goal='Lift the timber at the left start and cooperatively hand it to the far-right finish zone.',
                coordinate_frame='+X rightward transport, +Y north, +Z up; beam initially spans north/south',
                rules=[
                    'Use real gripper contact; no welding, snapping, teleportation or simulator reset.',
                    'During transport keep the timber off the table and floor, supported by at least two robots; initial pickup, final placement and declared drop recovery are separate phases.',
                    'Receiving robots establish their support before the current sending robots release. Grasp locations, poses, arm grouping and motion order are policy choices.',
                    'The initial sending pair is A/B. Custody advances only after a verified airborne handover, not merely because the timber moves into another region.',
                    'A drop counts as a failed transfer attempt. Only the last established sending team may retrieve it and retry; the intended receiving team must not pick it up to bypass the handover.',
                    'If the sending team cannot physically retrieve a drop, stop and report failure; do not reset or use the receiver as an undeclared recovery shortcut.',
                    'Finish by releasing the timber stably inside the marked right-hand zone.'
                ],
                planning_note='Choose grasp spacing that permits simultaneous sender/receiver access and withdrawal. This is a general geometry consideration, not a prescribed alternating-grasp solution.',
                implementation_boundary='The passive preview does not enforce rules. task.json selects the shared policy runtime and private process evaluator; Oracle engineering commands remain separate from policy inputs.',
                two_arm_requirement='Explicit task rule. The preview does not prove that a single Panda is physically incapable of carrying this timber.')
    return env, task


def _zone(world, name, cx, cy, sx, sy, z, color):
    for i, (x, y, hx, hy) in enumerate(((cx-sx/2, cy, .004, sy/2),
                                       (cx+sx/2, cy, .004, sy/2),
                                       (cx, cy-sy/2, sx/2, .004),
                                       (cx, cy+sy/2, sx/2, .004))):
        ET.SubElement(world, 'geom', name=f'{name}_{i}', type='box',
                      pos=vector([x, y, z]), size=vector([hx, hy, .00015]),
                      rgba=color, contype='0', conaffinity='0')


def create_scene(directory, parameters, env):
    directory = Path(directory)
    root = build_xml(env, directory)
    root.set('model', 'RoboFrontier | eight-arm airborne beam handoff | preview')
    world, asset = root.find('worldbody'), root.find('asset')
    height = parameters['table']['height_m']
    # Keep the existing table/robot contacts; replace the long plinth by legs.
    for geom in list(world.findall('geom')):
        if (geom.get('name') or '').startswith('table_support_'):
            world.remove(geom)
    leg_height = height - parameters['table']['thickness_m']
    table = parameters['table']
    for i, x in enumerate(v*table['length_m'] for v in (-.44,-.15,.15,.44)):
        for j, y in enumerate((-table['width_m']/2+.13,table['width_m']/2-.13)):
            ET.SubElement(world, 'geom', name=f'leg_{i}_{j}', type='box',
                          pos=vector([x,y,leg_height/2]), size=vector([.045,.045,leg_height/2]),
                          rgba='.30 .35 .38 1')
    # Distinct walnut-like procedural grain; geometry remains one rigid box.
    rng = np.random.default_rng(87)
    base_rgb = tuple(round(v*255) for v in parameters['appearance']['beam_rgba'][:3])
    grain = Image.new('RGB', (256,1024), base_rgb)
    draw = ImageDraw.Draw(grain)
    for i in range(85):
        x = float(rng.uniform(0,256)); tone = int(rng.integers(-24,25))
        points = [(x+4*math.sin(y/160+i),y) for y in range(0,1024,12)]
        draw.line(points,fill=tuple(min(255,max(0,v+tone)) for v in base_rgb),width=2)
    grain.save(directory/'assets/timber-grain.png')
    ET.SubElement(asset,'texture',name='timber_grain',type='2d',file='assets/timber-grain.png')
    ET.SubElement(asset,'material',name='timber_wood',texture='timber_grain',
                  texuniform='false',specular='.08',shininess='.1')
    beam = env['beam']
    body = ET.SubElement(world,'body',name=beam['id'],pos=vector(beam['initial_center_world_m']))
    ET.SubElement(body,'freejoint',name='timber_free')
    ET.SubElement(body,'geom',name='timber_geom',type='box',
                  size=vector([beam['width_m']/2,beam['length_m']/2,beam['height_m']/2]),
                  mass=str(beam['mass_kg']),material='timber_wood',
                  friction=vector(env['physics']['contact_friction']),condim='4')
    _zone(world,'start',*beam['initial_center_xy_m'],.22,1.36,height+.0004,'.20 .40 .72 1')
    goal = env['goal']
    _zone(world,'finish',*goal['center_xy_m'],goal['zone_length_x_m'],
          goal['zone_width_y_m'],height+.0004,'.08 .52 .40 1')
    for name, text, x, rgb in [('start_word','START',beam['initial_center_xy_m'][0],(35,70,125)),
                                ('finish_word','FINISH',goal['center_xy_m'][0],(20,100,80))]:
        image = Image.new('RGB',(256,80),(226,237,242)); d = ImageDraw.Draw(image)
        d.text((10,8),text,font=_font(45),fill=rgb)
        image.save(directory/f'assets/{name}.png')
        ET.SubElement(asset,'texture',name=name,type='2d',file=f'assets/{name}.png')
        ET.SubElement(asset,'material',name=name,texture=name,specular='0')
        ET.SubElement(world,'geom',name=name,type='box',pos=vector([x,-.755,height+.0004]),
                      size='.10 .032 .00015',material=name,contype='0',conaffinity='0')
    # World-static visual arrows, not collision rails or handoff buffers.
    for x in (-.66,.06,.78):
        ET.SubElement(world,'geom',type='box',pos=vector([x,0,height+.0004]),
                      size='.08 .006 .00015',rgba='.30 .43 .50 1',contype='0',conaffinity='0')
        for sign in (-1,1):
            ET.SubElement(world,'geom',type='box',pos=vector([x+.06,sign*.02,height+.0004]),
                          size='.031 .006 .00015',euler=vector([0,0,-sign*math.pi/4]),
                          rgba='.30 .43 .50 1',contype='0',conaffinity='0')
    for cam in list(world.findall('camera')):
        if cam.get('name') in ('global','global_side','top'):
            world.remove(cam)
    _camera(world,'global',[3.4,-3.5,3.4],np.array([0,0,.52]),43)
    _camera(world,'global_side',[-3.3,-3.3,2.65],np.array([0,0,.48]),50)
    ET.SubElement(world,'camera',name='top',pos='0 0 4.2',xyaxes='1 0 0 0 1 0',fovy='43')
    for pair in env['pairs']:
        x = pair['center_x_m']
        _camera(world,f'pair_{pair["id"]}',[x,-2.1,2.0],np.array([x,0,.45]),55)
    root.find('statistic').attrib.update(center='0 0 .5',extent='4.5')
    return root
