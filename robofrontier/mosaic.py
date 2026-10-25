"""Four-arm mosaic scene assets. No policy, placement planner or action bridge."""
from collections import Counter
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

from .cube_transport import ROOT, _font, build_xml, layout, xyz
from .scene import vector

DIRECTORY = ROOT/'configs/environments/mosaic/four-arm-10x10'
# Explicit draft rather than treating the illustration's irregular cells as truth.
PATTERN = [
    'BBBBBBBBBB',
    'BRRBBBBRRB',
    'BWYRBBRYWB',
    'BWYRRRRYWB',
    'BRRRRRRRRB',
    'BRWBRRBWRB',
    'BWRWWBWRWB',
    'BBYWWWWWBB',
    'BBRYYYYRYB',
    'BRRRYYRYBY',
]
PATTERN_8X8 = [
    'BBBBBBBB',
    'BRWYYWRB',
    'BYYYYYYB',
    'YWBYYBWY',
    'YYWRRWYY',
    'YWRYYRWY',
    'BYYYYYYB',
    'BBBBBBBB',
]
PATTERN_7X7 = [
    'WWWRWWW',
    'WWRRRWW',
    'WBRRRBW',
    'WBRYRBW',
    'BBRYRBB',
    'BRYYYRB',
    'RWWBWWR',
]
PATTERN_6X6 = [
    'WWRRWW',
    'WRRRRW',
    'WBRRBW',
    'BBYYBB',
    'BRYYRB',
    'RYYYYR',
]
POCKET_OPENING_M = .050
POCKET_WALL_M = .001
COLORS = {'B': ('blue', [.12, .36, .85]), 'R': ('red', [.85, .13, .13]),
          'Y': ('yellow', [.98, .74, .08]), 'W': ('white', [.93, .93, .90])}


def scene_directory(size=10, recessed=False):
    if size not in (6,7,8,10):
        raise ValueError('Supported mosaic sizes are 6, 7, 8 and 10')
    return DIRECTORY.parent/(f'four-arm-{size}x{size}'+('-recessed' if recessed else ''))


def specification(size=10, recessed=False):
    scene_directory(size, recessed)
    pattern = {6:PATTERN_6X6,7:PATTERN_7X7,8:PATTERN_8X8,10:PATTERN}[size]
    middle = (size-1)/2
    env = layout(4)
    env.update(id=f'mosaic-four-arm-{size}x{size}', schema='robofrontier.mosaic.environment.v1',
               collection='mosaic', rules_version=1, topology='shared_table', edges=[],
               tables=[{'center_m': [0, 0, .375], 'half_size_m': [.80, .80, .025]}])
    env['objects'] = []
    counts = Counter(''.join(pattern))
    # Separated stock rows with space around each base and between adjacent
    # supplies. Avoid the close-in region of a downward-facing Panda wrist.
    stock = [(r, (j-6.5)*.06) for r in (.30, .36, .42, .48, .54, .60) for j in range(14)
             if abs((j-6.5)*.06) <= r-.06+1e-9
             and math.hypot(.65-r, (j-6.5)*.06) >= .28]
    assert len(stock) >= max(counts.values())
    for station, code in zip(env['stations'], COLORS):
        arm = station['id']; color, rgb = COLORS[code]
        positions = [xyz(station, .65-r, lateral, .42) for r, lateral in stock[:counts[code]]]
        station.update(home_tcp_m=xyz(station, .28, 0, .72), source_positions_m=positions,
                       goal_positions_m=[], parking_positions_m=[], supply_color=color)
        for index, position in enumerate(positions, 1):
            env['objects'].append({'id': f'{arm}_{color}_{index:02d}', 'source': arm,
                                   'color': color, 'rgb': rgb, 'shape': 'cube', 'type_code': arm.lower(),
                                   'initial_position_m': position})
    env['grid'] = {'rows': size, 'columns': size, 'cell_side_m': .04, 'cube_side_m': .04,
                   'center_world_m': [0, 0, .4], 'row1': 'north', 'column1': 'west',
                   'lines': 'visual texture only; no collision, snap or physical partitions'}
    if recessed:
        depth=.04/3
        env['id']+='-recessed'
        env['grid'].update(cell_side_m=round(POCKET_OPENING_M+POCKET_WALL_M,6),
            lines='Static physical pocket walls and floor; no snapping, welding or object pose correction.',
            fixture={'kind':'recessed_square_pockets','opening_side_m':POCKET_OPENING_M,'wall_thickness_m':POCKET_WALL_M,
                     'depth_m':depth,'rim_z_m':.4,'floor_z_m':.4-depth})
    pitch=env['grid']['cell_side_m']
    goal_z=.42-(.04/3 if recessed else 0)
    task = {'schema': 'robofrontier.mosaic.task.v1', 'id': f'mosaic-four-arm-{size}x{size}-draft',
            'status': 'draft transcription; not a validated manipulation benchmark',
            'reference_image': 'assets/reference-original.png', 'matrix_codes': list(pattern),
            'code_to_color': {code: value[0] for code, value in COLORS.items()},
            'inventory_counts': {COLORS[k][0]: v for k, v in counts.items()},
            'transcription_note': ('The supplied illustration has irregular half-cells in row 7. This draft puts its central blue cell at column 6; blue shading is one color. Confirm or edit the matrix before scoring an experiment.' if size==10 else f'Transcribed from the supplied regular {size}x{size} illustration, rows north to south and columns west to east; B blue, R red, Y yellow, W white. No rescaling or cropping of another target.'),
            'assignment': 'No arm/color ownership or placement order imposed; supply color is only initial location.',
            'goal_cells': [{'row': row+1, 'column': col+1, 'color': COLORS[code][0],
                            'cube_center_world_m': [round((col-middle)*pitch, 5), round((middle-row)*pitch, 5), goal_z]}
                           for row, line in enumerate(pattern) for col, code in enumerate(line)],
            'evaluation_status': 'runtime adapter includes private stable pose/color/release evaluation; dense manipulation feasibility remains unvalidated'}
    if size in (6,7):
        task['execution_objective']={
            'priority':'Complete the recognizable full color pattern efficiently, with visually cohesive, aligned, compact placement and no material gaps.',
            'quality_tradeoff':'Small residual errors are acceptable under the human preference. Compare local correction with later consolidation by expected benefit, disturbance and execution/coordination cost; select the order and methods yourself.',
            'audit':'The original strict completion thresholds remain a separate diagnostic. Report practical progress, residual gaps/pose uncertainty and strict claims honestly; do not require every cell to pass the strict audit before making other useful progress.'}
    if recessed:
        task['id']+='-recessed'
        task['execution_objective']={
            'priority':'Efficiently reproduce the full color pattern with one released, stable, upright cube seated in each intended pocket.',
            'quality_tradeoff':'Small residual offsets within a pocket are acceptable. Divider and clearance gaps are intentional; do not try to make neighboring cubes touch across the fixture.',
            'audit':'A cube resting on a wall, on another cube or in the wrong pocket is not seated. Report observed seating and uncertainty honestly; the strict completion thresholds remain a separate diagnostic from practical seating within the pocket. Fixture dimensions and goal coordinates are static public task information; no live object state is supplied.'}
    return env, task


def add_recessed_fixture(world, env):
    """Cut only the board footprint out of the tabletop; retain the supply surface."""
    grid=env['grid'];f=grid['fixture'];pitch=grid['cell_side_m']
    hx=(grid['columns']*pitch+f['wall_thickness_m'])/2
    hy=(grid['rows']*pitch+f['wall_thickness_m'])/2
    table=world.find("geom[@name='table_0']")
    attrs=dict(table.attrib);world.remove(table)
    bottom=.35;rim=f['rim_z_m'];floor=f['floor_z_m']
    pieces=[('west',[-(.8+hx)/2,0,(bottom+rim)/2],[(.8-hx)/2,.8,(rim-bottom)/2]),
            ('east',[(.8+hx)/2,0,(bottom+rim)/2],[(.8-hx)/2,.8,(rim-bottom)/2]),
            ('south',[0,-(.8+hy)/2,(bottom+rim)/2],[hx,(.8-hy)/2,(rim-bottom)/2]),
            ('north',[0,(.8+hy)/2,(bottom+rim)/2],[hx,(.8-hy)/2,(rim-bottom)/2]),
            ('pocket_floor',[0,0,(bottom+floor)/2],[hx,hy,(floor-bottom)/2])]
    for name,pos,size in pieces:
        ET.SubElement(world,'geom',**dict(attrs,name='table_'+name,pos=vector(pos),size=vector(size)))
    wall=f['wall_thickness_m'];z=(floor+rim)/2
    for i in range(grid['columns']+1):
        ET.SubElement(world,'geom',name=f'pocket_wall_x_{i}',type='box',
            pos=vector([(i-grid['columns']/2)*pitch,0,z]),size=vector([wall/2,hy,f['depth_m']/2]),
            rgba='.38 .49 .56 1',friction=attrs['friction'],condim=attrs['condim'])
    # Segments avoid coincident wall volumes at lattice intersections.
    for i in range(grid['rows']+1):
        for j in range(grid['columns']):
            ET.SubElement(world,'geom',name=f'pocket_wall_y_{i}_{j}',type='box',
                pos=vector([(j-(grid['columns']-1)/2)*pitch,(i-grid['rows']/2)*pitch,z]),
                size=vector([f['opening_side_m']/2,wall/2,f['depth_m']/2]),
                rgba='.38 .49 .56 1',friction=attrs['friction'],condim=attrs['condim'])


def number_geom(world, asset, directory, name, text, position):
    image = Image.new('RGB', (128, 128), (229, 240, 246))
    draw = ImageDraw.Draw(image); font = _font(82)
    bounds = draw.textbbox((0, 0), text, font=font)
    draw.text(((128-(bounds[2]-bounds[0]))/2, (128-(bounds[3]-bounds[1]))/2-bounds[1]), text, font=font, fill=(24, 47, 68))
    image.save(directory/'assets'/f'{name}.png')
    ET.SubElement(asset, 'texture', name=name, type='2d', file=f'assets/{name}.png')
    ET.SubElement(asset, 'material', name=name, texture=name, specular='0')
    ET.SubElement(world, 'geom', name=name, type='box', size='.012 .012 .0001', pos=vector(position),
                  material=name, contype='0', conaffinity='0', group='1')


def create_scene(directory, env):
    root = build_xml(env, directory)
    root.set('model', 'RoboFrontier four-arm cube mosaic')
    world, asset = root.find('worldbody'), root.find('asset')
    # Four legs end at the underside of the tabletop.
    for geom in list(world.findall('geom')):
        if (geom.get('name') or '').startswith('table_support_'):
            world.remove(geom)
    for i, (x, y) in enumerate(((-.69, -.69), (-.69, .69), (.69, -.69), (.69, .69))):
        ET.SubElement(world, 'geom', name=f'table_leg_{i}', type='box', pos=vector([x, y, .175]),
                      size='.045 .045 .175', rgba='.36 .29 .23 1')
    image = Image.new('RGB', (1600, 1600), (225, 235, 239)); draw = ImageDraw.Draw(image)
    rows,columns=env['grid']['rows'],env['grid']['columns']
    side=env['grid']['cell_side_m'];half_x=columns*side/2;half_y=rows*side/2
    fixture=env['grid'].get('fixture')
    if fixture:add_recessed_fixture(world,env)
    for i in range(columns+1):
        p=min(1599,round(i*1600/columns))
        draw.line((p,0,p,1599),fill=(84,113,129),width=4)
    for i in range(rows+1):
        p=min(1599,round(i*1600/rows))
        draw.line((0,p,1599,p),fill=(84,113,129),width=4)
    image.save(directory/'assets/grid.png')
    ET.SubElement(asset, 'texture', name='mosaic_grid', type='2d', file='assets/grid.png')
    ET.SubElement(asset, 'material', name='mosaic_grid', texture='mosaic_grid', specular='0', emission='.08')
    grid_z=fixture['floor_z_m']+.0002 if fixture else .4002
    ET.SubElement(world, 'geom', name='mosaic_grid', type='box', size=vector([half_x,half_y,.0001]), pos=vector([0,0,grid_z]),
                  material='mosaic_grid', contype='0', conaffinity='0', group='1')
    for i in range(columns):
        number_geom(world, asset, directory, f'column_{i+1}', str(i+1), [(i-(columns-1)/2)*side, half_y+.028, .4003])
    for i in range(rows):
        number_geom(world, asset, directory, f'row_{i+1}', str(i+1), [-half_x-.028, ((rows-1)/2-i)*side, .4003])
    # Top image fills more of the viewport while retaining all four robot bases.
    root.find("worldbody/camera[@name='top']").set('pos', '0 0 2.65')
    ET.SubElement(world, 'camera', name='board_top', pos='0 0 1.10', xyaxes='1 0 0 0 1 0', fovy='40')
    return root


def target_preview(task, path):
    rows=len(task['matrix_codes']);columns=len(task['matrix_codes'][0]);height=180+rows*74
    image = Image.new('RGB', (140+columns*74, height), (234, 243, 248)); draw = ImageDraw.Draw(image)
    title=f'TARGET / {rows} x {columns}' + (' / row 7 normalized' if rows==10 else '')
    draw.text((24, 12), title, font=_font(27), fill=(20, 42, 65))
    for i in range(columns):
        draw.text((98+i*74, 65), str(i+1), font=_font(23), fill=(20, 42, 65))
    for i in range(rows):
        draw.text((40, 112+i*74), str(i+1), font=_font(23), fill=(20, 42, 65))
    for row, line in enumerate(task['matrix_codes']):
        for col, code in enumerate(line):
            rgb=tuple(round(c*255) for c in COLORS[code][1]);x,y=80+col*74,100+row*74
            draw.rectangle((x,y,x+73,y+73),fill=rgb,outline=(50,65,80),width=1)
    draw.text((24, height-50), 'B blue  R red  Y yellow  W white | no placement order', font=_font(23), fill=(20,42,65))
    image.save(path)


def runtime_task(env, draft, scene_sha256):
    """Shared outcomes, with no allocation to individual robots."""
    value = {
        'id':env['id']+'-v1','kind':'shared_grid',
        'status':'draft target; engineering integration only',
        'description':'Reproduce the shared color grid. Choose allocation, order, approach, grasp orientation and transfers. Supply color is only an initial location.',
        'scene_asset':str(scene_directory(env['grid']['rows'],bool(env['grid'].get('fixture'))).relative_to(ROOT)), 'scene_sha256':scene_sha256,
        'arms':{s['id']:{'base_position_m':s['base_position_m'],'yaw_rad':s['yaw_rad']} for s in env['stations']},
        'objects':env['objects'],
        'shared_goals':[{'goal_id':f"r{g['row']:02d}c{g['column']:02d}",'row':g['row'],'column':g['column'],
                         'position_m':g['cube_center_world_m'],'requires':{'color':g['color'],'shape':'cube'}} for g in draft['goal_cells']],
        'grid':env['grid'],'target_image':'target-pattern-draft.png','initial_top_image':'initial-top.png',
        'initial_image_metadata':'initial-top.json','initial_images_are_runtime_observations':False,
        'transcription_note':draft['transcription_note'],
        'workspace_world_m':{'lower':[-.80,-.80,.405],'upper':[.80,.80,.95]},
        'cameras':['global','global_side','top','board_top']+[a+'_'+v for a in 'ABCD' for v in ('front','side')],
        'completion':{'xy_tolerance_m':.002,'z_tolerance_m':.003,'max_linear_speed_m_s':.005,
                      'stable_duration_sim_s':1.,'max_sample_gap_sim_s':.025,'axis_alignment_tolerance_rad':.035},
        'observation_contract':{'initial_positions_public':True,
            'runtime':'RGB + robot proprioception + action status; no live object poses, contact IDs or depth',
            'camera_selection':'agent','image_loading':'agent'},
    }
    if 'execution_objective' in draft:
        value['execution_objective']=dict(draft['execution_objective'])
    if env['grid'].get('fixture'):
        value['completion'].update(mode='pocket_seating',containment_tolerance_m=.0002,upright_tolerance_rad=.035,max_angular_speed_rad_s=.05)
    return value
