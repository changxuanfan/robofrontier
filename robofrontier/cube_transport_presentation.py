"""Public, static robot identification and camera context, without task plans."""
import numpy as np
from PIL import Image, ImageDraw

from .cube_transport import COLLECTION, _font


def project_point(model, data, camera_id, point, width, height):
    rotation = data.cam_xmat[camera_id].reshape(3, 3)
    local = (np.asarray(point)-data.cam_xpos[camera_id]) @ rotation
    if local[2] >= 0:
        return None
    focal = height/(2*np.tan(np.deg2rad(model.cam_fovy[camera_id])/2))
    return [float(width/2+focal*local[0]/(-local[2])),
            float(height/2-focal*local[1]/(-local[2]))]


def annotate(image, model, data, env, camera_name):
    """Unscaled raw RGB in a reserved viewport; ID boxes stay in the gutters.

    Only the thin base leader lines enter the raw viewport. Camera intrinsics
    refer to render_size_pixel; scene_viewport_pixel records the translation.
    """
    raw_width, raw_height = image.size
    left, top = 128, 126
    right, bottom = left+raw_width, top+raw_height
    width, height = raw_width+256, raw_height+252
    canvas = Image.new('RGB', (width, height), (232, 244, 255))
    canvas.paste(image, (left, top))
    draw = ImageDraw.Draw(canvas)
    camera_id = model.camera(camera_name).id
    anchors, labels = [], []
    for station in env['stations']:
        point = np.array(station['base_position_m'])+[0, 0, .12]
        pixel = project_point(model, data, camera_id, point, raw_width, raw_height)
        if pixel is None or not (0 <= pixel[0] < raw_width and 0 <= pixel[1] < raw_height):
            continue
        anchors.append({'robot_id': station['id'], 'pixel': [pixel[0]+left, pixel[1]+top]})
    if anchors:
        points = np.asarray([a['pixel'] for a in anchors])
        center = (points.min(axis=0)+points.max(axis=0))/2
        extent = np.maximum((points.max(axis=0)-points.min(axis=0))/2, 1)
        groups = {side: [] for side in ('left', 'right', 'top', 'bottom')}
        for anchor in anchors:
            dx, dy = (np.asarray(anchor['pixel'])-center)/extent
            side = ('left' if dx < 0 else 'right') if abs(dx) > abs(dy) else ('top' if dy < 0 else 'bottom')
            groups[side].append(anchor)
        for side, members in groups.items():
            axis = 1 if side in ('left', 'right') else 0
            members.sort(key=lambda a: a['pixel'][axis])
            lower, upper, gap = (top+24, bottom-24, 55) if axis == 1 else (left+35, right-35, 68)
            positions = []
            for member in members:
                positions.append(max(lower, member['pixel'][axis], positions[-1]+gap if positions else lower))
            if positions and positions[-1] > upper:
                offset = positions[-1]-upper
                positions = [v-offset for v in positions]
            for member, value in zip(members, positions):
                if not lower <= value <= upper:
                    raise ValueError('Insufficient annotation gutter capacity')
                cx, cy = (64 if side == 'left' else width-64, value) if axis == 1 else (value, 86 if side == 'top' else height-62)
                member['box'] = [cx-27, cy-21.5, cx+27, cy+21.5]
    for anchor in anchors:
        x, y = anchor['pixel']; box = anchor['box']
        draw.line((x, y, (box[0]+box[2])/2, (box[1]+box[3])/2), fill=(65, 95, 125), width=1)
        draw.ellipse((x-2, y-2, x+2, y+2), fill=(20, 42, 75))
    for anchor in anchors:
        box = anchor['box']
        overlap = sum(max(0, min(box[2], other['box'][2])-max(box[0], other['box'][0]))*
                      max(0, min(box[3], other['box'][3])-max(box[1], other['box'][1]))
                      for other in anchors if other is not anchor)
        draw.rounded_rectangle(box, radius=7, fill=(245, 251, 255), outline=(20, 42, 75), width=2)
        draw.text((box[0]+14, box[1]+2), anchor['robot_id'], font=_font(34), fill=(12, 32, 65))
        labels.append({'robot_id': anchor['robot_id'], 'base_anchor_pixel': anchor['pixel'],
                       'label_box_pixel': list(box), 'overlap_area_pixel2': overlap})
    draw.rectangle((0, 0, width, 45), fill=(23, 48, 75))
    title = f"{env['id']} | camera={camera_name} | INITIAL PREVIEW | sim={data.time:.3f}s"
    draw.text((16, 8), title, font=_font(22), fill='white')
    return canvas, {'schema': 'robofrontier.static_preview.v2', 'camera_id': camera_name,
                   'sim_time_s': float(data.time), 'frame_scope': 'initialization preview; not live',
                   'image_size_pixel': [width, height], 'robot_labels': labels,
                   'render_size_pixel': [raw_width, raw_height],
                   'scene_viewport_pixel': [left, top, right, bottom],
                   'label_boxes_outside_scene': True,
                   'annotation_source': 'fixed robot base geometry only; no object annotations'}


def startup_context(env, model, data):
    cameras = []
    for i in range(model.ncam):
        name = model.camera(i).name
        station_id = name.split('_')[0] if name[0] in [s['id'] for s in env['stations']] else None
        cameras.append({'id': name, 'station_id': station_id,
                        'kind': 'top_down_north_up' if name == 'top' else 'fixed_oblique',
                        'position_world_m': data.cam_xpos[i].tolist(),
                        'rotation_camera_to_world_row_major': data.cam_xmat[i].tolist(),
                        'fovy_deg': float(model.cam_fovy[i])})
    inventory = []
    for station in env['stations']:
        obj = next(o for o in env['objects'] if o['source'] == station['id'])
        inventory.append({'station': station['id'], 'type_code': obj['type_code'],
                          'color': obj['color'], 'shape': obj['shape'], 'count': 3,
                          'initial_positions_m': station['source_positions_m']})
    return {'schema': 'robofrontier.cube_transport.startup.v4', 'environment': env['id'],
            'scope': 'public initialization and fixed robot/camera configuration; not live object state',
            'coordinate_frame': {'world': '+X east, +Y north, +Z up',
                                  'robot_local': '+X inward, +Y robot left, +Z up',
                                  'top_image': 'right=east, up=north'},
            'robots': [{'robot_id': s['id'], 'side': s['side'],
                        'base_position_world_m': s['base_position_m'], 'yaw_rad': s['yaw_rad']}
                       for s in env['stations']],
            'neighbors': [e['stations'] for e in env['edges']],
            'initial_inventory': inventory, 'cameras': cameras,
            'raw_render_size_pixel': [1280, 900],
            'annotated_image': {'size_pixel': [1536, 1152], 'scene_viewport_pixel': [128, 126, 1408, 1026],
                                'calibration': 'camera intrinsics use raw render size; add viewport origin for annotated pixels'},
            'initial_top_image': f'../../../../benchmarks/{COLLECTION}/previews/{env["id"]}-top.png',
            'adapter_status': 'offline assets; robot_id names are not yet bound to the live action service'}


def startup_markdown(env, context):
    lines = [f'# {env["id"]} 启动说明（规则 v4）', '',
             '先打开本目录 startup.json 中 initial_top_image 指向的带编号俯视图，核对各基座标签。',
             '本说明和图片描述初始化；执行后以新 RGB 和机器人本体观测为准。', '',
             '机器人 ID 与图中的大写基座标签一致。世界 +X 向东、+Y 向北、+Z 向上；俯视图上北右东。',
             '相邻关系：'+', '.join('—'.join(pair) for pair in context['neighbors'])+'。', '',
             '| 工位 / robot_id | 安装侧 | 初始物料（各三件） |', '| --- | --- | --- |']
    sides = {'south': '南', 'east': '东', 'north': '北', 'west': '西', 'line': '直线排列'}
    colors = {'red': '红', 'blue': '蓝', 'green': '绿', 'orange': '橙', 'purple': '紫', 'cyan': '青'}
    for station, kind in zip(env['stations'], context['initial_inventory']):
        shape = '方块' if kind['shape'] == 'cube' else '圆柱'
        lines.append(f"| {station['id']} | {sides[station['side']]} | {colors[kind['color']]}{shape}（{kind['type_code']}） |")
    lines += ['', '读取本次指定的 seed 任务文件获取三个槽位的颜色＋形状要求。同类物品可互换；槽号不规定执行顺序。',
              '使用相邻工位交接，路线、分工、操作次序和并行安排由策略选择。', '',
              f"相机共 {len(context['cameras'])} 台：global、global_side 为两侧全局斜视，top 为俯视；每台另有 <ID>_front、<ID>_side。",
              '局部相机面向指定工位，也可能包含邻居。依据观测需要选择视角；固定相机几何见 startup.json。',
              '运行图像应注明相机名与仿真时间。静态预览不能代替运行观测。', '',
              '当前是离线资产包，尚未绑定运行服务；本说明不提供动作入口。', '']
    return '\n'.join(lines)
