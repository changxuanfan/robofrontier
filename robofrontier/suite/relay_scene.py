"""Relay scene revisions: simple end regions and forward-facing openings.

Clone r10 assets without stepping physics. Apparatus placement changes are a
single rigid yaw of each complete assembly, including its independent weight.
Controllers, apparatus parameters and robot mounts stay put. R12 adds small
same-height end extensions where the outside regions exceed the old tabletop.
"""
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from . import v2
from .scenes import box, prism_mesh
from ..robot_control import matrix_quat, quat_matrix
from ..run_setup import write_json

HALF = .70
PAINT = (.95, .93, .86, 1)
ARROW = (.10, .30, .43, 1)


def endpoint_half(revision):
    return np.array([.40, .68]) if revision >= 12 else np.array([HALF, HALF])


def endpoint_centers(arms, revision=11, stations=None, route=None):
    if revision >= 13 and arms != 8:
        result=[]
        for index,region in enumerate((route[0],route[-1])):
            yaw=region['nominal_yaw_rad'];forward=np.array([math.cos(yaw),math.sin(yaw)])
            # Original r10 route already includes 0.38m of end clearance.
            result.append(np.asarray(region['center_m'])[:2]+(-.10 if index==0 else .10)*forward)
        return result
    if revision >= 12:
        extent = max(abs(s['base_position_m'][0]) for s in stations)
        return [np.array([-extent-.54, 0.]), np.array([extent+.54, 0.])]
    return [np.array([-1.30, 0.]), np.array([1.30, 0.])] if arms == 8 else [
        np.array([-1.70, -.10]), np.array([1.70, -.10])]


def apply_arc_layout(root,env,source_model):
    """One module-wise rigid layout transform; apparatus internal poses stay intact."""
    arms=env['arm_count']
    if arms==8:return
    world=root.find('worldbody')
    wide=env['suite_case']['scene_revision']>=14 and arms==12
    radial_shift=1.08 if wide else 0.
    def pose(element,turn,translation=np.zeros(3)):
        element.set('pos',v2.vector(turn@np.fromstring(element.get('pos','0 0 0'),sep=' ')+translation))
        rotation=quat_matrix(source_model.cam_quat[source_model.camera(element.get('name')).id]) if element.tag=='camera' else quat_matrix(np.fromstring(element.get('quat','1 0 0 0'),sep=' '))
        for attribute in ('xyaxes','zaxis','euler','axisangle'):element.attrib.pop(attribute,None)
        element.set('quat',v2.vector(matrix_quat(turn@rotation)))
    if arms==12:
        old_angles=np.linspace(math.pi,0,6);new_angles=np.deg2rad(np.linspace(135.,45.,6) if wide else np.linspace(150.,30.,6))
        deltas=new_angles-old_angles
        unequal=env['suite_case']['scene_revision']>=15
        ring_angles=np.r_[np.linspace(135.,45.,7),np.linspace(135.,45.,5)] if unequal else None
        for index,station in enumerate(env['stations']):
            if unequal:
                outside=index<7;theta=math.radians(float(ring_angles[index]));radius=3.46 if outside else 2.16
                new_yaw=theta+(math.pi if outside else 0.)
                delta=new_yaw-station['yaw_rad'];turn=v2.rz(delta)
                target=np.array([radius*math.cos(theta),radius*math.sin(theta),station['base_position_m'][2]])
                translation=target-turn@np.asarray(station['base_position_m'])
                station['ring']='outer' if outside else 'inner'
            else:
                delta=float(deltas[index//2]);turn=v2.rz(delta)
                theta=new_angles[index//2];translation=radial_shift*np.array([math.cos(theta),math.sin(theta),0.])
            pose(world.find(f"body[@name='{station['id']}_link0']"),turn,translation)
            for geom in world.findall('geom'):
                if geom.get('name','').startswith(station['id']+'_'):pose(geom,turn,translation)
            station['base_position_m']=(turn@np.asarray(station['base_position_m'])+translation).tolist()
            station['home_tcp_m']=(turn@np.asarray(station['home_tcp_m'])+translation).tolist()
            station['yaw_rad']+=delta
            for camera in world.findall('camera'):
                if camera.get('name','').startswith(station['id']+'_'):pose(camera,turn,translation)
        for index,region in enumerate(env['route']):
            turn=v2.rz(float(deltas[index]))
            theta=new_angles[index];translation=radial_shift*np.array([math.cos(theta),math.sin(theta),0.])
            region['center_m']=(turn@np.asarray(region['center_m'])+translation).tolist()
            region['nominal_yaw_rad']+=float(deltas[index])
        key=root.find("keyframe/key[@name='initial']");q=np.fromstring(key.get('qpos'),sep=' ')
        for assembly in env['mechanism']['payload_assemblies']:
            ident=assembly['id'];index=next(i for i,r in enumerate(env['route']) if r['region_id']==assembly['initial_route_stage_id'])
            delta=float(deltas[index]);turn=v2.rz(delta)
            theta=new_angles[index];translation=radial_shift*np.array([math.cos(theta),math.sin(theta),0.])
            if unequal:
                # Keep the distributed rods clear of the newly added outer
                # mount at105deg and the new inner mount at67.5deg.
                distance={'payload_02':.32,'payload_03':.72}.get(ident,0.)
                translation+=distance*np.array([math.sin(theta),-math.cos(theta),0.])
            for body in world.findall('body'):
                name=body.get('name','');free=body.find('freejoint')
                if free is None or not(name==ident or name.startswith(ident+'_')):continue
                address=int(source_model.jnt_qposadr[source_model.joint(free.get('name')).id])
                q[address:address+3]=turn@q[address:address+3]+translation
                q[address+3:address+7]=matrix_quat(turn@quat_matrix(q[address+3:address+7]))
                body.set('pos',v2.vector(q[address:address+3]));body.set('quat',v2.vector(q[address+3:address+7]))
            obj=next(o for o in env['objects'] if o['id']==ident)
            obj['initial_position_m']=(turn@np.asarray(obj['initial_position_m'])+translation).tolist()
            if obj.get('initial_quaternion_wxyz') is not None:obj['initial_quaternion_wxyz']=matrix_quat(turn@quat_matrix(obj['initial_quaternion_wxyz'])).tolist()
            for field in ('initial_position_m','pickup_center_m'):
                if field in assembly:assembly[field]=(turn@np.asarray(assembly[field])+translation).tolist()
            if 'initial_opening_direction_world_xyz' in assembly:assembly['initial_opening_direction_world_xyz']=(turn@np.asarray(assembly['initial_opening_direction_world_xyz'])).tolist()
            assembly['pickup_yaw_rad']+=delta
            assembly['layout_yaw_change_rad']=delta
            assembly['layout_translation_world_m']=translation.tolist()
            for camera in world.findall('camera'):
                if camera.get('name','').startswith(ident+'_') or (ident=='payload_01' and camera.get('name')=='payload_close'):pose(camera,turn,translation)
        key.set('qpos',v2.vector(q))
        if unequal:
            for obj in env['objects']:
                obj['source']=min(env['stations'],key=lambda s:np.linalg.norm(np.asarray(s['base_position_m'])-np.asarray(obj['initial_position_m'])))['id']
    # Rebuild one continuous annular worktop and its normal end landings.
    # No external wing tables are added.
    for geom in list(world.findall('geom')):
        if geom.get('name','').startswith('table_'):world.remove(geom)
    inner,outer=(1.60,3.89) if wide else (.80,2.81)
    route_radius=1.73+radial_shift
    arc_degrees=90 if wide else 120 if arms==12 else 180
    angles=np.deg2rad(np.linspace(45.,135.,25) if wide else np.linspace(30.,150.,25)) if arms==12 else np.linspace(0.,math.pi,31)
    height=env['physics']['table_height_m']
    footprint=[]
    for index,(a,b) in enumerate(zip(angles,angles[1:])):
        points=[(r*np.array([math.cos(t),math.sin(t)])).tolist() for r,t in [(inner,a),(outer,a),(outer,b),(inner,b)]]
        footprint.extend(points)
        name=f'table_relay_arc_{index}';prism_mesh(root,name,points,height-.05,height)
        ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,material='table_oak',mass='0',friction='1 .005 .0001')
        if index%3==0:
            point=(inner+outer)/2*np.array([math.cos(a),math.sin(a)])
            box(world,name+'_leg',[*point,(height-.05)/2],[.04,.04,(height-.05)/2],(.32,.36,.40,1),mass=0)
    for index,angle in enumerate((angles[-1],angles[0])):
        radial=np.array([math.cos(angle),math.sin(angle)]);forward=np.array([math.sin(angle),-math.cos(angle)])
        d0,d1=(-.92,0.) if index==0 else (0.,.92)
        points=[(r*radial+d*forward).tolist() for r,d in [(inner,d0),(outer,d0),(outer,d1),(inner,d1)]]
        # radial/clockwise-tangent is a left-handed planar basis.
        points.reverse();footprint.extend(points)
        name=f'table_relay_landing_{index}';prism_mesh(root,name,points,height-.05,height)
        ET.SubElement(world,'geom',name=name,type='mesh',mesh=name,material='table_oak',mass='0',friction='1 .005 .0001')
        for j,r in enumerate((inner+.15,outer-.15)):
            point=r*radial+(-.72 if index==0 else .72)*forward
            box(world,name+f'_leg{j}',[*point,(height-.05)/2],[.04,.04,(height-.05)/2],(.32,.36,.40,1),mass=0)
    env['table_geometry']=dict(kind='annular_arc_with_integrated_end_landings',center_m=[0.,0.],
        inner_radius_m=inner,outer_radius_m=outer,radius_m=(inner+outer)/2,radial_half_width_m=(outer-inner)/2,
        angle_range_rad=[float(angles[0]),float(angles[-1])],angle_degrees=arc_degrees,
        route_centerline_radius_m=route_radius,end_landing_depth_m=.92,external_side_tables=False)
    env['mechanism']['layout']=f'upper_arc_{arc_degrees}_degrees' if arms==12 else 'upper_semicircle'
    env['mechanism']['table_inner_diameter_m']=2*inner
    env['mechanism']['layout_arc_degrees']=arc_degrees
    env['mechanism']['curve_radius_m']=route_radius
    if arms==12:
        env['mechanism'].update(inner_arms=5 if unequal else 6,outer_arms=7 if unequal else 6,inner_robot_radius_m=1.08+radial_shift,outer_robot_radius_m=2.38+radial_shift)
        if unequal:
            env['mechanism'].update(outer_robot_ids=[s['id'] for s in env['stations'][:7]],inner_robot_ids=[s['id'] for s in env['stations'][7:]],flexible_teams=True)
    low,high=np.min(footprint,axis=0),np.max(footprint,axis=0)
    env['tables']=[dict(center_m=[*((low+high)/2).tolist(),height-.025],half_size_m=[*((high-low)/2).tolist(),.025])]


def split_polygon(polygon, a, b):
    """Split one convex polygon at an oriented edge, keeping both real pieces."""
    inside, outside = [], []
    cross = lambda p: float((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]))
    for p, q in zip(polygon, polygon[1:]+polygon[:1]):
        u, v = cross(p), cross(q)
        (inside if u >= 0 else outside).append(p)
        if (u >= 0) != (v >= 0):
            point = p+(q-p)*(u/(u-v))
            inside.append(point); outside.append(point)
    return inside, outside


def polygon_area(polygon):
    return abs(sum(float(a[0]*b[1]-a[1]*b[0]) for a, b in zip(polygon, polygon[1:]+polygon[:1])))/2 if len(polygon)>2 else 0.


def subtract_convex(polygon, cutter):
    remaining = polygon; pieces = []
    for a, b in zip(cutter, cutter[1:]+cutter[:1]):
        if not remaining: break
        remaining, outside = split_polygon(remaining, a, b)
        if polygon_area(outside) > 1e-8: pieces.append(outside)
    return pieces


def merge_convex_pieces(pieces):
    """Remove redundant coplanar seams when two disjoint pieces form one hull."""
    from scipy.spatial import ConvexHull
    while True:
        merged = False
        for i in range(len(pieces)):
            for j in range(i+1,len(pieces)):
                points = np.unique(np.vstack((pieces[i],pieces[j])),axis=0)
                hull = ConvexHull(points)
                if abs(hull.volume-polygon_area(pieces[i])-polygon_area(pieces[j])) < 1e-8:
                    pieces[i] = [points[k] for k in hull.vertices]
                    pieces.pop(j); merged = True; break
            if merged: break
        if not merged: return pieces


def add_end_extensions(root, env, source_model, centers, half):
    """Fill only missing tabletop area, avoiding overlapping support slabs."""
    if env['arm_count'] == 8:
        return []
    from scipy.spatial import ConvexHull
    data = mujoco.MjData(source_model)
    mujoco.mj_resetDataKeyframe(source_model, data, source_model.key('initial').id)
    mujoco.mj_forward(source_model, data)
    covers = []
    for g in range(source_model.ngeom):
        name = source_model.geom(g).name or ''
        if not name.startswith('table_') or 'leg' in name or not (source_model.geom_contype[g] or source_model.geom_conaffinity[g]): continue
        rotation = data.geom_xmat[g].reshape(3, 3)
        if source_model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = source_model.geom_dataid[g]; start = source_model.mesh_vertadr[mid]; count = source_model.mesh_vertnum[mid]
            local = source_model.mesh_vert[start:start+count]
        else:
            local = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*source_model.geom_size[g]
        vertices = local@rotation.T+data.geom_xpos[g]
        points = np.unique(vertices[:,:2], axis=0)
        covers.append([points[i] for i in ConvexHull(points).vertices])
    world = root.find('worldbody'); height = env['physics']['table_height_m']; result = []
    template = world.find("geom[@name='table_endcap_1']")
    for side, center in zip((-1,1), centers):
        outer = half+.04
        pieces = [[center+outer*np.array(v) for v in [(-1,-1),(1,-1),(1,1),(-1,1)]]]
        for cover in covers:
            pieces = [part for piece in pieces for part in subtract_convex(piece, cover)]
        pieces = merge_convex_pieces(pieces)
        for index, piece in enumerate(pieces):
            name = f'table_relay_end_{side}_{index}'
            prism_mesh(root, name, [p.tolist() for p in piece], height-.05, height)
            attributes = {key:value for key,value in template.attrib.items()
                          if key not in ('name','pos','quat','type','size','mesh')}
            attributes.update(name=name,type='mesh',mesh=name,mass='0')
            ET.SubElement(world,'geom',**attributes)
        for index, y in enumerate((-.54,.54)):
            box(world,f'table_relay_end_leg_{side}_{index}',[center[0]+side*.30,y,(height-.05)/2],
                [.035,.035,(height-.05)/2],(.30,.32,.34,1),mass=0)
        result.append(dict(center_world_xy_m=center.tolist(),half_size_xy_m=outer.tolist(),
                           nonoverlapping_added_pieces=len(pieces),added_area_m2=sum(polygon_area(p) for p in pieces)))
    env['table_geometry']['relay_end_extensions'] = result
    for table in env['tables']:
        table['half_size_m'][0] = float(abs(centers[1][0])+half[0]+.04)
    return result


def flip_assemblies(root, env, model):
    """Rotate free roots about the beam origin; nested hinge axes rotate with it."""
    if env['suite_case']['level'] == 1:
        return
    world = root.find('worldbody')
    turn = v2.rz(math.pi)
    key = root.find("keyframe/key[@name='initial']")
    q = np.fromstring(key.get('qpos'), sep=' ')
    for assembly in env['mechanism']['payload_assemblies']:
        ident = assembly['id']
        beam = world.find(f"body[@name='{ident}']")
        beam_joint = model.joint(beam.find('freejoint').get('name')).id
        anchor = q[int(model.jnt_qposadr[beam_joint]):][:3].copy()
        old_rotation = quat_matrix(q[int(model.jnt_qposadr[beam_joint])+3:][:4])
        for body in world.findall('body'):
            name = body.get('name', '')
            if name != ident and not name.startswith(ident+'_'):
                continue
            free = body.find('freejoint')
            if free is None:
                continue
            address = int(model.jnt_qposadr[model.joint(free.get('name')).id])
            position = anchor+turn@(q[address:address+3]-anchor)
            rotation = turn@quat_matrix(q[address+3:address+7])
            q[address:address+3] = position
            q[address+3:address+7] = matrix_quat(rotation)
            body.set('pos', v2.vector(position))
            body.set('quat', v2.vector(q[address+3:address+7]))
        obj = next(o for o in env['objects'] if o['id'] == ident)
        obj['initial_quaternion_wxyz'] = matrix_quat(turn@old_rotation).tolist()
        assembly['pickup_center_m'] = (anchor+turn@(
            np.asarray(assembly['pickup_center_m'])-anchor)).tolist()
        assembly['pickup_yaw_rad'] += math.pi
        assembly['initial_opening_direction_world_xyz'] = (old_rotation@np.array([1., 0., 0.])).tolist()
        assembly['initial_assembly_yaw_change_rad'] = math.pi
    key.set('qpos', v2.vector(q))


def revise(root, env, folder, source_model):
    world = root.find('worldbody')
    # All removed bodies are massless paint, never a support fixture.
    for body in list(world.findall('body')):
        if body.get('name', '').startswith(('route_zone_', 'route_arrow_', 'pickup_')):
            world.remove(body)
    for geom in list(world.findall('geom')):
        if geom.get('name', '').startswith('route_label_'):
            world.remove(geom)
    for tag in ('texture', 'material', 'mesh'):
        for asset in list(root.find('asset').findall(tag)):
            if asset.get('name', '').startswith(('route_label_', 'route_arrow_')):
                root.find('asset').remove(asset)
    flip_assemblies(root, env, source_model)
    revision = env['suite_case']['scene_revision']
    if revision>=13:apply_arc_layout(root,env,source_model)
    half = endpoint_half(revision)
    endpoints = endpoint_centers(env['arm_count'], revision, env['stations'],env['route'])
    extensions = add_end_extensions(root, env, source_model, endpoints, half) if revision == 12 else []
    height = env['physics']['table_height_m']
    for index, (name, center) in enumerate(zip(('start', 'goal'), endpoints)):
        region = env['route'][0 if index == 0 else -1]
        yaw=region['nominal_yaw_rad'] if revision>=13 and env['arm_count']!=8 else 0.
        v2.outline(root, 'relay_'+name, [*center, height+.00035], half.tolist(),yaw=yaw,color=PAINT)
        frame = world.find(f"body[@name='relay_{name}']")
        for geom in frame.findall('geom'):
            size = np.fromstring(geom.get('size'), sep=' ')
            size[np.argmin(size[:2])] = .0025
            geom.set('size', v2.vector(size))
        label_offset = [(-.14 if index == 0 else .14), -.55] if revision >= 12 else [(-.36 if index == 0 else .36), -.57]
        if revision>=13 and env['arm_count']!=8:label_offset=[(-.22 if index==0 else .22),.48]
        v2.label(root, folder, 'relay_'+name+'_label', name.upper(),
                 [*list(center+(v2.rz(yaw)@np.r_[label_offset,0.])[:2]), height+.0007], half=(.15, .045),yaw=yaw)
        region.update(center_m=[*center, height], half_size_m=half.tolist(),
                      nominal_yaw_rad=yaw, painted=True, label=name.upper())
    for region in env['route'][1:-1]:
        region['painted'] = False
    points = [(np.array([x, 0.]), 0.) for x in ((-.9,-.3,.3,.9) if revision >= 12 else (-.30,.30))] if env['arm_count'] == 8 else [
        (1.73*np.array([math.cos(t), math.sin(t)]), t-math.pi/2)
        for t in np.deg2rad([150., 120., 90., 60., 30.])]
    if revision>=13:
        if env['arm_count']==8:points=[(np.array([x,0.]),0.) for x in np.linspace(-.92,.92,7)]
        else:
            angles=np.linspace(128.,52.,9) if revision>=14 and env['arm_count']==12 else np.linspace(142.,38.,9) if env['arm_count']==12 else np.linspace(165.,15.,11)
            radius=env['table_geometry']['route_centerline_radius_m']
            points=[(radius*np.array([math.cos(t),math.sin(t)]),t-math.pi/2) for t in np.deg2rad(angles)]
    for index, (center, yaw) in enumerate(points):
        name = f'relay_direction_{index+1:02d}'
        arrow = ET.SubElement(world, 'body', name=name, pos=v2.vector([*center, height+.0005]),
                              quat=v2.vector(matrix_quat(v2.rz(yaw))))
        box(arrow, name+'_shaft', [-.0375,0.,0.] if revision>=13 else [-.027, 0., 0.],
            [.0825,.015,.00012] if revision>=13 else [.040, .008, .00012], ARROW, visual=True)
        prism_mesh(root, name+'_tip', [[.025,-.055],[.15,0.],[.025,.055]] if revision>=13 else [[.005, -.031], [.073, 0.], [.005, .031]], -.00012, .00012)
        ET.SubElement(arrow, 'geom', name=name+'_tip', type='mesh', mesh=name+'_tip',
                      rgba=v2.vector(ARROW), contype='0', conaffinity='0', mass='0')
    for assembly in env['mechanism']['payload_assemblies']:
        assembly['source_region_painted'] = False
    env['mechanism']['presentation'] = dict(
        revision=revision, endpoint_size_xy_m=(2*half).tolist(),
        start_center_world_m=[*endpoints[0], height], goal_center_world_m=[*endpoints[1], height],
        intermediate_region_frames=False, per_payload_pickup_frames=False,
        arrows=[dict(center_world_m=[*p, height], yaw_rad=y) for p, y in points],
        opening='L2/L3 whole assembly yaw reversed by180 degrees; opening faces the original local route-forward direction',
        end_extensions=extensions,
        physics_scope='Solver, apparatus, robot mounts and controller unchanged; r12 adds non-overlapping same-height end tabletop pieces where required')
    if revision>=13:
        env['mechanism']['presentation']['physics_scope']='Apparatus local mechanics and controller unchanged; arc layout poses and integrated worktop geometry follow the declared radii and angles. No external side tables.'
    env['relay_scene_instructions'] = [
        f'Only START and GOAL are outlined; each is {2*half[0]:.2f} m by {2*half[1]:.2f} m. The GOAL outline is the actual final acceptance boundary, not decoration.',
        'Numbered apparatuses still start at separate positions along the route. START marks the route origin; it does not require all apparatuses to start there or return there. Follow the sparse arrows toward GOAL.',
        'Unpainted route reference anchors do not prescribe handover locations, grasp points, robot pairs or an execution order. Scheduling and independent concurrent work remain planner choices.',
        'Curved and elbow apparatuses initially open forward along the local route (right on the straight table). The weight, closed eyes and any passive hinge move with the complete apparatus and retain their physical connections.'
    ]
    if revision>=15 and env['arm_count']==12:
        env['relay_scene_instructions'].append('The twelve-arm layout has seven outer robots A-G and five inner robots H-L. Ring membership identifies physical placement, not fixed collaboration pairs; choose cooperating robots from their actual reach and clearance.')
    if revision >= 12:
        env['relay_scene_instructions'].append(f"START/GOAL occupy the two normal end landings within the arc tabletop; there are no external side tables. This layout spans {env['table_geometry']['angle_degrees']} degrees. The inner tabletop opening is {2*env['table_geometry']['inner_radius_m']:.2f} m in diameter." if revision>=13 and env['arm_count']!=8 else 'START lies entirely left of all robot mounts; GOAL lies entirely right of them. Existing scattered initial apparatus positions are retained; these markers do not relocate every apparatus to START.')
        # Save fitted camera poses in the actual new source, not just the PNGs.
        from scripts.export_frontier_contact_sheets import fit_camera
        temporary = folder/'.camera-fit.xml'
        try:
            ET.ElementTree(root).write(temporary,encoding='unicode')
            preview_model = mujoco.MjModel.from_xml_path(str(temporary))
            preview_data = mujoco.MjData(preview_model)
            mujoco.mj_resetDataKeyframe(preview_model,preview_data,preview_model.key('initial').id)
            mujoco.mj_forward(preview_model,preview_data)
            for view in ('top','global','global_rear'):
                fit_camera(preview_model,preview_data,view)
                world.find(f"camera[@name='{view}']").set('pos',v2.vector(preview_model.cam_pos[preview_model.camera(view).id]))
        finally:
            temporary.unlink(missing_ok=True)


def apparatus_bounds(model, data, ident):
    bounds = []
    for g in range(model.ngeom):
        name = model.body(int(model.geom_bodyid[g])).name or ''
        if (name == ident or name.startswith(ident+'_')) and (model.geom_contype[g] or model.geom_conaffinity[g]):
            rotation = data.geom_xmat[g].reshape(3, 3)
            center = data.geom_xpos[g]+rotation@model.geom_aabb[g, :3]
            extent = abs(rotation)@model.geom_aabb[g, 3:]
            bounds.append((center-extent, center+extent))
    return np.min([b[0] for b in bounds], axis=0), np.max([b[1] for b in bounds], axis=0)


def validate(folder, source, persist=True):
    """Static checks only: original mechanics, rigid relative poses and paint support."""
    model = mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id)
    mujoco.mj_forward(model, data)
    old = mujoco.MjModel.from_xml_path(str(source/'scene.xml'))
    old_data = mujoco.MjData(old)
    mujoco.mj_resetDataKeyframe(old, old_data, old.key('initial').id)
    mujoco.mj_forward(old, old_data)
    task = json.loads((folder/'task.json').read_text())
    half = endpoint_half(task['scale']['scene_revision'])
    assert model.nq == old.nq and model.nv == old.nv and model.nu == old.nu
    for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat'):
        for i in range(1, old.nbody):
            name = old.body(i).name
            if not name or name.startswith(('route_zone_', 'route_arrow_', 'pickup_')):
                continue
            assert np.allclose(getattr(model, field)[model.body(name).id], getattr(old, field)[i]), (field, name)
    for field in ('actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange',
                  'actuator_ctrlrange', 'jnt_range', 'jnt_stiffness', 'dof_damping'):
        assert np.array_equal(getattr(model, field), getattr(old, field)), field
    for field in ('timestep', 'solver', 'iterations', 'noslip_iterations', 'gravity'):
        assert np.array_equal(getattr(model.opt, field), getattr(old.opt, field)), field
    # Apparatus and robot local geometry stay unchanged. R13 rebuilds the table.
    for g in range(old.ngeom):
        if not (old.geom_contype[g] or old.geom_conaffinity[g]):
            continue
        body = old.body(int(old.geom_bodyid[g])).name or 'world'
        if task['scale']['scene_revision']>=13 and task['scale']['arms']!=8 and (old.geom(g).name or '').startswith('table_'):
            continue
        ordinal = list(np.flatnonzero(old.geom_bodyid == old.geom_bodyid[g])).index(g)
        peers = np.flatnonzero(model.geom_bodyid == model.body(body).id)
        new_g = int(peers[ordinal])
        for field in ('geom_type', 'geom_pos', 'geom_quat', 'geom_size', 'geom_friction', 'geom_solref', 'geom_solimp', 'geom_condim'):
            assert np.allclose(getattr(model, field)[new_g], getattr(old, field)[g]), (field, body, ordinal)
    # A downward ray restricted to tabletop colliders checks both full regions.
    model.geom_group[:] = 5
    tabletop = []
    for g in range(model.ngeom):
        name = model.geom(g).name or ''
        if name.startswith('table_') and 'leg' not in name and (model.geom_contype[g] or model.geom_conaffinity[g]):
            tabletop.append(g); model.geom_group[g] = 4
    group = np.array([0, 0, 0, 0, 1, 0], dtype=np.uint8)
    hit = np.zeros(1, dtype=np.int32)
    samples = 0
    for region in (task['route'][0], task['route'][-1]):
        assert region['half_size_m'] == half.tolist()
        for x in np.linspace(-half[0]-.003, half[0]+.003, 25):
            for y in np.linspace(-half[1]-.003, half[1]+.003, 25):
                point = np.array(region['center_m'])+v2.rz(region['nominal_yaw_rad'])@np.array([x, y, .05])
                distance = mujoco.mj_ray(model, data, point, np.array([0., 0., -1.]), group, 1, -1, hit)
                assert abs(distance-.05) < .001, ('region outside tabletop', region['region_id'], point, distance)
                samples += 1
    first = task['mechanism']['payload_assemblies'][0]['id']
    start = np.array(task['route'][0]['center_m'])
    frame=v2.rz(task['route'][0]['nominal_yaw_rad']);local_bounds=[]
    for g in range(model.ngeom):
        name=model.body(int(model.geom_bodyid[g])).name or ''
        if not(name==first or name.startswith(first+'_')) or not(model.geom_contype[g] or model.geom_conaffinity[g]):continue
        rotation=data.geom_xmat[g].reshape(3,3);center=frame.T@(data.geom_xpos[g]+rotation@model.geom_aabb[g,:3]-start);extent=abs(frame.T@rotation)@model.geom_aabb[g,3:]
        local_bounds.extend([center-extent,center+extent])
    lo,hi=np.min(local_bounds,axis=0),np.max(local_bounds,axis=0)
    first_inside = bool(np.all(lo[:2] >= -half+.01) and np.all(hi[:2] <= half-.01))
    if task['scale']['scene_revision'] < 12: assert first_inside
    if task['scale']['scene_revision'] == 12 or task['scale']['arms']==8:
        mounts = [arm['base_position_m'][0] for arm in task['arms'].values()]
        assert start[0]+half[0] <= min(mounts)-.10
        assert task['route'][-1]['center_m'][0]-half[0] >= max(mounts)+.10
    transformed = []
    apparatus_tabletop_samples = 0
    for assembly in task['mechanism']['payload_assemblies']:
        ident = assembly['id']; root_new = data.body(ident); root_old = old_data.body(ident)
        for g in range(model.ngeom):
            name=model.body(int(model.geom_bodyid[g])).name or ''
            if not(name==ident or name.startswith(ident+'_')) or not(model.geom_contype[g] or model.geom_conaffinity[g]):continue
            rotation=data.geom_xmat[g].reshape(3,3)
            if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                mid=int(model.geom_dataid[g]);offset=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid]);vertices=model.mesh_vert[offset:offset+n]
            else:
                vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*model.geom_aabb[g,3:]+model.geom_aabb[g,:3]
            for point in vertices@rotation.T+data.geom_xpos[g]:
                distance=mujoco.mj_ray(model,data,np.array([point[0],point[1],task['table_height_m']+.05]),np.array([0.,0.,-1.]),group,1,-1,hit)
                assert abs(distance-.05)<.001,('initial apparatus over table edge/hole',ident,point,distance)
                apparatus_tabletop_samples+=1
        turn = root_new.xmat.reshape(3, 3)@root_old.xmat.reshape(3, 3).T
        assert np.allclose(root_new.xpos,v2.rz(assembly.get('layout_yaw_change_rad',0.))@root_old.xpos+np.asarray(assembly.get('layout_translation_world_m',[0.,0.,0.])))
        for i in range(1, old.nbody):
            name = old.body(i).name or ''
            if name != ident and not name.startswith(ident+'_'):
                continue
            assert np.allclose(data.body(name).xpos, root_new.xpos+turn@(old_data.body(i).xpos-root_old.xpos), atol=1e-8), name
            assert np.allclose(data.body(name).xmat.reshape(3,3), turn@old_data.body(i).xmat.reshape(3,3), atol=1e-8), name
        transformed.append(ident)
    forbidden_initial_contacts = []
    for contact in data.contact:
        names = [model.body(int(model.geom_bodyid[g])).name or 'world' for g in (contact.geom1, contact.geom2)]
        payloads = [next((a['id'] for a in task['mechanism']['payload_assemblies']
                         if name == a['id'] or name.startswith(a['id']+'_')), None) for name in names]
        robot_payload = any(payloads[i] and names[1-i].split('_')[0] in task['arms'] for i in (0, 1))
        robots=[name.split('_')[0] for name in names]
        robot_robot=all(r in task['arms'] for r in robots) and robots[0]!=robots[1]
        different_payloads = all(payloads) and payloads[0] != payloads[1]
        if contact.dist < -.001 and (robot_payload or robot_robot or different_payloads):
            forbidden_initial_contacts.append(dict(bodies=names, penetration_m=-float(contact.dist)))
    assert not forbidden_initial_contacts, forbidden_initial_contacts
    result = dict(status='static_checks_passed', endpoint_size_xy_m=(2*half).tolist(),
        tabletop_samples=samples, endpoint_regions_fully_supported=True,
        complete_assembly_footprint_tabletop_samples=apparatus_tabletop_samples,
        first_complete_assembly_inside_start=first_inside, complete_rigid_transform_checked=transformed,
        endpoint_regions_outside_all_robot_mounts=task['scale']['scene_revision']==12 or task['scale']['arms']==8,
        endpoint_regions_within_integrated_arc_landings=task['scale']['scene_revision']>=13 and task['scale']['arms']!=8,
        added_table_end_extensions=task['mechanism']['presentation'].get('end_extensions',[]),
        original_apparatus_and_robot_local_mechanics_preserved=True,
        table_geometry=task.get('mechanism',{}).get('layout'),
        initial_robot_payload_or_different_payload_penetrations_over_1mm=forbidden_initial_contacts,
        source_directory=str(source.relative_to(v2.ROOT)), physics_steps=0,
        oracle_verified=False, policy_verified=False,
        scope='Static initial geometry and metadata only; no grasp, dynamics, collision-free motion or complete-task feasibility claim')
    if persist:write_json(folder/'static-checks.json', result)
    return result


def build(case, replace_draft=False, render=True):
    from .reframe import build as clone
    source_case = dict(case, scene_revision=10)
    source = v2.directory(source_case)
    source_model = mujoco.MjModel.from_xml_path(str(source/'scene.xml'))
    folder = clone(case, replace_draft, render=False,
        transform=lambda root, env: revise(root, env, v2.directory(case), source_model))
    if case['level']==1 and case['scene_revision'] in (13,15):
        from .relay_contract import write_task
        write_task(folder)
    result = validate(folder, source)
    saved = json.loads((folder/'validation.json').read_text())
    saved.update(status='static_checks_passed', scope=result['scope'])
    write_json(folder/'validation.json', saved)
    if render:
        model = mujoco.MjModel.from_xml_path(str(folder/'scene.xml')); data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id); mujoco.mj_forward(model, data)
        model.vis.global_.offwidth=1920; model.vis.global_.offheight=1200
        with mujoco.Renderer(model, width=1920, height=1200, max_geom=max(5000, model.ngeom+1000)) as renderer:
            for view in ('top', 'global'):
                renderer.update_scene(data, camera=view)
                from PIL import Image
                Image.fromarray(renderer.render()).save(folder/('preview-'+view+'.png'))
    return folder
