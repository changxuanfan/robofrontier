"""Run with Blender --background --python SCRIPT -- --asset DIR --output DIR.

Offline presentation only. Source FK geometry is carried exactly; lighting is
render-specific. No controller, IK, contacts, or task physics run in Blender.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess
import time

import bpy
import bmesh
import numpy as np
from mathutils import Matrix,Vector


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--views',nargs='+',default=['interior_south','interior_diagonal'])
    parser.add_argument('--width',type=int,default=1280);parser.add_argument('--height',type=int,default=800)
    parser.add_argument('--exposure',type=float,default=-1.5);parser.add_argument('--samples',type=int,default=64);parser.add_argument('--device',choices=['METAL','CPU'],default='METAL')
    parser.add_argument('--save-blend',action='store_true');parser.add_argument('--usd',action='store_true')
    parser.add_argument('--recorded-pose',type=Path,help='Derived recorded-state pose, matching this exact factory; no physics')
    parser.add_argument('--animation',type=Path,help='Common source-bound animation.json; 30fps independent station loops')
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:]);started=time.monotonic()
    args.asset=args.asset.resolve();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    metadata=json.loads((args.asset/'scene.json').read_text());arrays=np.load(args.asset/'meshes.npz')
    bpy.ops.wm.read_factory_settings(use_empty=True);scene=bpy.context.scene
    scene.render.engine='CYCLES';scene.cycles.samples=args.samples;scene.cycles.use_denoising=True
    scene.render.use_persistent_data=True
    scene.render.resolution_x=args.width;scene.render.resolution_y=args.height;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
    scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1.
    scene.view_settings.view_transform='AgX';scene.view_settings.look='AgX - Medium High Contrast';scene.view_settings.exposure=args.exposure
    scene.cycles.max_bounces=8;scene.cycles.diffuse_bounces=4;scene.cycles.glossy_bounces=4
    prefs=bpy.context.preferences.addons['cycles'].preferences
    if args.device=='METAL':
        prefs.compute_device_type='METAL';prefs.get_devices()
        for device in prefs.devices:device.use=device.type=='METAL'
        if not any(d.use for d in prefs.devices):raise RuntimeError('Requested Metal GPU is unavailable')
        scene.cycles.device='GPU'
    else:scene.cycles.device='CPU';scene.render.threads_mode='FIXED';scene.render.threads=4
    images={};textures={x['id']:x for x in metadata['textures']}
    def image(tid,noncolor=False):
        key=(tid,noncolor)
        if key not in images:
            im=bpy.data.images.load(str(args.asset/textures[tid]['file']),check_existing=not noncolor)
            if noncolor:im.colorspace_settings.name='Non-Color'
            images[key]=im
        return images[key]
    materials={}
    def make_material(spec):
        mat=bpy.data.materials.new(spec['name']);mat.use_nodes=True;nodes=mat.node_tree.nodes;links=mat.node_tree.links
        bsdf=nodes.get('Principled BSDF');bsdf.inputs['Base Color'].default_value=spec['rgba']
        bsdf.inputs['Metallic'].default_value=spec['metallic'];bsdf.inputs['Roughness'].default_value=max(.22,spec['roughness'])
        bsdf.inputs['Alpha'].default_value=spec['rgba'][3]
        if 'glass' in spec['name']:
            bsdf.inputs['Transmission Weight'].default_value=.85;bsdf.inputs['Roughness'].default_value=.12
        roles=spec.get('textures',{})
        for role in ('mjTEXROLE_RGB','mjTEXROLE_RGBA','mjTEXROLE_ROUGHNESS','mjTEXROLE_NORMAL'):
            tid=roles.get(role)
            if tid not in textures:continue
            tex=nodes.new('ShaderNodeTexImage');tex.image=image(tid,role in ('mjTEXROLE_ROUGHNESS','mjTEXROLE_NORMAL'))
            tex.extension='REPEAT';uv=nodes.new('ShaderNodeUVMap');mapping=nodes.new('ShaderNodeVectorMath');mapping.operation='MULTIPLY'
            mapping.inputs[1].default_value=(*spec['texrepeat'],1.)
            links.new(uv.outputs['UV'],mapping.inputs[0]);links.new(mapping.outputs['Vector'],tex.inputs['Vector'])
            if role=='mjTEXROLE_NORMAL':
                normal=nodes.new('ShaderNodeNormalMap');normal.inputs['Strength'].default_value=.3
                links.new(tex.outputs['Color'],normal.inputs['Color']);links.new(normal.outputs['Normal'],bsdf.inputs['Normal'])
            elif role=='mjTEXROLE_ROUGHNESS':links.new(tex.outputs['Color'],bsdf.inputs['Roughness'])
            else:
                mix=nodes.new('ShaderNodeMixRGB');mix.blend_type='MULTIPLY';mix.inputs[0].default_value=1.;mix.inputs[2].default_value=spec['rgba']
                links.new(tex.outputs['Color'],mix.inputs[1]);links.new(mix.outputs[0],bsdf.inputs['Base Color'])
        return mat
    for spec in metadata['materials']:materials[spec['id']]=make_material(spec)
    fallback={}
    def geom_material(g):
        if g['material']>=0:return materials[g['material']]
        color=tuple(g['rgba'])
        if color not in fallback:fallback[color]=make_material(dict(name='rgba_'+str(len(fallback)),rgba=color,roughness=.45,metallic=0.,texrepeat=[1,1]))
        return fallback[color]
    appearance_materials={}
    material_specs={m['id']:m for m in metadata['materials']}
    def appearance_material(original,rgba):
        if np.array_equal(rgba,np.asarray(original['rgba'],dtype=np.float32)):
            return geom_material(original)
        if original['material']<0:return geom_material(dict(original,rgba=list(rgba)))
        key=(original['material'],tuple(rgba))
        if key not in appearance_materials:
            spec=dict(material_specs[original['material']],rgba=list(rgba))
            appearance_materials[key]=make_material(spec)
        return appearance_materials[key]
    meshes={}
    for spec in metadata['meshes']:
        i=spec['id'];mesh=bpy.data.meshes.new(spec['name']);mesh.from_pydata(arrays[f'v{i}'].tolist(),[],arrays[f'f{i}'].tolist());mesh.update()
        if spec['textured']:
            uv=mesh.uv_layers.new(name='UVMap');coords=arrays[f'uv{i}'];indices=arrays[f'fuv{i}']
            for polygon,faceuv in zip(mesh.polygons,indices):
                for loop,index in zip(polygon.loop_indices,faceuv):
                    if 0<=index<len(coords):uv.data[loop].uv=(float(coords[index][0]),1.-float(coords[index][1])) # MuJoCo-authored image origin is top-left.
        for polygon in mesh.polygons:polygon.use_smooth=not spec['textured']
        mesh.materials.append(next(iter(materials.values())));meshes[('mesh',i)]=mesh
    def primitive(kind):
        key=('primitive',kind)
        if key in meshes:return meshes[key]
        mesh=bpy.data.meshes.new('primitive_'+str(kind));bm=bmesh.new()
        if kind==6:bmesh.ops.create_cube(bm,size=2.)
        elif kind==5:bmesh.ops.create_cone(bm,cap_ends=True,cap_tris=False,segments=40,radius1=1.,radius2=1.,depth=2.)
        elif kind==3:bmesh.ops.create_uvsphere(bm,u_segments=24,v_segments=12,radius=1.)
        bm.to_mesh(mesh);bm.free();mesh.update();uv=mesh.uv_layers.new(name='UVMap')
        for polygon in mesh.polygons:
            axis=max(range(3),key=lambda i:abs(polygon.normal[i]))
            for li in polygon.loop_indices:
                co=mesh.vertices[mesh.loops[li].vertex_index].co
                ij=((1,2),(0,2),(0,1))[axis];uv.data[li].uv=((co[ij[0]]+1)/2,(co[ij[1]]+1)/2)
            polygon.use_smooth=kind==3 or (kind==5 and abs(polygon.normal.z)<.5)
        mesh.materials.append(next(iter(materials.values())));meshes[key]=mesh;return mesh
    max_position_error=0.;max_rotation_error=0.;objects={}
    for g in metadata['geoms']:
        mesh=meshes[('mesh',g['mesh'])] if g['kind']==7 else primitive(g['kind'])
        obj=bpy.data.objects.new(g['name'] or f'geom_{g["id"]}',mesh);scene.collection.objects.link(obj)
        obj.rotation_mode='QUATERNION';obj.rotation_quaternion=Matrix(g['rotation']).to_quaternion();obj.location=g['position']
        if g['kind']==6:obj.scale=g['size']
        elif g['kind']==5:obj.scale=(g['size'][0],g['size'][0],g['size'][1])
        elif g['kind']==3:obj.scale=(g['size'][0],)*3
        obj.material_slots[0].link='OBJECT';obj.material_slots[0].material=geom_material(g)
        obj['source_geom_id']=g['id'];obj['source_geom_name']=g['name'];obj['presentation_only']=True;objects[g['id']]=obj
        max_position_error=max(max_position_error,float(np.max(abs(np.asarray(obj.location,dtype=float)-np.array(g['position'])))))
        # Blender stores object transforms as float32, including unit rotations.
        reconstructed=np.asarray(obj.rotation_quaternion.to_matrix());max_rotation_error=max(max_rotation_error,float(np.max(abs(reconstructed-np.array(g['rotation'])))))
    if max_position_error>1e-5 or max_rotation_error>1e-6:raise RuntimeError(('Visual transform conversion exceeds float32 tolerance',max_position_error,max_rotation_error))
    recorded=None
    if args.recorded_pose:
        recorded=json.loads(args.recorded_pose.read_text())
        if (recorded.get('schema')!='robofrontier.factory-recorded-pose.v1'
                or recorded.get('factory_scene_sha256')!=metadata['source_scene_sha256']):
            raise ValueError('Recorded pose does not match the factory asset')
        specs={g['id']:g for g in metadata['geoms']};seen=set()
        for pose in recorded['geoms']:
            gid=pose['id'];obj=objects[gid];original=specs[gid]
            if gid in seen or pose['name']!=original['name']:raise ValueError('Recorded geometry identity mismatch')
            seen.add(gid);position=np.asarray(pose['position']);rotation=np.asarray(pose['rotation']);rgba=np.asarray(pose['rgba'])
            if (position.shape!=(3,) or rotation.shape!=(3,3) or rgba.shape!=(4,)
                    or not all(np.isfinite(x).all() for x in (position,rotation,rgba))
                    or np.max(abs(rotation.T@rotation-np.eye(3)))>1e-6
                    or abs(np.linalg.det(rotation)-1)>1e-6 or np.any(rgba<0) or np.any(rgba>1)):
                raise ValueError('Invalid recorded visual transform/appearance')
            obj.location=position;obj.rotation_quaternion=Matrix(rotation.tolist()).to_quaternion()
            if 'size' in pose:
                size=np.asarray(pose['size'],dtype=float)
                if size.shape!=(3,)or not np.isfinite(size).all()or np.any(size<0):raise ValueError('Invalid recorded geometry dimensions')
                if original['kind']==6:obj.scale=size
                elif original['kind']==5:obj.scale=(size[0],size[0],size[1])
                elif original['kind']==3:obj.scale=(size[0],)*3
            max_position_error=max(max_position_error,float(np.max(abs(np.asarray(obj.location)-position))))
            max_rotation_error=max(max_rotation_error,float(np.max(abs(np.asarray(obj.rotation_quaternion.to_matrix())-rotation))))
            if not np.allclose(rgba,original['rgba'],rtol=0,atol=1e-7):
                # Appearance events are per geometry, never a shared-material
                # mutation that could change other workstations or digits.
                spec=dict(original,material=-1,rgba=rgba.tolist())
                obj.material_slots[0].material=geom_material(spec)
        metadata['cameras'].append(recorded['camera'])
        if max_position_error>1e-5 or max_rotation_error>1e-6:raise RuntimeError('Recorded visual transform precision exceeded')
    # Offline interior lighting: large soft ceiling fixtures, calibrated separately
    # from Filament intensity units. These lights never enter task source models.
    world=bpy.data.worlds.new('Factory ambient');scene.world=world;world.use_nodes=True
    world.node_tree.nodes['Background'].inputs[0].default_value=(.65,.72,.85,1.)
    world.node_tree.nodes['Background'].inputs[1].default_value=.25
    for x in (-20.,-12.,-4.,4.,12.,20.):
        for y in (-20.,-10.,0.,10.,20.):
            lamp=bpy.data.lights.new('Ceiling softbox','AREA');lamp.energy=1800.;lamp.shape='RECTANGLE';lamp.size=4.;lamp.size_y=2.
            obj=bpy.data.objects.new(lamp.name,lamp);scene.collection.objects.link(obj);obj.location=(x,y,6.5)
    sun=bpy.data.lights.new('Daylight','SUN');sun.energy=1.5;sun.angle=math.radians(8.)
    obj=bpy.data.objects.new('Daylight',sun);scene.collection.objects.link(obj);obj.rotation_euler=(math.radians(25),math.radians(-30),math.radians(-15))
    camera_data=bpy.data.cameras.new('Factory camera');camera=bpy.data.objects.new('Factory camera',camera_data);scene.collection.objects.link(camera);scene.camera=camera
    camera_data.clip_start=.03;camera_data.clip_end=250.
    report=dict(schema='robofrontier.blender-render.v1',source_scene_sha256=metadata['source_scene_sha256'],blender_version=bpy.app.version_string,
        device=args.device,exposure_ev=args.exposure,physics_stepped=False,visual_objects=len(metadata['geoms']),shared_meshes=len(meshes),
        max_position_error_m=max_position_error,max_rotation_matrix_error=max_rotation_error,
        scope='Offline visual rendering; no physics/Isaac dynamics conversion',views=[])
    if recorded:
        report['recorded_pose']=dict(case_id=recorded['case_id'],time_s=recorded['selected_recorded_time_s'],geometries=len(recorded['geoms']),
            source_scene_sha256=recorded['source_scene_sha256'],motion_index_sha256=recorded['motion_index_sha256'])
    def render_camera(source,file):
        position=Vector(source['position_m']);target=Vector(source['target_m']);camera.location=position
        camera.rotation_euler=(target-position).to_track_quat('-Z','Y').to_euler()
        if source.get('projection')=='orthographic':
            camera_data.type='ORTHO';camera_data.ortho_scale=source['orthographic_height_m']*args.width/args.height
        else:
            camera_data.type='PERSP';camera_data.sensor_fit='VERTICAL';camera_data.sensor_height=24.;camera_data.lens=12./math.tan(math.radians(source['vertical_fov_deg'])/2)
        hidden=set(source.get('hidden_layers',[]));layers={'structure':0,'roof':1,'north':2,'south':3,'west':4,'east':5}
        for g in metadata['geoms']:
            obj=objects[g['id']];obj.hide_render=g['group'] in {layers[x] for x in hidden if x in layers}
        scene.render.filepath=str(file);before=time.monotonic();bpy.ops.render.render(write_still=True)
        return dict(camera=source.get('id','tour'),file=scene.render.filepath,wall_s=time.monotonic()-before,samples=args.samples,resolution=[args.width,args.height])
    if args.animation:
        if args.recorded_pose:raise ValueError('Select one animation or one recorded pose')
        animation=json.loads(args.animation.read_text())
        if (animation['schema']!='robofrontier.factory-animation.v1' or animation['fps']!=30
                or animation['source_scene_sha256']!=metadata['source_scene_sha256']):
            raise ValueError('Animation source or cadence differs')
        trace=args.animation.parent/'frames.jsonl'
        if hashlib.sha256(trace.read_bytes()).hexdigest()!=animation['source_trace_sha256']:
            raise ValueError('Animation source trace differs')
        specs={g['id']:g for g in metadata['geoms']};last_rgba={}
        for index,frame in enumerate(animation['frames']):
            path=args.animation.parent/frame['file']
            if path.resolve().parent!=args.animation.parent.resolve() or hashlib.sha256(path.read_bytes()).hexdigest()!=frame['sha256']:
                raise ValueError('Animation pose differs')
            with np.load(path,allow_pickle=False) as pose:
                ids=pose['geom_ids'];positions=pose['position'];rotations=pose['rotation'];colors=pose['rgba']
                if (positions.shape!=(len(ids),3) or rotations.shape!=(len(ids),3,3) or colors.shape!=(len(ids),4)
                        or len(set(ids.tolist()))!=len(ids) or not all(np.isfinite(a).all() for a in (positions,rotations,colors))):
                    raise ValueError('Invalid animation pose arrays')
                for gid,position,rotation,rgba in zip(ids,positions,rotations,colors):
                    gid=int(gid);obj=objects[gid]
                    obj.location=position;obj.rotation_quaternion=Matrix(rotation).to_quaternion()
                    color=tuple(rgba)
                    if color!=last_rgba.get(gid,tuple(specs[gid]['rgba'])):
                        obj.material_slots[0].material=appearance_material(specs[gid],rgba)
                    last_rgba[gid]=color
            report['views'].append(render_camera(frame['camera'],args.output/f'frame-{index:06d}.png'))
            (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        count=len(animation['frames'])
        if not count:raise ValueError('Empty animation')
        subprocess.run(['ffmpeg','-v','error','-threads','1','-framerate','30','-i',str(args.output/'frame-%06d.png'),
            '-frames:v',str(count),'-c:v','libx264','-threads','1','-pix_fmt','yuv420p','-crf','18','-movflags','+faststart',str(args.output/'factory.mp4')],check=True)
        subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(args.output/'factory.mp4'),'-f','null','-'],check=True)
        report['animation']=dict(frames=count,fps=30,stations=animation['stations'],loop=animation['loop'],
            source_trace=str(trace.resolve()),full_decode_passed=True)
    else:
      for name in args.views:
        source=next(c for c in metadata['cameras'] if c['id']==name)
        report['views'].append(render_camera(source,args.output/(name+'.png')))
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.save_blend:bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'factory.blend'))
    if args.usd:
        bpy.ops.wm.usd_export(filepath=str(args.output/'factory.usdc'),export_materials=True,export_textures_mode='NEW',export_custom_properties=True,xform_op_mode='MAT')
        report['usd_scope']='Visual export only; target Isaac compatibility not validated'
    report['total_wall_s']=time.monotonic()-started;(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
