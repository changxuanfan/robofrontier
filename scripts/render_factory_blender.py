"""Run with Blender --background --python SCRIPT -- --asset DIR --output DIR.

Offline presentation only. Source FK frames are preserved; the fast profile
simplifies robot visual meshes. No controller, IK, contacts or physics run here.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess
import time
import os
import platform
import plistlib
import threading

import bpy
import bmesh
import numpy as np
from mathutils import Matrix,Vector

sys.path.insert(0,str(Path(__file__).resolve().parent))
from factory_blender_materials import resolve as resolve_material, robot_role, task_role, ROBOT_BODY

SCRIPT_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--views',nargs='+',default=['interior_south','interior_diagonal'])
    parser.add_argument('--width',type=int,default=1280);parser.add_argument('--height',type=int,default=800)
    parser.add_argument('--exposure',type=float);parser.add_argument('--samples',type=int,default=64);parser.add_argument('--device',choices=['METAL','CPU'],default='METAL')
    parser.add_argument('--engine',choices=['cycles','eevee'],default='cycles')
    parser.add_argument('--profile',choices=['reference','fast'],default='reference')
    parser.add_argument('--benchmark-frames',type=int,default=1)
    parser.add_argument('--instances',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--raytracing',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--lighting-config',type=Path,help='Offline lighting overrides; source task state stays frozen')
    parser.add_argument('--camera-config',type=Path,help='Named offline camera overrides for matched-engine look development')
    parser.add_argument('--bake-gi',action='store_true',help='Bake static architecture GI once; moving task geometry excluded')
    parser.add_argument('--denoise',action=argparse.BooleanOptionalAction,default=True,help='GPU denoising for low-sample output')
    parser.add_argument('--save-blend',action='store_true');parser.add_argument('--usd',action='store_true')
    parser.add_argument('--recorded-pose',type=Path,help='Derived recorded-state pose, matching this exact factory; no physics')
    parser.add_argument('--animation',type=Path,help='Common source-bound animation.json; 30fps independent station loops')
    parser.add_argument('--frame-indices',nargs='+',type=int,help='Render selected source frames for matched-state quality benchmarks')
    parser.add_argument('--encode',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--keep-frames',action=argparse.BooleanOptionalAction,default=False,
        help='Keep all animation PNGs; otherwise retain three stills after video verification')
    parser.add_argument('--metalrt',choices=['AUTO','ON','OFF'],default='AUTO')
    parser.add_argument('--noise-threshold',type=float,default=.1,help='Cycles adaptive threshold override; geometry and lighting stay fixed')
    parser.add_argument('--min-samples',type=int,default=4,help='Minimum samples before adaptive stopping; zero uses the Cycles automatic value')
    parser.add_argument('--png-compression',type=int,default=15)
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:]);started=time.monotonic()
    if min(args.samples,args.width,args.height,args.benchmark_frames)<=0:raise ValueError('Positive samples, resolution and repeats required')
    if not 0<=args.png_compression<=100:raise ValueError('PNG compression must be in [0,100]')
    if args.frame_indices is not None and not args.animation:raise ValueError('Frame selection requires an animation')
    if args.noise_threshold is not None and not 0<args.noise_threshold<=1:raise ValueError('Noise threshold must be in (0,1]')
    if args.min_samples is not None and not 0<=args.min_samples<=args.samples:raise ValueError('Minimum samples must be between zero and maximum samples')
    if (args.output/'report.json').exists():raise FileExistsError('Choose a new render output; existing reports and frames are preserved')
    gpu_samples=[];gpu_stop=threading.Event();gpu_hardware={}
    def monitor_gpu():
        while not gpu_stop.is_set():
            try:
                devices=plistlib.loads(subprocess.check_output(['/usr/sbin/ioreg','-a','-r','-c','AGXAccelerator']))
                if devices:
                    record=devices[0];gpu_hardware.update(model=record.get('model'),cores=record.get('gpu-core-count'))
                    stats=record.get('PerformanceStatistics',{})
                    gpu_samples.append(dict(elapsed_s=time.monotonic()-started,utilization_percent=stats.get('Device Utilization %')))
            except (OSError,ValueError,subprocess.CalledProcessError):return
            gpu_stop.wait(.5)
    if platform.system()=='Darwin':threading.Thread(target=monitor_gpu,daemon=True).start()
    args.asset=args.asset.resolve();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    metadata=json.loads((args.asset/'scene.json').read_text());arrays=np.load(args.asset/'meshes.npz')
    if args.camera_config:
        overrides=json.loads(args.camera_config.read_text())
        metadata['cameras']=[overrides.get(c['id'],c)for c in metadata['cameras']]
    lighting=metadata.get('offline_lighting',{})
    if args.lighting_config:lighting=dict(lighting,**json.loads(args.lighting_config.read_text()))
    if args.exposure is None:args.exposure=lighting.get('exposure_ev',-1.5)
    animation=json.loads(args.animation.read_text()) if args.animation else None
    bpy.ops.wm.read_factory_settings(use_empty=True);scene=bpy.context.scene
    scene.render.engine='CYCLES' if args.engine=='cycles' else 'BLENDER_EEVEE'
    scene.cycles.samples=args.samples;scene.cycles.use_denoising=args.denoise
    scene.render.use_persistent_data=True
    scene.render.resolution_x=args.width;scene.render.resolution_y=args.height;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
    scene.render.image_settings.compression=args.png_compression
    scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1.
    scene.view_settings.view_transform='AgX';scene.view_settings.look=lighting.get('look','AgX - Medium High Contrast');scene.view_settings.exposure=args.exposure
    scene.cycles.max_bounces=8;scene.cycles.diffuse_bounces=4;scene.cycles.glossy_bounces=4
    if args.profile=='fast':
        scene.cycles.max_bounces=3;scene.cycles.diffuse_bounces=2;scene.cycles.glossy_bounces=2
        scene.cycles.transmission_bounces=2;scene.cycles.volume_bounces=0
        scene.cycles.adaptive_threshold=.1;scene.cycles.adaptive_min_samples=min(4,args.samples)
    if args.noise_threshold is not None:scene.cycles.adaptive_threshold=args.noise_threshold
    scene.cycles.use_adaptive_sampling=True
    if args.min_samples is not None:scene.cycles.adaptive_min_samples=args.min_samples
    scene.eevee.taa_render_samples=args.samples
    scene.eevee.use_raytracing=args.raytracing;scene.eevee.use_fast_gi=True
    scene.eevee.ray_tracing_options.resolution_scale=str(lighting.get('eevee_ray_resolution',4))
    scene.eevee.fast_gi_ray_count=lighting.get('eevee_gi_rays',2);scene.eevee.fast_gi_step_count=8
    scene.eevee.shadow_ray_count=1 if args.profile=='fast' else 2
    scene.eevee.shadow_step_count=4 if args.profile=='fast' else 8
    if args.denoise and args.engine=='eevee':
        graph=bpy.data.node_groups.new('Factory GPU denoising','CompositorNodeTree')
        graph.interface.new_socket(name='Image',in_out='OUTPUT',socket_type='NodeSocketColor')
        scene.compositing_node_group=graph
        layer=graph.nodes.new('CompositorNodeRLayers');denoise=graph.nodes.new('CompositorNodeDenoise');output=graph.nodes.new('NodeGroupOutput')
        scene.view_layers[0].use_pass_normal=True;scene.view_layers[0].use_pass_diffuse_color=True
        graph.links.new(layer.outputs['Image'],denoise.inputs['Image'])
        graph.links.new(layer.outputs['Normal'],denoise.inputs['Normal']);graph.links.new(layer.outputs['Diffuse Color'],denoise.inputs['Albedo'])
        graph.links.new(denoise.outputs['Image'],output.inputs['Image'])
        scene.render.compositor_device='GPU';scene.render.compositor_denoise_device='GPU'
        scene.render.compositor_denoise_final_quality='BALANCED'if args.profile=='fast'else 'HIGH'
    prefs=bpy.context.preferences.addons['cycles'].preferences
    scene.cycles.denoising_use_gpu=True
    if args.device=='METAL':
        prefs.compute_device_type='METAL';prefs.metalrt=args.metalrt;prefs.get_devices()
        for device in prefs.devices:device.use=device.type=='METAL'
        if not any(d.use for d in prefs.devices):raise RuntimeError('Requested Metal GPU is unavailable')
        scene.cycles.device='GPU'
    else:scene.cycles.device='CPU';scene.render.threads_mode='AUTO'
    images={};textures={x['id']:x for x in metadata['textures']}
    def image(tid,noncolor=False):
        key=(tid,noncolor)
        if key not in images:
            im=bpy.data.images.load(str(args.asset/textures[tid]['file']),check_existing=not noncolor)
            if noncolor:im.colorspace_settings.name='Non-Color'
            images[key]=im
        return images[key]
    materials={};material_audit=[]
    def make_material(spec,role=None):
        spec=dict(spec,**lighting.get('material_overrides',{}).get(spec['name'],{}))
        spec=resolve_material(spec,role,lighting.get('robot_materials',True))
        if role:spec['name']+='__'+role
        material_audit.append(dict(name=spec['name'],role=role,rgba=spec['rgba'],roughness=spec['roughness'],metallic=spec['metallic']))
        mat=bpy.data.materials.new(spec['name']);mat.use_nodes=True;nodes=mat.node_tree.nodes;links=mat.node_tree.links
        bsdf=nodes.get('Principled BSDF');bsdf.inputs['Base Color'].default_value=spec['rgba']
        bsdf.inputs['Metallic'].default_value=spec['metallic'];bsdf.inputs['Roughness'].default_value=spec['roughness']
        bsdf.inputs['Specular IOR Level'].default_value=spec.get('specular',.25)
        bsdf.inputs['Alpha'].default_value=spec['rgba'][3]
        if spec.get('emission',0):
            bsdf.inputs['Emission Color'].default_value=spec['rgba']
            bsdf.inputs['Emission Strength'].default_value=spec['emission']*lighting.get('emission_scale',4.)
        if 'glass' in spec['name']:
            bsdf.inputs['Base Color'].default_value=(.92,.96,1.,1.)
            bsdf.inputs['Transmission Weight'].default_value=1.;bsdf.inputs['Roughness'].default_value=.025
            bsdf.inputs['Alpha'].default_value=1.
            mat.use_raytrace_refraction=True
            if args.engine=='cycles':
                path=nodes.new('ShaderNodeLightPath');clear=nodes.new('ShaderNodeBsdfTransparent')
                clear.inputs[0].default_value=(.94,.94,.94,1)
                mix=nodes.new('ShaderNodeMixShader');links.new(path.outputs['Is Shadow Ray'],mix.inputs[0])
                links.new(bsdf.outputs['BSDF'],mix.inputs[1]);links.new(clear.outputs[0],mix.inputs[2])
                links.new(mix.outputs[0],nodes.get('Material Output').inputs['Surface'])
            if args.engine=='eevee':
                # Distant planar panes need no expensive/noisy screen-space
                # refraction; smooth alpha blending preserves the yard view.
                mat.use_raytrace_refraction=False;mat.surface_render_method='BLENDED'
                bsdf.inputs['Transmission Weight'].default_value=0.;bsdf.inputs['Alpha'].default_value=.10
        roles=spec.get('textures',{})
        for texture_role in ('mjTEXROLE_RGB','mjTEXROLE_RGBA','mjTEXROLE_ROUGHNESS','mjTEXROLE_NORMAL'):
            tid=roles.get(texture_role)
            if tid not in textures:continue
            tex=nodes.new('ShaderNodeTexImage');tex.image=image(tid,texture_role in ('mjTEXROLE_ROUGHNESS','mjTEXROLE_NORMAL'))
            tex.extension='REPEAT';uv=nodes.new('ShaderNodeUVMap');mapping=nodes.new('ShaderNodeVectorMath');mapping.operation='MULTIPLY'
            mapping.inputs[1].default_value=(*spec['texrepeat'],1.)
            links.new(uv.outputs['UV'],mapping.inputs[0]);links.new(mapping.outputs['Vector'],tex.inputs['Vector'])
            if texture_role=='mjTEXROLE_NORMAL':
                normal=nodes.new('ShaderNodeNormalMap');normal.inputs['Strength'].default_value=.3
                links.new(tex.outputs['Color'],normal.inputs['Color']);links.new(normal.outputs['Normal'],bsdf.inputs['Normal'])
            elif texture_role=='mjTEXROLE_ROUGHNESS':
                if spec['name']=='factory_floor' or spec['name'].startswith('factory_zone_floor_'):
                    finish=nodes.new('ShaderNodeMapRange')
                    finish.inputs['To Min'].default_value=.27;finish.inputs['To Max'].default_value=.47
                    links.new(tex.outputs['Color'],finish.inputs['Value']);links.new(finish.outputs['Result'],bsdf.inputs['Roughness'])
                else:links.new(tex.outputs['Color'],bsdf.inputs['Roughness'])
            else:
                mix=nodes.new('ShaderNodeMixRGB');mix.blend_type='MULTIPLY';mix.inputs[0].default_value=1.;mix.inputs[2].default_value=spec['rgba']
                color=tex.outputs['Color']
                if lighting.get('normalize_paint_textures',False) and spec['name'] in ('factory_wall','factory_roof'):
                    # These grayscale images describe paint/rib variation, not
                    # a second dark paint layer over the chosen base albedo.
                    normalization=nodes.new('ShaderNodeVectorMath');normalization.operation='SCALE'
                    normalization.inputs['Scale'].default_value=1.8 if spec['name']=='factory_wall' else 1.45
                    links.new(color,normalization.inputs[0]);color=normalization.outputs['Vector']
                links.new(color,mix.inputs[1]);links.new(mix.outputs[0],bsdf.inputs['Base Color'])
        if role=='painted_wood' or spec['name']=='factory_bench_wood':
            # Fine sealed grain, in stable local UVs; no displacement or color
            # changes to task loads and their printed markings.
            uv=nodes.new('ShaderNodeUVMap');stretch=nodes.new('ShaderNodeVectorMath');stretch.operation='MULTIPLY'
            stretch.inputs[1].default_value=(3.,65.,1.)
            noise=nodes.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=2.;noise.inputs['Detail'].default_value=2.
            bump=nodes.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.10;bump.inputs['Distance'].default_value=.00015
            links.new(uv.outputs['UV'],stretch.inputs[0]);links.new(stretch.outputs[0],noise.inputs['Vector'])
            links.new(noise.outputs['Fac'],bump.inputs['Height']);links.new(bump.outputs['Normal'],bsdf.inputs['Normal'])
        return mat
    for spec in metadata['materials']:materials[spec['id']]=make_material(spec)
    fallback={};robot_materials={};task_materials={}
    body_names={b['id']:b['name'] for b in metadata.get('bodies',[])}
    robot_geoms={g['id'] for g in metadata['geoms'] if ROBOT_BODY.search(body_names.get(g.get('body_id'),''))}
    source_materials={m['id']:m for m in metadata['materials']}
    def geom_material(g):
        if g['id'] in robot_geoms and g['material']>=0:
            mid=g['material']
            if mid not in robot_materials:
                spec=source_materials[mid];robot_materials[mid]=make_material(spec,robot_role(spec))
            return robot_materials[mid]
        role=task_role(g)
        if role:
            key=(g['material'],tuple(g['rgba']),role)
            if key not in task_materials:
                spec=source_materials[g['material']]if g['material']>=0 else dict(name='task_'+str(len(task_materials)),rgba=g['rgba'],roughness=-1.,metallic=-1.,texrepeat=[1,1])
                task_materials[key]=make_material(spec,role)
            return task_materials[key]
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
            appearance_materials[key]=make_material(spec,task_role(original))
        return appearance_materials[key]
    meshes={}
    flat_meshes={g['mesh']for g in metadata['geoms']if g['kind']==7 and
        g['material']>=0 and 'factory_bench_wood'in material_specs[g['material']]['name']}
    for spec in metadata['meshes']:
        i=spec['id'];mesh=bpy.data.meshes.new(spec['name']);mesh.from_pydata(arrays[f'v{i}'].tolist(),[],arrays[f'f{i}'].tolist());mesh.update()
        if spec['textured']:
            uv=mesh.uv_layers.new(name='UVMap');coords=arrays[f'uv{i}'];indices=arrays[f'fuv{i}']
            for polygon,faceuv in zip(mesh.polygons,indices):
                for loop,index in zip(polygon.loop_indices,faceuv):
                    if 0<=index<len(coords):uv.data[loop].uv=(float(coords[index][0]),1.-float(coords[index][1])) # MuJoCo-authored image origin is top-left.
        for polygon in mesh.polygons:
            polygon.use_smooth=not spec['textured'] and spec['id']not in flat_meshes and not spec['name'].startswith('factory_')
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
    grouped=None
    if metadata.get('bodies') and not args.recorded_pose and (not animation or 'appearance_geom_ids'in animation):
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from factory_blender_objects import build
        camera_positions=[f['camera']['position_m']for f in animation['frames']]if animation else [c['position_m']for c in metadata['cameras']if c['id']in args.views]
        grouped=build(metadata,meshes,primitive,geom_material,scene,set(animation['appearance_geom_ids'])if animation else set(),args.instances,camera_positions,args.profile=='fast',lighting.get('robot_lod',{}))
        objects=grouped['objects'];max_position_error=grouped['position_error'];max_rotation_error=grouped['rotation_error']
    for g in ([] if grouped else metadata['geoms']):
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
    # from interactive renderer intensity units. These lights never enter task source models.
    world=bpy.data.worlds.new('Factory ambient');scene.world=world;world.use_nodes=True
    world.node_tree.nodes['Background'].inputs[0].default_value=(*lighting.get('world_color',[.65,.72,.85]),1.)
    world.node_tree.nodes['Background'].inputs[1].default_value=lighting.get('world_strength',.25)
    # Visible sky is independently exposed; changing its camera appearance must
    # not silently flood the factory with additional indirect light.
    nodes=world.node_tree.nodes;links=world.node_tree.links
    sky=nodes.new('ShaderNodeBackground');sky.inputs[0].default_value=(.76,.86,1.,1.)
    sky.inputs[1].default_value=lighting.get('visible_sky_strength',1.6)
    rays=nodes.new('ShaderNodeLightPath');mix=nodes.new('ShaderNodeMixShader')
    links.new(rays.outputs['Is Camera Ray'],mix.inputs[0]);links.new(nodes.get('Background').outputs[0],mix.inputs[1])
    links.new(sky.outputs[0],mix.inputs[2]);links.new(mix.outputs[0],nodes.get('World Output').inputs['Surface'])
    building=metadata.get('building',dict(width_m=55.,depth_m=50.,eave_height_m=7.2))
    bounced=lighting.get('ceiling_mode')=='bounced'
    for ix,x in enumerate([] if bounced else np.linspace(-building['width_m']/2+5,building['width_m']/2-5,7)):
        for iy,y in enumerate(np.linspace(-building['depth_m']/2+4,building['depth_m']/2-4,6)):
            key=ix in (1,5) and iy in (1,4)
            lamp=bpy.data.lights.new('Ceiling softbox','AREA');lamp.energy=lighting.get('ceiling_key_w'if key else 'ceiling_fill_w',lighting.get('ceiling_energy_w',1800.));lamp.shape='RECTANGLE';lamp.size=lighting.get('fixture_size_m',4.);lamp.size_y=lighting.get('fixture_width_m',2.)
            if args.engine=='eevee' and args.profile=='fast':
                lamp.use_shadow=lighting.get('eevee_all_shadows',False) or (ix+2*iy)%5==0
            # Emit below the lamp's lower face, not inside its opaque mesh.
            obj=bpy.data.objects.new(lamp.name,lamp);scene.collection.objects.link(obj);obj.location=(x,y,building['eave_height_m']-1.04)
    if bounced:
        # Approximate the overlapping, diffuse ceiling bounce as one broad
        # source. Keep the authored fixture meshes; avoid 42 isolated pools and
        # their repeated rough specular highlights on the epoxy floor.
        lamp=bpy.data.lights.new('Diffuse ceiling bounce','AREA');lamp.shape='RECTANGLE'
        lamp.size=building['width_m']-6;lamp.size_y=building['depth_m']-6
        lamp.energy=lighting.get('ceiling_bounce_w',14000.);lamp.specular_factor=0.
        lamp.color=(.94,.97,1.);obj=bpy.data.objects.new(lamp.name,lamp);scene.collection.objects.link(obj)
        obj.location=(0,0,building['eave_height_m']-1.05)
    sun=bpy.data.lights.new('Daylight','SUN');sun.energy=lighting.get('sun_energy',1.5);sun.angle=math.radians(lighting.get('sun_angle_deg',1.4))
    obj=bpy.data.objects.new('Daylight',sun);scene.collection.objects.link(obj)
    sun.color=lighting.get('sun_color',[1.,.94,.85])
    obj.rotation_euler=tuple(math.radians(x)for x in lighting.get('sun_rotation_deg',[25,-30,-15]))
    # Exterior daylight cards sit beyond the actual glazed side openings. They
    # provide a visible sky/treeline instead of black transmission in Eevee.
    # This is presentation scenery, shared by both offline engines.
    exterior_objects=[]
    if lighting.get('exterior_daylight',True):
        sky=bpy.data.materials.new('Exterior sky and distant treeline');sky.use_nodes=True
        n=sky.node_tree.nodes;l=sky.node_tree.links;n.clear()
        output=n.new('ShaderNodeOutputMaterial');emission=n.new('ShaderNodeEmission');emission.inputs['Strength'].default_value=1.8
        coords=n.new('ShaderNodeTexCoord');separate=n.new('ShaderNodeSeparateXYZ');l.new(coords.outputs['Generated'],separate.inputs[0])
        noise=n.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=16.;noise.inputs['Detail'].default_value=2.;l.new(coords.outputs['Generated'],noise.inputs['Vector'])
        height=n.new('ShaderNodeMath');height.operation='MULTIPLY_ADD';height.inputs[1].default_value=.13;height.inputs[2].default_value=.11;l.new(noise.outputs['Fac'],height.inputs[0])
        trees=n.new('ShaderNodeMath');trees.operation='LESS_THAN';l.new(separate.outputs['Z'],trees.inputs[0]);l.new(height.outputs[0],trees.inputs[1])
        mix=n.new('ShaderNodeMixRGB');mix.inputs[1].default_value=(.68,.78,.90,1);mix.inputs[2].default_value=(.10,.16,.07,1)
        l.new(trees.outputs[0],mix.inputs[0]);l.new(mix.outputs[0],emission.inputs[0]);l.new(emission.outputs[0],output.inputs[0])
        if metadata.get('exterior_texture'):
            panorama=n.new('ShaderNodeTexImage');panorama.image=bpy.data.images.load(str(args.asset/metadata['exterior_texture']))
            l.new(panorama.outputs['Color'],emission.inputs[0])
        for side in (-1,1):
          for cy in (-32.,32.):
            mesh=bpy.data.meshes.new('Exterior daylight backdrop');x=side*38.
            mesh.from_pydata([(x,cy-32,-9.8),(x,cy+32,-9.8),(x,cy+32,11.5333),(x,cy-32,11.5333)],[],[(0,1,2,3)]);mesh.materials.append(sky)
            uv=mesh.uv_layers.new(name='UVMap')
            for loop,coord in zip(uv.data,[(0,0),(1,0),(1,1),(0,1)]):loop.uv=coord
            backdrop=bpy.data.objects.new(mesh.name,mesh);scene.collection.objects.link(backdrop)
            backdrop.visible_shadow=False;backdrop.visible_diffuse=False;backdrop.visible_glossy=True
            exterior_objects.append(backdrop)
    for side,energy in [(-1,lighting.get('west_window_w',0)),(1,lighting.get('east_window_w',0))]:
        if not energy:continue
        lamp=bpy.data.lights.new('Window daylight','AREA');lamp.energy=energy;lamp.shape='RECTANGLE';lamp.size=42.;lamp.size_y=3.
        lamp.color=lighting.get('window_color',[1.,.96,.89]);lamp.use_shadow=lighting.get('window_shadows',True)
        obj=bpy.data.objects.new(lamp.name,lamp);scene.collection.objects.link(obj)
        obj.location=(side*26.9,0,3.2);obj.rotation_euler=(Vector((0,0,1.))-obj.location).to_track_quat('-Z','Y').to_euler()
    camera_data=bpy.data.cameras.new('Factory camera');camera=bpy.data.objects.new('Factory camera',camera_data);scene.collection.objects.link(camera);scene.camera=camera
    camera_data.clip_start=.03;camera_data.clip_end=250.
    bake_s=0.
    if args.bake_gi:
        if args.engine!='eevee' or not grouped or not grouped.get('instances'):raise ValueError('Static GI bake needs Eevee instances')
        before=time.monotonic();probe=bpy.data.lightprobes.new('Static factory indirect light','VOLUME')
        probe.resolution_x=16;probe.resolution_y=16;probe.resolution_z=4
        probe.bake_samples=128;probe.surfel_density=4;probe.capture_world=True;probe.capture_distance=70.
        obj=bpy.data.objects.new(probe.name,probe);scene.collection.objects.link(obj);obj.location=(0,0,3.5);obj.scale=(27.,24.,3.4)
        dynamic=[c['obj']for c in grouped['instances'].clouds if not c['static']]
        for obj in dynamic:obj.hide_render=True
        baked=bpy.ops.object.lightprobe_cache_bake(subset='ALL')
        if baked!={'FINISHED'}:raise RuntimeError(('Static light cache bake did not finish',baked))
        for obj in dynamic:obj.hide_render=False
        scene.eevee.fast_gi_method='AMBIENT_OCCLUSION_ONLY';bake_s=time.monotonic()-before
    report=dict(schema='robofrontier.blender-render.v1',source_scene_sha256=metadata['source_scene_sha256'],blender_version=bpy.app.version_string,
        engine=args.engine,profile=args.profile,device=args.device if args.engine=='cycles' else 'GPU',exposure_ev=args.exposure,physics_stepped=False,
        visual_objects=len(metadata['geoms']),retained_objects=grouped['objects_count']if grouped else len(objects),shared_meshes=grouped['unique_meshes']if grouped else len(meshes),
        max_position_error_m=max_position_error,max_rotation_matrix_error=max_rotation_error,
        scope='Offline visual rendering; no physics/Isaac dynamics conversion',views=[])
    report.update(setup_wall_s=time.monotonic()-started,cpu_threads='AUTO',cpu_logical_cores=os.cpu_count(),
        static_gi_bake_s=bake_s,
        compositor_gpu_denoise=args.denoise and args.engine=='eevee',
        compositor_denoise_quality=scene.render.compositor_denoise_final_quality,
        lighting=lighting,raytracing=args.raytracing,material_audit=material_audit,
        acceleration=dict(metalrt=prefs.metalrt,persistent_data=scene.render.use_persistent_data,
            adaptive_sampling=scene.cycles.use_adaptive_sampling,max_samples=scene.cycles.samples,min_samples=scene.cycles.adaptive_min_samples,
            light_tree=scene.cycles.use_light_tree,noise_threshold=scene.cycles.adaptive_threshold,
            png_compression=args.png_compression,enabled_devices=[dict(name=d.name,type=d.type)for d in prefs.devices if d.use]),
        glass_mode='blended-thin-pane'if args.engine=='eevee'else 'physical-transmission-with-thin-shadow',
        cycles_gpu_denoising=bool(args.engine=='cycles'and scene.cycles.use_denoising and scene.cycles.denoising_use_gpu),
        shadow_area_lights=sum(l.use_shadow for l in bpy.data.lights if l.type=='AREA'),
        geometry=dict(robot_visual_lod=args.profile=='fast',
            render_triangles=grouped.get('render_triangles')if grouped else None,
            instance_count=grouped.get('instance_count')if grouped else None,
            lod_templates=grouped.get('lod_templates',[])if grouped else []))
    if recorded:
        report['recorded_pose']=dict(case_id=recorded['case_id'],time_s=recorded['selected_recorded_time_s'],geometries=len(recorded['geoms']),
            source_scene_sha256=recorded['source_scene_sha256'],motion_index_sha256=recorded['motion_index_sha256'])
    hidden_state=None
    def render_camera(source,file):
        nonlocal hidden_state
        position=Vector(source['position_m']);target=Vector(source['target_m']);camera.location=position
        camera.rotation_euler=(target-position).to_track_quat('-Z','Y').to_euler()
        if source.get('projection')=='orthographic':
            camera_data.type='ORTHO';camera_data.sensor_fit='HORIZONTAL'
            camera_data.ortho_scale=source['orthographic_height_m']*args.width/args.height
        else:
            camera_data.type='PERSP';camera_data.sensor_fit='VERTICAL';camera_data.sensor_height=24.;camera_data.lens=12./math.tan(math.radians(source['vertical_fov_deg'])/2)
        hidden=set(source.get('hidden_layers',[]));camera_hidden=set(source.get('camera_hidden_layers',[]))
        layers={'structure':0,'roof':1,'north':2,'south':3,'west':4,'east':5}
        state=(frozenset(hidden),frozenset(camera_hidden))
        if state!=hidden_state:
            visibility=grouped['visibility']if grouped else [(objects[g['id']],g['group'])for g in metadata['geoms']]
            for obj,group in visibility:
                obj.hide_render=group in {layers[x] for x in hidden if x in layers}
                obj.visible_camera=group not in {layers[x] for x in camera_hidden if x in layers}
                # Eevee does not implement Cycles camera-ray visibility. Its
                # cutaway omits the roof; this approximation is reported.
                if args.engine=='eevee' and group in {layers[x] for x in camera_hidden if x in layers}:obj.hide_render=True
            for obj in exterior_objects:obj.hide_render=bool(hidden or camera_hidden)
            hidden_state=state
        offset=lighting.get('top_exposure_offset_ev',0.) if source.get('projection')=='orthographic' else 0.
        if args.engine=='eevee' and source.get('projection')=='orthographic':offset=lighting.get('eevee_top_exposure_offset_ev',offset)
        scene.view_settings.exposure=args.exposure+source.get('exposure_offset_ev',offset)
        scene.render.filepath=str(file);before=time.monotonic();bpy.ops.render.render(write_still=True)
        return dict(camera=source.get('id','tour'),file=scene.render.filepath,wall_s=time.monotonic()-before,
            exposure_ev=scene.view_settings.exposure,hidden_layers=sorted(hidden),camera_hidden_layers=sorted(camera_hidden),
            cutaway_mode=('camera-only' if args.engine=='cycles' else 'roof-omitted') if camera_hidden else 'none',
            started_s=before-started,ended_s=time.monotonic()-started,samples=args.samples,resolution=[args.width,args.height])
    if args.animation:
        if args.recorded_pose:raise ValueError('Select one animation or one recorded pose')
        animation=json.loads(args.animation.read_text())
        if (animation['schema']!='robofrontier.factory-animation.v1' or animation['fps']!=30
                or animation['source_scene_sha256']!=metadata['source_scene_sha256']):
            raise ValueError('Animation source or cadence differs')
        trace=args.animation.parent/'frames.jsonl'
        if hashlib.sha256(trace.read_bytes()).hexdigest()!=animation['source_trace_sha256']:
            raise ValueError('Animation source trace differs')
        selected=args.frame_indices if args.frame_indices is not None else list(range(len(animation['frames'])))
        if not selected or any(i<0 or i>=len(animation['frames']) for i in selected) or selected!=sorted(set(selected)):
            raise ValueError('Frame indices must be nonempty, unique, increasing and within the source clip')
        specs={g['id']:g for g in metadata['geoms']};last_rgba={};last_body=None
        for ordinal,index in enumerate(selected):
            frame=animation['frames'][index]
            frame_started=time.monotonic()
            path=args.animation.parent/frame['file']
            if path.resolve().parent!=args.animation.parent.resolve() or hashlib.sha256(path.read_bytes()).hexdigest()!=frame['sha256']:
                raise ValueError('Animation pose differs')
            with np.load(path,allow_pickle=False) as pose:
                ids=pose['geom_ids'];positions=pose['position'];rotations=pose['rotation'];colors=pose['rgba']
                if (positions.shape!=(len(ids),3) or rotations.shape!=(len(ids),3,3) or colors.shape!=(len(ids),4)
                        or len(set(ids.tolist()))!=len(ids) or not all(np.isfinite(a).all() for a in (positions,rotations,colors))):
                    raise ValueError('Invalid animation pose arrays')
                if grouped:
                    bids=pose['body_ids'];bp=pose['body_position'];br=pose['body_rotation']
                    if (bp.shape!=(len(bids),3)or br.shape!=(len(bids),3,3)or not np.isfinite(bp).all()or not np.isfinite(br).all()):raise ValueError('Invalid body pose')
                    if grouped.get('instances'):grouped['instances'].update(bids,bp,br)
                    else:
                        changed=np.ones(len(bids),bool)if last_body is None else np.any(bp!=last_body[0],axis=1)|np.any(br!=last_body[1],axis=(1,2))
                        for bid,position,rotation in zip(bids[changed],bp[changed],br[changed]):
                            for obj in grouped['by_body'].get(int(bid),[]):
                                obj.location=position;obj.rotation_quaternion=Matrix(rotation).to_quaternion()
                        last_body=(bp.copy(),br.copy())
                mutable=np.isin(ids,animation.get('appearance_geom_ids',ids))if grouped else np.ones(len(ids),bool)
                if grouped and ordinal==0:
                    frozen=np.asarray([specs[int(gid)]['rgba']for gid in ids[~mutable]],dtype=np.float32)
                if grouped and not np.array_equal(colors[~mutable],frozen):raise ValueError('Appearance events missing from mutable geometry set')
                for gid,position,rotation,rgba in zip(ids[mutable],positions[mutable],rotations[mutable],colors[mutable]):
                    gid=int(gid);obj=objects[gid]
                    if not grouped:obj.location=position;obj.rotation_quaternion=Matrix(rotation).to_quaternion()
                    color=tuple(rgba)
                    if color!=last_rgba.get(gid,tuple(specs[gid]['rgba'])):
                        if grouped and grouped.get('instances'):grouped['instances'].appearance(gid,appearance_material(specs[gid],rgba))
                        else:
                            slot=grouped['slots'][gid][1]if grouped else 0
                            obj.material_slots[slot].material=appearance_material(specs[gid],rgba)
                    last_rgba[gid]=color
                if grouped and grouped.get('instances'):grouped['instances'].flush()
                if grouped and grouped.get('instances') and ordinal in (0,len(selected)//2,len(selected)-1):
                    override={int(gid):(p,r)for gid,p,r in zip(ids,positions,rotations)}
                    audit_metadata=dict(metadata,geoms=[dict(g,position=override[g['id']][0],rotation=override[g['id']][1])if g['id']in override else g for g in metadata['geoms']])
                    audit=grouped['instances'].audit(audit_metadata)
                    if audit['position_error']>1e-5 or audit['rotation_error']>1e-6:raise ValueError(('Animated instance FK differs',index,audit))
                    report.setdefault('animation_transform_audits',[]).append(dict(frame=index,**audit))
            update_s=time.monotonic()-frame_started
            row=render_camera(frame['camera'],args.output/f'frame-{ordinal:06d}.png')
            row.update(source_frame=index,update_s=update_s,frame_wall_s=time.monotonic()-frame_started)
            report['views'].append(row)
            (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        count=len(selected)
        if not count:raise ValueError('Empty animation')
        encoding_started=time.monotonic()
        if args.encode:
          subprocess.run(['ffmpeg','-v','error','-threads','0','-framerate','30','-i',str(args.output/'frame-%06d.png'),
            '-frames:v',str(count),'-c:v','libx264','-threads','0','-pix_fmt','yuv420p','-crf','18','-movflags','+faststart',str(args.output/'factory.mp4')],check=True)
          report['encoding_wall_s']=time.monotonic()-encoding_started
          subprocess.run(['ffmpeg','-v','error','-threads','0','-i',str(args.output/'factory.mp4'),'-f','null','-'],check=True)
          probe=subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_entries',
            'stream=nb_read_frames,avg_frame_rate,width,height','-of','json',str(args.output/'factory.mp4')],capture_output=True,text=True,check=True)
          video=json.loads(probe.stdout)['streams'][0]
          if int(video['nb_read_frames'])!=count or video['avg_frame_rate']!='30/1':raise ValueError('Encoded video count/cadence differs')
        else:video=None;report['encoding_wall_s']=0.
        report['animation']=dict(frames=count,fps=30,stations=animation['stations'],loop=animation['loop'],
            source_frame_indices=selected,source_trace=str(trace.resolve()),full_decode_passed=True if args.encode else None,video=video,
            camera_motion=animation.get('camera_motion'))
    else:
      for name in args.views:
        source=next(c for c in metadata['cameras'] if c['id']==name)
        for repeat in range(args.benchmark_frames):
            row=render_camera(source,args.output/(name+(''if repeat==0 else '-'+str(repeat))+'.png'))
            row['repeat']=repeat;report['views'].append(row)
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.save_blend:
        bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(args.output/'factory.blend'))
    if args.usd:
        bpy.ops.wm.usd_export(filepath=str(args.output/'factory.usdc'),export_materials=True,export_textures_mode='NEW',export_custom_properties=True,xform_op_mode='MAT')
        report['usd_scope']='Visual export only; target Isaac compatibility not validated'
    rows=report['views'];warm=[r.get('frame_wall_s',r['wall_s'])for r in rows[1:]if args.animation or r.get('repeat',0)>0]
    report['timing']=dict(first_render_s=rows[0]['wall_s'],
        first_frame_including_setup_s=report['setup_wall_s']+rows[0].get('frame_wall_s',rows[0]['wall_s']),
        warm_frames=len(warm),warm_mean_s=float(np.mean(warm))if warm else None,
        warm_median_s=float(np.median(warm))if warm else None,warm_p95_s=float(np.percentile(warm,95))if warm else None,
        warm_max_s=max(warm)if warm else None,
        includes_pose_update_and_png=bool(args.animation),camera_moving=bool(args.animation and len({json.dumps(f['camera'],sort_keys=True)for f in animation['frames']})>1))
    if not args.animation:
        report['timing']['by_camera']={name:dict(first_s=next(r['wall_s']for r in rows if r['camera']==name),
            warm_mean_s=float(np.mean([r['wall_s']for r in rows if r['camera']==name and r['repeat']>0]))if args.benchmark_frames>1 else None)
            for name in dict.fromkeys(r['camera']for r in rows)}
    gpu_stop.set();report['gpu']=dict(hardware=gpu_hardware,samples=gpu_samples,
        scope='System-wide AGX utilization, including desktop and other applications; cores scheduled by Metal driver')
    report['provenance']=dict(renderer_sha256=SCRIPT_SHA256,
        frame_storage_helper_sha256=hashlib.sha256((Path(__file__).parent/'factory_video_frames.py').read_bytes()).hexdigest(),
        instance_helper_sha256=hashlib.sha256((Path(__file__).parent/'factory_blender_objects.py').read_bytes()).hexdigest(),
        material_helper_sha256=hashlib.sha256((Path(__file__).parent/'factory_blender_materials.py').read_bytes()).hexdigest(),
        asset_metadata_sha256=hashlib.sha256((args.asset/'scene.json').read_bytes()).hexdigest(),
        animation_sha256=hashlib.sha256(args.animation.read_bytes()).hexdigest()if args.animation else None)
    if args.animation and args.encode and not args.keep_frames:
        from factory_video_frames import prune_frames
        prune_frames(args.output,report)
    else:report['frame_storage']=dict(status='all_retained')
    report['total_wall_s']=time.monotonic()-started;(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
