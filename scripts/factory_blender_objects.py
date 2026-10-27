"""Retain exact rigid body frames with shared meshes and isolated material slots.

Imported only inside Blender. Mutable appearance geoms keep separate slots;
Rigid geometry is combined per body/layer and instanced across identical robots.
An optional camera-aware LOD changes robot skins only, never source task state.
"""
from collections import defaultdict
import hashlib
import json
import re

import bpy
import bmesh
import numpy as np
from mathutils import Euler,Matrix


def eulers(rotation):
    r=np.asarray(rotation);sy=np.sqrt(r[:,0,0]**2+r[:,1,0]**2);regular=sy>1e-6
    return np.column_stack([np.where(regular,np.arctan2(r[:,2,1],r[:,2,2]),np.arctan2(-r[:,1,2],r[:,1,1])),
        np.arctan2(-r[:,2,0],sy),np.where(regular,np.arctan2(r[:,1,0],r[:,0,0]),0.)]).astype(np.float32)


class Instances:
    """GPU instances driven by bulk point attributes, including per-geom colors."""
    def __init__(self,records,combined,body_specs,scene):
        self.records=records;self.combined=combined;self.templates=bpy.data.collections.new('Rigid templates')
        self.scene=scene;self.reduced={};self.lod_statistics=[]
        self.choices={};self.clouds=[];self.visibility=[];self.geometry={}
        for layer,static in sorted({(r['layer'],r['static'])for r in records}):
            rows=[r for r in records if r['layer']==layer and r['static']==static]
            mesh=bpy.data.meshes.new('instance_points_'+str(layer))
            mesh.from_pydata([[0,0,0]]*len(rows),[],[])
            rotation=mesh.attributes.new('instance_rotation','FLOAT_VECTOR','POINT')
            index=mesh.attributes.new('instance_index','INT','POINT')
            indices=np.array([self.choice(r['signature'],detail=r['detail'])for r in rows],dtype=np.int32)
            index.data.foreach_set('value',indices)
            obj=bpy.data.objects.new('instances_layer_'+str(layer),mesh);scene.collection.objects.link(obj)
            graph=bpy.data.node_groups.new('Rigid instances '+str(layer),'GeometryNodeTree')
            graph.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry')
            graph.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
            nodes,links=graph.nodes,graph.links
            start=nodes.new('NodeGroupInput');end=nodes.new('NodeGroupOutput')
            collection=nodes.new('GeometryNodeCollectionInfo');collection.inputs['Collection'].default_value=self.templates
            collection.inputs['Separate Children'].default_value=True;collection.inputs['Reset Children'].default_value=True
            node=nodes.new('GeometryNodeInstanceOnPoints');node.inputs['Pick Instance'].default_value=True
            angle=nodes.new('GeometryNodeInputNamedAttribute');angle.data_type='FLOAT_VECTOR';angle.inputs['Name'].default_value='instance_rotation'
            kind=nodes.new('GeometryNodeInputNamedAttribute');kind.data_type='INT';kind.inputs['Name'].default_value='instance_index'
            links.new(start.outputs['Geometry'],node.inputs['Points']);links.new(collection.outputs['Instances'],node.inputs['Instance'])
            links.new(kind.outputs['Attribute'],node.inputs['Instance Index']);links.new(angle.outputs['Attribute'],node.inputs['Rotation'])
            links.new(node.outputs['Instances'],end.inputs['Geometry'])
            modifier=obj.modifiers.new('Rigid GPU instances','NODES');modifier.node_group=graph
            cloud=dict(mesh=mesh,rotation=rotation,index=index,indices=indices,rows=rows,
                static=static,obj=obj,appearance_dirty=False,bodies=np.array([r['body']for r in rows]),local_position=np.array([r['local_position']for r in rows]),
                local_rotation=np.array([r['local_rotation']for r in rows]))
            self.clouds.append(cloud);self.visibility.append((obj,layer))
            for i,row in enumerate(rows):
                for gid in row['gids']:self.geometry[gid]=(cloud,i,row['signature'],row['detail'])
        ids=np.array(sorted(body_specs));positions=np.array([body_specs[i]['position']for i in ids]);rotations=np.array([body_specs[i]['rotation']for i in ids])
        self.update(ids,positions,rotations)

    def choice(self,signature,material=None,detail=1.):
        key=(signature,material.name if material else None,detail)
        if key not in self.choices:
            mesh=self.combined[signature][0]
            if detail<1 and len(mesh.polygons)>2000:
                lod=(signature,detail)
                if lod not in self.reduced:
                    # Source visual meshes duplicate vertices at authored normals/UV
                    # seams. Weld only the temporary robot LOD before decimation;
                    # collapsing disconnected triangles produces holes and spikes.
                    welded=mesh.copy();bm=bmesh.new();bm.from_mesh(welded)
                    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-6)
                    bm.to_mesh(welded);bm.free();welded.update()
                    temporary=bpy.data.objects.new('LOD preparation',welded);self.scene.collection.objects.link(temporary)
                    modifier=temporary.modifiers.new('Visual LOD','DECIMATE');modifier.ratio=detail;modifier.use_collapse_triangulate=True
                    graph=bpy.context.evaluated_depsgraph_get()
                    reduced=bpy.data.meshes.new_from_object(temporary.evaluated_get(graph),depsgraph=graph)
                    self.lod_statistics.append(dict(original_faces=len(mesh.polygons),render_faces=len(reduced.polygons),ratio=detail))
                    self.reduced[lod]=reduced;bpy.data.objects.remove(temporary,do_unlink=True)
                    bpy.data.meshes.remove(welded)
                mesh=self.reduced[lod]
            if material is not None:
                # Collection Info carries mesh materials, not OBJECT slot
                # overrides, into evaluated instances. Give each cached color
                # variant its own material binding without changing the base.
                mesh=mesh.copy()
                for i in range(len(mesh.materials)):mesh.materials[i]=material
                mesh.update()
            obj=bpy.data.objects.new('instance_'+str(len(self.choices)).zfill(6),mesh)
            self.templates.objects.link(obj)
            for i,slot in enumerate(obj.material_slots):
                slot.link='DATA';slot.material=mesh.materials[i]
            self.choices[key]=len(self.choices)
        return self.choices[key]

    def update(self,ids,positions,rotations):
        lookup={int(b):i for i,b in enumerate(ids)}
        for cloud in self.clouds:
            mask=np.array([int(b)in lookup for b in cloud['bodies']]);indices=np.flatnonzero(mask)
            if not len(indices):continue
            # Non-task background points retain their initialized transforms.
            p=np.empty(len(cloud['mesh'].vertices)*3,dtype=np.float32)
            cloud['mesh'].vertices.foreach_get('co',p);p=p.reshape(-1,3)
            r=np.empty(len(cloud['mesh'].vertices)*3,dtype=np.float32)
            cloud['rotation'].data.foreach_get('vector',r);r=r.reshape(-1,3)
            source=np.array([lookup[int(b)]for b in cloud['bodies'][mask]])
            orientation=rotations[source]
            p[mask]=positions[source]+np.einsum('nij,nj->ni',orientation,cloud['local_position'][mask])
            r[mask]=eulers(orientation@cloud['local_rotation'][mask])
            cloud['mesh'].vertices.foreach_set('co',p.ravel());cloud['rotation'].data.foreach_set('vector',r.ravel())
            cloud['index'].data.foreach_set('value',cloud['indices']);cloud['mesh'].update()

    def appearance(self,gid,material):
        cloud,index,signature,detail=self.geometry[gid]
        choice=self.choice(signature,material,detail)
        if cloud['indices'][index]!=choice:
            cloud['indices'][index]=choice;cloud['appearance_dirty']=True

    def flush(self):
        for cloud in self.clouds:
            if cloud['appearance_dirty']:
                cloud['index'].data.foreach_set('value',cloud['indices']);cloud['mesh'].update()
                cloud['appearance_dirty']=False

    def audit(self,metadata):
        """Read back the actual point attributes and reconstruct source FK frames."""
        specs={g['id']:g for g in metadata['geoms']};position_error=rotation_error=0.
        names=[o.name for o in sorted(self.templates.objects,key=lambda o:o.name)]
        if names!=['instance_'+str(i).zfill(6)for i in range(len(names))]:
            raise ValueError('Instance template order differs')
        triangles=0
        for cloud in self.clouds:
            p=np.empty(len(cloud['rows'])*3,dtype=np.float32);r=np.empty_like(p)
            cloud['mesh'].vertices.foreach_get('co',p);cloud['rotation'].data.foreach_get('vector',r)
            actual_indices=np.empty(len(cloud['rows']),dtype=np.int32);cloud['index'].data.foreach_get('value',actual_indices)
            if not np.array_equal(actual_indices,cloud['indices']):raise ValueError('Appearance instance attributes differ')
            for point,angle,row in zip(p.reshape(-1,3),r.reshape(-1,3),cloud['rows']):
                rotation=np.asarray(Euler(tuple(float(x)for x in angle),'XYZ').to_matrix())
                template=self.templates.objects[names[self.choice(row['signature'],detail=row['detail'])]]
                triangles+=len(template.data.polygons)
                for gid in row['gids']:
                    spec=specs[gid]
                    local_position=spec['local_position']if len(row['gids'])>1 else [0,0,0]
                    local_rotation=spec['local_rotation']if len(row['gids'])>1 else np.eye(3)
                    position_error=max(position_error,float(np.max(abs(point+rotation@local_position-spec['position']))))
                    rotation_error=max(rotation_error,float(np.max(abs(rotation@local_rotation-spec['rotation']))))
        return dict(position_error=position_error,rotation_error=rotation_error,render_triangles=triangles)


def build(metadata,meshes,primitive,geom_material,scene,mutable_ids,use_instances=True,camera_positions=None,robot_lod=False,lod_config=None):
    groups=defaultdict(list)
    for geom in metadata['geoms']:
        groups[(geom['body_id'],geom['group'],geom['id'] if geom['id']in mutable_ids else -1)].append(geom)
    body_specs={b['id']:b for b in metadata['bodies']}
    pieces={};combined={};objects={};slots={};by_body=defaultdict(list);visibility=[];records=[]
    def piece(g):
        key=('mesh',g['mesh'])if g['kind']==7 else ('primitive',g['kind'])
        if key not in pieces:
            mesh=meshes[key]if g['kind']==7 else primitive(g['kind'])
            mesh.calc_loop_triangles()
            vertices=np.array([v.co[:] for v in mesh.vertices])
            faces=np.array([f.vertices[:]for f in mesh.loop_triangles],dtype=np.int32)
            smooth=np.array([mesh.polygons[f.polygon_index].use_smooth for f in mesh.loop_triangles])
            uv=np.array([[mesh.uv_layers.active.data[i].uv[:]for i in f.loops]for f in mesh.loop_triangles])if mesh.uv_layers.active else np.zeros((len(faces),3,2))
            pieces[key]=(vertices,faces,smooth,uv)
        return pieces[key]
    position_error=rotation_error=0.
    for (bid,layer,mutable),geoms in groups.items():
        original_geoms=geoms
        local_position=geoms[0]['local_position']if len(geoms)==1 else [0,0,0]
        local_rotation=geoms[0]['local_rotation']if len(geoms)==1 else np.eye(3).tolist()
        if use_instances and len(geoms)==1:geoms=[dict(geoms[0],local_position=[0,0,0],local_rotation=np.eye(3).tolist())]
        signature=hashlib.sha256(json.dumps([{k:g[k]for k in ('kind','mesh','size','local_position','local_rotation','material','rgba')}for g in geoms],sort_keys=True).encode()).hexdigest()
        if signature not in combined:
            vertices=[];faces=[];smoothing=[];uvs=[];material_indices=[];materials=[];indices=[];offset=0
            for g in geoms:
                v,f,smooth,uv=piece(g);v=v.copy()
                if g['kind']==6:v*=g['size']
                elif g['kind']==5:v*=np.array([g['size'][0],g['size'][0],g['size'][1]])
                elif g['kind']==3:v*=g['size'][0]
                v=v@np.array(g['local_rotation']).T+g['local_position']
                mat=geom_material(g)
                if mat.name=='factory_bench_wood':uv=v[f][...,:2]*2.
                vertices.append(v);faces.append(f+offset);offset+=len(v);smoothing.extend(smooth);uvs.append(uv)
                if mat not in materials:materials.append(mat)
                index=materials.index(mat);indices.append(index);material_indices.extend([index]*len(f))
            mesh=bpy.data.meshes.new('rigid_'+signature[:16])
            mesh.from_pydata(np.concatenate(vertices).tolist(),[],np.concatenate(faces).tolist());mesh.update()
            for mat in materials:mesh.materials.append(mat)
            mesh.polygons.foreach_set('use_smooth',np.asarray(smoothing,dtype=np.bool_))
            mesh.polygons.foreach_set('material_index',np.asarray(material_indices,dtype=np.int32))
            uv=mesh.uv_layers.new(name='UVMap');uv.data.foreach_set('uv',np.concatenate(uvs).ravel())
            combined[signature]=(mesh,indices)
        mesh,indices=combined[signature];body=body_specs[bid]
        if use_instances:
            detail=1.
            if robot_lod and re.search(r'__[A-Z]_(?:link\d|hand|leftfinger|rightfinger|left_finger|right_finger)',body['name']):
                distance=max(0.,float(np.min(np.linalg.norm(np.asarray(camera_positions)-body['position'],axis=1)))-1.)
                settings=lod_config or {}
                detail=(settings.get('hero',.35) if distance<4 else settings.get('near',.12) if distance<10
                        else settings.get('middle',.03) if distance<22 else settings.get('far',.01))
                if not 0<detail<=1:raise ValueError('Robot visual LOD must be in (0,1]')
            records.append(dict(signature=signature,body=bid,layer=layer,static='__'not in body['name'],gids=[g['id']for g in original_geoms],local_position=local_position,local_rotation=local_rotation,detail=detail))
            continue
        obj=bpy.data.objects.new('rigid_'+str(bid)+'_'+str(layer)+'_'+str(mutable),mesh);scene.collection.objects.link(obj)
        obj.location=body['position'];obj.rotation_mode='QUATERNION';obj.rotation_quaternion=Matrix(body['rotation']).to_quaternion()
        by_body[bid].append(obj);visibility.append((obj,layer))
        for index,slot in enumerate(obj.material_slots):
            slot.link='OBJECT';slot.material=mesh.materials[index]
        for g,index in zip(geoms,indices):
            objects[g['id']]=obj;slots[g['id']]=(obj,index)
            rotation=np.array(obj.rotation_quaternion.to_matrix());position=np.array(obj.location)+rotation@np.array(g['local_position'])
            position_error=max(position_error,float(np.max(abs(position-g['position']))))
            rotation_error=max(rotation_error,float(np.max(abs(rotation@np.array(g['local_rotation'])-g['rotation']))))
    if position_error>1e-5 or rotation_error>1e-6:raise ValueError('Grouped visual transforms differ')
    if use_instances:
        instances=Instances(records,combined,body_specs,scene)
        audit=instances.audit(metadata)
        if audit['position_error']>1e-5 or audit['rotation_error']>1e-6:
            raise ValueError(('Instance visual transforms differ',audit))
        return dict(instances=instances,objects={g['id']:None for g in metadata['geoms']},visibility=instances.visibility,
            position_error=audit['position_error'],rotation_error=audit['rotation_error'],unique_meshes=len(combined),objects_count=len(instances.visibility),
            instance_count=len(records),render_triangles=audit['render_triangles'],lod_templates=instances.lod_statistics)
    return dict(objects=objects,slots=slots,by_body=by_body,visibility=visibility,
        position_error=position_error,rotation_error=rotation_error,unique_meshes=len(combined),objects_count=len(visibility))
