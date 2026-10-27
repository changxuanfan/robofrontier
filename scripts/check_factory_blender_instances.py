"""Blender integration checks for rigid instancing and isolated appearance events.

Run: Blender --background --python-exit-code 1 --python SCRIPT
"""
from pathlib import Path
import sys
import unittest

import bpy
import numpy as np
from mathutils import Matrix

sys.path.insert(0,str(Path(__file__).resolve().parent))
from factory_blender_objects import build


class RigidInstanceChecks(unittest.TestCase):
    def test_evaluated_geometry_and_color_isolation_after_motion(self):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        scene=bpy.context.scene
        material=bpy.data.materials.new('base')
        cube=bpy.data.meshes.new('cube')
        vertices=np.array([[x,y,z]for x in (-1,1)for y in (-1,1)for z in (-1,1)],dtype=float)
        cube.from_pydata(vertices.tolist(),[],[(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)])
        cube.update();cube.materials.append(material)
        rotation=np.asarray(Matrix.Rotation(.43,3,'Z'))
        bodies=[dict(id=1,name='cell__A_link1',position=[1,2,0],rotation=np.eye(3).tolist()),
                dict(id=2,name='cell__B_link1',position=[-2,1,0],rotation=rotation.tolist())]
        geoms=[]
        for gid,(bid,local)in enumerate([(1,[.2,0,0]),(1,[0,.3,0]),(1,[0,0,.4]),(2,[.2,0,0]),(2,[0,.3,0]),(2,[0,0,.4])]):
            body=bodies[bid-1];r=np.asarray(body['rotation'])
            geoms.append(dict(id=gid,name='geom_'+str(gid),body_id=bid,group=0,kind=6,mesh=None,
                size=[.1,.12,.08],material=-1,rgba=[1,1,1,1],local_position=local,local_rotation=np.eye(3).tolist(),
                position=(np.asarray(body['position'])+r@local).tolist(),rotation=r.tolist()))
        metadata=dict(bodies=bodies,geoms=geoms)
        grouped=build(metadata,{},lambda kind:cube,lambda g:material,scene,{2,5},True,[[0,0,5]],False)
        instances=grouped['instances']
        # Two immutable geoms combine per body; two mutable geoms stay separate.
        self.assertEqual(grouped['instance_count'],4)
        self.assertEqual(grouped['unique_meshes'],2)
        red=bpy.data.materials.new('red');red.diffuse_color=(1,0,0,1)
        instances.appearance(2,red);instances.flush()
        cloud,index,signature,detail=instances.geometry[2]
        other,other_index,_,_=instances.geometry[5]
        self.assertNotEqual(cloud['indices'][index],other['indices'][other_index])
        chosen=instances.templates.objects['instance_'+str(cloud['indices'][index]).zfill(6)]
        self.assertEqual(chosen.material_slots[0].material,red)
        original=instances.templates.objects['instance_'+str(other['indices'][other_index]).zfill(6)]
        self.assertEqual(original.material_slots[0].material,material)
        # A real late-created variant must preserve Collection Info index order.
        positions=np.array([[3,1,.5],[-1,-2,.4]])
        rotations=np.array([np.asarray(Matrix.Rotation(.8,3,'X')),np.asarray(Matrix.Rotation(-.6,3,'Z'))])
        instances.update(np.array([1,2]),positions,rotations)
        moved=[]
        for geom in geoms:
            i=geom['body_id']-1
            moved.append(dict(geom,position=positions[i]+rotations[i]@geom['local_position'],rotation=rotations[i]))
        audit=instances.audit(dict(metadata,geoms=moved))
        self.assertLess(audit['position_error'],1e-5)
        self.assertLess(audit['rotation_error'],1e-6)
        # Realize the actual node output, then compare every evaluated vertex.
        for obj,_ in instances.visibility:
            graph=obj.modifiers[0].node_group
            output=next(n for n in graph.nodes if n.bl_idname=='NodeGroupOutput')
            source=output.inputs['Geometry'].links[0].from_socket
            realize=graph.nodes.new('GeometryNodeRealizeInstances')
            graph.links.new(source,realize.inputs['Geometry']);graph.links.new(realize.outputs['Geometry'],output.inputs['Geometry'])
        bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get()
        actual=[];red_centers=[]
        for obj,_ in instances.visibility:
            evaluated=obj.evaluated_get(deps);mesh=evaluated.to_mesh()
            actual.extend([v.co[:]for v in mesh.vertices])
            # Evaluated meshes carry the authoritative face material attribute;
            # polygon.material_index may expose stale RNA values in Blender 5.1.
            indices=np.zeros(len(mesh.polygons),dtype=np.int32)
            if 'material_index'in mesh.attributes:mesh.attributes['material_index'].data.foreach_get('value',indices)
            for polygon,material_index in zip(mesh.polygons,indices):
                if mesh.materials[material_index].original==red:
                    red_centers.append(np.mean([mesh.vertices[i].co[:]for i in polygon.vertices],axis=0))
            evaluated.to_mesh_clear()
        expected=np.concatenate([vertices*np.array(g['size'])@np.asarray(g['rotation']).T+g['position']for g in moved])
        def ordered(values):
            a=np.round(np.asarray(values),5);return a[np.lexsort((a[:,2],a[:,1],a[:,0]))]
        np.testing.assert_allclose(ordered(actual),ordered(expected),rtol=0,atol=1e-5)
        self.assertEqual(len(red_centers),12)
        np.testing.assert_allclose(np.mean(red_centers,axis=0),moved[2]['position'],rtol=0,atol=1e-5)
        # Rewind/reset must restore the first frame's color immediately, without
        # waiting for another body update or leaking into the other instance.
        instances.appearance(2,material);instances.flush();bpy.context.view_layer.update()
        for obj,_ in instances.visibility:
            evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
            indices=np.zeros(len(mesh.polygons),dtype=np.int32)
            if 'material_index'in mesh.attributes:mesh.attributes['material_index'].data.foreach_get('value',indices)
            self.assertTrue(all(mesh.materials[i].original==material for i in indices))
            evaluated.to_mesh_clear()


if __name__=='__main__':unittest.main(argv=[sys.argv[0]])
