"""Audit the generated background's portability, reserved areas and static scope."""
import argparse,collections,hashlib,json,shutil,tempfile
from pathlib import Path

import mujoco,numpy as np
from PIL import Image

from robofrontier.factory_scene import ROOT


def check(scene, previews=None):
    scene=Path(scene).resolve();manifest=json.loads((scene.parent/'manifest.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    findings=[]
    asset_checks=0
    for asset in manifest['assets']:
        path=scene.parent/'assets'/asset['path']
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=asset['sha256']:findings.append('Missing/changed asset: '+asset['path'])
        asset_checks+=1
    if model.nv or model.nu or model.njnt:findings.append('Background has dynamic degrees of freedom or actuators')
    if np.any(model.geom_contype) or np.any(model.geom_conaffinity):findings.append('Display geometry has active collision masks')
    for name in ['roof','north_wall','south_wall','east_wall','west_wall']:
        if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'factory_'+name)<0:findings.append('Missing enclosure: '+name)
    regions=manifest['regions'];W,D,_=manifest['dimensions_m']
    if len(regions)!=4 or {r['id'] for r in regions}!={'transport','mosaic','relay','dial'}:findings.append('Expected four experiment regions')
    def intersects(a,b):return min(a[2],b[2])-max(a[0],b[0])>1e-5 and min(a[3],b[3])-max(a[1],b[1])>1e-5
    for i,r in enumerate(regions):
        x0,y0,x1,y1=r['bounds_xy_m']
        if not (-W/2<x0<x1<W/2 and -D/2<y0<y1<D/2):findings.append('Region outside building: '+r['id'])
        for other in regions[i+1:]:
            if intersects(r['bounds_xy_m'],other['bounds_xy_m']):findings.append('Overlapping regions')
    equipment=[]
    for obj in manifest['objects']:
        if obj['category']!='perimeter_equipment':continue
        bid=model.body(obj['id']).id;geoms=np.flatnonzero(model.geom_bodyid==bid)
        if not len(geoms):findings.append('Empty equipment body: '+obj['id']);continue
        low=[];high=[]
        for g in geoms:
            rot=data.geom_xmat[g].reshape(3,3)
            if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                # MuJoCo aligns a mesh to its inertia frame. Rotating that frame's
                # AABB again overestimates asymmetric batches; inspect real vertices.
                mesh=model.geom_dataid[g];start=model.mesh_vertadr[mesh];count=model.mesh_vertnum[mesh]
                vertices=model.mesh_vert[start:start+count]@rot.T+data.geom_xpos[g]
                low.append(vertices.min(axis=0));high.append(vertices.max(axis=0))
            else:
                center=data.geom_xpos[g]+rot@model.geom_aabb[g,:3];extent=np.abs(rot)@model.geom_aabb[g,3:]
                low.append(center-extent);high.append(center+extent)
        lo=np.min(low,axis=0);hi=np.max(high,axis=0);bounds=[float(lo[0]),float(lo[1]),float(hi[0]),float(hi[1])]
        blocked=[r['id'] for r in regions if intersects(bounds,r['bounds_xy_m'])]
        if blocked:findings.append(obj['id']+' intrudes into '+','.join(blocked))
        if lo[2]<-.005:findings.append(obj['id']+' penetrates floor')
        equipment.append(dict(id=obj['id'],bounds_xy_m=bounds,min_z_m=float(lo[2]),region_intersections=blocked))
    task_bodies=[model.body(i).name for i in range(1,model.nbody) if any(s in model.body(i).name for s in ['_table','_link0','_workstation'])]
    if task_bodies:findings.append('Unexpected task workstations')
    portable=False
    with tempfile.TemporaryDirectory(prefix='robo-factory-portable-') as temp:
        target=Path(temp)/'factory';shutil.copytree(scene.parent,target)
        other=mujoco.MjModel.from_xml_path(str(target/scene.name))
        portable=other.ngeom==model.ngeom and other.ntex==model.ntex
        if not portable:findings.append('Portable scene copy does not match')
    images=[]
    if previews:
        for file in sorted(Path(previews).glob('*.png')):
            a=np.asarray(Image.open(file).convert('RGB'));std=float(a.std())
            images.append(dict(path=str(file),pixel_std=std,width=a.shape[1],height=a.shape[0]))
            if std<8:findings.append('Nearly uniform/unusable rendered view: '+file.name)
    mesh_instances=sum(int(model.mesh_facenum[model.geom_dataid[i]]) for i in range(model.ngeom) if model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH)
    kinds=collections.Counter(mujoco.mjtGeom(int(t)).name for t in model.geom_type)
    decoded=model.ntexdata/2**20;config=json.loads((scene.parent/'build-config.json').read_text())
    if mesh_instances>config['appearance']['material_budget_mesh_triangles']:findings.append('Instanced mesh triangle budget exceeded')
    if decoded>config['appearance']['material_budget_texture_mib']:findings.append('Decoded texture budget exceeded')
    return dict(passed=not findings,findings=findings,scope='Static factory engineering audit, not robot manipulation or cross-engine validation',scene=str(scene),asset_hashes_checked=asset_checks,task_workstations=0,dynamic_dofs=model.nv,actuators=model.nu,collision_masks_disabled=True,enclosure_bodies_present=True,region_count=len(regions),perimeter_equipment=equipment,portable_copy_compiled=portable,decoded_texture_mib=decoded,model_buffer_mib=model.nbuffer/2**20,mesh_definition_triangles=model.nmeshface,mesh_instance_triangles=mesh_instances,primitive_geometry_counts=dict(kinds),triangle_scope='Mesh counts are exact. Primitive tessellation is renderer-dependent and excluded; no total-GPU-triangle guarantee.',images=images,linux_execution_verified=False,isaac_conversion_verified=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,default=ROOT/'outputs/robo-factory/scene/scene.xml');p.add_argument('--previews',type=Path,default=ROOT/'outputs/robo-factory/previews');p.add_argument('--report',type=Path,default=ROOT/'outputs/robo-factory/reports/scene-check.json')
    a=p.parse_args();report=check(a.scene,a.previews);a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['perimeter_equipment','images']}))
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
