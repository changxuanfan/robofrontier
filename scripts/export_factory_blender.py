"""Export compiled visual geometry for offline Blender; never step task physics.

This carries stable geom names, exact FK transforms, shared meshes and authored
PBR textures. The output is a presentation asset, not an Isaac physics model.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image


def export(scene,output):
    scene=Path(scene).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((scene.parent/'manifest.json').read_text())
    digest=hashlib.sha256(scene.read_bytes()).hexdigest()
    if digest!=manifest['scene_sha256']:raise ValueError('Frozen factory scene hash mismatch')
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model)
    mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data)
    arrays={};meshes=[]
    for mid in range(model.nmesh):
        va=int(model.mesh_vertadr[mid]);vn=int(model.mesh_vertnum[mid]);fa=int(model.mesh_faceadr[mid]);fn=int(model.mesh_facenum[mid])
        arrays[f'v{mid}']=model.mesh_vert[va:va+vn];arrays[f'f{mid}']=model.mesh_face[fa:fa+fn]
        ta=int(model.mesh_texcoordadr[mid]);tn=int(model.mesh_texcoordnum[mid])
        if ta>=0:
            arrays[f'uv{mid}']=model.mesh_texcoord[ta:ta+tn];arrays[f'fuv{mid}']=model.mesh_facetexcoord[fa:fa+fn]
        meshes.append(dict(id=mid,name=model.mesh(mid).name,textured=ta>=0))
    np.savez_compressed(output/'meshes.npz',**arrays)
    textures=[];(output/'textures').mkdir()
    for tid in range(model.ntex):
        if int(model.tex_type[tid])!=int(mujoco.mjtTexture.mjTEXTURE_2D):continue
        width=int(model.tex_width[tid]);height=int(model.tex_height[tid]);channels=int(model.tex_nchannel[tid]);adr=int(model.tex_adr[tid])
        image=model.tex_data[adr:adr+width*height*channels].reshape(height,width,channels)
        file=f'textures/{tid:03d}.png';Image.fromarray(image.squeeze(-1) if channels==1 else image).save(output/file)
        textures.append(dict(id=tid,name=model.texture(tid).name,file=file))
    materials=[]
    for mid in range(model.nmat):
        materials.append(dict(id=mid,name=model.mat(mid).name,rgba=model.mat_rgba[mid].tolist(),
            specular=float(model.mat_specular[mid]),
            roughness=float(model.mat_roughness[mid]),metallic=float(model.mat_metallic[mid]),
            emission=float(model.mat_emission[mid]),texrepeat=model.mat_texrepeat[mid].tolist(),
            texuniform=bool(model.mat_texuniform[mid]),textures={role.name:int(model.mat_texid[mid,int(role)]) for role in
                (mujoco.mjtTextureRole.mjTEXROLE_RGB,mujoco.mjtTextureRole.mjTEXROLE_RGBA,mujoco.mjtTextureRole.mjTEXROLE_ROUGHNESS,mujoco.mjtTextureRole.mjTEXROLE_NORMAL) if model.mat_texid[mid,int(role)]>=0}))
    geoms=[]
    for gid in range(model.ngeom):
        kind=int(model.geom_type[gid])
        if kind not in (3,5,6,7):raise ValueError(('Unsupported visual primitive',gid,kind))
        local_rotation=np.empty(9);mujoco.mju_quat2Mat(local_rotation,model.geom_quat[gid])
        geoms.append(dict(id=gid,name=model.geom(gid).name,kind=kind,mesh=int(model.geom_dataid[gid]) if kind==7 else None,
            body_id=int(model.geom_bodyid[gid]),local_position=model.geom_pos[gid].tolist(),
            local_rotation=local_rotation.reshape(3,3).tolist(),
            size=model.geom_size[gid].tolist(),position=data.geom_xpos[gid].tolist(),rotation=data.geom_xmat[gid].reshape(3,3).tolist(),
            material=int(model.geom_matid[gid]),rgba=model.geom_rgba[gid].tolist(),group=int(model.geom_group[gid])))
    result=dict(schema='robofrontier.blender-visual.v1',source_scene=str(scene),source_scene_sha256=digest,
        units='metres',coordinate_frame='+X east,+Y north,+Z up',physics_stepped=False,
        scope='Exact visual FK export, not dynamics conversion',meshes=meshes,textures=textures,materials=materials,
        geoms=geoms,cameras=manifest['cameras'])
    result['bodies']=[dict(id=i,name=model.body(i).name,position=data.xpos[i].tolist(),rotation=data.xmat[i].reshape(3,3).tolist())for i in range(model.nbody)]
    config=json.loads((scene.parent/'build-config.json').read_text())if (scene.parent/'build-config.json').exists()else {}
    result['offline_lighting']=config.get('offline_lighting',{})
    result['building']=config.get('building',dict(width_m=55.,depth_m=50.,eave_height_m=7.2))
    exterior=scene.parent/'assets/textures/exterior-daylight-v1.png'
    if exterior.exists():
        shutil.copy2(exterior,output/'textures/exterior-daylight-v1.png')
        result['exterior_texture']='textures/exterior-daylight-v1.png'
    (output/'scene.json').write_text(json.dumps(result,separators=(',',':'))+'\n')
    print(json.dumps(dict(output=str(output),geoms=len(geoms),unique_meshes=len(meshes),physics_stepped=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();export(a.scene,a.output)
