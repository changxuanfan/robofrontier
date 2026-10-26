"""Table-only geometry edits on an independent FK rendering model.

This helper is never called by a physical executor or used as a new task pass.
It preserves the joint/geometry registry so recorded poses and colors still map.
"""
import re
import numpy as np


def scale_rectangular_opening_for_render(model,scale=.5):
    if not 0<float(scale)<=1:raise ValueError('Opening scale must be in (0,1]')
    if scale==1:return None
    ids=[i for i in range(model.ngeom) if re.fullmatch(r'table_\d+',model.geom(i).name or '')]
    if len(ids)!=4:return None  # Four-arm Transport already has a solid top.
    if any(model.geom_bodyid[i]!=0 or not np.allclose(model.geom_quat[i],[1,0,0,0]) for i in ids):return None
    old_pos=model.geom_pos[ids].copy();old_size=model.geom_size[ids].copy()
    if any(abs(p[0])>1e-9 and abs(p[1])>1e-9 for p in old_pos):return None
    vertical=[j for j,p in enumerate(old_pos) if abs(p[0])<1e-9 and abs(p[1])>1e-9]
    horizontal=[j for j,p in enumerate(old_pos) if abs(p[1])<1e-9 and abs(p[0])>1e-9]
    if len(vertical)!=2 or len(horizontal)!=2:return None
    ix=min(abs(old_pos[j,0])-old_size[j,0] for j in horizontal)
    iy=min(abs(old_pos[j,1])-old_size[j,1] for j in vertical)
    if ix<=0 or iy<=0:return None
    ox=max(abs(p[0])+s[0] for p,s in zip(old_pos,old_size))
    oy=max(abs(p[1])+s[1] for p,s in zip(old_pos,old_size))
    nx,ny=ix*scale,iy*scale
    for j,i in enumerate(ids):
        p,s=old_pos[j].copy(),old_size[j].copy()
        if j in vertical:
            p[1]=np.sign(p[1])*(oy+ny)/2;s[0]=ox;s[1]=(oy-ny)/2
        else:
            p[0]=np.sign(p[0])*(ox+nx)/2;s[0]=(ox-nx)/2;s[1]=ny
        model.geom_pos[i]=p;model.geom_size[i]=s
        model.geom_rbound[i]=np.linalg.norm(s)
        model.geom_aabb[i,:3]=0.;model.geom_aabb[i,3:]=s
    return dict(kind='tabletop-opening-only',linear_scale=float(scale),
        old_opening_m=[2*ix,2*iy],new_opening_m=[2*nx,2*ny],
        outer_envelope_m=[2*ox,2*oy],table_geoms=[model.geom(i).name for i in ids],
        legs_arms_objects_goals_unchanged=True,recorded_poses_unchanged=True,
        source_physical_scene_modified=False,presentation_model_geometry_modified=True,
        physical_reexecution=False)
