# Frozen pre-vectorization predicates for independent differential tests.
"""Private geometry-driven v2 acceptance; no policy observations or fixed pairs."""
import math
import numpy as np
import mujoco
from robofrontier.suite.evaluate import geom_extent


def placement_candidates(mon,positions,stable):
    task=mon.task;m,d=mon.m,mon.d;goals=task['goals'];out=np.zeros((len(goals),len(mon.names)),bool)
    side=task['cube_size_m'];tol=.0003
    for i,g in enumerate(goals):
        target=np.asarray(g['position_m'])
        if task['family']=='transport':
            board=d.body(g['board']);br=board.xmat.reshape(3,3);bv=mon.velocity(board.id)
            if np.linalg.norm(bv[3:])>.01 or np.linalg.norm(bv[:3])>.1:continue
        for j,o in enumerate(mon.objects):
            if not stable[j] or any(o.get(k)!=v for k,v in g['requires'].items()):continue
            body=d.body(o['id']);r=body.xmat.reshape(3,3)
            if task['family']=='mosaic':
                half=side/2*np.sum(abs(r),axis=1);delta=positions[j]-target
                if np.min(np.max(abs(r),axis=0))<math.cos(math.radians(8)):continue
                if task['level']==1:
                    opening=task['grid']['pocket_opening_m']/2
                    if np.any(abs(delta[:2])+half[:2]>opening+tol):continue
                    if abs(positions[j,2]-half[2]-task['grid']['floor_z_m'])>.0008:continue
                    root=m.geom(o['id']+'_geom').id
                    touching=any(root in c.geom and m.geom(int(c.geom[1] if c.geom[0]==root else c.geom[0])).name=='pocket_bottom' and c.dist<=.0002 for c in d.contact)
                    if not touching and mujoco.mj_geomDistance(m,d,root,m.geom('pocket_bottom').id,.00021,None)>.0002:continue
                elif np.linalg.norm(delta[:2])>.005 or abs(delta[2])>.003:continue
            else:
                p=br.T@(positions[j]-board.xpos);rr=br.T@r;local=np.asarray(g['local_position_m'])
                if abs(p[2]-local[2])>.0015 or rr[2,2]<math.cos(math.radians(5)):continue
                if task['level']==1:
                    root=m.geom(o['id']+'_geom').id;gr=d.geom_xmat[root].reshape(3,3);typ=o['shape']
                    extent=(side/2*np.sum(abs(br.T@gr),axis=1) if typ=='cube' else side/2*(abs((br.T@gr)[:,2])+np.sqrt(np.maximum(0,1-(br.T@gr)[:,2]**2))))
                    if np.any(abs(p[:2]-local[:2])+extent[:2]>(side+.010)/2+tol):continue
                else:
                    mechanism=task['mechanism'];spacing=mechanism['blade_spacing_m'];opening=np.asarray(mechanism['slot_opening_m'])/2;depth=mechanism['slot_depth_m'];valid=True
                    for blade in (0,1):
                        gid=m.geom(o['id']+f'_blade{blade}').id;center=br.T@(d.geom_xpos[gid]-board.xpos);rot=br.T@d.geom_xmat[gid].reshape(3,3);extent=abs(rot)@m.geom_size[gid]
                        expected=np.array([(-1 if blade==0 else 1)*spacing/2,local[1]])
                        # 180-degree placement swaps the two identical blades.
                        nearest=min((np.array([-spacing/2,local[1]]),np.array([spacing/2,local[1]])),key=lambda q:np.linalg.norm(center[:2]-q))
                        if np.any(abs(center[:2]-nearest)+extent[:2]>opening+tol) or center[2]+extent[2]>.0005 or center[2]-extent[2]<-depth-tol:valid=False;break
                    if not valid:continue
            out[i,j]=True
    return out

