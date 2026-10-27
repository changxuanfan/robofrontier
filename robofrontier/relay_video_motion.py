"""Offline scene-wide motion decisions for removing replay waiting.

The 0.25 s and 2 s tests use net change and directional coherence; summing
noise alone must not turn a stationary force hold into useful motion.
"""
from dataclasses import asdict,dataclass
import numpy as np


@dataclass(frozen=True)
class MotionThresholds:
    short_window_s:float=.25
    long_window_s:float=2.
    robot_joint_strong_rad:float=.0015
    robot_joint_short_rad:float=.002
    robot_joint_long_rad:float=.0015
    jaw_strong_m:float=.0005
    jaw_short_m:float=.0002
    jaw_long_m:float=.0003
    object_strong_m:float=.001
    object_short_m:float=.00075
    object_long_m:float=.0005
    object_strong_rad:float=.005
    object_short_rad:float=.0025
    object_long_rad:float=.0015
    independent_pendant_relocation_m:float=.08

    def manifest(self):return asdict(self)


def window_indices(n,width):
    width=max(1,int(width));i=np.arange(n)
    return np.maximum(0,i-width//2),np.minimum(n-1,i+(width-width//2))


def scalar_signal(x,hz,strong,short,long,short_floor,local_floor):
    x=np.asarray(x,dtype=np.float32)
    if x.ndim==1:x=x[:,None]
    n=len(x);sl,sh=window_indices(n,round(.25*hz));ll,lh=window_indices(n,round(2*hz));il,ih=window_indices(n,2)
    path=np.vstack([np.zeros((1,x.shape[1]),dtype=np.float64),np.cumsum(abs(np.diff(x,axis=0)),axis=0,dtype=np.float64)])
    ds=abs(x[sh]-x[sl]);dl=abs(x[lh]-x[ll]);di=abs(x[ih]-x[il])
    cs=ds/np.maximum(path[sh]-path[sl],1e-12);cl=dl/np.maximum(path[lh]-path[ll],1e-12)
    useful=(di>=strong)|((ds>=short)&(cs>=.6)&(di>=local_floor))|((dl>=long)&(cl>=.5)&(ds>=short_floor)&(di>=local_floor))
    weak=((ds>=short*.5)|(dl>=long*.5))&(di>=local_floor*.5)
    score=np.max(np.maximum(ds/max(short,1e-12),dl/max(long,1e-12)),axis=1)
    return np.any(useful,axis=1),np.any(weak,axis=1),score


def quaternion_angle(a,b):
    a=np.asarray(a,dtype=np.float64);b=np.asarray(b,dtype=np.float64)
    a=a/np.linalg.norm(a,axis=-1,keepdims=True);b=b/np.linalg.norm(b,axis=-1,keepdims=True)
    return 2*np.arccos(np.clip(abs(np.sum(a*b,axis=-1)),0,1))


def rigid_signal(pose,hz,cfg):
    x=np.asarray(pose);n=len(x);sl,sh=window_indices(n,round(.25*hz));ll,lh=window_indices(n,round(2*hz));il,ih=window_indices(n,2)
    p=x[:,:3];q=x[:,3:7]
    def decisions(ds,dl,di,path,strong,short,long,floor,minimum_short):
        cs=ds/np.maximum(path[sh]-path[sl],1e-12);cl=dl/np.maximum(path[lh]-path[ll],1e-12)
        useful=(di>=strong)|((ds>=short)&(cs>=.6)&(di>=floor))|((dl>=long)&(cl>=.5)&(ds>=minimum_short)&(di>=floor))
        weak=((ds>=short*.5)|(dl>=long*.5))&(di>=floor*.5)
        return useful,weak,cl
    path=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1),dtype=np.float64)]
    ds=np.linalg.norm(p[sh]-p[sl],axis=1);dl=np.linalg.norm(p[lh]-p[ll],axis=1);di=np.linalg.norm(p[ih]-p[il],axis=1)
    pm,pw,pc=decisions(ds,dl,di,path,cfg.object_strong_m,cfg.object_short_m,cfg.object_long_m,.000015,.00008)
    relocation=(dl>=cfg.independent_pendant_relocation_m)&(pc>=.5)&(di>=.002)
    angular_path=np.r_[0.,np.cumsum(quaternion_angle(q[1:],q[:-1]),dtype=np.float64)]
    rs=quaternion_angle(q[sh],q[sl]);rl=quaternion_angle(q[lh],q[ll]);ri=quaternion_angle(q[ih],q[il])
    rm,rw,_=decisions(rs,rl,ri,angular_path,cfg.object_strong_rad,cfg.object_short_rad,cfg.object_long_rad,.00003,.0001)
    return pm|rm,pw|rw,relocation


def classify_motion(q,hz,layout,cfg=None):
    cfg=cfg or MotionThresholds();n=len(q);robot=np.zeros(n,bool);primary=robot.copy();pendant=robot.copy();anomaly=robot.copy();weak=robot.copy();robot_score=np.zeros(n,np.float32)
    for group in layout['robot_hinges']:
        useful,ambiguous,score=scalar_signal(q[:,group],hz,cfg.robot_joint_strong_rad,cfg.robot_joint_short_rad,cfg.robot_joint_long_rad,.0001,.00002)
        robot|=useful;weak|=ambiguous;robot_score=np.maximum(robot_score,score)
    for group in layout['robot_slides']:
        useful,ambiguous,score=scalar_signal(q[:,group],hz,cfg.jaw_strong_m,cfg.jaw_short_m,cfg.jaw_long_m,.000025,.000003)
        robot|=useful;weak|=ambiguous;robot_score=np.maximum(robot_score,score)
    for address in layout['primary_free']:
        useful,ambiguous,_=rigid_signal(q[:,address:address+7],hz,cfg);primary|=useful;weak|=ambiguous
    for address in layout['primary_hinges']:
        useful,ambiguous,_=scalar_signal(q[:,address],hz,cfg.object_strong_rad,cfg.object_short_rad,cfg.object_long_rad,.0001,.00003);primary|=useful;weak|=ambiguous
    for address in layout['pendant_free']:
        useful,ambiguous,large_relocation=rigid_signal(q[:,address:address+7],hz,cfg);pendant|=useful|ambiguous;anomaly|=large_relocation
    keep=robot|primary|anomaly
    return dict(keep=keep,robot=robot,primary=primary,pendant=pendant,pendant_relocation=anomaly,weak=weak,robot_score=robot_score,
        visual_candidates=(~keep)&(pendant|weak),weight_only=(~robot)&(~primary)&pendant&(~anomaly))


def ranges(mask):
    padded=np.r_[False,np.asarray(mask,dtype=bool),False];starts=np.flatnonzero(np.diff(padded.astype(np.int8))==1);ends=np.flatnonzero(np.diff(padded.astype(np.int8))==-1)
    return list(zip(starts.tolist(),(ends-1).tolist()))


def image_motion_evidence(before_rgb,after_rgb,before_ids,after_ids,robot_geoms,primary_geoms,pendant_geoms,pixel_threshold=.8):
    """Actual camera-space cross-check, excluding moving-weight occlusion.

    Compare visible geometry centroids and silhouettes, rather than allowing
    a swinging pendant or its shadow to count as robot movement.
    """
    before_ids=np.asarray(before_ids);after_ids=np.asarray(after_ids)
    excluded=np.isin(before_ids,list(pendant_geoms))|np.isin(after_ids,list(pendant_geoms))
    color_change=np.max(abs(before_rgb.astype(np.int16)-after_rgb.astype(np.int16)),axis=2)>12
    result={}
    for label,geoms in [('robot',robot_geoms),('primary',primary_geoms)]:
        moved=False;max_shift=0.;max_changed=0
        visible=set(np.unique(before_ids))|set(np.unique(after_ids))
        for gid in set(geoms)&visible:
            left=(before_ids==gid)&(~excluded);right=(after_ids==gid)&(~excluded);nl=int(left.sum());nr=int(right.sum())
            if min(nl,nr)<4:continue
            cy,cx=np.nonzero(left);dy,dx=np.nonzero(right);shift=float(np.hypot(cx.mean()-dx.mean(),cy.mean()-dy.mean()));union=left|right;changed=int(np.count_nonzero(color_change&union));intersection=int(np.count_nonzero(left&right));iou=intersection/max(1,int(union.sum()))
            max_shift=max(max_shift,shift);max_changed=max(max_changed,changed)
            if (shift>=pixel_threshold and changed>=3) or (min(nl,nr)>=10 and iou<.8 and changed>=8):moved=True
        result[label+'_effective']=moved;result[label+'_max_geom_shift_px']=max_shift;result[label+'_max_changed_pixels']=max_changed
    pendant_mask=np.isin(before_ids,list(pendant_geoms))|np.isin(after_ids,list(pendant_geoms));result['pendant_changed_pixels']=int(np.count_nonzero(color_change&pendant_mask))
    return result
