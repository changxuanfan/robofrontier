"""Transport motion signals and optional image verification primitives.

The native source editor uses signal_flags/quaternion_path. Image helpers verify
historical delivered cuts; frozen generator snapshots retain their original code.
"""
import math
import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d, uniform_filter1d, label

IMAGE_IGNORE_TOP_PX = 90
IMAGE_MIN_CONNECTED_PIXELS = 8

def odd(seconds, dt, n):
    return max(3, min(int(round(seconds/dt))|1, n if n%2 else n-1))

def excursion(x,w):
    return maximum_filter1d(x,w,mode='nearest')-minimum_filter1d(x,w,mode='nearest')

def boxsum(x,w):return uniform_filter1d(x,w,mode='nearest')*w

def trend_confident(x,w,reversals,tol,cap,dt=1/60):
    """OLS drift, with effective samples limited by oscillation cycles, not Hz."""
    n=len(x);h=w//2;p=np.pad(x,(h,h),mode='edge');idx=np.arange(len(p),dtype=float)
    sx=np.r_[0.,np.cumsum(p)];six=np.r_[0.,np.cumsum(idx*p)];sxx=np.r_[0.,np.cumsum(p*p)]
    total=sx[w:]-sx[:-w];weighted=six[w:]-six[:-w];mean=total/w
    slope=(weighted-(np.arange(n)+h)*total)/(w*(w*w-1)/12)
    variance=np.maximum(0,(sxx[w:]-sxx[:-w])/w-mean*mean)
    residual=np.maximum(0,variance-slope*slope*(w*w-1)/12)
    neff=np.maximum(2,boxsum(reversals,w)/2)
    drift=np.abs(slope)*(w-1);error=3*np.sqrt(12*residual/neff)
    return (drift>np.maximum(2*tol,error)) & (drift/max(1.,w-1)*(2/dt)>tol) & (excursion(x,w)<=cap)

def signal_flags(values,dt,tol,cap,protect=None):
    """Return effective, raw, isolated-jitter masks for one physical coordinate."""
    x=np.asarray(values,dtype=float);x=x-x[0];n=len(x)
    if n<5:
        raw=np.r_[False,np.abs(np.diff(x))>tol];return raw,raw,np.zeros(n,bool)
    sm=uniform_filter1d(x,3,mode='nearest');dx=np.diff(sm,prepend=sm[0]);sign=np.sign(dx)
    sign[np.abs(dx)<tol*.005]=0;last=np.maximum.accumulate(np.where(sign!=0,np.arange(n),0));sign=sign[last]
    rev=np.r_[0.,(sign[1:]*sign[:-1]<0).astype(float)]
    short=odd(.25,dt,n);long=odd(2,dt,n);cyclic=odd(16,dt,n);trend=odd(64,dt,n)
    raw=(excursion(x,short)>tol)|(excursion(x,long)>tol)
    jitter=np.zeros(n,bool)
    for w in sorted({long,cyclic,trend}):
        h=w//2;indices=np.arange(n);net=np.abs(sm[np.minimum(n-1,indices+h)]-sm[np.maximum(0,indices-h)])
        path=boxsum(np.abs(dx),w)
        jitter|=(excursion(sm,w)<=cap)&(boxsum(rev,w)>=3)&(path>8*tol)&(net<.3*path)
    drift=trend_confident(sm,cyclic,rev,tol,cap,dt)|trend_confident(sm,trend,rev,tol,cap,dt)
    micro=excursion(x,5)>max(tol*.001,1e-12)
    effective=(raw|drift)&micro&(~jitter|drift)
    if protect is not None:effective|=raw&micro&protect
    return effective,raw,raw&~effective&jitter

def quaternion_path(q):
    q=np.asarray(q,dtype=float);q/=np.linalg.norm(q,axis=1)[:,None]
    p,v=q[:-1,0],q[:-1,1:];s,w=q[1:,0],q[1:,1:]
    scalar=s*p+np.sum(w*v,axis=1);vector=p[:,None]*w-s[:,None]*v+np.cross(v,w)
    vector*=np.where(scalar<0,-1.,1.)[:,None];length=np.linalg.norm(vector,axis=1)
    factor=np.divide(2*np.arctan2(length,np.abs(scalar)),length,out=np.full_like(length,2.),where=length>1e-12)
    return np.vstack((np.zeros(3),np.cumsum(vector*factor[:,None],axis=0)))

def bounded_rattle(features,dt,tolerances,caps):
    """Stationary-envelope repeated motion in an arm-quiet stretch; preserve outliers.
    Robust envelopes exclude short real falls/transitions instead of erasing them.
    A persistent drift in ANY coordinate prevents suppressing the entity.
    """
    x=np.asarray(features,dtype=float);n=len(x);result=np.zeros(n,bool)
    if n*dt<8:return result
    tol=np.asarray(tolerances);caps=np.asarray(caps);low,high=np.quantile(x,[.005,.995],axis=0);spread=high-low
    quarter=max(1,n//4);first=x[:quarter].mean(axis=0);last=x[-quarter:].mean(axis=0)
    drift2=np.abs(last-first)/max(dt*(n-quarter),dt)*2
    if np.any((drift2>tol)&(spread>2*tol)):return result
    eligible=spread>2*tol
    if not np.any(eligible)or np.any(spread[eligible]>caps[eligible]):return result
    repeated=False;first_turn=0
    for col in np.flatnonzero(eligible):
        y=uniform_filter1d(x[:,col],max(3,int(round(.1/dt))|1),mode='nearest');dx=np.diff(y);sign=np.sign(dx);sign[np.abs(dx)<tol[col]*.005]=0
        nz=sign[sign!=0];reversals=int(np.sum(nz[1:]*nz[:-1]<0));travel=float(np.abs(dx).sum())
        if reversals>=4 and travel>4*spread[col] and abs(last[col]-first[col])<.35*spread[col]:
            nonzero=np.flatnonzero(sign);turns=nonzero[1:][sign[nonzero[1:]]*sign[nonzero[:-1]]<0];first_turn=int(turns[0])+1 if len(turns)else 0;repeated=True;break
    if not repeated:return result
    centre=(low+high)/2;radius=np.maximum(spread*.625,tol*3)
    result=np.all(abs(x-centre)<=radius,axis=1);result[:first_turn+5]=False
    return result

class JitterRegions:
    def __init__(self,trial,names,width,height):
        import mujoco
        self.mj=mujoco;self.m=mujoco.MjModel.from_xml_path(str(trial/'scene.xml'));self.d=mujoco.MjData(self.m);self.camera=self.m.camera('global').id;self.w=width;self.h=height;self.groups=[]
        for name in names:
            if name.startswith('robot:'):
                actor=name.split(':')[1];bodies={i for i in range(self.m.nbody)if self.m.body(i).name.startswith(actor+'_')}
            else:
                body=self.m.body(name.split(':',1)[1]).id;bodies={body}
                for i in range(body+1,self.m.nbody):
                    if int(self.m.body_parentid[i])in bodies:bodies.add(i)
            self.groups.append([i for i in range(self.m.ngeom)if int(self.m.geom_bodyid[i])in bodies])
    def mask(self,q,jitter):
        mask=np.zeros((self.h,self.w),bool)
        if not np.any(jitter):return mask
        m,d=self.m,self.d;d.qpos[:]=q;self.mj.mj_kinematics(m,d);self.mj.mj_comPos(m,d);self.mj.mj_camlight(m,d)
        pos=d.cam_xpos[self.camera];rot=d.cam_xmat[self.camera].reshape(3,3);f=.5*self.h/np.tan(np.radians(m.cam_fovy[self.camera]/2))
        def rectangle(p,r):
            v=rot.T@(p-pos);depth=-v[2]
            if depth<=r:return
            x=self.w/2+f*v[0]/depth;y=self.h/2-f*v[1]/depth;radius=f*r/max(.01,depth-r)+12
            x0,x1=max(0,int(x-radius)),min(self.w,int(x+radius)+1);y0,y1=max(0,int(y-radius)),min(self.h,int(y+radius)+1)
            if x1>x0 and y1>y0:mask[y0:y1,x0:x1]=True
        for active,geoms in zip(jitter,self.groups):
            if not active:continue
            for g in geoms:
                p=d.geom_xpos[g];radius=max(.004,float(m.geom_rbound[g]));rectangle(p,radius)
                for light in range(m.nlight):
                    if not m.light_castshadow[light]:continue
                    directional=bool(m.light_directional[light])if hasattr(m,'light_directional')else int(m.light_type[light])==1
                    ray=d.light_xdir[light]if directional else p-d.light_xpos[light]
                    if ray[2]>=-1e-6:continue
                    for z in [0.,.8]:
                        if p[2]<=z:continue
                        shadow=p+ray*((z-p[2])/ray[2]);rectangle(shadow,radius*(1+abs(ray[0]/ray[2])*.2))
        return mask

def changed_tiles(current,previous,long_reference,mask,threshold=8):
    diff=np.minimum(np.abs(current.astype(np.int16)-previous),np.abs(current.astype(np.int16)-long_reference))
    changed=(diff>threshold)&~mask;changed[:IMAGE_IGNORE_TOP_PX]=False
    if changed.sum()<IMAGE_MIN_CONNECTED_PIXELS:return np.zeros((math.ceil(current.shape[0]/32),math.ceil(current.shape[1]/32)),bool),0
    components,count=label(changed);sizes=np.bincount(components.ravel());sizes[0]=0;valid=sizes>=IMAGE_MIN_CONNECTED_PIXELS;changed=valid[components]
    tiles=np.zeros((math.ceil(current.shape[0]/32),math.ceil(current.shape[1]/32)),bool)
    yy,xx=np.nonzero(changed);tiles[yy//32,xx//32]=True
    return tiles,int(changed.sum())

def load_state_cache(path):
    # NPZ __getitem__ decompresses on every access; load once before frame loops.
    with np.load(path,allow_pickle=False)as archive:return {key:archive[key]for key in archive.files}
