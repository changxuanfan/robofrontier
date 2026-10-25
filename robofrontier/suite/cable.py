"""Declared, passive first-party elastic cable; no policy-side tuning."""
import math
import xml.etree.ElementTree as ET
import numpy as np

PROFILE = dict(
    model='mujoco.elasticity.cable', length_m=.20, radius_m=.002, segments=16,
    linear_mass_kg_m=.010, bend_pa=1e6, twist_pa=3e5,
    relaxation_time_s=.005, armature_kg_m2=0., stress_free_straight=True,
    timestep_s=.0005, endpoint_solref_s=.002,
    material='Soft braided cord equivalent; engineering assumption, not measured material',
    physical_calibration_verified=False,
)


def rotation_to_z(direction):
    z=np.asarray(direction,dtype=float);z/=np.linalg.norm(z)
    trial=np.array([1.,0,0]) if abs(z[0])<.85 else np.array([0,1.,0])
    x=np.cross(trial,z);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))


def install(root,parent,parent_p,parent_r,points,target_site,name,profile=None):
    from ..robot_control import matrix_quat
    from .scenes import vector
    cfg=dict(PROFILE if profile is None else profile);n=len(points)-1
    length=float(sum(np.linalg.norm(b-a) for a,b in zip(points,points[1:])))
    if n!=cfg['segments'] or not math.isclose(length,cfg['length_m'],abs_tol=1e-9):raise ValueError('Cable geometry/profile mismatch')
    segment=length/n;radius=cfg['radius_m'];mass=cfg['linear_mass_kg_m']*segment
    damping=cfg['relaxation_time_s']*cfg['bend_pa']*(math.pi*radius**4/4)/segment
    extension=root.find('extension')
    if extension is None:extension=ET.SubElement(root,'extension')
    plugin=next((e for e in extension.findall('plugin') if e.get('plugin')==PROFILE['model']),None)
    if plugin is None:plugin=ET.SubElement(extension,'plugin',plugin=PROFILE['model'])
    instance=ET.SubElement(plugin,'instance',name=name+'_elastic')
    for key,value in dict(bend=cfg['bend_pa'],twist=cfg['twist_pa'],flat='true' if cfg['stress_free_straight'] else 'false',vmax=0).items():
        ET.SubElement(instance,'config',key=key,value=str(value))
    previous_p=np.asarray(parent_p);previous_r=np.asarray(parent_r);previous_name=None
    contact=root.find('contact')
    for i,(start,end) in enumerate(zip(points,points[1:])):
        r=rotation_to_z(end-start)
        body=ET.SubElement(parent,'body',name=name+f'_link{i}',pos=vector(previous_r.T@(start-previous_p)),quat=vector(matrix_quat(previous_r.T@r)))
        ET.SubElement(body,'joint',name=name+f'_joint{i}',type='ball',damping=str(damping),armature=str(cfg['armature_kg_m2']))
        ET.SubElement(body,'geom',name=name+f'_geom{i}',type='capsule',fromto=vector([0,0,0,0,0,segment]),size=str(radius),mass=str(mass),rgba='.22 .31 .38 1',contype='4',conaffinity='5',friction='.6 .003 .0001')
        ET.SubElement(body,'plugin',instance=name+'_elastic')
        if previous_name:ET.SubElement(contact,'exclude',body1=previous_name,body2=body.get('name'))
        previous_name=body.get('name');previous_p=start;previous_r=r;parent=body
    site=name+'_end'
    ET.SubElement(parent,'site',name=site,pos=vector([0,0,segment]),size='.002',rgba='0 0 0 0')
    ET.SubElement(root.find('equality'),'connect',name=name+'_attachment',site1=site,site2=target_site,solref=f"{cfg['endpoint_solref_s']} 1",solimp='.999 .9999 .0001 .5 2')
    return parent.get('name')
