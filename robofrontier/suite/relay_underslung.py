"""R16: user-declared underside eyes and uniform wooden pendulums.

Manufactured source revision only. Existing scenes, L1, live states, robot
mounts, actuator parameters and acceptance rules are not edited.
"""
from copy import deepcopy
import hashlib,json,math,os,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from . import v2
from .rigid_relay import PROFILE, METAL, capsule, ring, spring_hinge
from .scenes import cylinder,box,vector,rz
from ..robot_control import matrix_quat,quat_matrix
from ..run_setup import write_json

REVISION=16
SHAFT=.170
TIP_DISTANCE=.100


def design(arms,level):
    if arms not in (8,12,16) or level not in (2,3):raise ValueError('R16 changes only six L2/L3 cases')
    chord=1.2 if arms==16 else 1.15
    if level==2:
        angle=math.radians(63.);radius=chord/(2*math.sin(angle/2))
        return dict(chord_m=chord,circular_angle_deg=63.,radius_m=radius,
                    arc_length_m=radius*angle,bend_rise_m=radius*(1-math.cos(angle/2)))
    rise=chord/2*math.tan(math.radians(15.75))
    return dict(chord_m=chord,bend_rise_m=rise,interior_angle_deg=148.5,
                centerline_length_m=2*math.hypot(chord/2,rise))


def attachment_point(mechanism,level):
    if level==2:
        angle=math.radians(mechanism['circular_angle_deg']/2)-TIP_DISTANCE/mechanism['radius_m']
        r=mechanism['radius_m'];point=[r*(math.cos(angle)-math.cos(math.radians(mechanism['circular_angle_deg']/2))),r*math.sin(angle),-.0225]
    else:
        half=mechanism['chord_m']/2;length=math.hypot(half,mechanism['bend_rise_m'])
        point=[TIP_DISTANCE*mechanism['bend_rise_m']/length,half-TIP_DISTANCE*half/length,-.0225]
    return np.array(point)


def transverse_axis(mechanism):
    axis=np.array([mechanism['chord_m']/2,mechanism['bend_rise_m'],0.]);return axis/np.linalg.norm(axis)


def orient_hinge(beam,ident,mechanism):
    axis=transverse_axis(mechanism);angle=math.atan2(axis[1],axis[0]);turn=rz(angle)
    child=beam.find(f"body[@name='{ident}_distal']");joint=child.find('joint');joint.set('axis',vector(axis));pivot=np.fromstring(child.get('pos'),sep=' ')
    housing=beam.find(f"geom[@name='{ident}_hinge_housing']");housing.set('quat',vector(matrix_quat(turn@quat_matrix(np.fromstring(housing.get('quat'),sep=' ')))))
    for geom in beam.findall('geom'):
        if geom.get('name','').startswith(ident+'_hinge_coil_'):
            points=np.fromstring(geom.get('fromto'),sep=' ').reshape(2,3);geom.set('fromto',vector(((points-pivot)@turn.T+pivot).ravel()))
    return dict(mode='out_of_plane',axis_local_xyz=axis.tolist(),joint=ident+'_flexion',
        motion='Up/down in the moving half vertical plane; top-view centreline opening fixed when Root is horizontal',
        rest_angle_rad=0.,stiffness_nm_rad=18.,damping_nms_rad=.55,limit_rad=math.radians(25),
        horizontal_projection_angle_deg=148.5)


def make_weight(root,ident,material):
    body=ET.Element('body',name=ident+'_weight');ET.SubElement(body,'freejoint',name=ident+'_weight_free')
    ring_mass=ring(body,ident+'_weight_eye',[0,0,0],PROFILE['eye_centerline_radius_m'],(1,2))
    shaft_mass=600*math.pi*.005**2*SHAFT
    cylinder(body,ident+'_weight_shaft',[0,0,-.018-SHAFT/2],[.005,SHAFT/2],[.65,.41,.20,1],mass=shaft_mass,friction='.7 .003 .0001',solref='.004 1')
    center=np.array([0.,0.,-.018-SHAFT-.030])
    cylinder(body,ident+'_weight_base',center+[0,0,-.021],[.033,.004],METAL,mass=.6*.20,solref='.004 1')
    cylinder(body,ident+'_weight_core',center+[0,0,-.003],[.029,.014],METAL,mass=.6*.65,solref='.004 1')
    cylinder(body,ident+'_weight_neck',[0,0,-.191],[.009,.009],METAL,mass=.6*.01,solref='.004 1')
    vertices=[[r*math.cos(t),r*math.sin(t),z+center[2]] for r,z in ((.029,.011),(.013,.024)) for t in np.arange(24)*2*math.pi/24]
    mesh=ident+'_weight_shoulder_r16';ET.SubElement(root.find('asset'),'mesh',name=mesh,vertex=vector(np.ravel(vertices)))
    ET.SubElement(body,'geom',name=ident+'_weight_shoulder',type='mesh',mesh=mesh,mass=str(.6*.14),rgba=vector(METAL),solref='.004 1')
    if material:box(body,ident+'_weight_mass_label',center+[0,-.0292,-.001],[.022,.00015,.009],visual=True,material=material)
    return body,.6+ring_mass+shaft_mass


def exact_bottom(model,data,ids):
    minimum=math.inf
    for gid in ids:
        rotation=data.geom_xmat[gid].reshape(3,3);pos=data.geom_xpos[gid];size=model.geom_size[gid];kind=int(model.geom_type[gid])
        if kind==mujoco.mjtGeom.mjGEOM_MESH:
            mid=int(model.geom_dataid[gid]);start=int(model.mesh_vertadr[mid]);count=int(model.mesh_vertnum[mid]);value=np.min((model.mesh_vert[start:start+count]@rotation.T+pos)[:,2])
        elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE:value=pos[2]-abs(rotation[2,2])*size[1]-size[0]
        elif kind==mujoco.mjtGeom.mjGEOM_CYLINDER:value=pos[2]-abs(rotation[2,2])*size[1]-size[0]*math.sqrt(max(0,1-rotation[2,2]**2))
        else:value=pos[2]-(abs(rotation)@size)[2]
        minimum=min(minimum,float(value))
    return minimum


def fit_initial(model,data,task,hook_points):
    """Two real supports: low timber end and underside eye, not a hidden stand."""
    rows=[];table=task['table_height_m']
    for item in task['mechanism']['payload_assemblies']:
        ident=item['id'];free=data.joint(ident+'_free').qpos;xy=free[:2].copy();old=quat_matrix(free[3:].copy());yaw=math.atan2(old[1,0],old[0,0]);heading=rz(yaw)
        if task['level']==3:data.joint(ident+'_flexion').qpos[0]=0.
        rod=[g for g in range(model.ngeom) if (model.geom(g).name or '').startswith(ident+'_beam') and (model.geom_contype[g] or model.geom_conaffinity[g])]
        hooks=[g for g in range(model.ngeom) if (model.geom(g).name or '').startswith(ident+'_hook_')]
        def geometry(pitch):
            c,s=math.cos(pitch),math.sin(pitch);rotation=heading@np.array([[1,0,0],[0,c,-s],[0,s,c]])
            free[:]=np.r_[xy,0.,matrix_quat(rotation)];mujoco.mj_kinematics(model,data)
            return exact_bottom(model,data,hooks)-exact_bottom(model,data,rod),rotation
        lo,hi=0.,.15;assert geometry(lo)[0]<0<geometry(hi)[0],ident
        for _ in range(48):
            mid=(lo+hi)/2
            if geometry(mid)[0]<0:lo=mid
            else:hi=mid
        _,rotation=geometry((lo+hi)/2);free[2]=table-exact_bottom(model,data,rod)
        eye=free[:3]+rotation@(hook_points[ident]+np.array([-.014,0,-.014]));weight=data.joint(ident+'_weight_free').qpos
        gids=[g for g in range(model.ngeom) if model.geom_bodyid[g]==model.body(ident+'_weight').id and (model.geom_contype[g] or model.geom_conaffinity[g])]
        def weight_bottom(angle):
            c,s=math.cos(angle),math.sin(angle);r=rotation@np.array([[c,0,s],[0,1,0],[-s,0,c]])
            weight[:]=np.r_[eye,matrix_quat(r)];mujoco.mj_kinematics(model,data);return exact_bottom(model,data,gids)-table
        a=math.radians(80);fa=weight_bottom(a);bracket=None
        for degrees in np.arange(81.,112.,.5):
            b=math.radians(float(degrees));fb=weight_bottom(b)
            if fa<=0<=fb:bracket=(a,b);break
            a,fa=b,fb
        if bracket is None:raise ValueError('No nonpenetrating tabletop weight pose '+ident)
        lo2,hi2=bracket
        for _ in range(48):
            mid=(lo2+hi2)/2
            if weight_bottom(mid)<0:lo2=mid
            else:hi2=mid
        weight_bottom((lo2+hi2)/2)
        item['initial_position_m']=free[:3].tolist();item['initial_quaternion_wxyz']=free[3:].tolist()
        obj=next(o for o in task['objects'] if o['id']==ident);obj.update(initial_position_m=free[:3].tolist(),initial_quaternion_wxyz=free[3:].tolist())
        rows.append(dict(payload=ident,beam_pitch_deg=math.degrees((lo+hi)/2),weight_initial_tilt_deg=math.degrees((lo2+hi2)/2),timber_bottom_m=exact_bottom(model,data,rod),fixed_eye_bottom_m=exact_bottom(model,data,hooks),weight_bottom_m=exact_bottom(model,data,gids)))
    return rows


def build(case,replace_draft=False,render=True,source=None,destination=None):
    from ..workstation import Workstation
    if case['level']==1:raise ValueError('L1 is immutable; retain its existing source')
    source_case=dict(case,scene_revision=15 if case['arms']==12 else 13)
    source=Path(source) if source else v2.directory(source_case);frozen=Workstation(source)
    folder=Path(destination) if destination else v2.directory(dict(case,scene_revision=REVISION))
    if folder.exists():
        if not replace_draft:raise FileExistsError(folder)
        validation=json.loads((folder/'validation.json').read_text()) if (folder/'validation.json').exists() else {}
        if (folder/'FROZEN').exists() or validation.get('oracle_verified') or validation.get('frozen_input'):raise ValueError('Frozen r16 input; create a new revision')
        shutil.rmtree(folder)
    folder.mkdir(parents=True);(folder/'assets').mkdir()
    for asset in frozen.manifest['assets']:
        original=source/asset['path'];target=folder/asset['path'];target.parent.mkdir(parents=True,exist_ok=True)
        try:os.link(original,target)
        except OSError:shutil.copy2(original,target)
    root=ET.parse(source/'scene.xml').getroot();env=json.loads((source/'environment.json').read_text());task=json.loads((source/'task.json').read_text());world=root.find('worldbody')
    newcase=dict(case,scene_revision=REVISION,source_scene_revision=source_case['scene_revision']);newcase['directory']=str(folder.relative_to(v2.ROOT));task.update(scale=newcase,scene_asset=newcase['directory'],oracle_verified=False,execution_readiness='environment_preview_only');env['suite_case']=newcase
    params=design(case['arms'],case['level']);level=case['level'];hooks={};mass=None;replacements={};old_meshes=set()
    for item in task['mechanism']['payload_assemblies']:
        ident=item['id'];old=world.find(f"body[@name='{ident}']");oldweight=world.find(f"body[@name='{ident}_weight']")
        for body in [old,oldweight]:
            old_meshes.update(g.get('mesh') for g in body.iter('geom') if g.get('mesh'))
        label=old.find(f"geom[@name='{ident}_label']");material=label.get('material') if label is not None else None
        mark=oldweight.find(f"geom[@name='{ident}_weight_mass_label']");weight_material=mark.get('material') if mark is not None else None
        beam=ET.Element('body',name=ident,pos=old.get('pos'),quat=old.get('quat'));ET.SubElement(beam,'freejoint',name=ident+'_free')
        if level==2:shape=v2.curved_beam(root,beam,ident+'_beam',params['chord_m'],63.)
        else:shape=v2.angled_beam(root,beam,ident+'_beam',params['chord_m'],31.5)
        parent=beam;pivot=np.zeros(3)
        if level==3:
            parent,pivot=spring_hinge(root,beam,shape['bend_rise_m'],prefix=ident);item['elastic_hinge']=orient_hinge(beam,ident,shape)
        else:item.pop('elastic_hinge',None)
        mount=attachment_point(params,level);hook=mount+[0,0,-PROFILE['hook_centerline_radius_m']-PROFILE['ring_stock_radius_m']+.0015];hooks[ident]=hook
        ring(parent,ident+'_hook',hook-pivot,PROFILE['hook_centerline_radius_m'],(0,2))
        if material:
            x=params.get('bend_rise_m',params['radius_m']*(1-math.cos(math.radians(31.5)))) if level==2 else params['bend_rise_m']
            box(beam,ident+'_label',[x,0,.0228],[.017,.029,.0001],visual=True,material=material)
        weight,mass=make_weight(root,ident,weight_material);weight.set('pos',oldweight.get('pos'));weight.set('quat',oldweight.get('quat'))
        replacements[ident]=beam;replacements[ident+'_weight']=weight
        item.update(attachment=dict(face='underside',distance_from_tip_along_centerline_m=TIP_DISTANCE,surface_point_local_m=mount.tolist(),fixed_eye_center_local_m=hook.tolist(),bracket_stem_m=0.),beam_bodies=[ident,ident+'_distal'] if level==3 else [ident])
    for old in list(world):
        if old.get('name') in replacements:
            index=list(world).index(old);world.remove(old);world.insert(index,replacements[old.get('name')])
    used={g.get('mesh') for g in root.iter('geom') if g.get('mesh')}
    for mesh in list(root.find('asset').findall('mesh')):
        if mesh.get('name') in old_meshes and mesh.get('name') not in used:root.find('asset').remove(mesh)
    mechanism=task['mechanism'];mechanism.update(shape);mechanism.update(params)
    mechanism.update(kind='underslung_arc_closed_eye_wooden_pendulum' if level==2 else 'underslung_transverse_spring_elbow',suspended_mass_kg=mass,weight_mass_kg=.6,
        suspension='Underside direct fixed eye, physically contact-linked free eye,170mm wooden shaft and600g weight; no bracket, cable or weld',
        pendulum=dict(shaft_length_m=SHAFT,shaft_radius_m=.005,shaft_material='wood',same_shape_for_levels=[2,3],connection='Closed contact-linked eyes; independent free weight body'),
        rigid_suspension_profile=dict(PROFILE,pendulum_shaft_length_m=SHAFT,pendulum_shaft_material='wood'),
        initial_condition='Table-supported tilted beam and directly mounted underside eye; independent wooden pendulum lies on tabletop. No added support fixtures.',
        elastic_hinge=dict(mode='out_of_plane',axis='Transverse to moving half in neutral beam plane',axis_local_xyz=transverse_axis(params).tolist(),rest_angle_rad=0.,stiffness_nm_rad=18.,damping_nms_rad=.55,limit_rad=math.radians(25),mode_counts={'out_of_plane':len(mechanism['payload_assemblies'])}) if level==3 else None)
    task['rules']=[r.split(' L3 rods have different manufactured hinge directions:')[0] for r in task['rules']]
    mechanism['presentation']['apparatus_revision']=REVISION
    mechanism['presentation']['physics_scope']='R16 user-requested apparatus replacement; unchanged robot mounts, worktop, robot controllers, timestep and goal acceptance. Timber shape, suspension and L3 hinge direction follow the new declared geometry.'
    task['rules'].append('R16 manufactured geometry: direct underside fixed eye100mm along the timber from its end; uniform170mm wooden pendulum. All L3 hinges are transverse out-of-plane angular springs. When Root is horizontal their top-view centreline opening remains148.5degrees; projected length changes. Initial support is actual tabletop contact, not a hidden stand.')
    draft=folder/'.build.xml';ET.ElementTree(root).write(draft,encoding='unicode');model=mujoco.MjModel.from_xml_path(str(draft));data=mujoco.MjData(model)
    assert [model.joint(i).name for i in range(model.njnt)]==[frozen.model.joint(i).name for i in range(frozen.model.njnt)]
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);initial=fit_initial(model,data,task,hooks)
    root.find("keyframe/key[@name='initial']").set('qpos',vector(data.qpos))
    for ident,body in replacements.items():
        q=data.joint(ident+'_free').qpos;body.set('pos',vector(q[:3]));body.set('quat',vector(q[3:]))
    ET.indent(root);ET.ElementTree(root).write(folder/'scene.xml',encoding='unicode');draft.unlink();task['scene_sha256']=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest()
    task['apparatus_revision']=dict(revision=REVISION,source_directory=str(source.relative_to(v2.ROOT)),source_scene_sha256=frozen.manifest['scene_sha256'],user_requested=True,L1_unchanged=True,robot_and_table_geometry_unchanged=True,initial_support_fit=initial)
    env.update(objects=deepcopy(task['objects']),mechanism=deepcopy(mechanism));env['apparatus_revision']=deepcopy(task['apparatus_revision'])
    write_json(folder/'task.json',task);write_json(folder/'environment.json',env);write_json(folder/'public-task.json',v2.public_task(task));write_json(folder/'parameters.json',dict(physics=env['physics'],mechanism=mechanism,apparatus_revision=task['apparatus_revision']))
    model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data);v2.manifest(model,data,newcase,folder,env)
    result=validate(folder,source);write_json(folder/'validation.json',result)
    if render:render_views(folder)
    return folder


def validate(folder,source=None):
    from ..workstation import Workstation
    w=Workstation(folder);m,d=w.model,w.data;mujoco.mj_forward(m,d);task=json.loads((Path(folder)/'task.json').read_text());issues=[]
    for contact in d.contact:
        if contact.dist<-.0002:
            names=[m.geom(int(g)).name for g in contact.geom]
            if any('payload_' in (name or '') for name in names):issues.append(dict(geoms=names,penetration_m=-float(contact.dist)))
    if issues:raise ValueError('Initial apparatus penetrations '+str(issues[:12]))
    if source:
        old=Workstation(source)
        for field in ['actuator_gainprm','actuator_biasprm','actuator_forcerange','actuator_ctrlrange','jnt_range','jnt_stiffness','dof_damping']:
            np.testing.assert_allclose(getattr(m,field),getattr(old.model,field),rtol=0,atol=0)
        for ident,arm in task['arms'].items():
            np.testing.assert_allclose(d.body(ident+'_link0').xpos,arm['base_position_m'],rtol=0,atol=1e-12)
    assert not any('hook_bracket' in (m.geom(g).name or '') for g in range(m.ngeom))
    assert m.nu==8*len(task['arms']) and m.nplugin==0 and m.opt.timestep==.002
    links={item['id']:linking_number(m,d,item['id']) for item in task['mechanism']['payload_assemblies']}
    assert all(abs(abs(value)-1)<.03 for value in links.values()),links
    return dict(status='static_checks_passed',oracle_verified=False,policy_verified=False,frozen_input=False,scene_revision=REVISION,physics_steps=0,initial_support=task['apparatus_revision']['initial_support_fit'],initial_penetrations=issues,ring_linking_numbers=links,robot_mounts_preserved=True,independent_free_weights=True,scope='New user-declared apparatus; unchanged robots/worktop/contact and acceptance parameters; not Oracle success')


def linking_number(model,data,ident):
    """Gauss linking integral of the two closed centreline polygons (FK only)."""
    chains=[]
    for prefix in [ident+'_hook_',ident+'_weight_eye_']:
        pieces=[]
        for gid in range(model.ngeom):
            if not (model.geom(gid).name or '').startswith(prefix):continue
            p=data.geom_xpos[gid];axis=data.geom_xmat[gid].reshape(3,3)[:,2]*model.geom_size[gid][1]
            a,b=p-axis,p+axis
            for j in range(5):pieces.append((a+(b-a)*(j+.5)/5,(b-a)/5))
        chains.append((np.array([p for p,_ in pieces]),np.array([v for _,v in pieces])))
    (a,da),(b,db)=chains;delta=a[:,None,:]-b[None,:,:]
    return float(np.sum(np.sum(np.cross(da[:,None,:],db[None,:,:])*delta,axis=2)/np.linalg.norm(delta,axis=2)**3)/(4*math.pi))


def render_views(folder):
    from PIL import Image
    w=__import__('robofrontier.workstation',fromlist=['Workstation']).Workstation(folder);m,d=w.model,w.data;mujoco.mj_forward(m,d);m.vis.global_.offwidth=1600;m.vis.global_.offheight=1000
    with mujoco.Renderer(m,width=1600,height=1000,max_geom=max(5000,m.ngeom+1000)) as renderer:
        for camera in ['global','global_rear','top']:
            renderer.update_scene(d,camera=camera);Image.fromarray(renderer.render()).save(Path(folder)/('preview-'+camera+'.png'))
