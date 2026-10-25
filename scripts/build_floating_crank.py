"""Separate eight-arm floating crank variant; preserve fixed cranks and floating gears."""
import argparse,copy,hashlib,json,math,shutil,xml.etree.ElementTree as ET
from pathlib import Path
import mujoco,numpy as np
from PIL import Image
from robofrontier.scene import ROOT,vector
from robofrontier.cube_transport import _camera
from robofrontier.spring_dial import update_display
from robofrontier.run_setup import write_json
from scripts.build_spring_crank_dial import CRANK

SOURCE=ROOT/'configs/environments/spring_dial/floating-04-060-tight-noslip20'
DIRECTORY=ROOT/'configs/environments/spring_dial/floating-crank-04-060-tight-noslip20'


def resize_horizontal_layout(root,parameters,env,spacing):
    old=parameters['pair_spacing_m'];difference=spacing-old
    offsets=[(i-1.5)*difference for i in range(4)];world=root.find('worldbody')
    def translate(element,dx):
        pos=np.fromstring(element.get('pos'),sep=' ');pos[0]+=dx;element.set('pos',vector(pos))
    for i,dx in enumerate(offsets):
        for robot in 'ABCDEFGH'[2*i:2*i+2]:
            for collection in (parameters['arms'],env['stations']):
                entry=next(a for a in collection if a['id']==robot)
                for field in ('base_position_m','home_tcp_m'):entry[field][0]+=dx
            translate(world.find(f"body[@name='{robot}_link0']"),dx)
            translate(world.find(f"geom[@name='{robot}_label']"),dx)
            for side in ('front','side'):translate(world.find(f"camera[@name='{robot}_{side}']"),dx)
        a='ABCD'[i];translate(world.find(f"body[@name='dial_{a}_module']"),dx)
        env['dials'][i]['center_m'][0]+=dx
        for camera in world.findall('camera'):
            if camera.get('name')=='dial_'+a or camera.get('name').startswith('dial_'+a+'_'):translate(camera,dx)
    parameters['pair_spacing_m']=spacing;parameters['table']['length_m']+=3*difference
    env['tables'][0]['half_size_m'][0]+=1.5*difference
    for name in ('table_0','table_support_0'):
        geom=world.find(f"geom[@name='{name}']");size=np.fromstring(geom.get('size'),sep=' ');size[0]+=1.5*difference;geom.set('size',vector(size))
    ratio=(3*spacing+2.1)/(3*old+2.1);center=parameters['table'].get('center_x_m',0);target=np.array([center,0,.55])
    for name in ('global','global_side'):
        camera=world.find(f"camera[@name='{name}']");pos=np.fromstring(camera.get('pos'),sep=' ');fovy=float(camera.get('fovy'));world.remove(camera)
        _camera(world,name,target+(pos-target)*ratio,target,fovy)
    camera=world.find("camera[@name='top']");pos=np.fromstring(camera.get('pos'),sep=' ');pos[2]=.4+(pos[2]-.4)*ratio;camera.set('pos',vector(pos))
    return offsets


def build(directory=DIRECTORY,pair_spacing_m=.6):
    if not math.isfinite(pair_spacing_m) or pair_spacing_m<=0:raise ValueError('Positive finite horizontal spacing required')
    directory=Path(directory).resolve()
    if directory.exists():raise ValueError('Choose a new scene folder; earlier assets are preserved')
    directory.mkdir();shutil.copytree(SOURCE/'assets',directory/'assets')
    root=ET.parse(SOURCE/'scene.xml').getroot();world=root.find('worldbody');asset=root.find('asset')
    fixed=ET.parse(ROOT/'configs/environments/spring_dial/crank-04/scene.xml').getroot()
    arrow=fixed.find("asset/mesh[@name='dial_pointer_arrow']")
    if arrow is not None:asset.append(copy.deepcopy(arrow))
    parameters=json.loads((SOURCE/'parameters.json').read_text());env=json.loads((SOURCE/'environment.json').read_text())
    parameters['id']=env['id']='spring-dial-'+directory.name
    crank=copy.deepcopy(CRANK);crank.pop('id',None);crank.update(grip_center_radius_m=CRANK['radius_m']-.006,grip_center_above_hub_m=.0435,grip_mount='rigid to crank arm')
    parameters['crank']=crank
    module=parameters['module'];front=module['face_thickness_m']/2;origin=front+crank['hub_height_m']/2
    for a in 'ABCD':
        rotor=root.find(f"worldbody/body[@name='dial_{a}_module']/body[@name='dial_{a}_rotor']")
        rotor.set('pos',vector([0,0,origin]))
        for child in list(rotor):
            if child.tag=='geom':rotor.remove(child)
        template=fixed.find(f"worldbody/body[@name='dial_{a}']")
        for name in ('crank_hub','crank_arm','hand_grip'):
            g=copy.deepcopy(template.find(f"geom[@name='dial_{a}_{name}']"));g.set('material','module_red');rotor.append(g)
        length=.105;z=crank['hub_height_m']/2+.003+crank['arm_thickness_m']/2+.001
        ET.SubElement(rotor,'geom',name=f'dial_{a}_orange_pointer',type='box',pos=vector([0,(length-.024)/2,z]),size=vector([.004,(length-.024)/2,.0005]),rgba='1 .31 .035 1',mass='0',contype='0',conaffinity='0')
        if arrow is not None:ET.SubElement(rotor,'geom',name=f'dial_{a}_arrowhead',type='mesh',mesh='dial_pointer_arrow',pos=vector([0,length-.012,z]),rgba='1 .31 .035 1',mass='0',contype='0',conaffinity='0')
    previous_spacing=parameters['pair_spacing_m']
    offsets=resize_horizontal_layout(root,parameters,env,pair_spacing_m) if pair_spacing_m!=previous_spacing else [0.]*4
    # A genuine opposite global view, useful for inspecting the blue supports.
    frontcam=world.find("camera[@name='global']");pos=np.fromstring(frontcam.get('pos'),sep=' ');pos[1]*=-1
    _camera(world,'global_rear',pos,np.array([parameters['table'].get('center_x_m',0),0,.55]),float(frontcam.get('fovy')))
    root.set('model','RoboFrontier | Airborne Crank Codebreaker | eight arms')
    ET.indent(root);ET.ElementTree(root).write(directory/'scene.xml',encoding='unicode')
    m=mujoco.MjModel.from_xml_path(str(directory/'scene.xml'))
    if any(offsets):
        key=root.find("keyframe/key[@name='initial']");q=np.fromstring(key.get('qpos'),sep=' ')
        for a,dx in zip('ABCD',offsets):q[m.jnt_qposadr[m.joint('dial_'+a+'_module_free').id]]+=dx
        key.set('qpos',' '.join(repr(float(x)) for x in q));ET.ElementTree(root).write(directory/'scene.xml',encoding='unicode');m=mujoco.MjModel.from_xml_path(str(directory/'scene.xml'))
    d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    module['assembled_mass_kg']=float(m.body_subtreemass[m.body('dial_A_module').id])
    for k in ('knob_radius_m','knob_height_m','knob_mass_kg'):module.pop(k,None)
    update_display(m,dict(ready=False,correct_count=None,won=False))
    with mujoco.Renderer(m,height=800,width=1280) as r:
        for camera in ('global','global_rear','dial_A','dial_A_side','board'):
            r.update_scene(d,camera=camera);Image.fromarray(r.render()).save(directory/('preview-'+camera+'.png'))
    for name,v in [('parameters',parameters),('environment',env)]:write_json(directory/(name+'.json'),v)
    from robofrontier.floating_dial_task import make_task
    write_json(directory/'task.json',make_task(directory))
    write_json(directory/'validation.json',dict(passed=None,status='built',scope='Scene loading/appearance only; contact dynamics and release are not yet validated.'))
    write_json(directory/'provenance.json',dict(source_floating_scene=str(SOURCE),source_fixed_crank='configs/environments/spring_dial/crank-04',source_scene_sha256=hashlib.sha256((SOURCE/'scene.xml').read_bytes()).hexdigest(),blue_support_geometry_preserved=True,robot_layout_preserved=not any(offsets),previous_spacing_m=previous_spacing,horizontal_spacing_m=pair_spacing_m,same_row_spacing_increment_m=pair_spacing_m-previous_spacing,crank_geometry_and_physical_mass_preserved=True,policy_oracle_recipe_exported=False))
    (directory/'README.md').write_text(f"# Airborne Crank Codebreaker — {pair_spacing_m:.2f} m horizontal spacing\n\nSeparate four-module/eight-arm scene. Each row's adjacent robot bases are {pair_spacing_m:.2f} m apart. Table length is {parameters['table']['length_m']:.2f} m, with symmetric side margins; width, front/back pair separation, score panel, crank/blue-handle geometry, mass, friction, spring and NoSlip=20 are preserved. Robot and module detail cameras translate with their initial workspaces; global cameras widen to include the new layout.\n\nEarlier environments and runs remain intact. This asset has load/layout validation only; full dynamics and model performance require a new trial. Task rules and public/private observation boundaries use the existing floating apparatus adapter.\n")
    print(json.dumps({'directory':str(directory),'arms':len(env['stations']),'modules':4,'horizontal_spacing_m':pair_spacing_m,'table_length_m':parameters['table']['length_m'],'grip_diameter_m':crank['grip_diameter_m'],'grip_center_radius_m':crank['grip_center_radius_m'],'module_mass_kg':module['assembled_mass_kg']}));return directory

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,default=DIRECTORY);p.add_argument('--pair-spacing-m',type=float,default=.6);a=p.parse_args();build(a.directory,a.pair_spacing_m)
