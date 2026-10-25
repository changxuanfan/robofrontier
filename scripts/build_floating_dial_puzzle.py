"""Generate a separate eight-arm/four-free-module puzzle; preserve earlier assets."""
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.cube_transport import ROOT, build_xml, initialize, add_initial_keyframe, _camera
from robofrontier.scene import vector
from scripts.build_floating_spring_dial import PARAMETERS, _environment, _add_module, _bold_font

DIRECTORY = ROOT/'configs/environments/spring_dial/floating-04'
COMPACT_DIRECTORY = ROOT/'configs/environments/spring_dial/floating-04-compact'


def build(directory=DIRECTORY, pair_spacing_m=1.6, strict_contact_sequence=False, tight_table=False):
    if not math.isfinite(pair_spacing_m) or pair_spacing_m<=0:raise ValueError('Pair spacing must be positive; actual geometry and dynamics determine feasibility')
    centers=[(i-1.5)*pair_spacing_m for i in range(4)]
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=True)
    (directory/'assets').mkdir(exist_ok=True)
    p=copy.deepcopy(PARAMETERS)
    p['id']='spring-dial-'+directory.name;p['table']['length_m']=3*pair_spacing_m+2.1
    p['pair_spacing_m']=pair_spacing_m
    if strict_contact_sequence:p['contact_sequence']={'kind':'lift_before_knob_contact','airborne_duration_s':.25,'contact_tolerance_m':.0002}
    p['feedback']={'angle_tolerance_deg':4.,'max_speed_deg_s':5.,'stable_duration_s':.25,'max_sample_gap_s':.03}
    p['airborne']={'minimum_clearance_m':.02}
    originals=copy.deepcopy(p['arms']);p['arms']=[]
    for i,x in enumerate(centers):
        for j,a in enumerate(originals):
            a=copy.deepcopy(a);a['id']=chr(65+i*2+j)
            for field in ('base_position_m','home_tcp_m'):a[field][0]+=x
            p['arms'].append(a)
    if tight_table:
        xs=[a['base_position_m'][0] for a in p['arms']]
        p['table']['length_m']=max(xs)-min(xs)+.55
        p['table']['center_x_m']=(min(xs)+max(xs))/2
        front=p['arms'][::2];score_x=(front[1]['base_position_m'][0]+front[2]['base_position_m'][0])/2
        p['score_panel']={'center_xy_m':[score_x,-p['table']['width_m']/2+.20],'front_edge_inset_m':.02}
    env=_environment(p);env['tables'][0]['center_m'][0]=p['table'].get('center_x_m',0);env.update(id=p['id'],arm_count=8,topology='four_facing_pairs')
    env.pop('module');env['dials']=[{'id':chr(65+i),'nominal_robots':[chr(65+2*i),chr(66+2*i)],'center_m':[x,0,.515]} for i,x in enumerate(centers)]
    for i,s in enumerate(env['stations']):s['pair']=i//2+1
    root=build_xml(env,directory);root.set('model','RoboFrontier | Airborne Dial Codebreaker | eight arms')
    _add_module(root,directory,p)
    # Same printed geometry, now with the puzzle's symbol alphabet.
    image=Image.new('RGB',(1024,1024),(249,248,242));draw=ImageDraw.Draw(image)
    draw.ellipse((18,18,1006,1006),outline=(170,160,150),width=18)
    for degrees in range(0,360,5):
        t=math.radians(degrees);inner=354 if degrees%90 else 325
        draw.line((512+inner*math.sin(t),512-inner*math.cos(t),512+405*math.sin(t),512-405*math.cos(t)),fill=(0,0,0),width=12 if degrees%90==0 else 3)
    for value in range(4):
        t=value*math.pi/2;draw.text((512+275*math.sin(t),512-275*math.cos(t)),str(value),anchor='mm',font=_bold_font(105),fill=(0,0,0))
    image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(directory/'assets/dial_face.png')
    world=root.find('worldbody');template=world.find("body[@name='dial_module']");world.remove(template)
    for c in list(world.findall('camera')):
        if c.get('name').startswith('module_') or c.get('name') in ('global','global_side','top'):world.remove(c)
    # Positive hinge travel is clockwise when looking at the printed front.
    for i,x in enumerate(centers):
        a=chr(65+i);body=copy.deepcopy(template)
        for e in body.iter():
            if e.get('name'):e.set('name','dial_'+a+'_'+e.get('name').removeprefix('dial_'))
        body.set('pos',vector([x,0,.515]));body.find('.//joint').set('axis','0 0 -1')
        world.append(body)
        _camera(world,'dial_'+a,[x+.60,-1.30,1.0],np.array([x,0,.67]),36)
        _camera(world,'dial_'+a+'_side',[x+1.15,-.25,1.00],np.array([x,0,.68]),40)
        _camera(world,'dial_'+a+'_front',[x,-1.65,.88],np.array([x,0,.64]),38)
        _camera(world,'dial_'+a+'_high',[x+.4,-.75,1.5],np.array([x,0,.70]),38)
        # A public identity marker on the module, not an inferred pose overlay.
        image=Image.new('RGB',(256,256),(248,247,241));ImageDraw.Draw(image).text((128,128),a,anchor='mm',font=_bold_font(180),fill=(0,0,0))
        image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(directory/'assets'/f'module_{a}.png')
        asset=root.find('asset');ET.SubElement(asset,'texture',name=f'module_{a}_label',type='2d',file=f'assets/module_{a}.png')
        ET.SubElement(asset,'material',name=f'module_{a}_label',texture=f'module_{a}_label')
        ET.SubElement(body,'geom',name=f'dial_{a}_label',type='box',pos='0 -.075 .0047',size='.017 .017 .0002',material=f'module_{a}_label',contype='0',conaffinity='0',mass='0')
    # Share the established score-panel design, not its physics or code.
    fixed=ROOT/'configs/environments/spring_dial/facing-04'
    old=ET.parse(fixed/'scene.xml').getroot()
    for e in old.findall('asset/*'):
        if e.get('name')=='score_title':
            e=copy.deepcopy(e)
            if e.get('file'):
                shutil.copy2(fixed/e.get('file'),directory/e.get('file'))
            root.find('asset').append(e)
    for e in old.findall('worldbody/geom'):
        if e.get('name','').startswith('score_'):
            e=copy.deepcopy(e);pos=np.fromstring(e.get('pos'),sep=' ');panel=p.get('score_panel',{'center_xy_m':[0,-.95]})['center_xy_m'];pos[:2]+=panel;e.set('pos',vector(pos));world.append(e)
    scale=(3*pair_spacing_m+2.1)/6.9  # Frame the robot envelope, independently of trimmed table edges.
    center_x=p['table'].get('center_x_m',0)
    _camera(world,'global',[center_x,-5.8*scale,.55+4.65*scale],np.array([center_x,0,.55]),35)
    _camera(world,'global_side',[center_x-5*scale,-4.5*scale,.55+3.25*scale],np.array([center_x,0,.55]),55)
    ET.SubElement(world,'camera',name='top',pos=vector([center_x,0,.4+6.6*scale]),xyaxes='1 0 0 0 1 0',fovy='48')
    panel=p.get('score_panel',{'center_xy_m':[0,-.95]})['center_xy_m']
    ET.SubElement(world,'camera',name='board',pos=vector([*panel,1.15]),xyaxes='1 0 0 0 1 0',fovy='42')
    root.find('statistic').set('extent','8')
    path=directory/'scene.xml'
    def save():
        ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    save();model=mujoco.MjModel.from_xml_path(str(path));data=initialize(model,env)
    source=ROOT/'configs/environments/spring_dial/floating-module-01/scene.xml'
    sm=mujoco.MjModel.from_xml_path(str(source));sd=mujoco.MjData(sm);mujoco.mj_resetDataKeyframe(sm,sd,sm.key('initial').id)
    free=sd.joint('dial_module_free').qpos.copy()
    for i,x in enumerate(centers):
        q=free.copy();q[0]+=x;data.joint('dial_'+chr(65+i)+'_module_free').qpos[:]=q
    mujoco.mj_forward(model,data)
    for _ in range(1000):mujoco.mj_step(model,data)
    data.time=0;add_initial_keyframe(root,model,data);save()
    p['module']['assembled_mass_kg']=float(model.body_subtreemass[model.body('dial_A_module').id])
    from robofrontier.spring_dial import update_display
    update_display(model,dict(ready=False,correct_count=None,won=False))
    for camera in ('global','board','dial_A','dial_A_side'):
        with mujoco.Renderer(model,height=800,width=1280) as r:
            r.update_scene(data,camera=camera);Image.fromarray(r.render()).save(directory/f'preview-{camera}.png')
    # freeze_scene's established preview filename remains compatible.
    shutil.copy2(directory/'preview-global.png',directory/'preview-board.png')
    p['apparatus']='floating';p['module']['hinge_axis_local']=[0,0,-1]
    for name,value in [('parameters',p),('environment',env)]:
        (directory/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
    report={'passed':bool(np.isfinite(data.qpos).all()),'robot_count':8,'module_count':4,'free_modules':True,'dial_actuators':0,'source_single_scene_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    (directory/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return directory

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--directory',type=Path,default=DIRECTORY);parser.add_argument('--pair-spacing-m',type=float,default=1.6);parser.add_argument('--strict-contact-sequence',action='store_true');parser.add_argument('--tight-table',action='store_true');args=parser.parse_args()
    build(args.directory,args.pair_spacing_m,args.strict_contact_sequence,args.tight_table)
