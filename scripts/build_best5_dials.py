"""Nine versioned six-mark BEST scenes; clone R10 physics without stepping."""
from copy import deepcopy
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image,ImageDraw
from robofrontier.suite import v2
from robofrontier.suite.scenes import box,cylinder,texture,vector
from robofrontier.suite.evaluate import DIGITS
from robofrontier.suite.dial_feedback import DIGIT_DIM_RGB
from robofrontier.workstation import Workstation
from robofrontier.run_setup import write_json

ROOT=Path(__file__).resolve().parents[1]
CATALOG=ROOT/'configs/suites/dial-best5-r11.json'
SPEC=dict(version='best5-v1',mode='best_so_far',symbol_count=5,total_marks=6,step_deg=60.,
    angle_tolerance_deg=4.,dwell_sim_s=.25,zero_valid=False,require_red_grasp=True,
    initial_best=0,display_digits=1,display_brightness='Bright only when READY is green; otherwise dim but readable',record_scope='episode',settlement='automatic_stable_configuration',
    readiness='Grey invalid, amber qualifying, green ready; readiness is not password correctness')
RULES=[
    'The face has0,1,2,3,4,5 at60-degree intervals. Each private password digit is1..5;0 is an invalid initial position. Clockwise labels4 and5 may be reached as-120 and-60degrees; +180 and-180 are the same mark3.',
    'BEST starts at0 and shows the largest correct-count achieved by any one complete valid input in this episode. It is not the current correct-count. An unchanged record can mean equal or worse; never combine per-dial results from different attempts.',
    'READY is grey when the current input is invalid, amber while qualifying and green after every dial is within4degrees of a nonzero mark, actually grasped, and stable for0.25seconds. Green means a valid input, not a correct password.',
    'The BEST digit is bright only when READY is green. Otherwise its retained value stays visible in dim grey-brown, including while the amber lamp is qualifying. Brightness does not change the record or disclose current correctness.',
    'Only a complete valid input updates BEST. Off-mark, zero or lost grasp makes READY invalid while BEST remains. New stable configurations are evaluated automatically, including intermediate configurations during movement; bind a record change to an actually observed configuration, not an assumed target.',
    'L2/L3 require a separate robot to hold each blue handle with the whole module at least20mm clear of external supports. Maintain that support during red operation. A crank grip moves around the axle and may require coordinated translation.',
    'Unlock occurs only when all digits of one simultaneous valid input are correct. Success latches, while physical springs and support remain active. Inputs and actuation choices are Agent decisions.',
    'Only robot actuators and real contacts may affect the live world. Use RGB and permitted robot feedback; hidden passwords, per-dial correctness and live object truth are not policy inputs.'
]


def face(path):
    image=Image.new('RGB',(1024,1024),(249,248,242));draw=ImageDraw.Draw(image)
    draw.ellipse((4,4,1020,1020),outline=(145,144,137),width=6)
    radius=v2.DIAL_FACE_RADIUS
    for k in range(120):
        a=2*math.pi*k/120;u=np.array([math.sin(a),math.cos(a)])
        inner=.087 if k%20==0 else .092 if k%5==0 else .095
        p=512+inner/radius*512*u;q=512+.098/radius*512*u
        draw.line([tuple(p),tuple(q)],fill=(0,0,0),width=10 if k%20==0 else 3 if k%5==0 else 2)
    h=v2.DIAL_DIGIT_HEIGHT/radius*512;probe=v2.font(150);bb=probe.getbbox('0');font=v2.font(round(150*h/(bb[3]-bb[1])))
    for i in range(6):
        a=2*math.pi*i/6;c=512+v2.DIAL_DIGIT_RADIUS/radius*512*np.array([math.sin(a),math.cos(a)])
        glyph=Image.new('RGBA',(200,200),(0,0,0,0));g=ImageDraw.Draw(glyph);bb=font.getbbox(str(i))
        g.text(((200-bb[2]+bb[0])/2-bb[0],(200-bb[3]+bb[1])/2-bb[1]),str(i),font=font,fill=(0,0,0,255))
        glyph=glyph.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        image.paste(glyph,(round(c[0]-100),round(c[1]-100)),glyph)
    image.save(path)


def panel_layout(root,n,level):
    world=root.find('worldbody');base=world.find("geom[@name='score_base']")
    old=np.fromstring(base.get('pos'),sep=' ');scale=1.5 if level>1 and n>=6 else 1.
    center=old.copy();center[2]=.8+(old[2]-.8)*scale
    if n==8 and level==1:center[:2]=0.
    if n==6 and level>1:center[:2]=[0.,2.50]
    if n==8 and level>1:center[:2]=2.03*np.array([math.cos(-3*math.pi/8),math.sin(-3*math.pi/8)])
    for geom in world.findall('geom'):
        if not (geom.get('name') or '').startswith('score_'):continue
        geom.set('pos',vector(center+(np.fromstring(geom.get('pos'),sep=' ')-old)*scale))
        geom.set('size',vector(np.fromstring(geom.get('size'),sep=' ')*scale))
    camera=world.find("camera[@name='board']")
    camera.set('pos',vector(center+(np.fromstring(camera.get('pos'),sep=' ')-old)*scale))
    return center,scale


def verify_panel_support(model,data):
    """Ray-test the panel footprint against actual tabletop collision geometry."""
    groups=model.geom_group.copy()
    try:
        model.geom_group[:]=0
        for i in range(model.ngeom):
            name=model.geom(i).name or ''
            if name.startswith('table_') and 'leg' not in name:model.geom_group[i]=5
        base=model.geom('score_base');center=data.geom_xpos[base.id];half=base.size[:2]
        hit=np.zeros(1,dtype=np.int32);mask=np.array([0,0,0,0,0,1],dtype=np.uint8)
        for dx in np.linspace(-half[0],half[0],11):
            for dy in np.linspace(-half[1],half[1],11):
                origin=np.array([center[0]+dx,center[1]+dy,1.1])
                distance=mujoco.mj_ray(model,data,origin,np.array([0.,0.,-1.]),mask,1,-1,hit)
                if distance<0 or abs(origin[2]-distance-.8)>.001:raise ValueError('Score panel overhangs actual tabletop')
    finally:model.geom_group[:]=groups


def build(n,level,refresh_draft=False):
    source=ROOT/f'configs/environments/spring_dial/frontier-v2-r10-dial-n{n:02}-d{level}'
    old=Workstation(source);old_task=json.loads((source/'task.json').read_text());case=deepcopy(old_task['scale'])
    case.update(id=f'dial-best5-n{n:02}-d{level}',scene_revision=11,source_scene_revision=10)
    folder=v2.directory(case);case['directory']=str(folder.relative_to(ROOT))
    if folder.exists():
        validation=json.loads((folder/'validation.json').read_text()) if (folder/'validation.json').exists() else {}
        if not refresh_draft or validation.get('oracle_verified') or validation.get('frozen_input'):
            raise FileExistsError('Only unfrozen, unverified drafts may be refreshed: '+str(folder))
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    shutil.copytree(source/'assets',folder/'assets',copy_function=os.link)
    root=ET.parse(source/'scene.xml').getroot();world=root.find('worldbody')
    # New filenames ensure PIL never overwrites a linked old texture.
    face(folder/'assets/dial_face_best5.png')
    root.find("asset/texture[@name='dial_face_tex']").set('file','assets/dial_face_best5.png')
    title=Image.new('RGB',(1024,200),(244,241,233));ImageDraw.Draw(title).text((512,100),f'BEST / {n}',anchor='mm',font=v2.font(148),fill=(17,26,37));title.save(folder/'assets/score_title_best.png')
    root.find("asset/texture[@name='score_title']").set('file','assets/score_title_best.png')
    display_center=np.fromstring(world.find("geom[@name='score_base']").get('pos'),sep=' ')
    for index,segment in enumerate('abcdefg'):
        unused=world.find(f"geom[@name='score_1_{index}']");world.remove(unused)
        geom=world.find(f"geom[@name='score_0_{index}']")
        pos=np.fromstring(geom.get('pos'),sep=' ');size=np.fromstring(geom.get('size'),sep=' ')
        pos[0]=display_center[0]+(pos[0]-display_center[0]+.062)*1.25
        pos[1]=display_center[1]-.015+(pos[1]-display_center[1]+.015)*1.25
        size[:2]*=1.25
        geom.set('pos',vector(pos));geom.set('size',vector(size))
        geom.set('rgba',vector([*DIGIT_DIM_RGB,1 if segment in DIGITS[0] else 0]))
    base=world.find("geom[@name='score_base']");p=np.fromstring(base.get('pos'),sep=' ');z=float(p[2]+np.fromstring(base.get('size'),sep=' ')[2]+.0005)
    material=texture(root,folder,'score_ready_label','READY',(640,160),135)
    box(world,'score_ready_label_geom',[p[0]+.035,p[1]-.127,z],[.085,.020,.0002],visual=True,material=material)
    cylinder(world,'score_ready_light',[p[0]-.090,p[1]-.127,z],[.022,.0004],(.47,.51,.53,1),visual=True)
    panel_center,panel_scale=panel_layout(root,n,level)
    path=folder/'scene.xml';ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    env=json.loads((source/'environment.json').read_text());env['suite_case']=case
    env['mechanism'].update(feedback=deepcopy(SPEC),detent_labels=[str(i) for i in range(6)],score_label='BEST',
        feedback_rule_version=3,symbol_to_angle_deg={str(i):60*i for i in range(6)},
        score_panel_xy_m=panel_center[:2].tolist(),score_panel_scale=panel_scale)
    write_json(folder/'environment.json',env)
    task=deepcopy(old_task);task.update(id=case['id'],scale=case,scene_asset=str(folder.relative_to(ROOT)),
        scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),mechanism=deepcopy(env['mechanism']),
        rules=RULES,oracle_verified=False,execution_readiness='environment_preview_only',
        objective='Unlock the spring dial code by finding and simultaneously holding one correct nonzero input. Minimize execution time and valid attempts using only historical BEST feedback; preserve real grasps and required airborne support.')
    task['completion']=dict(mode='historical_best_count_latch',feedback_rule_version=3,angle_tolerance_deg=4,stable_duration_sim_s=.25)
    task['observation_contract']['policy']=['Selected timestamped RGB including BEST and READY','Robot proprioception and geometry','Action feedback']
    write_json(folder/'task.json',task);public=v2.public_task(task);public.pop('initial_supplies',None);write_json(folder/'public-task.json',public)
    write_json(folder/'parameters.json',dict(physics=env['physics'],mechanism=env['mechanism'],source_directory=str(source.relative_to(ROOT))))
    model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    before=v2.physical_summary(old.model);after=v2.physical_summary(model)
    # Authorized panel relocation/resizing changes only its static collider.
    physical=np.flatnonzero((model.geom_contype!=0)|(model.geom_conaffinity!=0)).tolist();row=physical.index(model.geom('score_base').id)
    for key in ('geom_pos','geom_size'):before[key][row]=after[key][row]
    assert before==after, 'Panel revision changed robot or dial physics'
    verify_panel_support(model,data)
    state=v2.manifest(model,data,case,folder,env)
    state['motion_recording_contract']['visual_events']=['BEST digits and ready-dependent brightness','READY grey/amber/green'];write_json(folder/'workstation.json',state)
    write_json(folder/'validation.json',dict(status='generated',oracle_verified=False,policy_verified=False,frozen_input=False,
        robot_and_dial_physics_equal_to_r10=True,panel_footprint_supported=True,score_panel_scale=panel_scale,
        physics_sha256=state['physics_sha256'],scope='Static compilation; only score panel collider relocated/resized; no grasp/rotation run'))
    with mujoco.Renderer(model,width=1280,height=800) as renderer:
        for view in ('global','global_rear','top','board','D01_face'):
            renderer.update_scene(data,camera=view);Image.fromarray(renderer.render()).save(folder/('preview-'+view+'.png'))
    case['status']='environment_preview_only';print(case['id'],flush=True);return case


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--refresh-draft',action='store_true');args=parser.parse_args()
    cases=[build(n,level,args.refresh_draft) for n in (4,6,8) for level in (1,2,3)]
    write_json(CATALOG,dict(schema='robofrontier.suite.v2',feedback=SPEC,cases=cases,oracle_verified=False))
    # Paired benchmark inputs live outside public scene assets.
    private=ROOT/'configs/evaluations/private/dial-best5-r11';private.mkdir(parents=True,exist_ok=True)
    entries=[]
    for c in cases:
        p=private/(c['id']+'.json')
        if not p.exists():write_json(p,dict(code_symbols=[secrets.randbelow(5)+1 for _ in range(c['dials'])]))
        entries.append(dict(case_id=c['id'],private_puzzle=str(p.relative_to(ROOT))))
    write_json(private/'manifest.json',dict(scope='Evaluator-only paired inputs; not policy or public-task inputs',cases=entries))
    print(CATALOG,flush=True)
