"""Build a preserved-geometry fixed-dial scene with red graspable crank handles."""
import argparse,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image
from robofrontier.scene import ROOT,vector
from robofrontier.spring_dial import PARAMETERS,specification,create_scene,DialFeedback,update_display
from robofrontier.cube_transport import initialize,add_initial_keyframe

DEFAULT=ROOT/'configs/environments/spring_dial/crank-04'
CRANK={'id':'spring-dial-crank-04','radius_m':.075,'arm_width_m':.016,'arm_thickness_m':.009,
       'grip_diameter_m':.022,'grip_height_m':.044,'hub_diameter_m':.024,'hub_height_m':.036,
       'assembly_mass_kg':.12,'contact_friction':[.8,.005,.0001]}

def build(directory=DEFAULT,design='panel'):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True);(directory/'assets').mkdir(exist_ok=True)
    p=json.loads(json.dumps(PARAMETERS));p['id']=CRANK['id'];p['appearance']['design']=design
    env=specification(p);env['id']=p['id'];root=create_scene(directory,p,env);world=root.find('worldbody')
    d=p['dial'];handle_mass=.03;arm_mass=.03;hub_mass=CRANK['assembly_mass_kg']-handle_mass-arm_mass
    for a in 'ABCD':
        body=root.find(f"worldbody/body[@name='dial_{a}']")
        for child in list(body):
            if child.get('name','').startswith(f'dial_{a}_tooth_'):body.remove(child)
        hub=body.find(f"geom[@name='dial_{a}_knob']")
        hub.set('size',vector([CRANK['hub_diameter_m']/2,CRANK['hub_height_m']/2]));hub.set('mass',str(hub_mass))
        hub.set('name',f'dial_{a}_crank_hub')
        # Short radial red arm joined to the central hub, with a round vertical palm grip.
        arm=ET.SubElement(body,'geom',name=f'dial_{a}_crank_arm',type='box',
            pos=vector([0,CRANK['radius_m']/2,CRANK['hub_height_m']/2+.003]),
            size=vector([CRANK['arm_width_m']/2,CRANK['radius_m']/2,CRANK['arm_thickness_m']/2]),
            mass=str(arm_mass),material='knob_metal',friction=vector(CRANK['contact_friction']),condim='4')
        grip=ET.SubElement(body,'geom',name=f'dial_{a}_hand_grip',type='cylinder',
            pos=vector([0,CRANK['radius_m']-.006,CRANK['hub_height_m']/2+.003+CRANK['arm_thickness_m']/2+CRANK['grip_height_m']/2-.004]),
            size=vector([CRANK['grip_diameter_m']/2,CRANK['grip_height_m']/2]),mass=str(handle_mass),
            material='knob_metal',friction=vector(CRANK['contact_friction']),condim='4')
        # Lower the non-contact pointer slightly so it remains visually distinct from the crank.
        pointer=body.find(f"geom[@name='dial_{a}_pointer']");tip=body.find(f"geom[@name='dial_{a}_arrowhead']")
        for geom in (pointer,tip):
            if geom is not None:
                pos=np.fromstring(geom.get('pos'),sep=' ');pos[2]=.004;geom.set('pos',vector(pos))
    scene=directory/'scene.xml';ET.indent(root);ET.ElementTree(root).write(scene,encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(scene));data=initialize(model,env)
    for _ in range(round(2/model.opt.timestep)):mujoco.mj_step(model,data)
    mujoco.mj_forward(model,data)
    bad=[{'a':model.geom(c.geom1).name,'b':model.geom(c.geom2).name,'depth_m':float(-c.dist)} for c in data.contact if c.dist<-.001]
    data.time=0;add_initial_keyframe(root,model,data);ET.indent(root);ET.ElementTree(root).write(scene,encoding='unicode')
    feedback=DialFeedback(p['feedback']);update_display(model,feedback.update(0,[0]*4,[0]*4))
    for camera in ('board','global','top','dial_A'):
        with mujoco.Renderer(model,height=900,width=1280) as renderer:
            renderer.update_scene(data,camera=camera);Image.fromarray(renderer.render()).save(directory/f'preview-{camera}.png')
    p['crank']=CRANK
    (directory/'parameters.json').write_text(json.dumps(p,indent=2)+'\n');(directory/'environment.json').write_text(json.dumps(env,indent=2)+'\n')
    report={'passed':not bad and bool(np.isfinite(data.qpos).all()),'arm_count':4,'fixed_white_dials':True,'crank_handles':4,'dial_actuators':0,'actuators':model.nu,'assembled_crank_mass_kg':CRANK['assembly_mass_kg'],'initial_penetrations_over_1mm':bad,'scope':'Initial scene load/settling only; crank grasp and rotation not yet validated.'}
    (directory/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    (directory/'README.md').write_text('# Fixed spring dials with red crank handles\n\nA separate copy of the fixed four-dial table scene. The old gear-knob environment is preserved. Each white dial stays fixed on the table; its passive red rotor has a radial arm and a vertical 22 mm grasp cylinder. The center hub, arm and grip preserve the original 0.12 kg rotating-assembly mass. Hinges, springs, angle labels, cameras, table and robot placements are unchanged.\n\nNo dial actuator or angle-setting API was added. The crank has not passed a physical grasp/rotation test yet.\n')
    print(json.dumps(report));
    if not report['passed']:raise SystemExit('Crank scene failed initial validation')
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--directory',type=Path,default=DEFAULT);ap.add_argument('--design',choices=('warm','silver','panel'),default='panel');args=ap.parse_args();build(args.directory,args.design)
