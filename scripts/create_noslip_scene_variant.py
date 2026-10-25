"""Clone a frozen-style scene, changing only NoSlip and variant identity; no policy run."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from robofrontier.floating_dial_task import make_task
from robofrontier.run_setup import write_json


def clone(source,target,iterations):
    source=Path(source).resolve();target=Path(target).resolve()
    if type(iterations) is not int or iterations<0:raise ValueError('Nonnegative integer NoSlip iterations required')
    if target.exists():raise ValueError('Variant destination already exists; earlier assets are preserved')
    if target.parent!=source.parent:raise ValueError('Keep sibling scene paths so relative mesh calibration is unchanged')
    target.mkdir();shutil.copytree(source/'assets',target/'assets')
    root=ET.parse(source/'scene.xml').getroot()
    def signature(e):
        attributes=dict(e.attrib)
        if e.tag=='option':attributes.pop('noslip_iterations',None)
        return [e.tag,sorted(attributes.items()),[signature(c) for c in e]]
    before=hashlib.sha256(json.dumps(signature(root)).encode()).hexdigest()
    root.find('option').set('noslip_iterations',str(iterations));ET.indent(root);ET.ElementTree(root).write(target/'scene.xml',encoding='unicode')
    after=hashlib.sha256(json.dumps(signature(ET.parse(target/'scene.xml').getroot())).encode()).hexdigest()
    if before!=after:raise RuntimeError('Unexpected non-NoSlip XML change')
    for name in ('parameters','environment'):
        p=json.loads((source/(name+'.json')).read_text());p['id']='spring-dial-'+target.name;p['physics']['noslip_iterations']=iterations
        write_json(target/(name+'.json'),p)
    for preview in source.glob('preview-*.png'):shutil.copy2(preview,target/preview.name)
    write_json(target/'task.json',make_task(target))
    write_json(target/'variant-provenance.json',dict(source_directory=str(source),only_xml_change='option.noslip_iterations',noslip_iterations=iterations,non_noslip_xml_sha256=after,initial_keyframe_unchanged=True,gripper_friction_mass_geometry_unchanged=True))
    write_json(target/'validation.json',dict(passed=None,status='prepared',scope='Physics variant construction only; full dynamics validation is separate.'))
    print(json.dumps({'directory':str(target),'only_physics_change':'noslip_iterations','iterations':iterations,'initial_keyframe_unchanged':True}))
    return target

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('target',type=Path);p.add_argument('--iterations',type=int,required=True);a=p.parse_args();clone(a.source,a.target,a.iterations)
