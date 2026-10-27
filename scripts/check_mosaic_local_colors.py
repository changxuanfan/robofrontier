"""Check all nine colour-only assets, local certificates and frozen R13 outputs."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from robofrontier.suite.mosaic_local_colors import (geometry_signature,validate_plan,nearest,COVERAGE)
from robofrontier.suite.mosaic_policy_package import (ROOT,read,write,checksum,verify_registry,
    project_public,_public_boundary,_check_target)


def check(engineering,package):
    engineering=Path(engineering);package=Path(package)
    baseline,_=verify_registry()
    registry=read(ROOT/'configs/task_families/mosaic/r14.json')
    selection=read(ROOT/'configs/task_families/mosaic/selection.json')
    assert selection['future_gpt_registry']=='configs/task_families/mosaic/r14.json'
    assert selection['oracle_and_factory_replay_registry']=='configs/task_families/mosaic/r13.json'
    assert registry['readiness']['policy_launch_ready'] is False and registry['readiness']['oracle_verified'] is False
    assert checksum(ROOT/registry['catalog'])==registry['catalog_sha256']
    assert checksum(ROOT/registry['baseline_oracle_registry'])==registry['baseline_oracle_registry_sha256']
    assert read(package/'engineering/registry.json')==registry
    for file,sha in registry['design_evidence'].items():assert checksum(file)==sha
    catalog=read(ROOT/registry['catalog'])['cases'];assert len(catalog)==9 and len({c['id'] for c in catalog})==9
    plans={n:read(engineering/f'a{n:02d}-plan.json') for n in (4,6,8)}
    rows=[]
    for case,entry in zip(catalog,registry['cases']):
        assert entry['case_id']==case['id']
        folder=ROOT/case['directory'];source=ROOT/next(e['environment'] for e in baseline['cases'] if e['case_id']==case['id'])
        for name,sha in entry['source_files'].items():assert checksum(folder/name)==sha
        task=read(folder/'task.json');old=read(source/'task.json');plan=plans[case['arms']]
        validate_plan(old,plan)
        assert task['execution_readiness']=='environment_preview_only' and task['oracle_verified'] is False
        assert old['goals']==task['goals'] and old['grid']==task['grid'] and old['arms']==task['arms']
        assert old['rules']==task['rules'] and old['physics']==task['physics']
        a,b=ET.parse(source/'scene.xml').getroot(),ET.parse(folder/'scene.xml').getroot()
        for obj in old['objects']:
            for root in (a,b):root.find(f"worldbody/body[@name='{obj['id']}']/geom[@name='{obj['id']}_geom']").attrib.pop('rgba')
        assert ET.tostring(a)==ET.tostring(b),'Non-colour XML change'
        assert geometry_signature(source)==geometry_signature(folder)
        assert read(source/'compiled-physics.json')==read(folder/'compiled-physics.json')
        actual={o['id']:o for o in task['objects']}
        for pairing in plan['assignments']:
            before=next(o for o in old['objects'] if o['id']==pairing['object_id']);after=actual[before['id']]
            assert {k:v for k,v in before.items() if k not in ('color','rgb')}=={k:v for k,v in after.items() if k not in ('color','rgb')}
            assert after['color']==pairing['colour']
            expected=[int(task['grid']['palette'][after['color']][i:i+2],16)/255 for i in (1,3,5)]
            assert after['rgb']==expected
        report=read(ROOT/COVERAGE/case['id']/'report.json')
        extra=read(plan['extra_geometry_file']) if plan['extra_geometry_sha256'] else {'checks':{}}
        if plan['extra_geometry_sha256']:assert checksum(plan['extra_geometry_file'])==plan['extra_geometry_sha256']
        for pair in plan['assignments']:
            if not pair['local']:continue
            pickup=next(r for r in report['sources'] if r['id']==pair['object_id'])
            assert pickup['confirmed'] and pickup['witness']['robot']==pair['robot']
            goal=next(r for r in report['goals'] if r['id']==pair['goal_id'])
            if goal['witness']['robot']!=pair['robot']:
                goal=extra['checks'][f"{case['id']}/{pair['robot']}/{pair['goal_id']}"]
            assert goal['confirmed'] and goal['witness']['robot']==pair['robot']
            assert goal['position_m']==next(g['position_m'] for g in old['goals'] if g['goal_id']==pair['goal_id'])
        published=read(folder/'public-task.json');_check_target(task,published)
        dest=package/'public/cases'/case['id'];public=read(dest/'task.json');_public_boundary(public)
        expected=project_public(task,read(folder/'environment.json'),read(folder/'workstation.json'),published,baseline['completion'])
        expected['scene_revision']=14;expected['guidance']['task_prompt']='../../guidance/mosaic-r14.md'
        expected['package_status']='colour_design_review_only_no_r14_oracle_qualification'
        assert public==expected
        # Reference images remain the same target (rendering the supplied matrix).
        from PIL import Image,ImageColor
        with Image.open(dest/'target-pattern.png') as image:
            for r,line in enumerate(task['grid']['pattern']):
                for c,colour in enumerate(line):assert image.getpixel((c*40+20,r*40+20))==ImageColor.getrgb(task['grid']['palette'][colour])
        validation=read(folder/'validation.json')
        assert validation['oracle_verified'] is False and validation['policy_verified'] is False and validation['physics_steps']==0
        rows.append(dict(case_id=case['id'],changed_colours=sum(a['colour']!=a['previous_colour'] for a in plan['assignments']),
            stock=len(plan['assignments']),certified_local_pairs=sum(a['local'] for a in plan['assignments']),
            per_robot=[{k:r[k] for k in ('robot','nearby_stock','certified_local_pairs','certified_local_fraction')} for r in plan['per_robot']]))
    manifest=read(package/'engineering/package-manifest.json')
    assert manifest['policy_launch_ready'] is False
    assert checksum(ROOT/'configs/task_families/mosaic/r14.json')==manifest['registry_sha256']
    for file,sha in manifest['builder_sources'].items():assert checksum(ROOT/file)==sha
    for file,sha in manifest['plan_files'].items():assert checksum(file)==sha
    actual={str(p.relative_to(package/'public')):checksum(p) for p in (package/'public').rglob('*') if p.is_file()}
    assert not any(p.is_symlink() for p in (package/'public').rglob('*'))
    assert actual==manifest['public_files'] and len(actual)==20
    previews=read(package/'previews/preview-manifest.json');assert len(previews['cases'])==9
    for entry in previews['cases']:
        for name,sha in entry['files'].items():assert checksum(package/'previews'/entry['case_id']/name)==sha
    preserved=read(engineering/'preserved-inputs.json')
    for file,sha in preserved.items():assert checksum(ROOT/file)==sha,'Historical file changed: '+file
    result=dict(passed=True,cases=rows,historical_files_unchanged=len(preserved),physics_steps=0,
        policy_runs_started=0,new_oracle_runs=0,policy_launch_ready=False,
        scope='Colour-only XML, compiled physical identity, private/public inventory, distinct local pairing certificates, source pins, historical media and new preview integrity')
    write(engineering/'verification.json',result)
    print('VERIFIED nine cases;',len(preserved),'historical files unchanged;',sum(r['certified_local_pairs'] for r in rows),'/',sum(r['stock'] for r in rows),'local pairs')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--engineering',type=Path,required=True);p.add_argument('--package',type=Path,required=True)
    a=p.parse_args();check(a.engineering,a.package)
