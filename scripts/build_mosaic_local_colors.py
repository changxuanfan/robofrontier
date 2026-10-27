"""Prepare R14 local stock colours and visual previews, leaving R13 immutable."""
import argparse
from pathlib import Path

from robofrontier.suite.mosaic_local_colors import plan_scale, build_case, CATALOG
from robofrontier.suite.mosaic_policy_package import ROOT, read, write, checksum, project_public, _target_image, verify_registry


def build(engineering, output):
    from scripts.preview_mosaic_local_colors import previews
    import shutil
    output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    verify_registry()
    plans={n:read(Path(engineering)/f'a{n:02d}-plan.json') for n in (4,6,8)}
    cases=[build_case(c,plans[c['arms']]) for c in read(ROOT/CATALOG)['cases']]
    catalog=dict(schema_version=2,case_count=9,scene_revision=14,
        status='Colour-only local-supply design; geometric equivalence checked, no R14 Oracle or policy run',cases=cases)
    write(ROOT/'configs/suites/frontier-mosaic-r14.json',catalog)
    baseline=read(ROOT/'configs/task_families/mosaic/r13.json')
    registry=dict(schema='robofrontier.mosaic-local-colour-family.v1',family='mosaic',scene_revision=14,
        purpose='Future GPT input; historical Oracle and factory replay retain R13',
        catalog='configs/suites/frontier-mosaic-r14.json',
        catalog_sha256=checksum(ROOT/'configs/suites/frontier-mosaic-r14.json'),
        baseline_oracle_registry='configs/task_families/mosaic/r13.json',
        baseline_oracle_registry_sha256=checksum(ROOT/'configs/task_families/mosaic/r13.json'),
        harness=baseline['harness'],harness_sha256=baseline['harness_sha256'],completion=baseline['completion'],
        guidance=dict(prompt='prompts/tasks/mosaic-r14.md',skill=baseline['guidance']['skill']),
        presentation_defaults=baseline['presentation_defaults'],
        readiness=dict(policy_launch_ready=False,oracle_verified=False,status='colour_design_review_shared_integration_pending',
            shared_dependencies=baseline['readiness']['shared_dependencies']),
        design_evidence={str((Path(engineering)/f'a{n:02d}-plan.json').resolve()):checksum(Path(engineering)/f'a{n:02d}-plan.json') for n in plans},
        cases=[])
    for case in cases:
        folder=ROOT/case['directory']; task=read(folder/'task.json'); env=read(folder/'environment.json'); workstation=read(folder/'workstation.json')
        published=project_public(task,env,workstation,read(folder/'public-task.json'),baseline['completion'])
        published['scene_revision']=14
        published['guidance']['task_prompt']='../../guidance/mosaic-r14.md'
        published['package_status']='colour_design_review_only_no_r14_oracle_qualification'
        dest=output/'public/cases'/case['id'];dest.mkdir(parents=True)
        write(dest/'task.json',published);_target_image(published,dest/'target-pattern.png')
        registry['cases'].append(dict(case_id=case['id'],environment=case['directory'],
            source_files={name:checksum(folder/name) for name in ('scene.xml','environment.json','task.json','public-task.json','workstation.json','validation.json')},
            baseline_scene_sha256=read(folder/'validation.json')['source_scene_sha256'],
            local_fraction_min=min(r['certified_local_fraction'] for r in plans[case['arms']]['per_robot'])))
    for key,relative in registry['guidance'].items():
        dest=output/'public/guidance'/('mosaic-r14.md' if key=='prompt' else 'mosaic-r13/SKILL.md')
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
    write(ROOT/'configs/task_families/mosaic/r14.json',registry)
    write(ROOT/'configs/task_families/mosaic/selection.json',dict(
        schema='robofrontier.mosaic-purpose-selection.v1',
        future_gpt_registry='configs/task_families/mosaic/r14.json',
        oracle_and_factory_replay_registry='configs/task_families/mosaic/r13.json',
        policy_launch_ready=False,
        note='Explicit future integration selection, not a shared runtime switch. Do not replay R13 poses into R14 colours.'))
    write(output/'engineering/registry.json',registry)
    write(output/'engineering/package-manifest.json',dict(schema='robofrontier.mosaic-local-colour-review.v1',
        scene_revision=14,cases=9,policy_launch_ready=False,physics_steps=0,
        public_files={str(p.relative_to(output/'public')):checksum(p) for p in sorted((output/'public').rglob('*')) if p.is_file()},
        registry_sha256=checksum(ROOT/'configs/task_families/mosaic/r14.json'),
        builder_sources={name:checksum(ROOT/name) for name in ('scripts/build_mosaic_local_colors.py','robofrontier/suite/mosaic_local_colors.py','scripts/preview_mosaic_local_colors.py')},
        plan_files={str((Path(engineering)/f'a{n:02d}-plan.json').resolve()):checksum(Path(engineering)/f'a{n:02d}-plan.json') for n in plans},
        scope='Mosaic-only colour design and review supplement; no shared runtime activation or new Oracle success'))
    previews(cases,plans,output/'previews')
    return registry


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--engineering',type=Path,required=True)
    p.add_argument('--arms',type=int,choices=[4,6,8],help='Plan one scale only; no source files written')
    p.add_argument('--local-fraction',type=float,default=.70)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.arms:
        if not .60<=a.local_fraction<=.80:p.error('Design fraction must be within the requested 60–80% band')
        plan_scale(a.arms,a.engineering,a.local_fraction)
    elif a.output:build(a.engineering,a.output)
    else:p.error('Provide --arms for planning or --output for building all nine')
