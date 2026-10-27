"""Build an audited Mosaic-only review package without creating a policy run.

Engineering evidence and policy-visible static specifications are separate trees.
Shared readiness, projection, prompt loading and runtime code are not changed.
"""
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import shutil

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = 'configs/task_families/mosaic/r13.json'
CONTROL_FILES = ('robot_control.py', 'sim.py', 'motion_check.py', 'task_config.py', 'pipeline.py')
FORBIDDEN_PUBLIC_KEYS = {'objects', 'goals', 'assignments', 'qpos', 'qvel', 'initial_position_m',
                         'initial_quaternion_wxyz', 'repair_recipe', 'oracle', 'oracle_mode',
                         'actions', 'source_trial', 'executor_sha256'}


def checksum(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def local(root, relative):
    value = PurePosixPath(relative)
    if value.is_absolute() or '..' in value.parts or not value.parts:
        raise ValueError('Expected a repository-relative reference')
    # runs/outputs are intentionally external data symlinks in managed worktrees.
    return Path(root).joinpath(*value.parts)


def _pinned(path, expected):
    if checksum(path) != expected:
        raise ValueError('Pinned source differs: '+str(path))


def _public_boundary(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_PUBLIC_KEYS:
                raise ValueError('Private field in public task: '+key)
            _public_boundary(child)
    elif isinstance(value, list):
        for child in value:
            _public_boundary(child)


def project_public(task, environment, workstation, published, completion):
    """Explicit static projection; never copy the full private task or mechanism."""
    grid = task['grid']
    target = {key:deepcopy(published['target'][key]) for key in
              ('rows','columns','colors','pitch_m','cube_side_m','pattern','pocket_opening_m','depth_m','palette')}
    target.update(center_world_m=deepcopy(published['target']['center_world_m']), row1='north',
                  column1='west', reference_image='target-pattern.png',
                  supplied_reference_orientation='Already world north-up; use as supplied',
                  floor_z_m=grid['floor_z_m'],
                  goal_cube_center_z_m=grid['floor_z_m']+task['cube_size_m']/2)
    placement = dict(completion['recessed'] if task['level']==1 else completion['flat'])
    placement['mode'] = 'complete_footprint_and_floor_contact' if task['level']==1 else 'center_position'
    result = dict(schema='robofrontier.mosaic-policy-task.v1', kind='frontier_suite',
        id=task['id'], family='mosaic', suite_version=2, scene_revision=13,
        source_scene_sha256=task['scene_sha256'], level=task['level'],
        arms=deepcopy(published['arms']), rules=deepcopy(published['rules']),
        objective=published['objective'], coordinate_frame=published['coordinate_frame'],
        observation_contract=deepcopy(published['observation_contract']),
        gripper_profile=published['gripper_profile'], cameras=deepcopy(published['cameras']),
        initial_supplies=deepcopy(published['initial_supplies']), target=target,
        mechanism={key:deepcopy(task['mechanism'][key]) for key in ('kind','stock_packing','stock_count')},
        static_geometry=dict(table_height_m=task['table_height_m'], tables=deepcopy(environment['tables']),
            cube_side_m=task['cube_size_m'], cube_mass_kg=task['physics']['object_mass_kg'],
            contact_friction=deepcopy(task['physics']['contact_friction']),
            workspace_world_m=deepcopy(task['workspace_world_m']),
            camera_calibration=deepcopy(workstation['cameras']), gripper=deepcopy(workstation['gripper'])),
        completion=dict(stable_duration_sim_s=completion['stable_duration_sim_s'],
            stable_linear_speed_m_s_exclusive=completion['stable_linear_speed_m_s_exclusive'],
            stable_angular_speed_rad_s_exclusive=completion['stable_angular_speed_rad_s_exclusive'],
            axis_alignment_tolerance_deg_inclusive=completion['axis_alignment_tolerance_deg_inclusive'],
            robot_contact_release_distance_m_inclusive=completion['robot_contact_release_distance_m_inclusive'],
            placement=placement, distinct_cube_per_cell=True, process_violations_clear_required=True,
            orientation='Cube axes aligned to workstation axes, including cube symmetries'),
        execution_readiness=task['execution_readiness'],
        package_status='review_supplement_only_shared_integration_pending',
        guidance=dict(task_prompt='../../guidance/mosaic-r13.md', skill='../../guidance/mosaic-r13/SKILL.md'))
    result['observation_contract']['policy'] = [item for item in result['observation_contract']['policy'] if item != 'Visible dial count']
    result['rules'].append('The supplied pattern matrix and reference image already have their final world orientation; do not apply the historical rotation or mirror again.')
    _public_boundary(result)
    return result


def _check_target(task, public):
    grid=task['grid'];rows=grid['rows'];cols=grid['columns'];pattern=public['target']['pattern']
    for key in ('rows','columns','colors','pitch_m','cube_side_m','pocket_opening_m','depth_m','palette'):
        if public['target'][key]!=grid[key]:raise ValueError('Public target geometry differs: '+key)
    if (public['arms']!=task['arms'] or public['level']!=task['level'] or public['id']!=task['id']
            or public['rules']!=task['rules'] or public['target']['center_world_m']!=[0,0,task['table_height_m']]):
        raise ValueError('Published identity/frame/rules differ from frozen task')

    if len(pattern)!=rows or any(len(row)!=cols for row in pattern):
        raise ValueError('Mosaic public target dimensions differ')
    colors=Counter(color for row in pattern for color in row)
    if colors!=Counter(obj['color'] for obj in task['objects']):
        raise ValueError('Mosaic public target and inventory differ')
    if len(task['goals'])!=rows*cols:
        raise ValueError('Mosaic goal count differs')
    goals={g['goal_id']:g for g in task['goals']}
    for row in range(rows):
        for col in range(cols):
            goal=goals[f'r{row+1}c{col+1}']
            expected=[(col-(cols-1)/2)*grid['pitch_m'],((rows-1)/2-row)*grid['pitch_m'],grid['floor_z_m']+task['cube_size_m']/2]
            if goal['requires']['color']!=pattern[row][col] or math.dist(goal['position_m'],expected)>1e-9:
                raise ValueError('Published pattern/frame differs from private target predicate')


def verify_registry(root=ROOT, registry_path=None, verify_recorded_chunks=False):
    root=Path(root);path=Path(registry_path) if registry_path else root/REGISTRY;registry=read(path)
    if registry['schema']!='robofrontier.mosaic-family.v1' or registry['scene_revision']!=13:
        raise ValueError('Expected the Mosaic R13 registry')
    expected={f'mosaic-a{n:02d}-{r}x{c}-d{level}' for n,r,c in ((4,6,12),(6,9,18),(8,12,24)) for level in (1,2,3)}
    if len(registry['cases'])!=9 or {c['case_id'] for c in registry['cases']}!=expected:
        raise ValueError('Registry must bind the nine unique Mosaic R13 cases')
    _pinned(local(root,registry['catalog']),registry['catalog_sha256'])
    _pinned(local(root,registry['harness']),registry['harness_sha256'])
    for name,sha in registry['evaluator_sources'].items():_pinned(local(root,name),sha)
    from ..task_config import physical_control_fingerprint
    harness=read(local(root,registry['harness']))
    physical=physical_control_fingerprint({f'robofrontier/{n}':checksum(root/'robofrontier'/n) for n in CONTROL_FILES},harness)
    catalog={c['id']:c for c in read(local(root,registry['catalog']))['cases']}
    verified=[]
    for entry in registry['cases']:
        ident=entry['case_id'];directory=local(root,entry['environment']);evidence=entry['oracle'];trial=local(root,evidence['trial']);frozen=trial/'motion/source'
        if catalog[ident]['directory']!=entry['environment'] or catalog[ident]['scene_revision']!=13:
            raise ValueError('Wrong catalogue revision: '+ident)
        for name,sha in entry['source_files'].items():_pinned(directory/name,sha)
        for name in ('scene.xml','task.json','environment.json','workstation.json'):
            if checksum(directory/name)!=checksum(frozen/name):raise ValueError('Current and recorded source differ: '+ident+'/'+name)
        _pinned(trial/'report.json',evidence['report_sha256']);_pinned(trial/'independent-motion-audit.json',evidence['audit_sha256']);_pinned(trial/'motion/stream.json',evidence['motion_index_sha256'])
        for log in ('actions','trace','process'):
            _pinned(trial/(log+('.json' if log=='actions' else '.jsonl')),evidence[log+'_sha256'])
        report=read(trial/'report.json');audit=read(trial/'independent-motion-audit.json');stream=read(trial/'motion/stream.json')
        if (not report['passed'] or report['commissioning_only'] or not audit['passed'] or audit['errors']
                or report['unresolved_robot_actions'] or report['live_object_state_writes_after_initialization']):
            raise ValueError('Unqualified full Oracle evidence: '+ident)
        if stream['status']!='complete' or stream['frames']!=entry['source_frames'] or audit['frames']!=entry['source_frames']:
            raise ValueError('Incomplete motion evidence: '+ident)
        if any(value!=entry['scene_sha256'] for value in (report['scene_sha256'],audit['source_scene_sha256'],stream['source_scene_sha256'])):
            raise ValueError('Scene identity differs: '+ident)
        if (report['physical_control_sha256']!=physical or evidence['physical_control_sha256']!=physical
                or read(trial/'executor-source/inputs/harness.json')!=harness):
            raise ValueError('Current controller differs from qualified Oracle: '+ident)
        if (report['oracle_mode']!=evidence['mode'] or report['assisted']!=evidence['assisted']
                or report['failed_robot_actions']!=evidence['failed_robot_actions']
                or report['executor_sha256']!=evidence['executor_sha256']
                or report['environment_logic_sha256']!=evidence['environment_logic_sha256']):
            raise ValueError('Oracle provenance flags differ: '+ident)
        for name,sha in registry['evaluator_sources'].items():
            _pinned(trial/'executor-source'/name,sha)
            if report['environment_logic_provenance']['sources'][name]!=sha:
                raise ValueError('Recorded evaluator differs: '+ident)
        task=read(directory/'task.json');environment=read(directory/'environment.json');workstation=read(directory/'workstation.json');public=read(directory/'public-task.json')
        if (task['id']!=ident or task['family']!='mosaic' or task['scale']['scene_revision']!=13
                or len(task['arms'])!=entry['arms'] or task['level']!=entry['level']
                or task['grid']['rows']!=entry['rows'] or task['grid']['columns']!=entry['columns']):
            raise ValueError('Task identity/scale differs: '+ident)
        if task['completion']['stable_duration_sim_s']!=registry['completion']['stable_duration_sim_s']:
            raise ValueError('Completion dwell differs: '+ident)
        evaluation=report['evaluation'];total=entry['rows']*entry['columns']
        if (not evaluation['task_success'] or evaluation['violations'] or evaluation['matched_goals']!=total or evaluation['total_goals']!=total):
            raise ValueError('Original full goal count not qualified: '+ident)
        for asset in workstation['assets']:
            file=(frozen/asset['path']).resolve()
            if not file.is_relative_to(frozen.resolve()):raise ValueError('Source asset escapes frozen workstation')
            _pinned(file,asset['sha256'])
        if verify_recorded_chunks:
            for chunk in stream['chunks']:_pinned(trial/'motion'/chunk['file'],chunk['sha256'])
        _check_target(task,public)
        verified.append((entry,task,environment,workstation,public))
    return registry,verified


def _target_image(public, destination):
    from PIL import Image,ImageDraw
    target=public['target'];cell=40;image=Image.new('RGB',(target['columns']*cell,target['rows']*cell));draw=ImageDraw.Draw(image)
    for row,line in enumerate(target['pattern']):
        for col,color in enumerate(line):draw.rectangle((col*cell,row*cell,(col+1)*cell-1,(row+1)*cell-1),fill=target['palette'][color],outline='#7C8B95')
    image.save(destination)


def build(output,root=ROOT,registry_path=None,verify_recorded_chunks=False):
    root=Path(root);output=Path(output).resolve()
    registry,verified=verify_registry(root,registry_path,verify_recorded_chunks)
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    for key,target in [('prompt','public/guidance/mosaic-r13.md'),('skill','public/guidance/mosaic-r13/SKILL.md')]:
        destination=output/target;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(local(root,registry['guidance'][key]),destination)
    for entry,task,environment,workstation,published in verified:
        destination=output/'public/cases'/entry['case_id'];destination.mkdir(parents=True)
        public=project_public(task,environment,workstation,published,registry['completion'])
        write(destination/'task.json',public);_target_image(public,destination/'target-pattern.png')
    files={str(p.relative_to(output/'public')):checksum(p) for p in sorted((output/'public').rglob('*')) if p.is_file()}
    context_hashes={key:checksum(local(root,value)) for key,value in registry['guidance'].items()}
    manifest=dict(schema='robofrontier.mosaic-review-package.v1',family='mosaic',cases=9,
        status='review_package_verified_shared_integration_pending',policy_launch_ready=False,
        registry_sha256=checksum(registry_path or root/REGISTRY),builder_sha256=checksum(__file__),guidance_sha256=context_hashes,public_files=files,
        raw_motion_chunks_rechecked=verify_recorded_chunks,
        evidence_scope='Pinned final reports/audits/stream indices, source assets, exact evaluator/control identities; raw motion checksums only when explicitly requested',
        presentation_defaults=registry['presentation_defaults'],shared_dependencies=registry['readiness']['shared_dependencies'])
    write(output/'engineering/registry.json',registry);write(output/'engineering/package-manifest.json',manifest)
    check(output,root,registry_path)
    return manifest


def check(output,root=ROOT,registry_path=None):
    output=Path(output).resolve();root=Path(root);registry,verified=verify_registry(root,registry_path)
    manifest=read(output/'engineering/package-manifest.json')
    if manifest.get('policy_launch_ready') is not False or manifest.get('cases')!=9:
        raise ValueError('Review package cannot activate policy execution')
    if manifest['builder_sha256']!=checksum(__file__):raise ValueError('Review builder changed')
    if manifest['registry_sha256']!=checksum(registry_path or root/REGISTRY) or read(output/'engineering/registry.json')!=registry:
        raise ValueError('Review registry changed')
    if (output/'public').is_symlink() or any(p.is_symlink() for p in (output/'public').rglob('*')):
        raise ValueError('Public symlink is outside the review artifact contract')
    actual={str(p.relative_to(output/'public')):p for p in (output/'public').rglob('*') if p.is_file()}
    expected={'guidance/mosaic-r13.md','guidance/mosaic-r13/SKILL.md'}
    expected.update(f'cases/{entry["case_id"]}/{name}' for entry,_,_,_,_ in verified for name in ('task.json','target-pattern.png'))
    if set(actual)!=expected or set(manifest['public_files'])!=expected:
        raise ValueError('Unexpected or missing public package file')
    for name,path in actual.items():
        if path.is_symlink() or not path.resolve().is_relative_to((output/'public').resolve()):
            raise ValueError('Public artifact escapes package')
        _pinned(path,manifest['public_files'][name])
    if manifest['guidance_sha256']!={key:checksum(local(root,path)) for key,path in registry['guidance'].items()}:
        raise ValueError('Guidance provenance changed')
    for key,name in [('prompt','guidance/mosaic-r13.md'),('skill','guidance/mosaic-r13/SKILL.md')]:
        if checksum(actual[name])!=checksum(local(root,registry['guidance'][key])):
            raise ValueError('Frozen family guidance differs')
    for entry,task,env,workstation,published in verified:
        public=read(actual[f'cases/{entry["case_id"]}/task.json']);_public_boundary(public)
        if public!=project_public(task,env,workstation,published,registry['completion']):
            raise ValueError('Public specification differs from verified source')
        from PIL import Image,ImageColor
        with Image.open(actual[f'cases/{entry["case_id"]}/target-pattern.png']) as image:
            image=image.convert('RGB');target=public['target']
            if image.size!=(40*target['columns'],40*target['rows']):raise ValueError('Target image dimensions differ')
            for row,line in enumerate(target['pattern']):
                for col,color in enumerate(line):
                    if image.getpixel((col*40+20,row*40+20))!=ImageColor.getrgb(target['palette'][color]):
                        raise ValueError('Target image orientation/colors differ')
    return dict(passed=True,cases=9,policy_launch_ready=False,shared_integration_pending=True)
