"""Resolve released family inputs and freeze only their permitted public facts.

Historical Oracle registries remain immutable. This release records which scene
and reviewed public projection a new policy trial actually uses.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
RELEASE = 'configs/task_families/current.json'
READINESS = 'prepared_policy_trial'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def resolve_case(case_id, purpose='gpt', root=ROOT):
    root = Path(root)
    release = read(root/RELEASE)
    if purpose not in ('gpt', 'oracle'):
        raise ValueError('Unknown task purpose')
    matches = [(name, family, c) for name, family in release['families'].items()
               for c in family[purpose] if c['id'] == case_id]
    if len(matches) != 1:
        raise ValueError('Case must identify exactly one released family input')
    name, family, case = matches[0]
    for path, checksum in case['source_files'].items():
        if sha(root/case['directory']/path) != checksum:
            raise ValueError('Released scene metadata changed: '+case_id+'/'+path)
    return name, family, case


def protocol_for(case, release_sha256, wall_limit_s=86400):
    import math
    if isinstance(wall_limit_s, bool) or not isinstance(wall_limit_s, (int,float)) or not math.isfinite(wall_limit_s) or not 0 < wall_limit_s <= 86400:
        raise ValueError('Invalid policy wall limit')
    return dict(kind='policy_trial', launch='explicit_manager_start',
        release_sha256=release_sha256, case_id=case['id'],
        source_task_sha256=case['source_files']['task.json'],
        source_scene_sha256=case['source_files']['scene.xml'],
        reference_evidence=case['qualification'], total_wall_limit_s=wall_limit_s,
        evaluation_semantics='terminal_outcome_v2',
        experience='fresh_run_only', presentation_loop=False)


def policy_task(case_id, root=ROOT, wall_limit_s=86400):
    name, family, case = resolve_case(case_id, root=root)
    task = read(Path(root)/case['directory']/'task.json')
    task['execution_readiness'] = READINESS
    task['run_protocol'] = protocol_for(case, sha(Path(root)/RELEASE), wall_limit_s)
    return task


def validate_policy_task(task, root=ROOT):
    protocol = task.get('run_protocol', {})
    if (protocol.get('kind') != 'policy_trial'
            or protocol.get('launch') != 'explicit_manager_start'
            or protocol.get('experience') != 'fresh_run_only'
            or protocol.get('presentation_loop') is not False
            or not isinstance(protocol.get('total_wall_limit_s'), (int, float))):
        raise ValueError('Invalid released policy protocol')
    _, _, case = resolve_case(task['id'], root=root)
    if (protocol != protocol_for(case, sha(Path(root)/RELEASE), protocol['total_wall_limit_s'])
            or protocol.get('case_id') != task['id']
            or protocol.get('source_task_sha256') != case['source_files']['task.json']
            or protocol.get('source_scene_sha256') != task['scene_sha256']
            or task.get('oracle_verified') is not False):
        raise ValueError('Policy source/release binding differs')
    original = read(Path(root)/case['directory']/'task.json')
    expected = deepcopy(original)
    expected['execution_readiness'] = READINESS
    expected['run_protocol'] = protocol
    if task != expected:
        raise ValueError('Policy task changed outside its execution protocol')
    return protocol


def freeze_public(task, directory, public, root=ROOT):
    """Copy allowlisted family guidance and overlay its reviewed static facts."""
    if task.get('execution_readiness') != READINESS:
        return None
    root, directory = Path(root), Path(directory)
    family_name, _, case = resolve_case(task['id'], root=root)
    projection = case.get('public_projection')
    review = {}
    if projection:
        if sha(root/projection['path']) != projection['sha256']:
            raise ValueError('Reviewed public task changed')
        review = read(root/projection['path'])
        if review['id'] != task['id'] or review['family'] != family_name:
            raise ValueError('Public projection belongs to another task')
    keys = ('completion', 'workspace_world_m', 'table_height_m', 'table_geometry',
            'static_geometry', 'static_workstation', 'static_camera_mounts',
            'process_requirements', 'task_revision', 'scene_revision',
            'source_scene_sha256', 'initial_supplies', 'target', 'mechanism',
            'rules', 'objective', 'observation_contract', 'gripper_profile', 'dials')
    for key in keys:
        if key in review:
            public[key] = deepcopy(review[key])
    public.setdefault('completion', deepcopy(task['completion']))
    public.setdefault('workspace_world_m', deepcopy(task['workspace_world_m']))
    terminal_relay = (family_name == 'relay' and task.get('run_protocol', {}).get('evaluation_semantics') == 'terminal_outcome_v2')
    if terminal_relay:
        rules = public.get('rules', [])
        public['rules'] = [rule for rule in rules if not rule.startswith((
            'Airborne transport requires', 'Inside GOAL, original qualified holders',
            'Drops and violations remain recorded.'))]
        public['rules'].append('Collisions, drops and other process events are recorded separately. Recover using ordinary robot actions; do not reset, respawn or erase history.')
        public['completion']['required_airborne_handover'] = False
        public['completion']['scope'] = 'All assemblies released, actually supported and inside GOAL tolerance; process events are reported separately.'
    family_dir = directory/'public/inputs/family'
    family_dir.mkdir()
    hashes = {}
    for name, source in case['public_files'].items():
        if Path(name).name != name or sha(root/source['path']) != source['sha256']:
            raise ValueError('Invalid/changed public family file: '+name)
        selected = root/'prompts/tasks/relay-terminal-v2.md' if terminal_relay and name == 'family-guidance.md' else root/source['path']
        shutil.copyfile(selected, family_dir/name)
        hashes['public/inputs/family/'+name] = sha(selected)
    public['family_guidance'] = 'family/family-guidance.md'
    public['family_materials'] = sorted('family/'+n for n in case['public_files'])
    for key, name in [('static_workstation', 'table-geometry.json')]:
        if key in public and name in case['public_files']:
            public[key]['geometry_reference'] = 'family/'+name
    if 'camera-calibration.json' in case['public_files']:
        public['camera_calibration'] = 'family/camera-calibration.json'
    if 'task-guide.md' in case['public_files']:
        public['task_guide'] = 'family/task-guide.md'
    if 'skill.md' in case['public_files']:
        public['task_skill'] = 'family/skill.md'
    if 'initial_reference_images' in review:
        images = review['initial_reference_images']
        if any(name not in case['public_files'] for name in images):
            raise ValueError('Unfrozen public reference image')
        public['initial_reference_images'] = ['family/'+name for name in images]
    private_release = directory/'private/inputs/family-release.json'
    shutil.copyfile(root/RELEASE, private_release)
    return dict(release_sha256=sha(private_release), family=family_name, case_id=task['id'],
                qualification=case['qualification'], public_files=hashes,
                public_projection=projection,
                public_overlay='relay-terminal-v2' if terminal_relay else None)


def prepare(case_id, directory, harness=None, private_puzzle_path=None, wall_limit_s=86400):
    """Prepare only; simulator, role sessions and model inference stay stopped."""
    import tempfile
    from .run_setup import prepare_run
    task = policy_task(case_id, wall_limit_s=wall_limit_s)
    with tempfile.TemporaryDirectory(prefix='robofrontier-policy-') as temporary:
        path = Path(temporary)/'task.json'
        path.write_text(json.dumps(task, ensure_ascii=False)+'\n')
        return prepare_run(directory, path, harness or ROOT/'configs/harnesses/policy.json', private_puzzle_path)


def validate_frozen_policy(task, directory):
    """START checks the actual frozen release and scene, including task metadata."""
    directory = Path(directory)
    release = directory/'private/inputs/family-release.json'
    protocol = task['run_protocol']
    if sha(release) != protocol['release_sha256']:
        raise ValueError('Frozen policy release differs')
    cases = [c for f in read(release)['families'].values() for c in f['gpt']
             if c['id'] == task['id']]
    if len(cases) != 1:
        raise ValueError('Policy case is not in the frozen release')
    case = cases[0]
    if protocol != protocol_for(case, sha(release), protocol['total_wall_limit_s']):
        raise ValueError('Frozen policy protocol differs')
    source = directory/Path(task['scene_asset']).parent
    for name, expected in case['source_files'].items():
        if sha(source/name) != expected:
            raise ValueError('Frozen policy scene metadata differs: '+name)
    original = read(source/'task.json')
    original.update(scene_asset=task['scene_asset'], execution_readiness=READINESS,
                    run_protocol=protocol)
    if original != task:
        raise ValueError('Frozen policy task differs from its released scene')
    return protocol
