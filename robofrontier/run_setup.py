"""Prepare immutable inputs and durable runtime, without starting simulation/inference."""
import hashlib
import json
import os
import tempfile
from pathlib import Path
import secrets
import shutil

from .runtime_store import RuntimeStore, packed
from .observation_image import DEFAULT_PROFILE, observation_profile
from .suite.provenance import environment_logic_manifest
from .task_config import ROOT, load_task, load_harness, public_task,physical_control_fingerprint


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    fd, temporary = tempfile.mkstemp(prefix='.'+path.name+'-',dir=path.parent)
    try:
        with os.fdopen(fd,'w') as out:
            os.fchmod(out.fileno(),0o600)
            out.write(payload);out.flush();os.fsync(out.fileno())
        os.replace(temporary,path)
    finally:
        if Path(temporary).exists():Path(temporary).unlink()


def prepare_run(directory, task_path=None, harness_path=None, private_puzzle_path=None):
    directory = Path(directory).resolve()
    if directory.exists():
        raise ValueError('Choose a new run directory; existing runs are never overwritten')
    task, cfg = load_task(task_path), load_harness(harness_path)
    # Freeze new-run defaults; absent fields in already prepared runs remain legacy.
    cfg.setdefault('observation_image_profile', DEFAULT_PROFILE)
    image_profile = observation_profile(cfg)
    puzzle=None;puzzle_relative=None
    if task.get('kind')=='spring_dial':
        from .spring_dial_task import private_code,CODE_PATH
        puzzle=private_code(private_puzzle_path)
        puzzle_relative=CODE_PATH
    elif task.get('kind')=='frontier_suite' and task.get('family')=='dial':
        n=len(task['dials'])
        from .suite.dial_feedback import alphabet
        values_allowed=alphabet(task)
        puzzle=json.loads(Path(private_puzzle_path).read_text()) if private_puzzle_path else {'code_symbols':[secrets.choice(values_allowed) for _ in range(n)]}
        values=puzzle.get('code_symbols')
        if set(puzzle)!={'code_symbols'} or not isinstance(values,list) or len(values)!=n or any(type(x) is not int or x not in values_allowed for x in values):raise ValueError('Expected one configured private nonzero symbol per suite dial')
        puzzle_relative=Path('private/inputs/suite-code.json')
    elif private_puzzle_path is not None:
        raise ValueError('Private puzzle input applies only to spring_dial')
    directory.mkdir(parents=True)
    for name in ('public/inputs/roles', 'public/observations', 'public/results', 'private/inputs',
                 'private/state/connections', 'private/state/client-state', 'private/outputs'):
        (directory/name).mkdir(parents=True)
    if task.get('scene_asset'):
        from .scene_task import freeze_scene
        task=freeze_scene(task,directory)
    write_json(directory/'private/inputs/task.json', task)
    if puzzle is not None:
        write_json(directory/puzzle_relative,puzzle)
    public = public_task(task)
    from .task_families import freeze_public
    family_release = freeze_public(task, directory, public)
    from .scene_task import camera_names
    public['cameras'] = camera_names(task)
    for key,name in (('robot_layout_image','robot-layout.png'),('camera_calibration','camera-guide.json')):
        if (directory/'public/inputs'/name).is_file():public[key]='public/inputs/'+name
    public['observation_image'] = {k: image_profile[k] for k in ('width', 'height', 'format')}
    if 'quality' in image_profile: public['observation_image']['quality'] = image_profile['quality']
    # Preserve the signed family asset; publish a run-specific calibration copy
    # only when its original declares an image size. Camera pose/FOV stays fixed.
    calibration_ref = public.get('camera_calibration')
    if calibration_ref:
        calibration_path = directory/'public/inputs'/calibration_ref.removeprefix('public/inputs/')
        if calibration_path.is_file():
            calibration = json.loads(calibration_path.read_text())
            if 'image_size_pixel' in calibration:
                calibration['image_size_pixel'] = [image_profile['width'], image_profile['height']]
                calibration['source_calibration'] = calibration_ref
                calibration['scope'] = str(calibration.get('scope', 'Camera calibration')) + '; image size matches this run observation profile'
                derived = directory/'public/inputs/observation-camera-calibration.json'
                write_json(derived, calibration)
                public['camera_calibration'] = 'public/inputs/observation-camera-calibration.json'
    write_json(directory/'public/inputs/task.json', public)
    write_json(directory/'private/inputs/harness.json', cfg)
    roles = {cfg['manager']: {'kind':'manager'}}
    if cfg['topology'] == 'hierarchical':
        roles[cfg['coordinator']] = {'kind':'coordinator'}
    if cfg['topology'] in ('centralized','adaptive'):
        roles['Controller'] = {'kind':'controller', 'robots':list(task['arms'])}
    else:
        roles.update({a: {'kind':'worker', 'robot':a} for a in task['arms']})
    write_json(directory/'public/inputs/roles.json', {'topology':cfg['topology'], 'manager':cfg['manager'], 'coordinator':cfg['coordinator'] if cfg['topology']=='hierarchical' else None, 'roles':roles})
    templates = ROOT/'prompts'
    common = (templates/'common.md').read_text().replace('{{RUN_DIR}}',str(directory))
    (directory/'public/inputs/common.md').write_text(common)
    shutil.copyfile(templates/'policy-rules.md', directory/'public/inputs/policy-rules.md')
    shutil.copytree(ROOT/'.agents/skills/robot-collaboration', directory/'public/inputs/skill')
    shutil.copy2(ROOT/'scripts/role_tools.js', directory/'public/inputs/role_tools.js')
    runbook=(ROOT/'docs/meta-runbook.md').read_text()
    for target in ('architecture.md','harness-v2-verification.md'):
        runbook=runbook.replace(']('+target+')','](frozen-source/docs/'+target+')')
    runbook=runbook.replace('](../.agents/','](frozen-source/.agents/')
    (directory/'private/inputs/meta-runbook.md').write_text(runbook)
    python = ROOT/'.venv/bin/python'
    for actor, info in roles.items():
        kind = info['kind']
        filename = {'manager':'meta-manager','coordinator':'coordinator','worker':'worker','controller':'controller'}[kind]
        if kind=='controller' and cfg['topology']=='adaptive':filename='adaptive-controller'
        values = {'RUN_DIR':str(directory),'REPO_ROOT':str(ROOT),'PYTHON':str(python), 'ACTOR':actor,
                  'CONNECTION':str(directory/'private/state/connections'/f'{actor}.json'),
                  'TOPOLOGY':cfg['topology'],
                  'FINISH_AUTHORITY':cfg['coordinator'] if cfg['topology']=='hierarchical' else 'any peer Worker' if cfg['topology']=='peer' else 'Controller',
                  'COMMUNICATION_ROUTE':('Coordinate joint commitments and shared resources through '+cfg['coordinator']+'. Worker-to-Worker business messages are unavailable.' if cfg['topology']=='hierarchical' else 'Negotiate affected commitments and shared resources directly with relevant Workers.')}
        text = (templates/'roles'/f'{filename}.md').read_text()
        for key,value in values.items():
            text = text.replace('{{'+key+'}}',value)
        (directory/'public/inputs/roles'/f'{actor}.md').write_text(text)
        connection = directory/'private/state/connections'/f'{actor}.json'
        write_json(connection, {'schema_version':3, 'run_id':directory.name, 'url':None,
                                'actor':actor, 'token':secrets.token_urlsafe(24)})
        connection.chmod(0o600)
    store = RuntimeStore(directory/'private/state/runtime.sqlite')
    store.initialize(directory.name, cfg, task['arms'])
    store.close()
    paths = list((ROOT/'robofrontier').rglob('*.py'))
    paths += list((ROOT/'prompts').rglob('*.md'))
    paths += list((ROOT/'.agents/skills/robot-collaboration').rglob('*'))
    paths += [ROOT/'scripts'/name for name in ('create_pipeline_run.py','meta_dispatcher.js','export_run_report.py','role_tools.js',
                                             'benchmark.py','codex_rpc.js','codex_policy_host.js','codex_preflight.js')]
    paths += [ROOT/'docs'/name for name in ('meta-runbook.md','architecture.md','harness-v2-verification.md')]
    source_hashes = {}
    for source in paths:
        if not source.is_file():
            continue
        relative = source.relative_to(ROOT)
        dest = directory/'private/inputs/frozen-source'/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,dest)
        source_hashes[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
    inputs = {}
    for p in sorted((directory/'public/inputs').rglob('*')):
        if p.is_file():inputs[str(p.relative_to(directory))] = hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (directory/'private/inputs/scene').rglob('*'):
        if p.is_file():inputs[str(p.relative_to(directory))]=hashlib.sha256(p.read_bytes()).hexdigest()
    if puzzle is not None:
        p=directory/puzzle_relative
        inputs[str(puzzle_relative)]=hashlib.sha256(p.read_bytes()).hexdigest()
    p=directory/'private/inputs/meta-runbook.md'
    inputs[str(p.relative_to(directory))]=hashlib.sha256(p.read_bytes()).hexdigest()
    if family_release is not None:
        p=directory/'private/inputs/family-release.json'
        inputs[str(p.relative_to(directory))]=hashlib.sha256(p.read_bytes()).hexdigest()
    logic=environment_logic_manifest(source_hashes,task) if task.get('kind')=='frontier_suite' else None
    write_json(directory/'private/inputs/manifest.json', {'schema_version':3,'run_id':directory.name,
        'task_sha256':hashlib.sha256(packed(task).encode()).hexdigest(),
        'harness_sha256':hashlib.sha256(packed(cfg).encode()).hexdigest(),
        'source_sha256':source_hashes,'input_sha256':inputs,
        'physical_control_sha256':physical_control_fingerprint(source_hashes,cfg),
        'environment_logic_sha256':logic['sha256'] if logic else None,
        'environment_logic_provenance':logic,
        'model_requested':cfg['model'],'effort_requested':cfg['effort'],
        'family_release':family_release,
        'observation_contract':task['observation_contract'],
        'manager':'Meta session executes deterministic dispatcher; no independent desktop takeover',
        'isolation':'Role authentication and protocol/integrity checks; not an OS filesystem sandbox',
        'physical_restart':'unsupported; interrupted worlds require a new run',
        'status':'prepared; no simulator or inference started'})
    return directory


def verify_inputs(directory):
    directory = Path(directory)
    try:
        manifest = json.loads((directory/'private/inputs/manifest.json').read_text())
    except (OSError,ValueError):
        return ['manifest:unreadable']
    if not isinstance(manifest,dict):
        return ['manifest:invalid']
    if manifest.get('schema_version') != 3:
        return ['manifest:unsupported_schema']
    if not all(isinstance(manifest.get(k),dict) for k in ('source_sha256','input_sha256')) or not all(isinstance(manifest.get(k),str) for k in ('task_sha256','harness_sha256')):
        return ['manifest:invalid']
    changed = []
    for filename,key in [('task.json','task_sha256'),('harness.json','harness_sha256')]:
        try:
            actual=hashlib.sha256(packed(json.loads((directory/'private/inputs'/filename).read_text())).encode()).hexdigest()
        except (OSError,ValueError):
            actual=None
        if actual!=manifest[key]:changed.append('private:'+filename)
    for relative,digest in manifest['source_sha256'].items():
        p = ROOT/relative
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
            changed.append('source:'+relative)
        frozen=directory/'private/inputs/frozen-source'/relative
        if not frozen.is_file() or hashlib.sha256(frozen.read_bytes()).hexdigest()!=digest:
            changed.append('frozen:'+relative)
    for relative,digest in manifest['input_sha256'].items():
        p = directory/relative
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
            changed.append('input:'+relative)
    return changed
