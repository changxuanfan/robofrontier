"""Prepare and supervise independent Codex CLI policy tasks; never infer on prepare."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import shlex

from robofrontier.task_families import ROOT, RELEASE, read, prepare
from robofrontier.run_setup import write_json, verify_inputs


def load_env(path):
    """Load literal KEY=value entries; existing process environment wins."""
    path = Path(path)
    if not path.is_file():
        return
    import re
    for number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[7:].lstrip()
        key, sep, raw = line.partition('=')
        if not sep or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key):
            raise ValueError(f'Invalid .env entry at line {number}')
        try:
            values = shlex.split(raw, comments=True, posix=True)
        except ValueError as error:
            raise ValueError(f'Invalid .env value at line {number}') from error
        if len(values) > 1:
            raise ValueError(f'Quote whitespace in .env value at line {number}')
        os.environ.setdefault(key, values[0] if values else '')


def profile(name, config=None):
    catalog = read(config or ROOT/'configs/models.json')
    p = deepcopy(catalog['profiles'][name])
    p.update(context_window=catalog['context_window'], auto_compact_token_limit=catalog['auto_compact_token_limit'])
    if not p['vision']:
        raise ValueError('This benchmark requires native images; text-only models need a separately declared experiment')
    if not 0 < p['auto_compact_token_limit'] < p['context_window'] <= p['context_capacity']:
        raise ValueError('Requested context exceeds the declared model capacity')
    if 'base_url_env' in p:
        p['base_url'] = os.environ.get(p['base_url_env'])
        if not p['base_url']:
            raise ValueError('Set '+p['base_url_env']+' to the official workspace/region Responses base URL')
    if p['provider'] != 'openai':
        from urllib.parse import urlsplit
        url = urlsplit(p['base_url'])
        if url.scheme != 'https' or url.username or url.password or url.query:
            raise ValueError('Use an HTTPS base URL without embedded credentials')
    return p


def launch_config(p, folder):
    args = []
    values = {'model':p['model'], 'model_provider':p['provider'],
              'model_reasoning_effort':p['effort'], 'model_context_window':p['context_window'],
              'model_auto_compact_token_limit':p['auto_compact_token_limit'],
              'web_search':'disabled', 'features.multi_agent':False}
    if p['provider'] != 'openai':
        prefix='model_providers.'+p['provider']+'.'
        values.update({prefix+'name':p['provider'], prefix+'base_url':p['base_url'],
            prefix+'env_key':p['env_key'], prefix+'wire_api':'responses',
            prefix+'requires_openai_auth':False, prefix+'supports_websockets':False,
            prefix+'request_max_retries':3, prefix+'stream_max_retries':3})
    # Local Codex model catalog teaches the CLI that this exact model accepts
    # images. No global provider/auth/default settings are modified.
    cache_path=Path(os.environ.get('CODEX_HOME',Path.home()/'.codex'))/'models_cache.json'
    cached = read(cache_path).get('models', []) if cache_path.is_file() else []
    # Preserve exact OpenAI model metadata if available; the checked-in generic
    # template also permits an empty CODEX_HOME on a new server.
    model=deepcopy(next((m for m in cached if m.get('slug')==p['model']),
                        read(ROOT/'configs/codex-model-template.json')))
    model.update(slug=p['model'], display_name=p['model'], description='Benchmark provider model',
        context_window=p['context_window'],max_context_window=p['context_window'],
        input_modalities=['text','image'],tool_mode=None,shell_type='shell_command',
        use_responses_lite=False,supports_search_tool=False,apply_patch_tool_type='freeform',
        experimental_supported_tools=[],node_repl_disabled=True,
        default_reasoning_level=p['effort'],supported_reasoning_levels=[{'effort':p['effort'],'description':'Declared benchmark effort'}])
    model['model_messages']=None
    model['base_instructions']='You are a RoboFrontier policy agent running in Codex. Follow the frozen public task, role, tool protocol and evidence boundaries. Use only your assigned robot tools. Preserve original call IDs across uncertain results.'
    model['availability_nux']=None
    model['service_tiers']=[];model['additional_speed_tiers']=[]
    model['available_access_programs']=None
    target=folder/'models.json';write_json(target,{'models':[model]})
    values['model_catalog_json']=str(target)
    for key,value in values.items():
        # JSON scalar/array strings are also valid TOML values; argv, never shell.
        args.extend(['-c',key+'='+json.dumps(value)])
    return dict(model=p['model'],provider=p['provider'],effort=p['effort'],args=args,
                profile=p,context_window=p['context_window'],auto_compact_token_limit=p['auto_compact_token_limit'])


def select_cases(families=None, cases=None, level=None, min_arms=False):
    """Resolve selection from released task metadata, without creating a world."""
    release=read(ROOT/RELEASE)
    if families and set(families)-set(release['families']):
        raise ValueError('Unknown task families')
    selected=[]
    for name,family in release['families'].items():
        if families and name not in families:continue
        rows=[]
        for case in family['gpt']:
            task=read(ROOT/case['directory']/'task.json')
            if cases and case['id'] not in cases or level is not None and task['level']!=level:continue
            rows.append(dict(case_id=case['id'],family=name,level=task['level'],arms=len(task['arms']),
                             directory=case['directory'],qualification=case['qualification']))
        if min_arms and rows:
            minimum=min(row['arms'] for row in rows)
            rows=[row for row in rows if row['arms']==minimum]
        selected.extend(rows)
    if not selected or cases and set(cases)-{row['case_id'] for row in selected}:
        raise ValueError('Unknown/empty released cases or conflicting selection filters')
    return selected


def prepare_batch(output, names, families=None, cases=None, config=None, level=None, min_arms=False):
    if not names or len(set(names))!=len(names):raise ValueError('Choose distinct model profiles')
    profiles={n:profile(n,config) for n in names}
    selected=[row['case_id'] for row in select_cases(families,cases,level,min_arms)]
    output=Path(output).absolute();output.mkdir(parents=True,exist_ok=False)
    batch=dict(schema='robofrontier.cli-benchmark.v1',status='preparing',created_at=time.time(),
        cwd=str(ROOT),python=sys.executable,context_window=next(iter(profiles.values()))['context_window'],
        auto_compact_token_limit=next(iter(profiles.values()))['auto_compact_token_limit'],
        task_wall_limit_s=86400,model_calls=0,jobs=[])
    write_json(output/'batch.json',batch)
    # Match private Dial instances across model arms of this batch. These are
    # new secrets, never copied from an Oracle or exposed in public inputs.
    puzzles={}
    import secrets
    from robofrontier.task_families import policy_task
    from robofrontier.suite.dial_feedback import alphabet
    for case in selected:
        task=policy_task(case)
        if task.get('family')=='dial':
            target=output/'private/dial'/(case+'.json')
            write_json(target,{'code_symbols':[secrets.choice(alphabet(task)) for _ in task['dials']]})
            puzzles[case]=target
    for name,p in profiles.items():
        config_dir=output/'profiles'/name;config_dir.mkdir(parents=True)
        harness=read(ROOT/'configs/harnesses/policy.json')
        harness.update(model=p['model'],effort=p['effort'],observation_renderer='classic',record_realtime_video=False)
        harness_path=config_dir/'harness.json';write_json(harness_path,harness)
        for case in selected:
            run=output/name/case
            prepare(case,run,harness_path,puzzles.get(case))
            launch=launch_config(p,run/'private/inputs')
            write_json(run/'private/inputs/codex.json',launch)
            # Host config/model catalog belong to immutable prepared inputs too.
            import hashlib
            manifest_path=run/'private/inputs/manifest.json';manifest=read(manifest_path)
            for file in [run/'private/inputs/codex.json',run/'private/inputs/models.json']:
                if file.exists():manifest['input_sha256'][str(file.relative_to(run))]=hashlib.sha256(file.read_bytes()).hexdigest()
            write_json(manifest_path,manifest)
            batch['jobs'].append(dict(profile=name,case_id=case,run_directory=str(run),status='prepared'))
            write_json(output/'batch.json',batch)
    batch['status']='prepared_not_started';write_json(output/'batch.json',batch)
    return output/'batch.json'


def run_batch(folder, parallel):
    folder=Path(folder).resolve();batch=read(folder/'batch.json')
    if batch['status']!='prepared_not_started':raise ValueError('Never automatically restart a batch/world')
    if not 1<=parallel<=36:raise ValueError('Parallel tasks must be 1..36')
    with (folder/'supervisor.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # Another supervisor may have changed the batch before the lock.
        batch=read(folder/'batch.json')
        if batch['status']!='prepared_not_started':raise ValueError('Never automatically restart a batch/world')
        return _run_batch_locked(folder, parallel, batch)


def _run_batch_locked(folder, parallel, batch):
    for job in batch['jobs']:
        run=Path(job['run_directory']);launch=read(run/'private/inputs/codex.json')
        if verify_inputs(run):raise ValueError('Prepared code/inputs changed; prepare a new batch')
        key=launch['profile'].get('env_key')
        if key and not os.environ.get(key):raise ValueError('Missing credential environment variable: '+key)
        receipt=ROOT/'runs/engineering/model-compatibility'/job['profile']/'report.json'
        if not receipt.is_file():raise ValueError('Run benchmark preflight for '+job['profile']+' before scored trials')
        checked=read(receipt)
        expected=__import__('hashlib').sha256(json.dumps(launch['profile'],sort_keys=True).encode()).hexdigest()
        if not checked.get('passed') or checked.get('profile_sha256')!=expected:
            raise ValueError('Model compatibility preflight missing/failed or provider configuration changed')
        catalog_sha=__import__('hashlib').sha256((run/'private/inputs/models.json').read_bytes()).hexdigest()
        version=subprocess.check_output(['codex','--version'],text=True).strip()
        if checked.get('catalog_sha256')!=catalog_sha or checked.get('codex_version')!=version:
            raise ValueError('Model catalog or Codex CLI changed; rerun preflight')
        # Validate configuration without inference before starting any task.
        subprocess.run(['codex','app-server',*launch['args'],'--print-startup-config'],
                       check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    batch.update(status='running',parallel=parallel,started_at=time.time(),model_calls=None,
                 model_calls_note='Per-task native usage events are recorded; aggregate API request count is not measured')
    write_json(folder/'batch.json',batch)
    def task(job):
        run=Path(job['run_directory'])
        if (run/'STOP').exists():return None
        with (run/'private/outputs/supervisor.log').open('a') as log:
            proc=subprocess.Popen(['node',str(ROOT/'scripts/codex_policy_host.js'),str(run),str(ROOT),sys.executable],
                                  cwd=ROOT,stdout=log,stderr=log)
            write_json(run/'private/state/host-process.json',dict(pid=proc.pid,started_at=time.time()))
            return proc.wait()
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        futures={pool.submit(task,job):job for job in batch['jobs']}
        for future in as_completed(futures):
            job=futures[future]
            try:
                result=future.result()
                job.update(status='stopped_before_start' if result is None else 'closed' if result==0 else 'needs_review')
            except Exception as error:job.update(status='needs_review',error=str(error))
            write_json(folder/'batch.json',batch)
    batch.update(status='needs_review' if any(j['status']=='needs_review' for j in batch['jobs']) else 'closed',
                 finished_at=time.time());write_json(folder/'batch.json',batch)


def main():
    load_env(ROOT/'.env')
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('models')
    a=sub.add_parser('preflight');a.add_argument('--models',nargs='+',default=['astra']);a.add_argument('--model-config',type=Path)
    a.add_argument('--smoke-only',action='store_true',help='Small image/tool round trip; does not qualify scored runs')
    for command in ('plan','prepare'):
        a=sub.add_parser(command)
        if command=='prepare':a.add_argument('--output',type=Path,required=True)
        else:a.add_argument('--parallel',type=int,default=4)
        a.add_argument('--models',nargs='+',default=['astra']);a.add_argument('--families',nargs='+')
        a.add_argument('--cases',nargs='+');a.add_argument('--model-config',type=Path)
        a.add_argument('--level',type=int,choices=(1,2,3))
        a.add_argument('--min-arms',action='store_true',help='Smallest released arm count within each selected family')
    a=sub.add_parser('run');a.add_argument('batch',type=Path);a.add_argument('--parallel',type=int,default=9)
    a=sub.add_parser('status');a.add_argument('batch',type=Path)
    a=sub.add_parser('stop');a.add_argument('batch',type=Path)
    args=p.parse_args()
    if args.command=='models':print(json.dumps(read(ROOT/'configs/models.json'),ensure_ascii=False,indent=2))
    elif args.command=='plan':
        catalog=read(args.model_config or ROOT/'configs/models.json')
        if not 1<=args.parallel<=36:raise ValueError('Parallel tasks must be 1..36')
        if not args.models or len(set(args.models))!=len(args.models):raise ValueError('Choose distinct model profiles')
        rows=select_cases(args.families,args.cases,args.level,args.min_arms)
        jobs=[dict(profile=name,model=catalog['profiles'][name]['model'],**row) for name in args.models for row in rows]
        print(json.dumps(dict(model_calls=0,simulator_started=False,parallel=args.parallel,
                              task_count=len(jobs),jobs=jobs),ensure_ascii=False,indent=2))
    elif args.command=='preflight':
        from PIL import Image, ImageDraw
        import hashlib
        for name in args.models:
            selected=profile(name,args.model_config)
            key=selected.get('env_key')
            if key and not os.environ.get(key):raise ValueError('Set '+key+' before live preflight')
            scope='model-smoke' if args.smoke_only else 'model-compatibility'
            target=ROOT/'runs/engineering'/scope/name;target.mkdir(parents=True,exist_ok=True)
            write_json(target/'launch.json',launch_config(selected,target))
            # A failed/interrupted new check must not leave an older pass usable
            # under the new provider/model fingerprint.
            write_json(target/'report.json',{'passed':False,'error':'Preflight did not complete'})
            image=Image.new('RGB',(256,128),'red');ImageDraw.Draw(image).rectangle((128,0,255,127),fill='blue');image.save(target/'colors.png')
            command=['node',str(ROOT/'scripts/codex_preflight.js'),str(target)]
            if args.smoke_only:command.append('--smoke-only')
            result=subprocess.run(command)
            receipt=read(target/'report.json');receipt['profile_sha256']=hashlib.sha256(json.dumps(selected,sort_keys=True).encode()).hexdigest()
            if result.returncode:receipt['passed']=False
            if receipt.get('passed'):
                receipt['catalog_sha256']=hashlib.sha256((target/'models.json').read_bytes()).hexdigest()
                receipt['codex_version']=subprocess.check_output(['codex','--version'],text=True).strip()
            write_json(target/'report.json',receipt)
            if result.returncode:raise SystemExit(result.returncode)
    elif args.command=='prepare':print(prepare_batch(args.output,args.models,args.families,args.cases,args.model_config,args.level,args.min_arms))
    elif args.command=='run':run_batch(args.batch,args.parallel)
    elif args.command=='status':
        batch=read(args.batch/'batch.json')
        for j in batch['jobs']:
            from robofrontier.runtime_store import RuntimeStore
            s=RuntimeStore(Path(j['run_directory'])/'private/state/runtime.sqlite')
            j.update(lifecycle=s.get('lifecycle'),actors={a:{k:v.get(k) for k in ('thread_id','session_status','kind')}
                     for a,v in s.agents().items()});s.close()
        print(json.dumps(batch,ensure_ascii=False,indent=2))
    else:
        for job in read(args.batch/'batch.json')['jobs']:Path(job['run_directory'],'STOP').touch()


if __name__=='__main__':main()
