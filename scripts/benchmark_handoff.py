"""Inspect a checkout and transfer only the runtime assets needed by policy tasks.

Source code travels through a Git branch/bundle. This archive contains no .env,
Codex login, benchmark results, prepared worlds or private Dial answers.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile

from robofrontier.task_families import ROOT, RELEASE, read
from scripts.benchmark import load_env, profile
from scripts.tasks import check_case_assets


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def scene_assets(root=ROOT):
    expected={}
    for family in read(root/RELEASE)['families'].values():
        for case in family['gpt']:
            directory=Path(case['directory'])
            for asset in read(root/directory/'workstation.json').get('assets', []):
                relative=PurePosixPath(asset['path'])
                if relative.is_absolute() or '..' in relative.parts:
                    raise ValueError('Unsafe released asset path')
                name=(directory/asset['path']).as_posix()
                if name in expected and expected[name]!=asset['sha256']:
                    raise ValueError('Conflicting asset hashes: '+name)
                expected[name]=asset['sha256']
    return expected


def panda_assets(source):
    manifest=source/'vendor/panda/SOURCE.json'
    source_info=read(manifest)
    expected={'vendor/panda/SOURCE.json':digest(manifest)}
    for name,checksum in source_info['sha256'].items():
        path=PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Unsafe Panda source path')
        expected['vendor/panda/'+name]=checksum
    return expected


def export_assets(source, output, root=ROOT):
    source,output=Path(source).resolve(),Path(output).absolute()
    expected=scene_assets(root)
    expected.update(panda_assets(source))
    sources={}
    for name,checksum in expected.items():
        path=source/name
        if not path.is_file() and name.startswith('configs/'):
            path=source/'runs/state-assets'/checksum
        if not path.is_file() or digest(path)!=checksum:
            raise ValueError('Missing/changed source asset: '+name)
        sources[name]=path
    manifest=dict(schema='robofrontier.policy-assets.v1',release_sha256=digest(root/RELEASE),files=expected)
    output.parent.mkdir(parents=True,exist_ok=True)
    # Exclusive creation prevents replacing a previously delivered artifact.
    with output.open('xb') as raw,tarfile.open(fileobj=raw,mode='w:gz') as archive:
        content=(json.dumps(manifest,sort_keys=True,indent=2)+'\n').encode()
        entry=tarfile.TarInfo('assets-manifest.json');entry.size=len(content)
        archive.addfile(entry,io.BytesIO(content))
        for name,path in sources.items():
            with path.open('rb') as stream:
                entry=tarfile.TarInfo(name);entry.size=path.stat().st_size;entry.mode=0o644
                archive.addfile(entry,stream)
    return dict(archive=str(output),sha256=digest(output),files=len(expected),bytes=output.stat().st_size)


def restore_assets(archive_path, root=ROOT):
    root=Path(root).resolve()
    with tarfile.open(archive_path,'r:gz') as archive,tempfile.TemporaryDirectory(prefix='rf-assets-') as temporary:
        members=archive.getmembers()
        if any(not m.isfile() for m in members):raise ValueError('Archive must contain regular files only')
        names=[m.name for m in members]
        if len(set(names))!=len(names):raise ValueError('Duplicate archive members')
        manifest=json.load(archive.extractfile('assets-manifest.json'))
        if manifest.get('schema')!='robofrontier.policy-assets.v1' or manifest['release_sha256']!=digest(root/RELEASE):
            raise ValueError('Asset archive belongs to a different task release')
        expected=scene_assets(root)
        for name,checksum in expected.items():
            if manifest['files'].get(name)!=checksum:raise ValueError('Scene asset manifest mismatch: '+name)
        panda=json.load(archive.extractfile('vendor/panda/SOURCE.json'))
        for name in panda['sha256']:
            relative=PurePosixPath(name)
            if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe Panda source path')
        expected.update({'vendor/panda/'+name:checksum for name,checksum in panda['sha256'].items()})
        expected['vendor/panda/SOURCE.json']=manifest['files']['vendor/panda/SOURCE.json']
        if expected!=manifest['files'] or set(names)!=set(expected)|{'assets-manifest.json'}:
            raise ValueError('Unexpected archive contents')
        staged=Path(temporary);pending=[];reused=0
        # Verify every byte and every existing destination before writing any file.
        for name,checksum in expected.items():
            relative=PurePosixPath(name)
            if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe archive path')
            path=staged/name;path.parent.mkdir(parents=True,exist_ok=True)
            with archive.extractfile(name) as stream,path.open('wb') as output:shutil.copyfileobj(stream,output)
            if digest(path)!=checksum:raise ValueError('Corrupt archive asset: '+name)
            destination=root/name
            if destination.exists():
                if not destination.is_file() or digest(destination)!=checksum:raise ValueError('Destination changed: '+name)
                reused+=1
            else:
                if not destination.resolve().is_relative_to(root):raise ValueError('Destination outside checkout: '+name)
                pending.append((path,destination))
        for source,destination in pending:
            destination.parent.mkdir(parents=True,exist_ok=True)
            with source.open('rb') as stream,destination.open('xb') as output:shutil.copyfileobj(stream,output)
    return dict(restored=len(pending),reused=reused,release_sha256=digest(root/RELEASE))


def doctor(names, root=ROOT):
    load_env(root/'.env')
    tools={}
    for tool,args in [('git',['--version']),('node',['--version']),('codex',['--version']),('ffmpeg',['-version'])]:
        binary=shutil.which(tool)
        if binary:
            result=subprocess.run([binary,*args],capture_output=True,text=True,timeout=15)
            tools[tool]=dict(available=result.returncode==0,version=(result.stdout or result.stderr).splitlines()[0])
        else:tools[tool]=dict(available=False)
    profiles={}
    for name in names:
        raw=read(root/'configs/models.json')['profiles'][name]
        key=raw.get('env_key')
        entry=dict(model=raw['model'],credential_env=key,credential_set=bool(os.environ.get(key)) if key else None)
        try:profile(name);entry['configuration_valid']=True
        except ValueError as error:entry.update(configuration_valid=False,error=str(error))
        profiles[name]=entry
    assets=[check_case_assets(case,root) for family in read(root/RELEASE)['families'].values() for case in family['gpt']]
    return dict(tools=tools,profiles=profiles,task_release_sha256=digest(root/RELEASE),
                assets=dict(missing=sum(len(a['missing']) for a in assets),changed=sum(len(a['changed']) for a in assets)),
                panda_manifest_available=(root/'vendor/panda/SOURCE.json').is_file(),
                note='Read-only readiness report; no network/model/physics test. Credential values are never printed.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('doctor');p.add_argument('--models',nargs='+',default=['kimi','qwen','glm','deepseek'])
    p=sub.add_parser('assets');p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('restore');p.add_argument('archive',type=Path)
    sub.add_parser('init-env')
    args=parser.parse_args()
    if args.command=='doctor':result=doctor(args.models)
    elif args.command=='assets':result=export_assets(args.source,args.output)
    elif args.command=='restore':result=restore_assets(args.archive)
    else:
        path=ROOT/'.env'
        fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as stream:stream.write((ROOT/'.env.example').read_text())
        result=dict(created=str(path),permissions='600',contains_credentials=False)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
