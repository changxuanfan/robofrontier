"""Verify protected historical bytes with inode-aware hashing; never repair data."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

from robofrontier.task_families import ROOT,RELEASE,read
from robofrontier.run_setup import write_json


def check_historical():
    cache={};reports=[]
    def digest(path):
        stat=path.stat();key=(stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns)
        if key not in cache:
            h=hashlib.sha256()
            with path.open('rb')as source:
                for block in iter(lambda:source.read(1024*1024),b''):h.update(block)
            cache[key]=h.hexdigest()
        return cache[key]
    mosaic=read(ROOT/'runs/engineering/mosaic-merge-cleanup-20261027/protected-files.json')['files']
    transport=json.loads(gzip.decompress((ROOT/'runs/oracle/transport-continuation-20261026/engineering/transport-cleanup-20261027/protected-before.json.gz').read_bytes()))
    removal=read(ROOT/'runs/oracle/transport-continuation-20261026/engineering/transport-cleanup-20261027/removed.json')
    retired=[Path(r['path'])for r in removal['removed']]
    relay={r['path']:r for r in read(ROOT/'runs/engineering/relay-cleanup-20261027/manifest.json')['protected_evidence']}
    for family,files,branch in [('mosaic',mosaic,'codex/mosaic-continuation'),('transport',transport,'codex/transport-continuation'),('relay',relay,'codex/relay-continuation')]:
        failed=[];verified=0;previously_removed=[];versioned=[]
        for name,row in files.items():
            path=ROOT/Path(name);expected=row['sha256']
            if not path.is_file():
                if family=='transport' and (any(path==r or path.is_relative_to(r)for r in retired) or '__pycache__'in path.parts):
                    previously_removed.append(name);continue
                failed.append(dict(path=name,error='missing'));continue
            if digest(path)!=expected:
                relative=path.relative_to(ROOT)
                if relative.parts[0]not in ('runs','outputs') and not str(relative).startswith('configs/environments/'):
                    old=subprocess.run(['git','show',branch+':'+str(relative)],capture_output=True)
                    if old.returncode==0 and hashlib.sha256(old.stdout).hexdigest()==expected:
                        versioned.append(dict(path=str(relative),preserved_in=branch));continue
                failed.append(dict(path=name,error='changed'));continue
            verified+=1
        reports.append(dict(family=family,verified_files=verified,failed=failed,
                            previously_removed_by_family_cleanup=previously_removed,
                            current_source_changes_preserved_in_git=versioned))
        print(family,'verified',verified,'failed',len(failed),flush=True)
    dial=read(ROOT/'configs/task_families/dial/best5-r11.json')
    pins=[(dial['selection'],dial['selection_sha256'])]
    for row in dial['cases']:
        pins.extend((str(Path(row['source_directory'])/name),expected)for name,expected in row['source_sha256'].items())
        evidence=row['oracle_evidence']
        pins.extend([(evidence['report'],evidence['report_sha256']),
                     (str(Path(evidence['recording'])/'stream.json'),evidence['motion_index_sha256'])])
    for row in read(ROOT/'runs/engineering/dial-video-edit-20261027/edit-manifest.json')['cases']:
        pins.append((row['source'],row['source_sha256']))
    failed=[dict(path=name,error='missing_or_changed')for name,expected in pins
            if not (ROOT/name).is_file() or digest(ROOT/name)!=expected]
    reports.append(dict(family='dial',verified_files=len(pins)-len(failed),failed=failed,
        scope='Pinned selected reports, source metadata, stream indices, selection and original rendered videos; raw chunks also verified while generating station edits'))
    print('dial','verified',len(pins)-len(failed),'failed',len(failed),flush=True)
    bindings=[]
    for family in read(ROOT/RELEASE)['families'].values():
        for row in family['oracle']:
            reader=read(Path(row['recording'])/'stream.json')
            source=Path(row['directory'])
            for name,expected in row['source_files'].items():
                if digest(source/name)!=expected:raise ValueError('Frozen source changed')
            bindings.append(dict(case_id=row['id'],recording=row['recording'],
                frames=reader['frames'],scene_sha256=reader['source_scene_sha256']))
    return dict(passed=not any(r['failed']for r in reports),families=reports,
                source_bindings=bindings,unique_inodes_hashed=len(cache),physics_steps=0)


def check():
    """Current retained dataset; retired historical trials are no longer pins."""
    snapshot=ROOT/'outputs/final/preservation.json'
    if not snapshot.exists():return check_historical()
    inventory=read(snapshot)['files'];failed=[];cache={}
    for name,row in inventory.items():
        p=ROOT/name
        if not p.is_file():failed.append(dict(path=name,error='missing'));continue
        st=p.stat();key=(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)
        if key not in cache:
            h=hashlib.sha256()
            with p.open('rb')as source:
                for block in iter(lambda:source.read(1024*1024),b''):h.update(block)
            cache[key]=h.hexdigest()
        if cache[key]!=row['sha256']:failed.append(dict(path=name,error='changed'))
    bindings=[]
    from robofrontier.task_families import resolve_case
    for f in read(ROOT/RELEASE)['families'].values():
        for c in f['oracle']:
            resolve_case(c['id'],'oracle')
            bindings.append(c['id'])
    return dict(passed=not failed,verified_files=len(inventory)-len(failed),failed=failed,
                source_bindings=bindings,physics_steps=0,
                scope='Current selected sources/final delivery after authorized retirement; use --historical only for the former full archive')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--historical',action='store_true',help='Audit the former full archive, including now-retired experiments')
    args=parser.parse_args();result=check_historical() if args.historical else check();write_json(args.output,result)
    raise SystemExit(0 if result['passed']else 1)
