"""Freeze the Oracle executor before importing mutable family references.

A one-time exec preserves the runner PID and signal supervision. Data directories
remain aliases to revisioned source assets; canonical asset hashes are verified
separately by Workstation/MotionWriter. This is reproducibility, not a sandbox.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid

MARKER='ROBOFRONTIER_ORACLE_FROZEN_ROOT'


def freeze_executor(root,destination,harness,catalog):
    root=Path(root).resolve();destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=False)
    hashes={}
    for package in ('robofrontier','scripts'):
        for source in sorted((root/package).rglob('*.py')):
            relative=source.relative_to(root);payload=source.read_bytes();target=destination/relative
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(payload)
            hashes[str(relative)]=hashlib.sha256(payload).hexdigest()
    (destination/'inputs').mkdir()
    for name,source in (('harness',harness),('catalog',catalog)):
        target=destination/'inputs'/f'{name}.json';payload=Path(source).read_bytes();json.loads(payload)
        target.write_bytes(payload);hashes[f'inputs/{name}.json']=hashlib.sha256(payload).hexdigest()
    for name in ('configs','vendor','assets','runs','outputs','docs','.agents','config.json'):
        source=root/name
        if source.exists():(destination/name).symlink_to(source,target_is_directory=source.is_dir())
    manifest=dict(schema='robofrontier.oracle-source.v1',source_root=str(root),created_unix_s=time.time(),
        files=hashes,scope='Frozen Python/config/catalog; versioned scene assets verified separately; no physical checkpoint')
    (destination/'snapshot.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return destination


def replace_option(argv,key,value):
    remaining=[];index=0
    while index<len(argv):
        argument=argv[index]
        if argument==key:index+=2;continue
        if argument.startswith(key+'='):index+=1;continue
        remaining.append(argument);index+=1
    return remaining+[key,str(value)]


def reexec_entry(root,output_root,entrypoint,argv,harness,catalog,cli_config=False):
    """Also supports a run-local commissioning entrypoint, frozen by bytes."""
    root=Path(root).resolve()
    if os.environ.get(MARKER)==str(root):return
    output=Path(output_root).resolve()
    target=output.parent/'_executor_launches'/('launch-'+str(os.getpid())+'-'+uuid.uuid4().hex[:10])
    frozen=freeze_executor(root,target,harness,catalog)
    entrypoint=Path(entrypoint).resolve()
    try:relative=entrypoint.relative_to(root)
    except ValueError:relative=Path('inputs/entry.py')
    if relative.parts[0] not in ('robofrontier','scripts'):relative=Path('inputs/entry.py')
    payload=entrypoint.read_bytes();(frozen/relative).write_bytes(payload)
    manifest_path=frozen/'snapshot.json';manifest=json.loads(manifest_path.read_text())
    manifest['files'][str(relative)]=hashlib.sha256(payload).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    for name in manifest['files']:(frozen/name).chmod(0o444)
    manifest_path.chmod(0o444)
    if cli_config:
        argv=replace_option(argv,'--harness',frozen/'inputs/harness.json')
        argv=replace_option(argv,'--catalog',frozen/'inputs/catalog.json')
    environment=os.environ.copy();environment[MARKER]=str(frozen);environment['PYTHONPATH']=str(frozen)
    environment['ROBOFRONTIER_ORACLE_FROZEN_HARNESS']=str(frozen/'inputs/harness.json')
    environment['ROBOFRONTIER_ORACLE_FROZEN_CATALOG']=str(frozen/'inputs/catalog.json')
    os.execve(sys.executable,[sys.executable,str(frozen/relative),*argv],environment)


def reexec_frozen(root,args):
    root=Path(root).resolve()
    if os.environ.get(MARKER)==str(root):return
    output=Path(args.output_root).resolve() if args.output_root else root/'runs/oracle/frontier-v2'
    harness=Path(args.harness).resolve() if args.harness else root/'configs/harness-frontier-v2.json'
    argv=list(sys.argv[1:])
    if args.output_root:argv=replace_option(argv,'--output-root',output)
    reexec_entry(root,output,root/'scripts/verify_frontier_v2.py',argv,harness,Path(args.catalog).resolve(),cli_config=True)
