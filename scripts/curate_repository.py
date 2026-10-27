"""Plan/apply user-requested generated-data cleanup, with pinned-source guards."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from robofrontier.run_setup import write_json


def strings(value):
    if isinstance(value,str):yield value
    elif isinstance(value,dict):
        for v in value.values():yield from strings(v)
    elif isinstance(value,list):
        for v in value:yield from strings(v)


def plan(root):
    root=Path(root).resolve();policy=json.loads((Path(__file__).resolve().parents[1]/'configs/retention.json').read_text())
    audit_start=root/'runs/engineering/cli-migration-cleanup-20261027/worktree-inventory.json'
    cutoff=audit_start.stat().st_mtime_ns if audit_start.exists() else 0
    protected=set()
    def protect(name):
        p=Path(name)
        if not p.is_absolute():p=root/p
        p=p.absolute()
        if p in {root, *(root/x for x in ('runs','outputs','runs/oracle','runs/engineering','docs/generated','configs/evaluations/private'))}:return
        if p.exists() and p.is_relative_to(root):protected.add(p)
    for name in policy['keep']:protect(name)
    for file in (root/'configs/task_families').rglob('*.json'):
        for name in strings(json.loads(file.read_text())):
            if name.startswith(('runs/','outputs/',str(root)+'/')):protect(name)
    # Entire selected attempts include source snapshots, chunks and action/process
    # evidence. Failed sibling attempts are not selected by this operation.
    release=json.loads((root/'configs/task_families/current.json').read_text())
    for f in release['families'].values():
        for c in f['oracle']:
            p=Path(c['recording']);p=p if p.is_absolute() else root/p
            protect(p.parent)
    # Retain old policy runs with substantive archived videos/results, but group
    # them separately in a later explicit migration rather than invalidate paths.
    for p in (root/'runs/policy').iterdir():
        if p.name.startswith('prepared'):continue
        protect(p)
    # All shared archive symlink targets reachable from selected sources survive.
    for p in list(protected):
        candidates=p.rglob('*') if p.is_dir() else [p]
        for x in candidates:
            if x.is_symlink():protect(x.resolve())
    def intersects(p):return any(p==q or p.is_relative_to(q) or q.is_relative_to(p) for q in protected)
    candidates=[]
    def add(p,reason):
        if not p.exists() or intersects(p):return
        # Another authorized session can be rendering in the shared output store.
        # Exclude data newer than this cleanup's initial inventory, recursively.
        if cutoff:
            children=p.rglob('*') if p.is_dir() and not p.is_symlink() else []
            if p.lstat().st_mtime_ns>cutoff or any(x.lstat().st_mtime_ns>cutoff for x in children):return
        candidates.append((p,reason))
    for p in (root/'outputs/robo-factory').iterdir():add(p,'Superseded factory study, renderer calibration or pose cache')
    for p in (root/'outputs/oracle').iterdir():add(p,'Historical Oracle media outside the current delivery selection')
    for p in (root/'runs/oracle').iterdir():add(p,'Unselected historical Oracle run or diagnostic')
    for p in (root/'runs').iterdir():
        if p.name in ('oracle','engineering','policy','state-assets','history'):continue
        # Preserve successful earlier experiments with their existing paths.
        success=False
        for name in ('private/outputs/evaluation.json','report.json'):
            f=p/name
            if f.is_file():
                try:
                    v=json.loads(f.read_text());success=success or v.get('task_success') is True or v.get('passed') is True
                except (ValueError,OSError):pass
        if not success:add(p,'Old failed trial, setup probe or transient engineering run')
    for name in policy['old_engineering_candidates']:add(root/name,'Superseded physical pick/place calibration')
    for p in (root/'runs/policy').glob('prepared*'):add(p,'Stale never-started policy preparation')
    # Keep the current final movie and receipts; intermediate PNGs can be
    # regenerated from the protected source stream and scene.
    current=root/'outputs/robo-factory/v10-orbit16-cycles'
    if policy['remove_factory_frame_cache']:
        for p in current.glob('frame-*.png'):candidates.append((p,'Encoded final movie frame cache'))
    rows=[]
    for p,reason in candidates:
        st=p.lstat()
        size=sum(x.stat().st_size for x in p.rglob('*') if x.is_file() and not x.is_symlink()) if p.is_dir() else st.st_size
        rows.append(dict(path=str(p.relative_to(root)),reason=reason,logical_bytes=size,
                         inode=st.st_ino,mtime_ns=st.st_mtime_ns))
    return dict(schema='robofrontier.retention-plan.v1',root=str(root),
        protected=[str(p.relative_to(root)) for p in sorted(protected)],remove=rows,
        logical_remove_bytes=sum(r['logical_bytes'] for r in rows),
        scope='Logical bytes may share APFS/hardlinks; not a promise of physical disk space reclaimed')


def apply(file):
    data=json.loads(Path(file).read_text());root=Path(data['root']).resolve()
    fresh=plan(root);allowed={r['path'] for r in fresh['remove']}
    for row in data['remove']:
        p=root/row['path']
        if row['path'] not in allowed or not p.is_relative_to(root):raise ValueError('Retention boundary changed: '+str(p))
        st=p.lstat()
        if (st.st_ino,st.st_mtime_ns)!=(row['inode'],row['mtime_ns']):raise ValueError('Candidate changed after review: '+str(p))
    # Validate all candidates before deleting any. Do not follow directory links.
    removed=[]
    for row in data['remove']:
        p=root/row['path']
        if p.is_dir() and not p.is_symlink():shutil.rmtree(p)
        else:p.unlink()
        removed.append(row)
    write_json(Path(file).with_name('removed.json'),dict(removed=removed,logical_bytes=sum(r['logical_bytes'] for r in removed)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--apply',action='store_true');a=p.parse_args()
    if a.apply:apply(a.plan)
    else:
        result=plan(a.root);write_json(a.plan,result)
        print(json.dumps(dict(candidates=len(result['remove']),logical_gib=result['logical_remove_bytes']/2**30)))


if __name__=='__main__':main()
