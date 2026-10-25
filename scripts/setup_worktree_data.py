"""Materialize ignored inputs without sharing writable environment asset files.

Tracked source and scene metadata stay in the checkout. Large model assets use
APFS copy-on-write where available, with ordinary copies as a fallback. Runtime
and result directories are shared external stores; always use unique run IDs.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


def clone(source,target):
    if target.exists() or target.is_symlink():return
    target.parent.mkdir(parents=True,exist_ok=True)
    result=subprocess.run(['cp','-cR',str(source),str(target)],capture_output=True)
    if result.returncode:
        if target.exists():
            if target.is_dir():shutil.rmtree(target)
            else:target.unlink()
        if source.is_dir():shutil.copytree(source,target)
        else:shutil.copy2(source,target)


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--catalog',type=Path,required=True);p.add_argument('--family')
    a=p.parse_args();root=Path(__file__).resolve().parents[1];source=a.source.resolve()
    if root==source:raise ValueError('Run from the destination checkout')
    for name in ('.venv','vendor','runs','outputs','docs/generated','configs/evaluations/private'):
        original=source/name;dest=root/name
        if not original.exists():continue
        if dest.exists() or dest.is_symlink():
            if not dest.is_symlink() or dest.resolve()!=original:raise ValueError('Refusing to replace existing data: '+str(dest))
        else:dest.parent.mkdir(parents=True,exist_ok=True);dest.symlink_to(original,target_is_directory=original.is_dir())
    catalog=json.loads((root/a.catalog).read_text());folders=set()
    for case in catalog['cases']:
        if a.family and case['family']!=a.family:continue
        folder=Path(case['directory']);folders.add(folder)
        params=source/folder/'parameters.json'
        if params.exists():
            parent=json.loads(params.read_text()).get('source_directory')
            if parent:folders.add(Path(parent))
    for folder in sorted(folders):
        if folder.is_absolute() or '..' in folder.parts:raise ValueError('Scene path must be repository relative')
        original=source/folder;dest=root/folder
        if not (dest/'scene.xml').exists():raise ValueError('Tracked scene metadata missing: '+str(dest))
        if (original/'assets').exists():clone(original/'assets',dest/'assets')
        for preview in original.glob('preview-*.png'):clone(preview,dest/preview.name)
    print(json.dumps(dict(checkout=str(root),scenes_materialized=len(folders),asset_mode='Independent COW copies where available; dependency/data stores linked')))


if __name__=='__main__':main()
