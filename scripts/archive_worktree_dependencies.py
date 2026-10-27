"""Preserve live historical aliases in a stable, content-deduplicated archive.

Only symlink targets change. Original file bytes and recorded source path strings
stay intact. Missing historical targets are reported, never reconstructed by guess.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def archive(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    baseline = json.loads((output/'worktree-links-before.json').read_text())
    objects = output/'dependency-objects'
    objects.mkdir(exist_ok=True)
    inode_cache = {}
    copied = {}
    records = []
    receipt = output/'dependency-aliases.json'
    previous = {r['path']: r for r in json.loads(receipt.read_text())} if receipt.exists() else {}

    def copy_file(source, destination):
        source, destination = Path(source), Path(destination)
        stat = source.stat()
        identity = (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
        checksum = inode_cache.get(identity)
        if checksum is None:
            digest = hashlib.sha256()
            with source.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(block)
            checksum = inode_cache[identity] = digest.hexdigest()
        blob = objects/checksum
        if not blob.exists():
            temporary = objects/(checksum+'.tmp')
            shutil.copyfile(source, temporary)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != checksum:
                raise ValueError('Source changed during dependency archival: '+str(source))
            temporary.replace(blob)
        destination.parent.mkdir(parents=True, exist_ok=True)
        os.link(blob, destination)
        copied[str(destination.relative_to(output))] = checksum
        return str(destination)

    for row in baseline:
        link = root/row['path']
        prior = previous.get(row['path'])
        if prior and prior.get('status') == 'archived' and link.is_symlink() and str(link.readlink()) == prior['stable_target']:
            records.append(prior)
            continue
        if not link.is_symlink() or str(link.readlink()) != row['target']:
            raise ValueError('Alias changed after inventory: '+str(link))
        target = Path(row['target'])
        if not target.exists():
            records.append(dict(row, status='historical_target_unavailable'))
            continue
        branch = target.parts[target.parts.index('worktrees')+1]
        name = target.name
        if name in ('runs', 'outputs', 'vendor'):
            destination = target.resolve()
            if not destination.is_relative_to(root):
                raise ValueError('Persistent data target outside the repository')
        else:
            destination = output/'dependencies'/branch/name
            if not destination.exists():
                if target.is_dir():
                    shutil.copytree(target, destination, copy_function=copy_file, symlinks=True,
                                    ignore=shutil.ignore_patterns('__pycache__', '.DS_Store', '*.pyc'))
                    for base, dirs, files in os.walk(destination, followlinks=False):
                        for nested_name in dirs+files:
                            nested = Path(base)/nested_name
                            if nested.is_symlink():
                                stable = nested.resolve(strict=True)
                                if not stable.is_relative_to(root):
                                    raise ValueError('Nested dependency is not persistent: '+str(nested))
                                nested.unlink()
                                nested.symlink_to(stable, target_is_directory=stable.is_dir())
                else:
                    copy_file(target, destination)
        records.append(dict(row, stable_target=str(destination), status='archived'))
        # Persist the intended rewrite before atomically replacing the alias.
        (output/'dependency-aliases.json').write_text(json.dumps(records, indent=2)+'\n')
        temporary = link.with_name(link.name+'.integration-link')
        temporary.symlink_to(destination, target_is_directory=destination.is_dir())
        temporary.replace(link)
    result = dict(schema='robofrontier.dependency-archive.v1', aliases=records,
                  archived_files=copied, unique_contents=len(set(copied.values())),
                  unresolved=sum(r['status']!='archived' for r in records),
                  note='Unresolved targets were already absent at baseline; original aliases retained.')
    (output/'dependency-archive.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(aliases=len(records), archived=len(records)-result['unresolved'],
                         unresolved=result['unresolved'], unique_contents=result['unique_contents'])))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    archive(args.root, args.output)
