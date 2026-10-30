"""Losslessly archive stopped benchmark evidence; never deletes source evidence."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
import zlib

IMAGE = re.compile(rb'(data:image/[^;,\s"\\]+;base64,)([A-Za-z0-9+/=]+)')
REF = re.compile(rb'(data:image/[^;,\s"\\]+;base64,)@RF_IMAGE_([a-f0-9]{64})@')
MARKER = b'@RF_IMAGE_'
B64_MARKER = b'@RF_B64_'
B64_REF = re.compile(rb'"@RF_B64_([a-f0-9]{64})@"')
BLOCK = 1024 * 1024


def checksum(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda: f.read(BLOCK), b''):
            h.update(part)
    return h.hexdigest()


def safe(root, name):
    p = PurePosixPath(name)
    if p.is_absolute() or not p.parts or any(x in ('..', '.', '') for x in name.split('/')):
        raise ValueError('Unsafe archive path: ' + name)
    return root.joinpath(*p.parts)


def put(root, source, objects):
    """Stream and hash source; compressed objects are addressed by raw SHA-256."""
    h = hashlib.sha256(); compressor = zlib.compressobj(6); size = 0
    with tempfile.NamedTemporaryFile(dir=root, delete=False) as tmp:
        name = Path(tmp.name)
        try:
            with source.open('rb') as f:
                for part in iter(lambda: f.read(BLOCK), b''):
                    h.update(part); size += len(part); tmp.write(compressor.compress(part))
            tmp.write(compressor.flush())
        except BaseException:
            name.unlink(missing_ok=True)
            raise
    key = h.hexdigest(); dest = root / 'objects' / key[:2] / key
    try:
        if key not in objects:
            dest.parent.mkdir(parents=True, exist_ok=True)
            os.replace(name, dest)
            objects[key] = {'size': size, 'stored_size': dest.stat().st_size, 'stored_sha256': checksum(dest)}
    finally:
        name.unlink(missing_ok=True)
    return key


def read_object(root, manifest, key):
    if not re.fullmatch(r'[a-f0-9]{64}', key): raise ValueError('Invalid object key')
    meta = manifest['objects'][key]; path = root/'objects'/key[:2]/key
    if checksum(path) != meta['stored_sha256']: raise ValueError('Corrupt stored object ' + key)
    data = zlib.decompress(path.read_bytes())
    if len(data) != meta['size'] or hashlib.sha256(data).hexdigest() != key:
        raise ValueError('Corrupt decoded object ' + key)
    return data


def image_strings(value):
    """Only recognize explicit MCP/Anthropic image structures, never arbitrary data."""
    if isinstance(value, list):
        for item in value:
            yield from image_strings(item)
    elif isinstance(value, dict):
        if value.get('type') == 'image':
            if isinstance(value.get('mimeType'), str) and value['mimeType'].startswith('image/') and isinstance(value.get('data'), str):
                yield value['data']
            source = value.get('source')
            if isinstance(source, dict) and source.get('type') == 'base64' and str(source.get('media_type', '')).startswith('image/') and isinstance(source.get('data'), str):
                yield source['data']
        for item in value.values():
            yield from image_strings(item)


def image_object(data, root, objects):
    with tempfile.NamedTemporaryFile(dir=root, delete=False) as image:
        img = Path(image.name); image.write(data)
    try:
        return put(root, img, objects)
    finally:
        img.unlink(missing_ok=True)


def transform_session(source, root, objects):
    # A model may quote this very implementation in its transcript. Never
    # reinterpret its literal marker as an archive reference on restore.
    with source.open('rb') as src:
        if any(MARKER in line or B64_MARKER in line for line in src):
            return None
    tmp = tempfile.NamedTemporaryFile(dir=root, delete=False)
    path = Path(tmp.name)
    try:
        with tmp, source.open('rb') as src:
            for line in src:
                def replace(m):
                    try:
                        data = base64.b64decode(m[2], validate=True)
                    except ValueError:
                        return m[0]
                    if base64.b64encode(data) != m[2]: return m[0]
                    key = image_object(data, root, objects)
                    return m[1] + MARKER + key.encode() + b'@'
                # Discover typed fields using JSON, but replace the literal bytes.
                # Re-serializing JSON would change formatting and provenance hashes.
                try:
                    values = set(image_strings(json.loads(line)))
                except (ValueError, UnicodeError):
                    values = set()  # Preserve interrupted/unknown records unchanged.
                for value in sorted(values):
                    try:
                        encoded = value.encode('ascii')
                        data = base64.b64decode(encoded, validate=True)
                    except (ValueError, UnicodeError):
                        continue
                    if not data or base64.b64encode(data) != encoded or b'"'+encoded+b'"' not in line:
                        continue  # Escaped/noncanonical encodings retain their raw bytes.
                    key = image_object(data, root, objects)
                    line = line.replace(b'"'+encoded+b'"', b'"'+B64_MARKER+key.encode()+b'@"')
                tmp.write(IMAGE.sub(replace, line))
        return path
    except BaseException:
        path.unlink(missing_ok=True); raise


def verify_collection(source, allow_incomplete=False):
    report = json.loads((source/'collection.json').read_text())
    generic = report.get('schema') == 'robofrontier.policy-collection.v1'
    if not generic and report.get('schema') != 'robofrontier.luna-collection.v1':
        raise ValueError('Unknown collection schema')
    if not report.get('complete') and not (generic and allow_incomplete):
        raise ValueError('A complete collection is required; stopped generic evidence can use --allow-incomplete')
    batch = json.loads((source/'batch'/'batch.json').read_text())
    if generic:
        state = json.loads((source/'batch'/'engineering-status.json').read_text()) if report.get('engine') == 'claude' else batch
        terminal = ('closed', 'needs_review', 'stopped', 'stopped_before_start')
        if report.get('writers_stopped') is not True or state.get('status') not in terminal or any(j.get('status') not in terminal for j in state.get('jobs', [])):
            raise ValueError('Evidence writers must be stopped before archival')
        expected = report.get('files')
        if not isinstance(expected, dict) or not expected:
            raise ValueError('Collection file inventory is missing')
        actual = {p.relative_to(source).as_posix() for p in source.rglob('*') if p.is_file()} - {'collection.json'}
        if actual != set(expected):
            raise ValueError('Collection inventory changed')
        for name, checksum_expected in expected.items():
            if checksum(safe(source, name)) != checksum_expected:
                raise ValueError('Collection file changed: '+name)
    elif batch.get('status') != 'closed' or any(j.get('status') != 'closed' for j in batch.get('jobs', [])):
        raise ValueError('Only closed legacy Luna batches are supported')
    for job in report['jobs']:
        for role in job['roles']:
            p = safe(source, role['session'])
            if not p.is_file() or checksum(p) != role['sha256']:
                raise ValueError('Missing or changed session: ' + str(p))
    return report


def create(source, output, allow_incomplete=False):
    source = source.resolve(); output = output.resolve()
    if output.exists() or output.is_relative_to(source): raise ValueError('Output must be new and outside collection')
    collection_hash = checksum(source/'collection.json')
    report = verify_collection(source, allow_incomplete)
    if checksum(source/'collection.json') != collection_hash:
        raise ValueError('Collection manifest changed during validation')
    expected = dict(report.get('files', {}))
    expected['collection.json'] = collection_hash
    for job in report['jobs']:
        for role in job['roles']:
            expected[role['session']] = role['sha256']
    generic = report['schema'] == 'robofrontier.policy-collection.v1'
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix='.rf-archive-', dir=output.parent)); objects = {}; records = []; dirs = []
    try:
        for parent, names, files in os.walk(source, followlinks=False):
            names.sort(); files.sort(); parent = Path(parent)
            for name in names:
                if (parent/name).is_symlink(): raise ValueError('Symlinks are not supported: ' + str(parent/name))
            dirs.append({'path': parent.relative_to(source).as_posix(), 'mode': stat.S_IMODE(parent.stat().st_mode)})
            for name in files:
                p = parent/name; before = p.lstat(); rel = p.relative_to(source).as_posix()
                if not stat.S_ISREG(before.st_mode): raise ValueError('Non-regular file: ' + str(p))
                original = checksum(p); session = rel.endswith('.jsonl')
                if (generic and rel not in expected) or (rel in expected and original != expected[rel]):
                    raise ValueError('Collection file changed after validation: '+rel)
                transformed = transform_session(p, tmp, objects) if session else p
                if transformed is None:
                    session = False
                    transformed = p
                try: key = put(tmp, transformed, objects)
                finally:
                    if session: transformed.unlink(missing_ok=True)
                after = p.lstat()
                if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino) or checksum(p) != original:
                    raise ValueError('Source changed during archival: ' + str(p))
                records.append({'path': rel, 'object': key, 'transform': 'jsonl_images_v2' if session else 'identity',
                                'sha256': original, 'size': before.st_size, 'mode': stat.S_IMODE(before.st_mode), 'mtime_ns': before.st_mtime_ns})
        recorded = {item['path']:item['sha256'] for item in records}
        current = {p.relative_to(source).as_posix() for p in source.rglob('*') if p.is_file()}
        if set(recorded) != current or generic and set(recorded) != set(expected):
            raise ValueError('Collection inventory changed during archival')
        for rel, original in recorded.items():
            p = safe(source, rel)
            if p.is_symlink() or checksum(p) != original:
                raise ValueError('Collection changed during archival: '+rel)
        if checksum(source/'collection.json') != collection_hash or verify_collection(source, allow_incomplete) != report:
            raise ValueError('Collection manifest changed during archival')
        manifest = {'schema': 'robofrontier.policy-archive.v1', 'objects': objects, 'files': records, 'directories': dirs,
                    'collection_complete': report['complete'], 'collection_schema': report['schema'], 'job_count': report['job_count'],
                    'note': 'Private evidence. Image data restored losslessly; source paths are not rewritten.'}
        (tmp/'manifest.json').write_text(json.dumps(manifest, separators=(',', ':'))+'\n')
        verify(tmp, None)
        if output.exists(): raise FileExistsError(output)
        os.rename(tmp, output)
        print(json.dumps({'archive': str(output), 'files': len(records), 'original_bytes': sum(x['size'] for x in records), 'archive_bytes': sum(p.stat().st_size for p in output.rglob('*') if p.is_file())}))
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True); raise


def _verify(root, output):
    root = root.resolve(); manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema') != 'robofrontier.policy-archive.v1': raise ValueError('Unknown archive schema')
    if output:
        output = output.resolve()
    seen = set()
    for item in manifest['files']:
        safe(Path('/validation'), item['path'])
        if item['path'] in seen:
            raise ValueError('Duplicate archive file path')
        seen.add(item['path'])
    for directory in manifest['directories']:
        if directory['path'] != '.': safe(Path('/validation'), directory['path'])
    for item in manifest['files']:
        if item['transform'] not in ('session', 'jsonl_images_v2', 'identity'): raise ValueError('Unknown transform')
        safe(Path('/validation'), item['path'])
        key = item['object']
        if not re.fullmatch(r'[a-f0-9]{64}', key): raise ValueError('Invalid object key')
        meta = manifest['objects'][key]; path = root/'objects'/key[:2]/key
        if checksum(path) != meta['stored_sha256']: raise ValueError('Corrupt stored object: ' + key)
        # Stream large files to a temporary file: memory use does not scale with
        # the size of an entire video, database or session.
        with tempfile.TemporaryFile() as decoded:
            h = hashlib.sha256(); stored = hashlib.sha256(); size = 0; decompressor = zlib.decompressobj()
            with path.open('rb') as src:
                for part in iter(lambda: src.read(BLOCK), b''):
                    stored.update(part)
                    while part:
                        data = decompressor.decompress(part, BLOCK)
                        part = decompressor.unconsumed_tail
                        h.update(data); size += len(data); decoded.write(data)
                data = decompressor.flush(); h.update(data); size += len(data); decoded.write(data)
            if not decompressor.eof or decompressor.unused_data or stored.hexdigest() != meta['stored_sha256'] or h.hexdigest() != key or size != meta['size']:
                raise ValueError('Corrupt object: ' + key)
            decoded.seek(0); digest = hashlib.sha256(); restored_size = 0
            target = None
            try:
                if output:
                    dest = safe(output, item['path']); dest.parent.mkdir(parents=True, exist_ok=True)
                    target = dest.open('xb')
                iterator = decoded if item['transform'] in ('session', 'jsonl_images_v2') else iter(lambda: decoded.read(BLOCK), b'')
                for part in iterator:
                    if item['transform'] in ('session', 'jsonl_images_v2'):
                        part = REF.sub(lambda m: m[1] + base64.b64encode(read_object(root, manifest, m[2].decode())), part)
                    if item['transform'] == 'jsonl_images_v2':
                        part = B64_REF.sub(lambda m: b'"'+base64.b64encode(read_object(root, manifest, m[1].decode()))+b'"', part)
                    digest.update(part); restored_size += len(part)
                    if target: target.write(part)
            finally:
                if target: target.close()
            if restored_size != item['size'] or digest.hexdigest() != item['sha256']:
                raise ValueError('Restored checksum mismatch: ' + item['path'])
            if output:
                dest.chmod(item['mode']); os.utime(dest, ns=(item['mtime_ns'], item['mtime_ns']))
    if output:
        for d in reversed(manifest['directories']):
            path = output if d['path'] == '.' else safe(output, d['path'])
            path.mkdir(parents=True, exist_ok=True); path.chmod(d['mode'])
    return {'verified_files': len(manifest['files']), 'restored': bool(output)}


def verify(root, output=None):
    """Publish a restore only when all file hashes pass; remove partial output."""
    root = Path(root).resolve()
    if output is None:
        return _verify(root, None)
    output = Path(output).resolve()
    if output.exists() or output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError('Restore output must be new and outside archive')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.rf-restore-', dir=output.parent))
    try:
        result = _verify(root, temporary)
        if output.exists(): raise FileExistsError(output)
        temporary.rename(output)
        return result
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main():
    os.umask(0o077)
    p = argparse.ArgumentParser(description=__doc__); sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('create'); c.add_argument('collection', type=Path); c.add_argument('--output', required=True, type=Path)
    c.add_argument('--allow-incomplete', action='store_true', help='Preserve stopped evidence marked incomplete; never qualifies its score')
    v = sub.add_parser('verify'); v.add_argument('archive', type=Path)
    r = sub.add_parser('restore'); r.add_argument('archive', type=Path); r.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    if a.command == 'create': create(a.collection, a.output, a.allow_incomplete)
    else: print(json.dumps(verify(a.archive, a.output if a.command == 'restore' else None)))


if __name__ == '__main__': main()
