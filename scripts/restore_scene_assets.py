"""Restore ignored workstation assets by their recorded SHA256; never regenerate."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def restore(root=ROOT):
    restored = reused = 0
    missing = []
    for manifest in sorted((root/'configs/environments').rglob('workstation.json')):
        for asset in json.loads(manifest.read_text()).get('assets', []):
            target = manifest.parent/asset['path']
            if target.is_file():
                if hashlib.sha256(target.read_bytes()).hexdigest() != asset['sha256']:
                    raise ValueError('Asset changed: '+str(target))
                reused += 1
                continue
            source = root/'runs/state-assets'/asset['sha256']
            if not source.is_file():
                missing.append(str(target.relative_to(root)))
                continue
            if hashlib.sha256(source.read_bytes()).hexdigest() != asset['sha256']:
                raise ValueError('Asset cache changed: '+str(source))
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            restored += 1
    return dict(restored=restored, reused=reused, unavailable=missing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report = restore()
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print('restored', report['restored'], 'reused', report['reused'],
          'unavailable', len(report['unavailable']))
    if report['unavailable']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
