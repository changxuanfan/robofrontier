"""Discard reproducible animation PNGs only after successful video validation."""
import json
from pathlib import Path


def prune_frames(output, report):
    output = Path(output).resolve()
    animation = report.get('animation', {})
    if animation.get('full_decode_passed') is not True:
        raise ValueError('Video must pass full decoding before removing frames')
    if not (output / 'factory.mp4').is_file() or not (output / 'factory.mp4').stat().st_size:
        raise ValueError('Validated video is missing')
    rows = report['views']
    count = animation['frames']
    if count != len(rows) or not count:
        raise ValueError('Frame manifest differs')
    files = [output / f'frame-{i:06d}.png' for i in range(count)]
    if any(p.is_symlink() or not p.is_file() for p in files):
        raise ValueError('Complete regular PNG sequence required before cleanup')
    keep = {0, count // 2, count - 1}
    stills = output / 'stills'
    if stills.exists():
        raise FileExistsError('Preserve existing stills; choose a fresh output')
    removed_bytes = sum(p.stat().st_size for i, p in enumerate(files) if i not in keep)
    report['frame_storage'] = dict(status='pruning', kept_ordinals=sorted(keep),
        removed_frames=count-len(keep), reclaimed_bytes=removed_bytes)
    manifest = output / 'report.json'
    manifest.write_text(json.dumps(report, indent=2) + '\n')
    stills.mkdir()
    for i, p in enumerate(files):
        if i in keep:
            target = stills / p.name
            p.rename(target)
            rows[i]['retained_file'] = str(target)
        else:
            p.unlink()
            rows[i]['retained_file'] = None
    report['frame_storage']['status'] = 'complete'
    manifest.write_text(json.dumps(report, indent=2) + '\n')
    return report['frame_storage']
