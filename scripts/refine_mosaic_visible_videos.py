"""Refine all nine state-based edits by removing visually stationary scenes."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np

from scripts.mosaic_motion_mask import frame_intervals
from scripts.mosaic_visual_motion import visual_mask, visible_frame_selection, longest_still_seconds


def load(path):
    return json.loads(Path(path).read_text())


def save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def refine(folder, workers=3):
    folder = Path(folder).resolve()
    backup = folder / '第一版去静止备份'
    backup.mkdir(exist_ok=True)
    index_file = backup / 'video-index.json'
    if not index_file.exists():
        index = load(folder / 'video-index.json')
        if index.get('variant') != 'idle_removed' or index.get('ready') != 9:
            raise ValueError('Require the completed nine state-based edits')
        for row in index['videos']:
            destination = backup / Path(row['file']).name
            if not destination.exists():
                os.link(row['file'], destination)
            row['file'] = str(destination)
        save(index_file, index)
    index = load(index_file)
    records = folder / '静止裁剪记录'
    batch = records / ('visual-v2-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
    batch.mkdir()

    def one(old):
        work = batch / old['case_id']
        work.mkdir()
        source = Path(old['file'])
        print('VISUAL_SCAN_START', old['label'], flush=True)
        active = visual_mask(source)
        keep = visible_frame_selection(active)
        if len(keep) != old['kept_frames']:
            raise ValueError('Input frame count mismatch')
        old_mask = np.load(Path(old['work_directory']) / 'motion-mask.npz')
        original_indices = np.flatnonzero(old_mask['video_keep'])
        combined = np.zeros_like(old_mask['video_keep'])
        combined[original_indices[keep]] = True
        np.savez_compressed(work / 'motion-mask.npz', video_keep=combined,
                            video_source_indices=old_mask['video_source_indices'],
                            source_time_s=old_mask['source_time_s'],
                            visual_active=active, input_video_keep=keep)
        ranges = frame_intervals(keep)
        terms = [f'between(n,{a},{b})' if a != b else f'eq(n,{a})' for a, b in ranges]
        filters = work / 'select-filter.txt'
        filters.write_text("select='" + '+'.join(terms) + "',setpts=N/(30*TB)\n")
        output = work / 'visible-motion.mp4'
        command = ['ffmpeg', '-v', 'error', '-threads', '1', '-filter_threads', '1',
                   '-i', str(source), '-filter_script:v', str(filters), '-an',
                   '-c:v', 'libx264', '-threads', '1', '-preset', 'fast', '-crf', '20',
                   '-pix_fmt', 'yuv420p', '-r', '30', '-fps_mode', 'cfr',
                   '-movflags', '+faststart', str(output)]
        print('VISUAL_ENCODE', old['label'], len(keep), '->', int(keep.sum()), flush=True)
        subprocess.run(command, check=True)
        subprocess.run(['ffmpeg', '-v', 'error', '-threads', '1', '-i', str(output),
                        '-threads', '1', '-f', 'null', '-'], check=True)
        probe = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-count_frames', '-show_entries',
            'stream=codec_type,width,height,nb_read_frames:format=duration',
            '-of', 'json', str(output)], text=True))
        stream = next(x for x in probe['streams'] if x['codec_type'] == 'video')
        if int(stream['nb_read_frames']) != int(keep.sum()):
            raise ValueError('Encoded frame count mismatch')
        if (stream['width'], stream['height']) != (1280, 800):
            raise ValueError('Encoded dimensions changed')
        final_active = visual_mask(output)
        longest = longest_still_seconds(final_active)
        if longest > .5:
            raise ValueError(f'Residual stationary run exceeds .5s: {longest}')
        preview = work / 'exact-final.png'
        subprocess.run(['ffmpeg', '-v', 'error', '-threads', '1', '-filter_threads', '1',
                        '-sseof', '-0.5', '-i', str(output), '-vf', 'reverse',
                        '-frames:v', '1', '-threads', '1', '-y', str(preview)], check=True)
        destination = folder / source.name
        duration = float(probe['format']['duration'])
        retained_reported = None
        if old['case_id'] == 'mosaic-a08-12x24-d1':
            retained_reported = int(keep[5100:5310].sum())
            if retained_reported > 6:
                raise ValueError('Reported 2:50–2:57 stationary span still retained')
        row = dict(old, file=str(destination), state='ready', work_directory=str(work),
                   duration_s=duration, kept_frames=int(keep.sum()),
                   removed_fraction=1 - int(keep.sum()) / len(combined),
                   removed_seconds=old['original_duration_s'] - duration,
                   decode_passed=True, final_preview=str(preview),
                   manifest=str(work / 'manifest.json'),
                   sha256=hashlib.sha256(output.read_bytes()).hexdigest())
        row.pop('encoding_progress', None)
        manifest = dict(
            kind='Visible-scene stationary wait removal after state-based edit',
            algorithm_version=2, source_video=str(source),
            source_video_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            source_trial=old['source_trial'], source_frames=old['source_frames'],
            file=str(destination), duration_s=duration, fps=30, speedup=60,
            original_duration_s=old['original_duration_s'],
            original_video_frames=len(combined), input_video_frames=len(keep),
            kept_video_frames=int(keep.sum()),
            visible_scene=dict(overlay_excluded_px=60, analysis_width=640,
                               analysis_height=370, color_channels='RGB',
                               channel_delta=20, changed_pixels=4,
                               reference='Last significant change; slow differences accumulate'),
            all_detected_visible_changes_preserved=bool(keep[active].all()),
            initial_and_final_states_preserved=bool(keep[0] and keep[-1]),
            longest_remaining_visually_still_s=longest,
            reported_old_span_kept_frames=retained_reported,
            decode_passed=True, encoded_frame_count=int(keep.sum()),
            video_sha256=row['sha256'], final_preview=str(preview),
            context='One encoded frame before/after each visible change, plus endpoints',
            physics_restarted=False,
            scope='Presentation only; subpixel or occluded motion may be omitted. Original full trajectories and videos remain intact.')
        save(work / 'manifest.json', manifest)
        print('VISUAL_READY', old['label'], round(duration, 2),
              'residual_idle_s', round(longest, 3), flush=True)
        return row, output, manifest

    with ThreadPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(one, index['videos']))
    # Publish only once all nine pass. Each old input remains in backup.
    for row, output, _ in results:
        temporary = folder / (Path(row['file']).name + '.tmp')
        shutil.copy2(output, temporary)
        temporary.replace(row['file'])
    current = dict(index, variant='visible_idle_removed', updated_at=datetime.now().astimezone().isoformat(),
                   videos=[row for row, _, _ in results], ready=9, status='complete',
                   previous_edit_backup=str(backup), visual_review='pending',
                   verification=str(batch / 'verification.json'))
    current.pop('all_exact_final_frames_visually_reviewed', None)
    save(folder / 'video-index.json', current)
    save(batch / 'verification.json', dict(
        passed=True, files=9, cases=[manifest for _, _, manifest in results],
        visual_review='pending', reported_span_regression_passed=True))
    print('ALL_NINE_VISIBLE_EDITS_READY', batch, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=3, choices=range(1, 4))
    args = parser.parse_args()
    refine(args.folder, args.workers)
