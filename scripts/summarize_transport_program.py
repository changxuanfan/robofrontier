"""Summarize proof scope, physical acceptance, failures and verified videos."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from robofrontier.cube_transport import ROOT


def summarize(directory):
    rows, media = [], []
    for count in range(3, 13):
        name = f'line-{count:02}' if count == 3 else f'ring-{count:02}'
        plan = json.loads((directory/name/'schedule.json').read_text())
        attempts = []
        for path in sorted((directory/name).glob('attempt-*/report.json')):
            report = json.loads(path.read_text())
            scene = ROOT/f'configs/environments/cube_transport/{name}/scene.xml'
            if hashlib.sha256(scene.read_bytes()).hexdigest() != report['scene_sha256']:
                raise ValueError('Scene changed: '+name)
            task = json.loads((ROOT/f'configs/tasks/cube_transport/{name}/seed-0.json').read_text())
            if json.loads((path.parent/'task.json').read_text()) != task:
                raise ValueError('Task changed: '+name)
            calls = 0
            for line in (path.parent/'trace.jsonl').read_text().splitlines():
                event = json.loads(line)
                if event['event'] == 'moves':
                    calls += len(event['targets'])
                elif event['event'] == 'grippers':
                    calls += len(event['commands'])
                elif event['event'] == 'gentle_release':
                    calls += len(event['robots'])
            videos = []
            for video in sorted(path.parent.glob('*.mp4')):
                info = json.loads(subprocess.check_output([shutil.which('ffprobe'), '-v', 'error',
                    '-show_entries', 'format=duration,size', '-show_entries', 'stream=width,height,r_frame_rate',
                    '-of', 'json', str(video)], text=True))
                item = {'path': str(video.relative_to(directory)), 'acceptance_passed': report['passed'],
                        'metadata': info, 'sha256': hashlib.sha256(video.read_bytes()).hexdigest()}
                videos.append(item['path']); media.append(item)
            attempts.append({'path': str(path.relative_to(directory)), 'passed': report['passed'],
                             'error': report.get('error'), 'evaluation': report.get('evaluation'),
                             'physical_batches': report.get('physical_batches'),
                             'arm_command_segments': calls, 'sim_time_s': report['sim_time_s'],
                             'peak_moving_arms': report['peak_moving_arms'], 'videos': videos})
        accepted = [r for r in attempts if r['passed']]
        rows.append({'arms': count, 'serial_unit_phases': plan['serial_phase_slots'],
                     'optimal_unit_rounds_fixed_routes': plan['parallel_feasible_phase_slots'],
                     'optimality_proven': plan['optimality_proven'], 'proof_scope': plan['proof_scope'],
                     'attempts': attempts, 'accepted_attempt': accepted[-1] if accepted else None})
    (directory/'index.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2)+'\n')
    (directory/'media-verification.json').write_text(json.dumps(media, indent=2)+'\n')
    lines = ['# 无 GPT 程序参考：离散最优与接触仿真', '',
             '使用 OR-Tools CP-SAT 求解固定最短路线、单位抓／放阶段下的最少轮数，十个规模均证明 OPTIMAL，并通过独立资源 replay 检查。此证明不涵盖路线替换、连续运动时间或真实机器人全局最优。', '',
             '物理执行由确定性程序调用机器人 IK 和位置执行器。程序读取物体真值，使用释放前校正、连续夹爪位置、运动试算及必要的错峰，属于工程可行性参考，不能直接当作与 RGB Agent 同观测／同接口的性能基线。',
             '场景、物料与物理参数没有修改，场景 SHA 和任务内容逐项核对。所有失败尝试保留，成功只依据整场稳定目标验收。', '',
             '| 臂数 | 串行单位阶段 | 离散最优轮数 | 物理验收 | 实际物理分组 |',
             '| ---: | ---: | ---: | --- | ---: |']
    for row in rows:
        accepted = row['accepted_attempt']
        status = f"通过 {accepted['evaluation']['matched_goals']}/{3*row['arms']}" if accepted else '未取得整场通过记录'
        groups = accepted['physical_batches'] if accepted else '—'
        lines.append(f"| {row['arms']} | {row['serial_unit_phases']} | {row['optimal_unit_rounds_fixed_routes']} | {status} | {groups} |")
    lines += ['', '实际物理分组会因几何冲突错开而多于离散轮数；每个单位阶段还需多个移动／夹爪段，不能与工具调用次数或 MuJoCo 积分步混为一谈。真实耗时未证明最优，仿真时间与程序墙钟时间分开记录。', '',
              '## 已通过的视频', '']
    for item in media:
        if item['acceptance_passed']:
            duration = float(item['metadata']['format']['duration'])
            lines.append(f"- [{item['path']}]({item['path']})，{duration:.1f} 秒。")
    lines += ['', 'reference-* 为执行时采样录像；replay-* 为已验收物理轨迹的离线渲染，逐帧选取真实保存的 qpos，不插值编造物体运动。各文件名和画面注明倍速；20 fps 输出，按仿真时间而非推理墙钟计时。', '',
              '## 未通过记录', '']
    for row in rows:
        for attempt in row['attempts']:
            if not attempt['passed']:
                lines.append(f"- {attempt['path']}：{attempt['error'] or '未通过／中断'}。")
    lines += ['', '球体组的已观察问题包括持物掉落和释放后漂移；即使某次全部运输执行结束，也必须满足全体目标稳定一秒才能算通过。failure-replay 文件始终标注失败，不混入成功视频。参考程序自身尚未通过的任务，当前不能用于将失败归因于模型智能，应先处理物理执行问题。', '',
              '## 复现', '',
              '求解器依赖单独安装在本运行 solver-deps 内，版本见 solver-packages.json；没有加入项目默认依赖。执行器源码每次保存在 attempt/executor-source.py，最终源码另在 final-source；初版源码在 frozen-source。', '',
              '从仓库根目录运行 scripts.solve_transport_program、scripts.render_transport_program、scripts.render_transport_replay 的 --help 查看参数。新尝试使用新的 attempt 编号，不能覆写旧记录。', '']
    (directory/'report.md').write_text('\n'.join(lines))
    print(json.dumps({'discrete_optimal_proofs': sum(r['optimality_proven'] for r in rows),
                      'accepted_arm_counts': [r['arms'] for r in rows if r['accepted_attempt']],
                      'verified_media_files': len(media)}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    summarize(args.directory.resolve())
