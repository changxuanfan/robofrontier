"""Generate offline 3..12-arm scenes, order seeds, references and RGB previews.

Run from repository root: .venv/bin/python -m scripts.generate_cube_transport
No server, session, policy, or robot action bridge is started.
"""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.cube_transport import (ROOT, COLLECTION, _font, add_initial_keyframe, build_xml,
                                       initialize, layout, make_task, reference,
                                       replay_reference)
from robofrontier.sim import RobotIK
from robofrontier.cube_transport_presentation import annotate, startup_context, startup_markdown


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def supported(env, position, margin=.021):
    # Footprints may straddle coplanar strip seams; test the union, not one box.
    return all(any(all(abs(position[i]+offset[i]-t['center_m'][i]) <= t['half_size_m'][i]+1e-8
                       for i in range(2)) for t in env['tables'])
               for offset in ((x, y) for x in (-margin, 0, margin) for y in (-margin, 0, margin)))


def validate(model, data, env):
    report = {'environment': env['id'], 'mujoco_version': mujoco.__version__,
              'arms': env['arm_count'], 'free_objects': len(env['objects']),
              'cameras': model.ncam, 'ik': [], 'checks': {},
              'scope': 'load, support, point IK and endpoint contacts with peers home, home contacts, 1 s settle; no transport rollout'}
    positions = [(o['id'], o['initial_position_m']) for o in env['objects']]
    positions += [(s['id']+'_goal'+str(i+1), p) for s in env['stations'] for i, p in enumerate(s['goal_positions_m'])]
    positions += [(s['id']+'_parking'+str(i+1), p) for s in env['stations'] for i, p in enumerate(s['parking_positions_m'])]
    positions += [(e['id'], e['position_m']) for e in env['edges']]
    unsupported = [name for name, p in positions if not supported(env, p)]
    report['checks']['table_support'] = {'passed': not unsupported, 'unsupported': unsupported}
    closest = min((max(abs(p[0]-q[0]), abs(p[1]-q[1])), a, b)
                  for (a, p), (b, q) in itertools.combinations(positions, 2))
    report['checks']['distinct_cube_footprints'] = {'passed': closest[0] >= .05-1e-8,
        'minimum_chebyshev_center_distance_m': closest[0], 'closest_pair': list(closest[1:]),
        'scope': 'axis aligned 40 mm cubes at marked centers; excludes gripper and swept volume'}
    endpoint = mujoco.MjData(model)
    endpoint.qpos[:] = data.qpos
    for station in env['stations']:
        arm = station['id']; ik = RobotIK(model, arm+'_')
        home = data.qpos[ik.qids].copy(); _, rotation = ik.pose(home)
        targets = [('source', p) for p in station['source_positions_m']]
        targets += [('goal', p) for p in station['goal_positions_m']]
        targets += [('parking', p) for p in station['parking_positions_m']]
        targets += [('handoff_'+e['id'], e['position_m']) for e in env['edges'] if arm in e['stations']]
        for kind, position in targets:
            q = home.copy()
            # Verify both a hovering approach and the fingertip gripping height.
            for z in (.64, .42):
                target = np.array([*position[:2], z])
                q, error = ik.solve(target, rotation, q, 180)
                _, actual_r = ik.pose(q)
                angle = float(np.arccos(np.clip((np.trace(rotation.T@actual_r)-1)/2, -1, 1)))
                endpoint.qpos[:] = data.qpos
                endpoint.qpos[ik.qids] = q
                mujoco.mj_forward(model, endpoint)
                collisions = []
                for contact in endpoint.contact:
                    if contact.dist >= -.001:
                        continue
                    a, b = (model.body(model.geom_bodyid[g]).name for g in (contact.geom1, contact.geom2))
                    if a.startswith(arm+'_') or b.startswith(arm+'_'):
                        collisions.append({'body1': a, 'body2': b, 'depth_m': float(-contact.dist)})
                report['ik'].append({'arm': arm, 'kind': kind, 'target_m': target.tolist(),
                                      'position_error_m': error, 'rotation_error_rad': angle,
                                      'endpoint_penetrations': collisions,
                                      'passed': error < .002 and angle < .02 and not collisions})
    unexpected = []
    for contact in data.contact:
        if contact.dist >= -.001:
            continue
        a, b = (model.body(model.geom_bodyid[g]).name for g in (contact.geom1, contact.geom2))
        unexpected.append({'body1': a, 'body2': b, 'depth_m': float(-contact.dist)})
    report['checks']['initial_penetrations'] = {'passed': not unexpected, 'contacts': unexpected}
    initial = np.array([data.body(o['id']).xpos.copy() for o in env['objects']])
    for _ in range(500):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    final = np.array([data.body(o['id']).xpos.copy() for o in env['objects']])
    drift = float(np.max(np.linalg.norm(initial-final, axis=1)))
    tcp_error = max(float(np.linalg.norm(data.site(s['id']+'_tcp').xpos-s['home_tcp_m'])) for s in env['stations'])
    report['checks']['passive_settle'] = {'passed': drift < .003 and tcp_error < .01 and np.isfinite(data.qpos).all().item(),
                                          'duration_sim_s': 1, 'max_object_drift_m': drift,
                                          'max_home_tcp_error_m': tcp_error}
    report['passed'] = all(x['passed'] for x in report['checks'].values()) and all(x['passed'] for x in report['ik'])
    return report


def generate(counts, render=True):
    base = ROOT/'benchmarks'/COLLECTION
    (base/'previews').mkdir(parents=True, exist_ok=True)
    rows, manifest, summaries, task_overview = [], [], [], []
    for count in counts:
        env = layout(count)
        print('Generating '+env['id'], flush=True)
        directory = ROOT/'configs/environments'/COLLECTION/env['id']
        directory.mkdir(parents=True, exist_ok=True)
        env.update(scene='scene.xml', initial_keyframe='initial')
        write_json(directory/'environment.json', env)
        xml = build_xml(env, directory)
        scene = directory/'scene.xml'
        ET.indent(xml)
        ET.ElementTree(xml).write(scene, encoding='unicode')
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = initialize(model, env)
        add_initial_keyframe(xml, model, data)
        ET.indent(xml); ET.ElementTree(xml).write(scene, encoding='unicode')
        # Check the final on-disk artifact, including the saved reset state.
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id)
        mujoco.mj_forward(model, data)
        report = validate(model, data, env)
        write_json(base/'validation'/f"{env['id']}.json", report)
        summaries.append({k: report[k] for k in ('environment', 'arms', 'free_objects', 'cameras', 'passed')})
        context = startup_context(env, model, data)
        write_json(directory/'startup.json', context)
        (directory/'startup.md').write_text(startup_markdown(env, context))
        if render:
            with mujoco.Renderer(model, height=900, width=1280) as renderer:
                for view in ('global', 'top', 'A_front'):
                    renderer.update_scene(data, camera=view)
                    raw_image = Image.fromarray(renderer.render())
                    raw_image.save(base/'previews'/f"{env['id']}-{view}-raw.png")
                    image, metadata = annotate(raw_image, model, data, env, view)
                    image.save(base/'previews'/f"{env['id']}-{view}.png")
                    write_json(base/'previews'/f"{env['id']}-{view}.json", metadata)
                    if view in ('global', 'top'):
                        assert len(metadata['robot_labels']) == count, 'Global camera misses a robot base'
                        assert not any(v['overlap_area_pixel2'] for v in metadata['robot_labels']), 'Overlapping robot labels'
        for obsolete in (base/'references').glob(f"{env['id']}-seed*.json"):
            obsolete.unlink()
        for seed in (0, 1, 2):
            task = make_task(env, seed)
            task['environment_config'] = str((directory/'environment.json').relative_to(ROOT))
            task_path = ROOT/'configs/tasks'/COLLECTION/env['id']/f'seed-{seed}.json'
            write_json(task_path, task)
            if seed == 0:
                kinds = {(o['color'], o['shape']): o['type_code'] for o in env['objects']}
                task_overview += [f"## {count} 臂：{task['id']}", '', '```text']
                task_overview += [a+' ← '+', '.join(kinds[(g['requires']['color'], g['requires']['shape'])] for g in goals)
                                  for a, goals in task['orders'].items()]
                task_overview += ['```', '', '全场最少交接次数 → 需求数量：'+
                                  ', '.join(f'{h} → {v}' for h, v in task['distance_distribution']['hop_counts'].items())+'。', '']
            result = reference(env, task)
            replay_reference(env, task, result)
            result['schedule_replay_passed'] = True
            if seed == 0:
                colors = {'red': '红', 'blue': '蓝', 'green': '绿', 'orange': '橙', 'purple': '紫', 'cyan': '青'}
                task_overview += ['| 机械臂 | 槽 1 | 槽 2 | 槽 3 | 本臂参考执行阶段数 |',
                                  '| --- | --- | --- | --- | ---: |']
                for a, goals in task['orders'].items():
                    names = [colors[g['requires']['color']]+('方块' if g['requires']['shape'] == 'cube' else '圆柱') for g in goals]
                    task_overview.append('| '+a+' | '+' | '.join(names)+' | '+str(result['arm_phase_load'][a])+' |')
                task_overview += ['', '本臂阶段数包含接力工作；串行与该并行参考中执行总量相同，改变的是同时执行与等待。', '']
            write_json(base/'references'/f"{task['id']}.json", result)
            rows.append({'environment': env['id'], 'arms': count, 'seed': seed,
                         'objects': 3*count, 'serial_phase_slots': result['serial_phase_slots'],
                         'parallel_feasible_phase_slots': result['parallel_feasible_phase_slots'],
                         'lower_bound_phase_slots': result['lower_bound_phase_slots_for_fixed_routes'],
                         'optimality_proven': result['optimality_proven'],
                         'reference_speedup': round(result['serial_phase_slots']/result['parallel_feasible_phase_slots'], 3),
                         **{f'hop_{h}_count': result['hop_histogram'].get(h, 0) for h in range(7)}})
        manifest.append({'environment': env['id'], 'config': str((directory/'environment.json').relative_to(ROOT)),
                         'scene': str(scene.relative_to(ROOT)), 'scene_sha256': hashlib.sha256(scene.read_bytes()).hexdigest(),
                         'seeds': [0, 1, 2], 'validation_passed': report['passed']})
        failed_ik = [x for x in report['ik'] if not x['passed']]
        print(f"  passed={report['passed']} IK failures={len(failed_ik)} checks={report['checks']}", flush=True)
    with (base/'references/summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    sources = ['robofrontier/cube_transport.py', 'robofrontier/cube_transport_presentation.py',
               'scripts/generate_cube_transport.py', 'scripts/verify_transport_objects.py', 'robofrontier/sim.py']
    write_json(base/'manifest.json', {'schema': 4, 'rules_version': 4, 'environments': manifest,
        'source_sha256': {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources},
        'robot_source': json.loads((ROOT/'vendor/panda/SOURCE.json').read_text())})
    write_json(base/'validation/summary.json', summaries)
    comparison = ['| 臂数 | 串行阶段数 | 并行参考轮数 | 固定路线下界 | 参考加速比 |',
                  '| ---: | ---: | ---: | ---: | ---: |']
    comparison += [f"| {r['arms']} | {r['serial_phase_slots']} | {r['parallel_feasible_phase_slots']} | {r['lower_bound_phase_slots']} | {r['reference_speedup']} |"
                   for r in rows if r['seed'] == 0]
    counting = ['## 串行与并行计数', '',
                '一个阶段是抓起并清区，或者放下并清区。一次工位内抓放有两个阶段；h 次交接有 h+1 次工位访问，因此总阶段数为 2×Σ(h+1)。',
                '串行每轮一个阶段；并行每轮可有多个互不冲突的阶段。工作总量并未减少。表中不是 MuJoCo 积分步、工具调用数或实际秒数。',
                '并行参考为已验证的可行上界；只有达到固定路线下界才证明该抽象模型下最优。未达到者，真实最优轮数介于下界与参考之间。', '',
                *comparison, '', '下方给出当前 seed 0 的完整订单，以及固定参考路线下包含接力的各臂工作量。', '', '']
    (base/'tasks.md').write_text('\n'.join([
        '# 任务清单：规则 v4 / seed 0', '',
        '大写字母是工位，小写字母是物料类型；箭头右侧依次对应槽位 1、2、3，不规定执行顺序。',
        'a～f 为红、蓝、绿、橙、紫、青方块；g～l 为同一顺序的六色圆柱。',
        '每个存在的工位初始三件对应类型；同色同形物品可互换。',
        '3～6 臂保留约定订单；7～12 臂在全场统计上拉平各最少交接距离的需求量。',
        f'seed 1、2 的完整订单见 configs/tasks/{COLLECTION} 对应环境目录，汇总见 references/summary.csv。',
        '这份清单是人工查阅材料，策略启动仅应接收本次选定的任务文件。', '', *counting, *task_overview]))
    if render:
        sheet = Image.new('RGB', (1600, ((len(counts)+1)//2)*415), (243, 246, 249))
        draw = ImageDraw.Draw(sheet)
        for i, count in enumerate(counts):
            name = layout(count)['id']; x, y = (i % 2)*800, (i//2)*415
            draw.text((x+20, y+8), f'{count} arms / {name}', font=_font(25), fill=(25, 43, 67))
            image = Image.open(base/'previews'/f'{name}-global.png')
            image.thumbnail((790, 365)); sheet.paste(image, (x+(800-image.width)//2, y+43))
        sheet.save(base/'previews/overview.jpg', quality=92)
    if not all(r['passed'] for r in summaries):
        raise SystemExit('Some engineering checks failed; see validation files. No policy trial was run.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--counts', type=int, nargs='+', default=list(range(3, 13)))
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()
    generate(args.counts, not args.no_render)
