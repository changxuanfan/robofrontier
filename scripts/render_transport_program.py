"""Execute saved deterministic schedules through Panda IK/actuators and film.

Engineering oracle reference, not a model policy: live object truth is used
for precise pickup and grasp-offset correction and for private evaluation.
No attachment, teleport, qpos modification after initial reset, or physics skip.
The original runtime service and agent harness are not started or changed.
"""
import argparse
import copy
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.cube_transport import ROOT, _font, evaluate_placement_window, reference_jobs, replay_reference
from robofrontier.cube_transport_presentation import annotate
from robofrontier.recording import Video
from robofrontier.sim import RobotIK


class RobotContact(RuntimeError):
    def __init__(self, pair):
        self.pair = tuple(sorted(pair))
        super().__init__('Cross-robot contact: '+', '.join(self.pair))


def smooth_curve(curve, progress):
    """Natural cubic interpolation prevents velocity jumps at IK knots."""
    n = len(curve); h = 1/(n-1)
    second = np.zeros_like(curve)
    if n > 2:
        matrix = np.diag(np.full(n-2, 4.))
        if n > 3:
            matrix += np.diag(np.ones(n-3), 1)+np.diag(np.ones(n-3), -1)
        second[1:-1] = np.linalg.solve(matrix, 6*(curve[2:]-2*curve[1:-1]+curve[:-2])/h**2)
    scaled = np.asarray(progress)*(n-1)
    index = np.minimum(scaled.astype(int), n-2)
    u = (scaled-index)[:, None]
    return ((1-u)*curve[index]+u*curve[index+1]+h*h/6*
            (((1-u)**3-(1-u))*second[index]+(u**3-u)*second[index+1]))


class ProgramArm:
    """Offline robot-only IK with smooth, bounded joint reference segments."""
    def __init__(self, model, data, name):
        self.name, self.model, self.data = name, model, data
        self.ik = RobotIK(model, name+'_')
        self.qids, self.dofs, self.site = self.ik.qids, self.ik.dofs, self.ik.site
        self.act = np.array([model.actuator(f'{name}_actuator{i}').id for i in range(1, 8)])
        self.grip = model.actuator(name+'_actuator8').id
        self.qref = data.ctrl[self.act].copy()
        _, self.rotation = self.ik.pose(self.qref)
        self.gripper = self.grip_goal = 255.
        self.velocity = np.zeros(7)
        self.goal, self.speed, self.job, self.status = None, .10, None, 'holding'
        self.tcp_ref, self.cartesian_integral = None, np.zeros(3)

    def observe(self):
        return {'tcp_m': self.data.site_xpos[self.site].tolist(),
                'joint_rad': self.data.qpos[self.qids].tolist(),
                'gripper_width_m': sum(float(self.data.joint(self.name+'_'+s).qpos[0]) for s in ('finger_joint1', 'finger_joint2'))}


class Program:
    def __init__(self, count, output, attempt, record, playback, collection='cube_transport', motion_time_scale=1.0):
        if not math.isfinite(motion_time_scale) or motion_time_scale < 1:
            raise ValueError('Motion time scale must be finite and at least one')
        self.motion_time_scale = motion_time_scale
        self.name = f'line-{count:02}' if count == 3 else f'ring-{count:02}'
        self.directory = output/self.name/f'attempt-{attempt:02}'
        self.directory.mkdir(parents=True, exist_ok=False)
        folder = ROOT/f'configs/environments/{collection}/{self.name}'
        self.env = json.loads((folder/'environment.json').read_text())
        self.task = json.loads((ROOT/f'configs/tasks/{collection}/{self.name}/seed-0.json').read_text())
        self.plan = json.loads((output/self.name/'schedule.json').read_text())
        if self.plan.get('collection', 'cube_transport') != collection:
            raise ValueError('Schedule and scene collections differ')
        replay_reference(self.env, self.task, self.plan)
        self.model = mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('initial').id)
        mujoco.mj_forward(self.model, self.data)
        self.arms = {s['id']: ProgramArm(self.model, self.data, s['id']) for s in self.env['stations']}
        self.stations = {s['id']: s for s in self.env['stations']}
        self.objects = {o['id']: o for o in self.env['objects']}
        jobs = reference_jobs(self.env, self.task)
        goals = [g for values in self.task['orders'].values() for g in values]
        self.routes = dict(jobs)
        self.targets = {name: goal['position_m'] for (name, _), goal in zip(jobs, goals)}
        self.edges = {frozenset(e['stations']): e['position_m'] for e in self.env['edges']}
        roots = {self.model.body(a+'_link0').id: i for i, a in enumerate(self.arms)}
        self.owners = np.full(self.model.ngeom, -1)
        for g in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[g])
            while body and body not in roots:
                body = int(self.model.body_parentid[body])
            if body in roots:
                self.owners[g] = roots[body]
        self.offsets = {}
        self.payload_owners = {}
        self.object_geoms = {name: self.model.geom(name+'_geom').id for name in self.objects}
        self.trace = (self.directory/'trace.jsonl').open('w')
        self.tail = deque(maxlen=61)
        self.trajectory, self.times = [], []
        self.video = Video(self.directory/f'reference-{playback:g}x.mp4', 1536, 1152, 20) if record else None
        self.renderer = mujoco.Renderer(self.model, height=900, width=1280) if record else None
        self.playback, self.next_frame = playback, 0.
        self.round, self.action_groups, self.physical_batches = 0, 0, 0
        self.screening, self.screen_checks, self.screen_rejections = False, 0, 0
        self.tick_count, self.max_active = 0, 0
        self.report = {'collection': collection, 'environment': self.name, 'task': self.task['id'], 'passed': False,
                       'scope': 'deterministic oracle engineering reference; no GPT decisions',
                       'object_truth_access': 'pickup targeting, held offset correction, checks and evaluator',
                       'optimality_proven_discrete': self.plan['optimality_proven'],
                       'discrete_rounds': len(self.plan['schedule']),
                       'discrete_total_phases': self.plan['serial_phase_slots'],
                       'physical_time_optimality_proven': False,
                       'motion_time_scale': motion_time_scale,
                       'playback_speed': playback, 'video_clock': 'simulation time / playback_speed',
                       'scene_sha256': hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest(),
                       'collision_scope': 'robot and held-payload cross-owner contacts checked every 2 ms physics step; stop at contact',
                       'mutations': 'actuator ctrl only after the initial keyframe reset'}
        self.report['joint_reference'] = {'profile': 'Cartesian lines / IK waypoints / quintic timing', 'joint_speed_bound_rad_s': .5,
                                          'joint_acceleration_bound_rad_s2': 1, 'tcp_endpoint_speed_design_m_s': .10,
                                          'robot_gravity_bias_feedforward': True}
        (self.directory/'executor-source.py').write_text(Path(__file__).read_text())
        (self.directory/'task.json').write_text(json.dumps(self.task, indent=2))
        (self.directory/'environment.json').write_text(json.dumps(self.env, indent=2))
        (self.directory/'schedule.json').write_text(json.dumps(self.plan, indent=2))

    def event(self, kind, **values):
        if self.screening:
            return
        self.trace.write(json.dumps({'event': kind, 'sim_time_s': float(self.data.time), **values})+'\n')
        self.trace.flush()

    def sample(self):
        support, robot_contact = set(), set()
        for c in self.data.contact:
            for g, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                name = self.model.body(self.model.geom_bodyid[g]).name
                if name not in self.objects:
                    continue
                if (self.model.geom(other).name or '').startswith('table_'):
                    support.add(name)
                if self.owners[other] >= 0:
                    robot_contact.add(name)
        states = {}
        for name in self.objects:
            velocity = np.zeros(6)
            body = self.model.body(name).id
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
            states[name] = {'position_m': self.data.body(name).xpos.tolist(),
                            'linear_velocity_m_s': velocity[3:].tolist(),
                            'table_supported': name in support, 'released': name not in robot_contact}
        self.tail.append({'time_s': float(self.data.time), 'objects': states})

    def frame(self):
        if self.screening:
            return
        self.times.append(float(self.data.time)); self.trajectory.append(self.data.qpos.copy())
        if not self.renderer:
            return
        self.renderer.update_scene(self.data, camera='global')
        image, _ = annotate(Image.fromarray(self.renderer.render()), self.model, self.data, self.env, 'global')
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, image.width, 45), fill=(23, 48, 75))
        draw.text((16, 8), f'PROGRAM / ORACLE | {self.name} | round {self.round}/{len(self.plan["schedule"])} | sim {self.data.time:.1f}s | {self.playback:g}x',
                  font=_font(22), fill='white')
        if self.video.error:
            raise RuntimeError(self.video.error)
        # Block the offline encoder rather than drop frames.
        self.video.queue.put(np.asarray(image).tobytes(), timeout=30)

    def tick(self):
        for arm in self.arms.values():
            arm.gripper += float(np.clip(arm.grip_goal-arm.gripper, -255*.02, 255*.02))
            self.data.ctrl[arm.grip] = arm.gripper
            bias = self.data.qfrc_bias[arm.dofs]/self.model.actuator_gainprm[arm.act, 0]
            self.data.ctrl[arm.act] = np.clip(arm.qref+bias, self.model.actuator_ctrlrange[arm.act, 0],
                                             self.model.actuator_ctrlrange[arm.act, 1])
        self.max_active = max(self.max_active, sum(a.goal is not None for a in self.arms.values()))
        owners = self.owners.copy()
        names = list(self.arms)
        for obj, arm in self.payload_owners.items():
            owners[self.object_geoms[obj]] = names.index(arm)
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
            contacts = self.data.contact
            if len(contacts):
                first, second = owners[contacts.geom1], owners[contacts.geom2]
                hit = np.flatnonzero((first >= 0) & (second >= 0) & (first != second) & (contacts.dist < 0))
                if len(hit):
                    c = contacts[int(hit[0])]
                    raise RobotContact((names[owners[c.geom1]], names[owners[c.geom2]]))
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError('Nonfinite physics state')
        self.tick_count += 1
        self.sample()
        if self.data.time+1e-9 >= self.next_frame:
            self.frame(); self.next_frame += self.playback/20

    def hold(self, seconds):
        for _ in range(round(seconds/.02)):
            self.tick()

    def moves(self, points):
        self.action_groups += 1
        self.event('moves', targets=points)
        segments = {}
        duration = 1.2
        for name, point in points.items():
            arm = self.arms[name]
            start = arm.qref.copy()
            start_position, _ = arm.ik.pose(start)
            distance = float(np.linalg.norm(np.array(point)-start_position))
            knots = max(2, math.ceil(distance/.008)+1)
            curve = [start]
            for progress in np.linspace(0, 1, knots)[1:]:
                target = start_position+(np.array(point)-start_position)*progress
                q, error = arm.ik.solve(target, arm.rotation, curve[-1], 80)
                if error > .002:
                    raise RuntimeError(f'Unreachable Cartesian reference: {name} {target}, {error}')
                curve.append(q)
            curve = np.array(curve)
            largest = float(np.max(np.abs(curve[-1]-start)))
            duration = max(duration, largest*1.875/.5, math.sqrt(6*largest), distance*1.875/.10)
            segments[name] = curve
            arm.goal = np.array(point)
        duration *= self.motion_time_scale
        # Time-scale the sampled references until finite-difference joint speed
        # and acceleration fit the declared bounds, including segment ends.
        for _ in range(8):
            ticks = math.ceil(duration/.02)
            t = np.linspace(0, 1, ticks+1)
            blends = 10*t**3-15*t**4+6*t**5
            references = {name: smooth_curve(curve, blends) for name, curve in segments.items()}
            ratio = 1.
            for name, values in references.items():
                velocity = np.diff(values, axis=0)/.02
                acceleration = np.diff(np.vstack((np.zeros((1, 7)), velocity, np.zeros((1, 7)))), axis=0)/.02
                ratio = max(ratio, float(np.max(np.abs(velocity)))/.5, math.sqrt(float(np.max(np.abs(acceleration)))))
            if ratio <= 1.00001:
                break
            duration *= ratio*1.02
        else:
            raise RuntimeError('Joint reference time-scaling failed')
        self.event('trajectory_profile', duration_sim_s=ticks*.02, reference_limit_ratio=ratio)
        for index in range(1, ticks+1):
            for name, values in references.items():
                arm = self.arms[name]
                arm.qref = values[index].copy()
            self.tick()
        self.hold(.35)
        for name, curve in segments.items():
            arm = self.arms[name]
            arm.qref = curve[-1].copy(); arm.goal = None

    def screen_copy(self):
        trial = copy.copy(self)
        trial.data = mujoco.MjData(self.model)
        mujoco.mj_copyData(trial.data, self.model, self.data)
        trial.arms = {name: ProgramArm(self.model, trial.data, name) for name in self.arms}
        for name, original in self.arms.items():
            trial.arms[name].gripper = original.gripper
            trial.arms[name].grip_goal = original.grip_goal
            trial.arms[name].rotation = original.rotation.copy()
            trial.arms[name].qref = original.qref.copy()
        trial.offsets = {name: offset.copy() for name, offset in self.offsets.items()}
        trial.payload_owners = self.payload_owners.copy()
        trial.tail, trial.times, trial.trajectory = deque(maxlen=61), [], []
        trial.video, trial.renderer, trial.screening = None, None, True
        return trial

    def guarded_phase(self, batch):
        remaining = list(batch)
        parked = set()
        if not remaining:
            self.phase([]); return
        while remaining:
            candidate = list(remaining)
            while candidate:
                trial = self.screen_copy()
                self.screen_checks += 1
                try:
                    trial.phase(candidate)
                    break
                except RobotContact as collision:
                    self.screen_rejections += 1
                    active = {op['arm'] for op in candidate}
                    involved = set(collision.pair) & active
                    self.event('preflight_rejected', pair=collision.pair, candidates=sorted(active))
                    if len(involved) == 2:
                        removed = sorted(involved)[-1]
                        candidate = [op for op in candidate if op['arm'] != removed]
                    else:
                        idle = next((a for a in collision.pair if a not in active), None)
                        if idle is None or idle in parked:
                            raise
                        point = self.stations[idle]['home_tcp_m'].copy(); point[2] = .82
                        parking_trial = self.screen_copy(); parking_trial.moves({idle: point})
                        self.event('clearance_park', robot=idle, target=point)
                        self.moves({idle: point}); parked.add(idle)
                except Exception as error:
                    self.event('preflight_execution_failed', error=repr(error),
                               objects=trial.tail[-1]['objects'] if trial.tail else {},
                               robots={a: r.observe() for a, r in trial.arms.items()})
                    raise
            if not candidate:
                raise RuntimeError('No collision-free physical subgroup found')
            self.physical_batches += 1
            self.phase(candidate)
            completed = {op['arm'] for op in candidate}
            remaining = [op for op in remaining if op['arm'] not in completed]
        if parked:
            points = {a: self.stations[a]['home_tcp_m'] for a in parked}
            trial = self.screen_copy(); trial.moves(points)
            self.moves(points)

    def grips(self, commands, gentle_open=()):
        self.action_groups += 1
        self.event('grippers', commands=commands)
        for name, closed in commands.items():
            self.arms[name].grip_goal = 0 if closed else (128 if name in gentle_open else 255)
        self.hold(1.4)
        if gentle_open:
            self.event('gentle_release', robots=list(gentle_open))
            self.hold(.8)
            for name in gentle_open:
                self.arms[name].grip_goal = 255
            self.hold(1.4)

    def phase(self, batch):
        points, commands, placement_targets = {}, {}, {}
        for op in batch:
            name, obj, j = op['arm'], op['object'], op['route_index']
            route = self.routes[obj]
            picking = op['phase'] == 'pick_clear'
            if picking:
                planned = self.objects[obj]['initial_position_m'] if j == 0 else self.edges[frozenset(route[j-1:j+1])]
                actual = self.data.body(obj).xpos.copy()
                if np.linalg.norm(actual-np.array(planned)) > .025:
                    raise RuntimeError(f'Object drifted out of pickup zone: {obj} {actual} expected {planned}')
                points[name] = actual.tolist()
            else:
                target = self.targets[obj] if j+1 == len(route) else self.edges[frozenset(route[j:j+2])]
                target = np.array(target, dtype=float)
                if self.objects[obj]['shape'] == 'sphere':
                    target[2] += .012  # Clear the tabletop before relaxing the jaws.
                points[name] = (target-self.offsets[name]).tolist()
                placement_targets[name] = (obj, target)
            commands[name] = picking
        self.event('phase_start', round=self.round, batch=batch, robot_state={a: r.observe() for a, r in self.arms.items()})
        if not batch:
            self.hold(.2); return
        self.moves({a: [p[0], p[1], .70] for a, p in points.items()})
        self.moves(points)
        # The pad/object offset changes under load along a joint trajectory.
        # Close the oracle reference loop before release, with actuator moves.
        for _ in range(4):
            corrections = {}
            for name, (obj, target) in placement_targets.items():
                if np.linalg.norm(self.data.body(obj).xpos-self.data.site_xpos[self.arms[name].site]) > .06:
                    raise RuntimeError('Lost held object before release: '+obj)
                error = target-self.data.body(obj).xpos
                if np.linalg.norm(error) > .003:
                    # Increment the nominal command, preserving its gravity-load
                    # compensation instead of accumulating measured servo sag.
                    nominal, _ = self.arms[name].ik.pose(self.arms[name].qref)
                    corrections[name] = (nominal+error).tolist()
            if not corrections:
                break
            self.event('oracle_prerelease_alignment', targets=corrections)
            self.moves(corrections)
        for name in placement_targets:
            points[name] = self.data.site_xpos[self.arms[name].site].copy().tolist()
        gentle = [a for a, (obj, _) in placement_targets.items() if self.objects[obj]['shape'] == 'sphere']
        if gentle:
            self.hold(1)
        for op in batch:
            if commands[op['arm']]:
                self.payload_owners[op['object']] = op['arm']
        self.grips(commands, gentle)
        for op in batch:
            if not commands[op['arm']]:
                self.payload_owners.pop(op['object'], None)
        self.moves({a: [p[0], p[1], .70] for a, p in points.items()})
        for op in batch:
            a, obj = op['arm'], op['object']
            if commands[a]:
                actual = self.data.body(obj).xpos.copy()
                if actual[2] < .58:
                    raise RuntimeError('Grasp failed: '+obj)
                self.offsets[a] = actual-self.data.site_xpos[self.arms[a].site]
                self.event('lift_verified', robot=a, object=obj, position_m=actual.tolist())
        self.moves({a: self.stations[a]['home_tcp_m'] for a in points})
        self.event('phase_end', round=self.round, object_state=self.tail[-1]['objects'])
        if not self.screening:
            print(json.dumps({'environment': self.name, 'round': self.round, 'of': len(self.plan['schedule']), 'sim_s': round(self.data.time, 2)}), flush=True)

    def run(self):
        started = time.monotonic()
        try:
            self.hold(1)
            for self.round, batch in enumerate(self.plan['schedule'], 1):
                self.guarded_phase(batch)
            self.hold(1.5)
            evaluation = evaluate_placement_window(self.env, self.task, list(self.tail))
            self.report.update(evaluation=evaluation, passed=evaluation['complete'])
            if not evaluation['complete']:
                raise RuntimeError('Final stable semantic goal check failed')
            if self.renderer:
                self.renderer.update_scene(self.data, camera='top')
                image, _ = annotate(Image.fromarray(self.renderer.render()), self.model, self.data, self.env, 'top')
                draw = ImageDraw.Draw(image); draw.rectangle((0, 0, image.width, 45), fill=(23, 48, 75))
                draw.text((16, 8), f'PROGRAM FINAL | {self.name} | {evaluation["matched_goals"]}/{evaluation["total_goals"]} stable goals', font=_font(22), fill='white')
                image.save(self.directory/'final-top.png')
        except Exception as error:
            self.report['error'] = repr(error)
        finally:
            self.report.update(sim_time_s=float(self.data.time), wall_time_s=time.monotonic()-started,
                               command_groups=self.action_groups, control_ticks=self.tick_count,
                               peak_moving_arms=self.max_active, physical_batches=self.physical_batches,
                               preflight_checks=self.screen_checks, preflight_rejections=self.screen_rejections)
            if self.video:
                self.report['video'] = self.video.close()
            if self.renderer:
                self.renderer.close()
            self.trace.close()
            np.savez_compressed(self.directory/'trajectory.npz', sim_time_s=self.times, qpos=self.trajectory)
            (self.directory/'final-samples.json').write_text(json.dumps(list(self.tail)))
            (self.directory/'report.json').write_text(json.dumps(self.report, indent=2)+'\n')
            print(json.dumps(self.report), flush=True)
        return self.report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--counts', type=int, nargs='+', default=[3])
    parser.add_argument('--attempt', type=int, default=1)
    parser.add_argument('--no-record', action='store_true')
    parser.add_argument('--playback', type=float, default=4)
    parser.add_argument('--collection', default='cube_transport')
    parser.add_argument('--motion-time-scale', type=float, default=1, help='Stretch motion time without changing mass, friction or actuator gains')
    args = parser.parse_args()
    failed = []
    for count in args.counts:
        if not Program(count, args.output.resolve(), args.attempt, not args.no_record, args.playback, args.collection, args.motion_time_scale).run()['passed']:
            failed.append(count)
    if failed:
        raise SystemExit('Physical reference failed for arm counts: '+str(failed))
