"""Oracle engineering trial: one Panda holds the blue handle while another turns the red dial.

The program uses scene geometry and the simulated hinge angle for setup and
private evaluation. All physical changes are made through PandaArm and contact;
the dial hinge has no actuator and is never set directly during the trial.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image

from robofrontier.robot_control import PandaArm, matrix_quat, rotation_vector
from robofrontier.scene import ROOT
from robofrontier.task_config import load_harness
from scripts.build_floating_spring_dial import DEFAULT_DIRECTORY


DEFAULT_OUTPUT = ROOT / "runs/floating-dial-oracle-20261023/attempt-01"


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ])


def knob_grasp_rotation(module_rotation, twist):
    z = -module_rotation[:, 2]
    y = module_rotation[:, 0]
    return axis_rotation(z, twist) @ np.column_stack((np.cross(y, z), y, z))


def choose_knob_twist(arm, center, module_rotation):
    """Oracle geometric preflight, not a collision-free path guarantee."""
    for degrees in range(0, 360, 45):
        twist = math.radians(degrees)
        rotation = knob_grasp_rotation(module_rotation, twist)
        q = arm.qref.copy()
        candidates = [(center - rotation[:, 2] * .08, rotation), (center, rotation)]
        candidates += [(center, axis_rotation(module_rotation[:, 2], math.radians(a)) @ rotation)
                       for a in (30, 60, 90)]
        for target, r in candidates:
            q, error = arm.ik.solve(target, r, q, 250)
            _, actual_r = arm.ik.pose(q)
            if error > .0025 or np.linalg.norm(rotation_vector(r, actual_r)) > .02:
                break
        else:
            return twist
    raise RuntimeError('No sampled knob grasp orientation supports the approach and quarter turn')


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class OracleTrial:
    def __init__(self, output):
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.frames_dir = self.output / "frames"
        self.frames_dir.mkdir()
        self.model = mujoco.MjModel.from_xml_path(str(DEFAULT_DIRECTORY / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("initial").id)
        mujoco.mj_forward(self.model, self.data)
        self.arms = {name: PandaArm(self.model, self.data, name) for name in ("A", "B")}
        self.cfg = load_harness()
        self.actions = []
        self.robot_collisions = []
        self.contact_log = {}
        self.telemetry = []
        self.frame_states = []
        self.frame_times = []
        self.support_contact_gap_s = 0.
        self.maximum_support_contact_gap_s = 0.
        self.airborne_table_contacts = 0
        self.step_index = 0
        self.frame_index = 0
        self.frame_stride = max(1, round(1 / (8 * self.model.opt.timestep)))
        self.renderer = mujoco.Renderer(self.model, height=560, width=800)
        self.monitor_airborne = False
        self.min_airborne_clearance_m = float("inf")
        self.hinge_id = self.model.joint("dial_hinge").id
        self.hinge_qadr = self.model.jnt_qposadr[self.hinge_id]
        self.hinge_dadr = self.model.jnt_dofadr[self.hinge_id]
        self.knob_geom = self.model.geom("dial_red_knob").id
        self.stem_geom = self.model.geom("dial_module_stem").id
        module_id = self.model.body("dial_module").id
        rotor_id = self.model.body("dial_rotor").id
        self.module_geoms = [g for g in range(self.model.ngeom)
                             if self.model.geom_bodyid[g] in (module_id, rotor_id)
                             and (self.model.geom_contype[g] or self.model.geom_conaffinity[g])]
        table_id = self.model.geom("table_0").id
        self.table_geom = table_id
        self.table_top = float(self.data.geom_xpos[table_id, 2] + self.model.geom_size[table_id, 2])
        self.target_deg = 90.0

    def state(self):
        module_id = self.model.body("dial_module").id
        return {
            "sim_time_s": float(self.data.time),
            "dial_angle_deg": float(np.degrees(self.data.qpos[self.hinge_qadr])),
            "dial_speed_deg_s": float(np.degrees(self.data.qvel[self.hinge_dadr])),
            "module_position_m": self.data.xpos[module_id].copy().tolist(),
            "module_quaternion_wxyz": self.data.xquat[module_id].copy().tolist(),
            "knob_center_m": self.data.geom(self.knob_geom).xpos.copy().tolist(),
            "stem_center_m": self.data.geom(self.stem_geom).xpos.copy().tolist(),
            "arms": {name: arm.observe() for name, arm in self.arms.items()},
        }

    def save_json(self, name, value):
        (self.output / name).write_text(json.dumps(_jsonable(value), indent=2, ensure_ascii=False) + "\n")

    def snapshot(self, label):
        self.renderer.update_scene(self.data, camera="module_global")
        image = Image.fromarray(self.renderer.render())
        image.save(self.output / f"{label}.png")
        image.save(self.frames_dir / f"frame_{self.frame_index:05d}.png")
        self.frame_states.append(self.data.qpos.copy())
        self.frame_times.append(float(self.data.time))
        self.frame_index += 1
        self.renderer.update_scene(self.data, camera="module_close")
        Image.fromarray(self.renderer.render()).save(self.output / f"{label}-close.png")

    def contact_summary(self):
        return list(self.contact_log.values())

    def module_clearance(self):
        lowest = float("inf")
        for gid in self.module_geoms:
            center = self.data.geom(gid).xpos.copy()
            rotation = self.data.geom(gid).xmat.reshape(3, 3)
            if self.model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
                h = self.model.geom_size[gid].copy()
                points = np.asarray([[x, y, z] for x in (-h[0], h[0])
                                     for y in (-h[1], h[1]) for z in (-h[2], h[2])])
                low_z = float((points @ rotation.T + center)[..., 2].min())
            elif self.model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                radius, half_length = self.model.geom_size[gid][:2]
                axis = rotation[:, 2]
                radial_z = math.sqrt(max(0.0, 1 - axis[2] * axis[2]))
                low_z = float(center[2] - radius * radial_z - half_length * abs(axis[2]))
            else:
                continue
            lowest = min(lowest, low_z)
        return lowest - self.table_top

    def _audit_contacts(self):
        for contact in self.data.contact:
            if contact.dist > 0:
                continue
            g1, g2 = int(contact.geom1), int(contact.geom2)
            body1 = self.model.body(self.model.geom_bodyid[g1]).name
            body2 = self.model.body(self.model.geom_bodyid[g2]).name
            owner1 = body1.split("_", 1)[0]
            owner2 = body2.split("_", 1)[0]
            if owner1 in self.arms and owner2 in self.arms and owner1 != owner2:
                item = {"time_s": float(self.data.time), "body_a": body1, "body_b": body2,
                        "penetration_m": float(-contact.dist)}
                if len(self.robot_collisions) < 100:
                    self.robot_collisions.append(item)
            if (owner1 in self.arms) != (owner2 in self.arms):
                key = (body1, self.model.geom(g1).name, body2, self.model.geom(g2).name)
                item = self.contact_log.get(key)
                if item is None:
                    item = {"body_a": body1, "geom_a": self.model.geom(g1).name,
                            "body_b": body2, "geom_b": self.model.geom(g2).name,
                            "first_time_s": float(self.data.time), "last_time_s": float(self.data.time),
                            "sample_count": 0, "max_penetration_m": 0.0}
                    self.contact_log[key] = item
                item["last_time_s"] = float(self.data.time)
                item["sample_count"] += 1
                item["max_penetration_m"] = max(item["max_penetration_m"], float(-contact.dist))

    def step(self):
        if self.step_index % 20 == 0:
            for arm in self.arms.values():
                arm.control(.02)
        mujoco.mj_step(self.model, self.data)
        self.step_index += 1
        self._audit_contacts()
        if self.monitor_airborne:
            self.min_airborne_clearance_m = min(self.min_airborne_clearance_m, self.module_clearance())
            self.airborne_table_contacts += sum(
                contact.dist <= 0 and self.table_geom in (contact.geom1, contact.geom2)
                and any(g in self.module_geoms for g in (contact.geom1, contact.geom2))
                for contact in self.data.contact)
            if len({c['finger_body'] for c in self.finger_contacts('B', 'dial_module_stem')}) < 2:
                self.support_contact_gap_s += self.model.opt.timestep
                self.maximum_support_contact_gap_s = max(self.maximum_support_contact_gap_s,
                                                         self.support_contact_gap_s)
            else:
                self.support_contact_gap_s = 0.
        if self.step_index % 100 == 0:
            forces = []
            for i, contact in enumerate(self.data.contact):
                names = [self.model.geom(g).name for g in (contact.geom1, contact.geom2)]
                if not any(n.startswith('dial_') for n in names):
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                forces.append({'geoms': names, 'distance_m': float(contact.dist),
                               'normal_force_n': float(force[0]), 'tangent_force_n': float(np.linalg.norm(force[1:3]))})
            self.telemetry.append({'time_s': float(self.data.time),
                                   'module_clearance_m': self.module_clearance(),
                                   'module_quaternion': self.data.body('dial_module').xquat.copy().tolist(),
                                   'B_grasp_drive_n': float(self.data.actuator_force[self.arms['B'].grip]),
                                   'B_tcp_m': self.data.site_xpos[self.arms['B'].site].copy().tolist(),
                                   'contacts': forces})
        if self.step_index % self.frame_stride == 0:
            self.renderer.update_scene(self.data, camera="module_global")
            Image.fromarray(self.renderer.render()).save(
                self.frames_dir / f"frame_{self.frame_index:05d}.png")
            self.frame_states.append(self.data.qpos.copy())
            self.frame_times.append(float(self.data.time))
            self.frame_index += 1

    def wait(self, seconds):
        end_time = self.data.time + seconds
        while self.data.time < end_time - 1e-10:
            self.step()

    def execute(self, label, commands):
        record = {"label": label, "before": self.state(), "commands": commands, "results": {}}
        pending = set()
        for name, (op, params) in commands.items():
            command = self.arms[name].validate(
                op, params, self.cfg,
                np.array([-1.0, -1.0, .401]), np.array([1.0, 1.0, 1.5]),
            )
            self.arms[name].start(label + "-" + name, command, self.data.time)
            pending.add(name)
        max_steps = max(
            int((self.arms[name].execution_timeout_s + 5) / self.model.opt.timestep)
            for name in pending
        )
        for _ in range(max_steps):
            self.step()
            if self.step_index % 20:
                continue
            for name in list(pending):
                result = self.arms[name].poll(self.data.time, self.cfg["action_timeout_sim_s"])
                if result:
                    record["results"][name] = result
                    pending.remove(name)
            if not pending:
                break
        record["after"] = self.state()
        self.actions.append(record)
        self.save_json("actions.json", self.actions)
        print(json.dumps({'action': label, 'time_s': round(float(self.data.time), 2),
                          'clearance_m': round(self.module_clearance(), 4),
                          'results': record['results']}), flush=True)
        if pending:
            raise RuntimeError(f"Timed out waiting for {label}: {sorted(pending)}")
        bad = [r for r in record["results"].values() if r.get("status") != "completed"]
        if bad:
            raise RuntimeError(f"Action failed at {label}: {bad}")
        return record

    def finger_contacts(self, arm_name, object_prefix):
        evidence = []
        for contact in self.data.contact:
            g1, g2 = int(contact.geom1), int(contact.geom2)
            geom_names = [self.model.geom(g1).name, self.model.geom(g2).name]
            body_names = [self.model.body(self.model.geom_bodyid[g]).name for g in (g1, g2)]
            target = next((n for n in geom_names if n.startswith(object_prefix)), None)
            finger = next((n for n in body_names
                           if n in (arm_name + "_left_finger", arm_name + "_right_finger")), None)
            if target and finger:
                evidence.append({"finger_body": finger, "object_geom": target,
                                 "contact_dist_m": float(contact.dist)})
        return evidence

    def write_video(self):
        if not self.frame_index:
            return None
        video = self.output / "oracle-two-arm-turn.mp4"
        subprocess.run([
            "ffmpeg", "-y", "-loglevel", "error", "-framerate", "8",
            "-i", str(self.frames_dir / "frame_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video),
        ], check=True)
        return video.name

    def close(self):
        self.renderer.close()


def run(output=DEFAULT_OUTPUT):
    trial = OracleTrial(output)
    target_records = {}
    report = {"scope": __doc__, "llm_used": False, "direct_dial_actuation": False,
              "result": "in_progress"}
    try:
        trial.wait(.25)
        initial = trial.state()
        trial.snapshot("00-initial")

        # Oracle setup uses exact geometry. The hand closes across X; the
        # later support roll changes the grasp's bearing direction in gravity.
        stem_R = trial.data.geom(trial.stem_geom).xmat.reshape(3, 3).copy()
        stem_center = trial.data.geom(trial.stem_geom).xpos.copy()
        # Grip the proximal, higher part of the narrow handle, away from its
        # tabletop contact end so the fingers have actual clearance.
        # Locate the pinch near the module COM along the stem; this reduces
        # gravity torque on the suspended free body.
        bar_target = stem_center + stem_R @ np.array([0.0, 0.0, .070])
        bar_rotation = trial.data.site_xmat[trial.arms['B'].site].reshape(3, 3).copy()
        bar_pregrasp = bar_target + np.array([0., 0., .10])
        trial.execute("B-handle-approach", {"B": ("move", {
            "target_m": bar_pregrasp.tolist(), "speed_m_s": .05,
            "quaternion_wxyz": matrix_quat(bar_rotation).tolist(), "angular_speed_rad_s": .4,
        })})
        trial.execute("B-handle-descend", {"B": ("move", {
            "target_m": bar_target.tolist(), "speed_m_s": .05,
            "quaternion_wxyz": matrix_quat(bar_rotation).tolist(), "angular_speed_rad_s": .25,
        })})
        trial.execute("B-grasp-blue-handle", {"B": ("gripper", {
            "mode": "grasp", "width_m": .016, "force_n": 70.0, "speed_m_s": .02,
            "epsilon_inner_m": .005, "epsilon_outer_m": .005,
        })})
        trial.wait(.35)
        target_records["B_blue_handle_finger_contacts"] = trial.finger_contacts("B", "dial_module_stem")
        trial.snapshot("01-blue-handle-held")
        if len({x["finger_body"] for x in target_records["B_blue_handle_finger_contacts"]}) < 2:
            raise RuntimeError("Blue handle grasp not confirmed by two finger-to-handle contacts")
        # Lift the whole free module before the second arm touches the knob.
        b_state = trial.arms["B"].observe()
        lift_target = np.asarray(b_state["tcp_m"], dtype=float) + np.array([0.0, 0.0, .18])
        trial.execute("B-lift-module-clear-of-table", {"B": ("move", {
            "target_m": lift_target.tolist(), "speed_m_s": .05,
            "quaternion_wxyz": b_state["tcp_quaternion_wxyz"], "angular_speed_rad_s": .25,
        })})
        trial.wait(.35)
        lift_clearance = trial.module_clearance()
        lifted_state = trial.state()
        q0 = np.asarray(initial["module_quaternion_wxyz"], dtype=float)
        q1 = np.asarray(lifted_state["module_quaternion_wxyz"], dtype=float)
        orientation_dot = float(np.clip(abs(q0 @ q1), 0.0, 1.0))
        orientation_change_deg = float(np.degrees(2 * np.arccos(orientation_dot)))
        report["module_clearance_after_lift_m"] = lift_clearance
        report["module_orientation_change_after_lift_deg"] = orientation_change_deg
        report["module_airborne_before_turn"] = lift_clearance >= .05 and orientation_change_deg <= 25.0
        trial.monitor_airborne = True
        trial.snapshot("03-module-airborne")
        if lift_clearance < .05:
            raise RuntimeError(f"Module did not clear the table: {lift_clearance:.4f} m")
        if orientation_change_deg > 25.0:
            raise RuntimeError(f"Module rotated too far during lift: {orientation_change_deg:.1f} degrees")

        # Rotate the entire grasped module through the robot wrist so its load
        # bears on the lower pad. This is a physical arm motion, not a reset of
        # the module pose. The preceding lift gives the palm table clearance.
        b_rotation = trial.data.site_xmat[trial.arms['B'].site].reshape(3, 3).copy()
        b_rotation = axis_rotation([0., 1., 0.], math.pi / 2) @ b_rotation
        trial.execute('B-roll-to-support-from-below', {'B': ('move', {
            'target_m': (lift_target + np.array([0., 0., .10])).tolist(),
            'quaternion_wxyz': matrix_quat(b_rotation).tolist(),
            'speed_m_s': .06, 'angular_speed_rad_s': .6,
        })})
        trial.wait(.5)
        target_records['B_after_support_roll'] = trial.finger_contacts('B', 'dial_module_stem')
        if len({c['finger_body'] for c in target_records['B_after_support_roll']}) < 2:
            raise RuntimeError('Handle contact was lost while reorienting the module')
        trial.snapshot('03b-support-from-below')

        # Approach the knob along its axis, with jaws closing across its diameter.
        knob_target = trial.data.geom(trial.knob_geom).xpos.copy()
        module_id = trial.model.body("dial_module").id
        module_rotation = trial.data.xmat[module_id].reshape(3, 3).copy()
        grasp_twist = choose_knob_twist(trial.arms['A'], knob_target, module_rotation)
        report['knob_grasp_twist_deg'] = math.degrees(grasp_twist)
        tool_z = -module_rotation[:, 2]
        front_rotation = knob_grasp_rotation(module_rotation, grasp_twist)
        front_pregrasp = knob_target - tool_z * .08
        trial.execute("A-knob-approach", {"A": ("move", {
            "target_m": front_pregrasp.tolist(), "speed_m_s": .06,
            "quaternion_wxyz": matrix_quat(front_rotation).tolist(), "angular_speed_rad_s": .4,
        })})
        if trial.min_airborne_clearance_m < .05:
            raise RuntimeError("Module lost its airborne clearance during A's approach")
        # Recenter from the live knob pose before the final axial approach.
        knob_target = trial.data.geom(trial.knob_geom).xpos.copy()
        module_rotation = trial.data.xmat[module_id].reshape(3, 3).copy()
        tool_z = -module_rotation[:, 2]
        front_rotation = knob_grasp_rotation(module_rotation, grasp_twist)
        front_pregrasp = knob_target - tool_z * .08
        if np.linalg.norm(front_pregrasp - trial.data.site_xpos[trial.arms["A"].site]) > .003:
            trial.execute("A-update-front-pregrasp", {"A": ("move", {
                "target_m": front_pregrasp.tolist(), "speed_m_s": .05,
                "quaternion_wxyz": matrix_quat(front_rotation).tolist(), "angular_speed_rad_s": .4,
            })})
        knob_target = trial.data.geom(trial.knob_geom).xpos.copy()
        trial.execute("A-knob-descend", {"A": ("move", {
            "target_m": knob_target.tolist(), "speed_m_s": .08,
            "quaternion_wxyz": matrix_quat(front_rotation).tolist(), "angular_speed_rad_s": .35,
        })})
        if trial.min_airborne_clearance_m < .05:
            raise RuntimeError("Module lost its airborne clearance during A's descent")
        for align_i in range(3):
            knob_target = trial.data.geom(trial.knob_geom).xpos.copy()
            tcp_now = trial.data.site_xpos[trial.arms["A"].site].copy()
            if np.linalg.norm(knob_target - tcp_now) <= .003:
                break
            trial.execute(f"A-knob-contact-alignment-{align_i+1}", {"A": ("move", {
                "target_m": knob_target.tolist(), "speed_m_s": .012,
                "quaternion_wxyz": matrix_quat(front_rotation).tolist(), "angular_speed_rad_s": .3,
            })})
            if trial.min_airborne_clearance_m < .05:
                raise RuntimeError("Module lost its airborne clearance during final knob alignment")
        trial.execute("A-grasp-red-knob", {"A": ("gripper", {
            "mode": "grasp", "width_m": .056, "force_n": 70.0, "speed_m_s": .04,
            "epsilon_inner_m": .008, "epsilon_outer_m": .008,
        })})
        trial.wait(.35)
        if trial.min_airborne_clearance_m < .05:
            raise RuntimeError("Module contacted the tabletop before rotation started")
        target_records["A_red_knob_finger_contacts"] = trial.finger_contacts("A", "dial_red_knob")
        trial.snapshot("04-two-grasps-airborne")
        if len({x["finger_body"] for x in target_records["A_red_knob_finger_contacts"]}) < 2:
            raise RuntimeError("Red knob grasp not confirmed by two finger-to-knob contacts")

        target_angle = math.radians(trial.target_deg)
        ratio = None
        turn_history = []
        for turn_i in range(8):
            current_angle = float(trial.data.qpos[trial.hinge_qadr])
            error = target_angle - current_angle
            if abs(math.degrees(error)) <= 3.0:
                break
            max_segment = math.radians(30)
            if ratio is None:
                commanded_delta = math.radians(15)
            else:
                if ratio <= .1:
                    raise RuntimeError('Unreliable wrist-to-hinge motion transfer; do not amplify the command')
                commanded_delta = float(np.clip(error / ratio, -max_segment, max_segment))
            current_axis = trial.data.xaxis[trial.hinge_id].copy()
            current_rot = trial.data.site_xmat[trial.arms["A"].site].reshape(3, 3).copy()
            current_target = trial.data.geom(trial.knob_geom).xpos.copy()
            desired_rot = axis_rotation(current_axis, commanded_delta) @ current_rot
            before_angle = current_angle
            trial.execute(f"A-turn-segment-{turn_i+1}", {"A": ("move", {
                "target_m": current_target.tolist(), "speed_m_s": .025,
                "quaternion_wxyz": matrix_quat(desired_rot).tolist(), "angular_speed_rad_s": .25,
            })})
            trial.wait(.30)
            if trial.min_airborne_clearance_m < .05:
                raise RuntimeError("Module contacted or approached the tabletop during the turn")
            after_angle = float(trial.data.qpos[trial.hinge_qadr])
            measured_delta = after_angle - before_angle
            segment_ratio = measured_delta / commanded_delta if abs(commanded_delta) > 1e-6 else 0.0
            ratio = segment_ratio if ratio is None else .5 * ratio + .5 * segment_ratio
            turn_history.append({"segment": turn_i + 1, "before_deg": math.degrees(before_angle),
                                 "commanded_wrist_delta_deg": math.degrees(commanded_delta),
                                 "after_deg": math.degrees(after_angle), "measured_ratio": segment_ratio})
            if abs(measured_delta) < math.radians(1.0):
                raise RuntimeError("Wrist moved but dial angle did not: likely knob-grasp slip or contact failure")

        trial.wait(1.)
        held_angle = float(trial.data.qpos[trial.hinge_qadr])
        target_records['B_handle_after_turn'] = trial.finger_contacts('B', 'dial_module_stem')
        target_records['A_knob_after_turn'] = trial.finger_contacts('A', 'dial_red_knob')
        for key in ('B_handle_after_turn', 'A_knob_after_turn'):
            if len({c['finger_body'] for c in target_records[key]}) < 2:
                raise RuntimeError(f'Two-finger contact lost at end of rotation: {key}')
        report["minimum_module_clearance_during_turn_m"] = trial.min_airborne_clearance_m
        report["airborne_through_rotation"] = trial.min_airborne_clearance_m >= .05
        report["module_table_contacts_while_airborne"] = trial.airborne_table_contacts
        trial.snapshot("05-held-at-target-airborne")
        report.update({
            "result": "completed",
            "initial_state": initial,
            "grasp_evidence": target_records,
            "turn_segments": turn_history,
            "target_angle_deg": trial.target_deg,
            "held_angle_deg": math.degrees(held_angle),
            "rotation_error_deg": math.degrees(target_angle - held_angle),
            "rotation_passed": abs(math.degrees(target_angle - held_angle)) <= 4.0,
            "final_held_state": trial.state(),
            "inter_robot_penetrations": trial.robot_collisions,
            "arm_environment_contacts": trial.contact_summary(),
            "passed": abs(math.degrees(target_angle - held_angle)) <= 4.0
                      and bool(report.get("module_airborne_before_turn"))
                      and report["airborne_through_rotation"] and not trial.robot_collisions,
        })
        trial.save_json("report-before-release.json", report)

        # Release the red knob and verify the passive spring return while B still
        # stabilizes the blue handle. This does not set the hinge angle directly.
        trial.execute("A-release-red-knob", {"A": ("gripper", {
            "mode": "move", "width_m": .08, "speed_m_s": .04,
        })})
        trial.wait(2.0)
        returned_angle = float(trial.data.qpos[trial.hinge_qadr])
        report["released_angle_deg"] = math.degrees(returned_angle)
        report["spring_return_passed"] = abs(math.degrees(returned_angle)) <= 4.0
        report["post_release_state"] = trial.state()
        report["inter_robot_penetrations"] = trial.robot_collisions
        report["arm_environment_contacts"] = trial.contact_summary()
        report["module_clearance_after_rotation_m"] = trial.module_clearance()
        report["airborne_after_spring_return_m"] = report["module_clearance_after_rotation_m"] >= .05
        report['maximum_handle_two_finger_contact_gap_s'] = trial.maximum_support_contact_gap_s
        report['minimum_module_clearance_lift_to_finish_m'] = trial.min_airborne_clearance_m
        report['module_table_contacts_lift_to_finish'] = trial.airborne_table_contacts
        report["passed"] = bool(report["passed"] and report["spring_return_passed"]
                                and report["airborne_after_spring_return_m"]
                                and trial.min_airborne_clearance_m >= .05
                                and trial.airborne_table_contacts == 0
                                and trial.maximum_support_contact_gap_s <= .02
                                and not trial.robot_collisions)
        report['result'] = 'completed' if report['passed'] else 'failed'
        trial.snapshot("05-spring-return-airborne")
    except Exception as exc:
        report.update({"result": "failed", "error": f"{type(exc).__name__}: {exc}",
                       "state_at_failure": trial.state(),
                       "minimum_airborne_clearance_seen_m": trial.min_airborne_clearance_m,
                       "inter_robot_penetrations": trial.robot_collisions,
                       "arm_environment_contacts": trial.contact_summary()})
        report["passed"] = False
    finally:
        trial.close()
        trial.save_json("telemetry.json", trial.telemetry)
        np.savez_compressed(trial.output / 'frame-states.npz',
                            qpos=np.asarray(trial.frame_states), time_s=np.asarray(trial.frame_times))
        report["action_count"] = len(trial.actions)
        report["actions"] = trial.actions
        report["contact_evidence"] = target_records
        try:
            report["video"] = trial.write_video()
        except Exception as exc:
            report["video_error"] = f"{type(exc).__name__}: {exc}"
        trial.save_json("report.json", report)
    print(json.dumps(_jsonable({k: v for k, v in report.items()
                                if k not in ("actions", "contact_evidence", "grasp_evidence", "arm_environment_contacts", "initial_state", "state_at_failure", "final_held_state", "post_release_state")}),
                     ensure_ascii=False), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = run(args.output)
    if not result.get('passed'):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
