"""Shared robot-only IK, bounded reference trajectories, and real physics."""
import hashlib
import json
import secrets
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from .scene import ROOT, build_scene


class RobotIK:
    """Damped least squares; only robot joint coordinates enter this solver.

    The kinematic scratch model contains only the fixed-base chain ending at
    this robot's TCP. No environment bodies, contacts or object poses enter FK.
    """
    def __init__(self, model, prefix=""):
        self.model = model
        self.joints = np.array([model.joint(f"{prefix}joint{i}").id for i in range(1, 8)])
        self.qids = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.site = model.site(f"{prefix}tcp").id
        self.limits = model.jnt_range[self.joints].copy()
        for i in range(7):
            actuator = model.actuator(f"{prefix}actuator{i+1}").id
            self.limits[i, 0] = max(self.limits[i, 0], model.actuator_ctrlrange[actuator, 0]) + 0.02
            self.limits[i, 1] = min(self.limits[i, 1], model.actuator_ctrlrange[actuator, 1]) - 0.02
        root=ET.Element('mujoco');ET.SubElement(root,'compiler',angle='radian',autolimits='true');world=ET.SubElement(root,'worldbody')
        body=int(model.site_bodyid[self.site]);chain=[]
        while body:chain.append(body);body=int(model.body_parentid[body])
        def numbers(values):return ' '.join(repr(float(x)) for x in values)
        parent=world;copied=set()
        for body in reversed(chain):
            parent=ET.SubElement(parent,'body',name=model.body(body).name,pos=numbers(model.body_pos[body]),quat=numbers(model.body_quat[body]))
            if model.body_mass[body]>0:ET.SubElement(parent,'inertial',mass=str(model.body_mass[body]),pos=numbers(model.body_ipos[body]),quat=numbers(model.body_iquat[body]),diaginertia=numbers(model.body_inertia[body]))
            for joint in range(model.body_jntadr[body],model.body_jntadr[body]+model.body_jntnum[body]):
                if joint not in self.joints or model.jnt_type[joint]!=mujoco.mjtJoint.mjJNT_HINGE:raise ValueError('RobotIK requires a fixed-base seven-hinge chain')
                ET.SubElement(parent,'joint',name=model.joint(joint).name,type='hinge',axis=numbers(model.jnt_axis[joint]),pos=numbers(model.jnt_pos[joint]),range=numbers(model.jnt_range[joint]),ref=str(model.qpos0[model.jnt_qposadr[joint]]));copied.add(joint)
        if copied!=set(self.joints):raise ValueError('Robot TCP does not descend from every arm joint')
        ET.SubElement(parent,'site',name=prefix+'tcp',pos=numbers(model.site_pos[self.site]),quat=numbers(model.site_quat[self.site]))
        self.kinematic_model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));self.data=mujoco.MjData(self.kinematic_model)
        local_joints=[self.kinematic_model.joint(model.joint(int(j)).name).id for j in self.joints]
        self.local_qids=self.kinematic_model.jnt_qposadr[local_joints];self.local_dofs=self.kinematic_model.jnt_dofadr[local_joints];self.local_site=self.kinematic_model.site(prefix+'tcp').id
        self.jp = np.zeros((3, self.kinematic_model.nv))
        self.jr = np.zeros((3, self.kinematic_model.nv))

    def pose(self, q):
        self.data.qpos[self.local_qids] = q
        mujoco.mj_kinematics(self.kinematic_model, self.data)
        mujoco.mj_comPos(self.kinematic_model, self.data)
        return self.data.site_xpos[self.local_site].copy(), self.data.site_xmat[self.local_site].reshape(3, 3).copy()

    def solve(self, target, rotation, seed, iterations=20):
        q = seed.copy()
        for _ in range(iterations):
            position, current_r = self.pose(q)
            # Quaternion log remains informative at a 180-degree rotation.
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, (rotation @ current_r.T).ravel())
            if quat[0] < 0: quat = -quat
            sine = np.linalg.norm(quat[1:])
            rot_error = (np.zeros(3) if sine < 1e-12 else
                         quat[1:] * (2*np.arctan2(sine, max(0., quat[0])) / sine))
            error = np.r_[target - position, 0.35 * rot_error]
            if np.linalg.norm(error[:3]) < 0.00008 and np.linalg.norm(rot_error) < 0.0005:
                break
            mujoco.mj_jacSite(self.kinematic_model, self.data, self.jp, self.jr, self.local_site)
            jac = np.vstack([self.jp[:, self.local_dofs], 0.35 * self.jr[:, self.local_dofs]])
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 0.012**2 * np.eye(6), error)
            delta *= min(1.0, 0.08 / max(np.max(np.abs(delta)), 1e-12))
            q = np.clip(q + delta, self.limits[:, 0], self.limits[:, 1])
        position, _ = self.pose(q)
        return q, float(np.linalg.norm(target - position))


class Simulation:
    def __init__(self, config_path=None, blind=False):
        self.config = json.loads(Path(config_path or ROOT / "config.json").read_text())
        if blind:
            # Independently randomized before the policy's first observation.
            # Do not print coordinates or provide them through the bridge.
            rng = np.random.default_rng(secrets.randbits(128))
            placed = []
            minimum_separation = float(self.config.get("blind_min_separation_m", 0.085))
            for color in self.config["cubes"]:
                for _ in range(10000):
                    point = rng.uniform([0.41, -0.17], [0.55, 0.17])
                    if all(np.linalg.norm(point - other) >= minimum_separation for other in placed):
                        placed.append(point)
                        self.config["cubes"][color] = point.tolist()
                        break
                else:
                    raise RuntimeError("Could not sample a separated layout")
        self.model, self.xml = build_scene(self.config)
        self.data = mujoco.MjData(self.model)
        self.ik = RobotIK(self.model)
        self.qids, self.dofs = self.ik.qids, self.ik.dofs
        self.site = self.ik.site
        self.home_q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
        _, home_rotation = self.ik.pose(self.home_q)
        self.home_q, error = self.ik.solve(np.array(self.config["home_tcp_m"]), home_rotation, self.home_q, 180)
        if error > 0.002:
            raise RuntimeError(f"Home TCP is unreachable ({error:.4f} m error)")
        self.data.qpos[self.qids] = self.home_q
        for name in ("finger_joint1", "finger_joint2"):
            self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]] = 0.04
        self.data.ctrl[:7] = self.home_q
        self.data.ctrl[7] = 255
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(1 / self.config["timestep_s"])):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0
        self.state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.initial_state = self.integration_state()
        self.initial_cube_z = {name: float(self.data.body(f"cube_{name}").xpos[2]) for name in self.config["cubes"]}
        self.reset()

    def integration_state(self):
        state = np.empty(mujoco.mj_stateSize(self.model, self.state_spec))
        mujoco.mj_getState(self.model, self.data, state, self.state_spec)
        return state

    def reset(self):
        mujoco.mj_setState(self.model, self.data, self.initial_state, self.state_spec)
        mujoco.mj_forward(self.model, self.data)
        self.q_ref = self.data.ctrl[:7].copy()
        self.ref_velocity = np.zeros(7)
        self.tcp_ref = self.data.site_xpos[self.site].copy()
        self.rotation_ref = self.data.site_xmat[self.site].reshape(3, 3).copy()
        self.gripper_ref = float(self.data.ctrl[7])
        self.gripper_goal = self.gripper_ref
        self.mode = "tcp"
        self.speed = 1
        self.status = "Ready — real-time physics"
        self.steps = 0
        self.hold_time = {name: 0.0 for name in self.config["cubes"]}

    def stop(self):
        self.q_ref = self.data.qpos[self.qids].copy()
        self.ref_velocity[:] = 0
        self.tcp_ref = self.data.site_xpos[self.site].copy()
        self.rotation_ref = self.data.site_xmat[self.site].reshape(3, 3).copy()
        self.data.ctrl[:7] = self.q_ref

    def set_mode(self, mode):
        self.stop()
        self.mode = mode

    def set_gripper(self, opened):
        self.gripper_goal = 255.0 if opened else 0.0

    def control(self, direction=None, yaw=0.0, joint_direction=None):
        dt = self.config["control_dt_s"]
        direction = np.asarray(direction if direction is not None else [0, 0, 0], dtype=float)
        joint_direction = np.asarray(joint_direction if joint_direction is not None else np.zeros(7), dtype=float)
        max_speed = np.deg2rad(self.config["joint_speed_deg_s"][self.speed])
        if self.mode == "tcp":
            moving = np.linalg.norm(direction) > 0 or yaw != 0
            if moving:
                direction /= max(1.0, np.linalg.norm(direction))
                actual = self.data.site_xpos[self.site]
                # Anti-windup: never keep a hidden target far beyond a contact.
                self.tcp_ref = actual + np.clip(self.tcp_ref - actual, -0.012, 0.012)
                self.tcp_ref += direction * self.config["linear_speed_m_s"][self.speed] * dt
                self.tcp_ref = np.clip(self.tcp_ref, self.config["workspace_min_m"], self.config["workspace_max_m"])
                angle = yaw * max_speed * dt
                c, s = np.cos(angle), np.sin(angle)
                self.rotation_ref = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ self.rotation_ref
                target_q, error = self.ik.solve(self.tcp_ref, self.rotation_ref, self.q_ref, 12)
                self.status = "IK limited / unreachable" if error > 0.003 else "Moving TCP"
                if error > 0.003:
                    self.tcp_ref = actual.copy()
                requested = np.clip((target_q - self.q_ref) / dt, -max_speed, max_speed)
            else:
                requested = np.zeros(7)
        else:
            moving = np.any(joint_direction)
            requested = np.clip(joint_direction, -1, 1) * max_speed
            self.status = "Moving joints" if moving else "Holding"
        max_accel = np.deg2rad(self.config["joint_acceleration_deg_s2"])
        # Distance-based braking before the soft joint bounds.
        lower, upper = self.ik.limits[:, 0], self.ik.limits[:, 1]
        velocity_lower = -np.sqrt(2 * max_accel * np.maximum(0, self.q_ref - lower))
        velocity_upper = np.sqrt(2 * max_accel * np.maximum(0, upper - self.q_ref))
        requested = np.clip(requested, velocity_lower, velocity_upper)
        self.ref_velocity += np.clip(requested - self.ref_velocity, -max_accel * dt, max_accel * dt)
        self.q_ref = np.clip(self.q_ref + self.ref_velocity * dt, lower, upper)
        actual_q = self.data.qpos[self.qids]
        if np.max(np.abs(self.q_ref - actual_q)) > 0.10:
            self.q_ref = np.clip(self.q_ref, actual_q - 0.10, actual_q + 0.10)
            self.ref_velocity[:] = 0
            self.status = "Tracking limited — release and reposition"
        self.data.ctrl[:7] = self.q_ref
        change = 255 * dt / self.config["gripper_full_travel_s"]
        self.gripper_ref += float(np.clip(self.gripper_goal - self.gripper_ref, -change, change))
        self.data.ctrl[7] = self.gripper_ref
        for _ in range(round(dt / self.config["timestep_s"])):
            mujoco.mj_step(self.model, self.data)
        # mj_step leaves some derived position fields at the preceding time.
        mujoco.mj_forward(self.model, self.data)
        if not moving and np.max(np.abs(self.ref_velocity)) < 1e-5:
            self.tcp_ref = self.data.site_xpos[self.site].copy()
            if self.status.startswith("Moving"):
                self.status = "Holding"
        self.steps += 1
        for name in self.hold_time:
            raised = self.data.body(f"cube_{name}").xpos[2] > self.initial_cube_z[name] + 0.05
            fingers = self.finger_contacts(name)
            self.hold_time[name] = self.hold_time[name] + dt if raised and all(fingers) else 0.0

    def finger_contacts(self, color):
        object_body = self.model.body(f"cube_{color}").id
        object_geoms = {geom for geom, body in enumerate(self.model.geom_bodyid)
                        if body == object_body}
        finger_bodies = [self.model.body(name).id for name in ("left_finger", "right_finger")]
        touching = [False, False]
        for contact in self.data.contact:
            if contact.geom1 not in object_geoms and contact.geom2 not in object_geoms:
                continue
            other = contact.geom2 if contact.geom1 in object_geoms else contact.geom1
            body = self.model.geom_bodyid[other]
            for i, finger in enumerate(finger_bodies):
                if body == finger and contact.dist < 0.001:
                    touching[i] = True
        return touching

    def observe(self, oracle=False):
        result = {"sim_time_s": float(self.data.time), "mode": self.mode, "speed_index": self.speed,
                  "tcp_m": self.data.site_xpos[self.site].tolist(),
                  "joint_rad": self.data.qpos[self.qids].tolist(),
                  "joint_velocity_rad_s": self.data.qvel[self.dofs].tolist(),
                  "gripper_width_m": float(sum(self.data.joint(name).qpos[0] for name in ("finger_joint1", "finger_joint2"))),
                  "status": self.status}
        if oracle:
            result["objects"] = {name: {"position_m": self.data.body(f"cube_{name}").xpos.tolist(),
                                       "finger_contacts": self.finger_contacts(name),
                                       "hold_time_s": self.hold_time[name]}
                                 for name in self.config["cubes"]}
        return result

    def metadata(self):
        return {"config": self.config, "mujoco_version": mujoco.__version__,
                "model_sha256": hashlib.sha256(self.xml.encode()).hexdigest(),
                "robot_source": json.loads((ROOT / "vendor/panda/SOURCE.json").read_text()),
                "initial_q_rad": self.home_q.tolist(), "time_basis": "wall_clock",
                "control": "robot-only DLS IK; upstream position actuators; no object attachment"}
