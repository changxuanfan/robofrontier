"""Physical acceptance checks, including three real-contact grasp/lift trials.

The reference policy is deliberately Oracle-based. It verifies the simulator
and shared movement controller, not a model's vision or reasoning ability.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from robofrontier.scene import ROOT
from robofrontier.sim import Simulation


def move_to(sim, goal, max_steps=2500):
    goal = np.asarray(goal, dtype=float)
    for _ in range(max_steps):
        error = goal - sim.data.site_xpos[sim.site]
        if np.linalg.norm(error) < 0.0015:
            for _ in range(40):
                sim.control()
            return
        sim.control(np.clip(error / 0.008, -1, 1))
    raise AssertionError(f"Unreachable target {goal}: {sim.observe()}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    parser.add_argument("--hold-seconds", type=float, default=1.5)
    args = parser.parse_args()
    sim = Simulation(args.config)
    results = {"mujoco_version": sim.metadata()["mujoco_version"], "checks": {}}
    original = sim.integration_state().copy()
    cubes = np.array([sim.data.body(f"cube_{name}").xpos.copy() for name in sim.config["cubes"]])
    for _ in range(3000):
        sim.control()
    drift = max(np.linalg.norm(sim.data.body(f"cube_{name}").xpos - cubes[i]) for i, name in enumerate(sim.config["cubes"]))
    assert drift < 0.001, f"Idle cube drift: {drift}"
    assert np.isfinite(sim.data.qpos).all()
    results["checks"]["30_second_idle"] = {"cube_drift_m": float(drift), "finite_state": True}

    sim.reset()
    assert np.allclose(sim.integration_state(), original, atol=1e-12, rtol=0)
    results["checks"]["reset"] = "complete integration state restored"

    trials = []
    for trial in range(3):
        sim.reset()
        move_to(sim, [0.45, -0.09, 0.59])
        move_to(sim, [0.45, -0.09, 0.422])
        sim.set_gripper(False)
        for _ in range(180):
            sim.control()
        assert all(sim.finger_contacts("blue")), "Both fingers must contact the cube"
        move_to(sim, [0.45, -0.09, 0.54])
        hold_start = sim.data.body("cube_blue").xpos.copy()
        for _ in range(round(args.hold_seconds / sim.config["control_dt_s"])):
            sim.control()
        objects = sim.observe(True)["objects"]
        lift = objects["blue"]["position_m"][2] - sim.initial_cube_z["blue"]
        assert lift >= 0.05 and sim.hold_time["blue"] >= 1.0, objects
        assert all(objects["blue"]["finger_contacts"])
        hold_drift = np.linalg.norm(sim.data.body("cube_blue").xpos - hold_start)
        if args.hold_seconds >= 30:
            assert hold_drift < 0.002, f"Long hold drift: {hold_drift} m"
        assert abs(objects["red"]["position_m"][2] - sim.initial_cube_z["red"]) < 0.001
        trials.append({"trial": trial + 1, "lift_m": lift, "continuous_hold_s": sim.hold_time["blue"],
                       "sim_time_s": sim.data.time, "both_fingers": True,
                       "requested_hold_seconds": args.hold_seconds, "hold_drift_m": float(hold_drift)})
    results["checks"]["grasp_trials"] = trials

    sim.reset()
    sim.set_mode("joint")
    for _ in range(100):
        sim.control(joint_direction=[1, 0, 0, 0, 0, 0, 1])
    moved = sim.data.qpos[sim.qids] - sim.home_q
    assert moved[0] > 0.05 and moved[6] > 0.05
    for _ in range(150):
        sim.control()
    assert np.max(np.abs(sim.data.qvel[sim.dofs])) < 0.005
    before = sim.data.qpos[sim.qids].copy()
    sim.set_mode("tcp")
    sim.control()
    assert np.max(np.abs(sim.data.qpos[sim.qids] - before)) < 0.002
    results["checks"]["multi_joint_release_and_mode_switch"] = "passed"

    sim.reset()
    for _ in range(1800):
        sim.control([0, 0, -1])
    assert np.linalg.norm(sim.tcp_ref - sim.data.site_xpos[sim.site]) < 0.025
    assert np.max(np.abs(sim.q_ref - sim.data.qpos[sim.qids])) <= 0.101
    for _ in range(100):
        sim.control()
    assert np.max(np.abs(sim.data.qvel[sim.dofs])) < 0.01
    results["checks"]["table_bound_and_anti_windup"] = "passed"

    suffix = "-" + Path(args.config).stem if args.config else ""
    path = ROOT / f"runs/verification{suffix}.json"
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
