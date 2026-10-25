"""Postmortem ONLY: replay a recorded grasp with simulator truth for diagnosis.

This is not the RGB policy. It does not modify a running demonstration.
Baseline exits nonzero when the recorded slip reproduces.
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from robofrontier.scene import ROOT
from robofrontier.sim import Simulation


def contact_stats(sim):
    cube = sim.model.geom("cube_blue_geom").id
    finger_ids = [sim.model.body(n).id for n in ("left_finger", "right_finger")]
    normal = [0.0, 0.0]
    contacts = []
    for i, contact in enumerate(sim.data.contact):
        if cube not in (contact.geom1, contact.geom2):
            continue
        other = contact.geom2 if contact.geom1 == cube else contact.geom1
        body = sim.model.geom_bodyid[other]
        if body not in finger_ids:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(sim.model, sim.data, i, force)
        normal[finger_ids.index(body)] += max(0.0, float(force[0]))
        contacts.append({"normal_N": float(force[0]), "friction": contact.friction.tolist(),
                         "dim": int(contact.dim), "position_m": contact.pos.tolist()})
    cube_p = sim.data.body("cube_blue").xpos
    tcp_p = sim.data.site_xpos[sim.site]
    return {"sim_time_s": float(sim.data.time), "cube_z_m": float(cube_p[2]),
            "tcp_m": tcp_p.tolist(), "cube_minus_tcp_m": (cube_p - tcp_p).tolist(),
            "normal_per_finger_N": normal, "tendon_force_N": float(sim.data.actuator_force[7]),
            "gripper_ctrl": float(sim.data.ctrl[7]), "gripper_width_m": sim.observe()["gripper_width_m"],
            "contacts": contacts}


def replay(episode, case, compact=False, hold_seconds=2):
    metadata = json.loads((episode / "metadata.json").read_text())
    output = ROOT / "runs/slip-diagnosis"
    output.mkdir(exist_ok=True)
    config = output / f"{case}-recorded-config.json"
    config.write_text(json.dumps(metadata["config"]))
    sim = Simulation(config)
    events = [json.loads(line) for line in (episode / "trace.jsonl").open()]
    closed = next(row for row in events if row["event"] == "sample" and 425.03 < row["observation"]["sim_time_s"] < 425.3)
    moves = [row for row in events if row["event"] == "tool_request" and row.get("type") == "move" and 430 < row["sim_time_s"] < 480]
    mujoco.mj_setState(sim.model, sim.data, np.asarray(closed["integration_state"]), sim.state_spec)
    mujoco.mj_forward(sim.model, sim.data)
    sim.q_ref = np.asarray(closed["q_ref"])
    sim.ref_velocity[:] = 0
    sim.tcp_ref = sim.data.site_xpos[sim.site].copy()
    sim.gripper_ref = float(sim.data.ctrl[7])
    sim.gripper_goal = 0
    sim.mode = "tcp"
    sim.speed = 1
    if case == "sliding_friction_3x":
        for geom in range(sim.model.ngeom):
            if sim.model.geom_bodyid[geom] in [sim.model.body(n).id for n in ("cube_blue", "left_finger", "right_finger")]:
                sim.model.geom_friction[geom, 0] *= 3
    elif case == "gripper_stiffness_4x":
        sim.model.actuator_gainprm[7, 0] *= 4
        sim.model.actuator_biasprm[7, 1] *= 4
    elif case == "torsional_friction_5x":
        for geom in range(sim.model.ngeom):
            if sim.model.geom_bodyid[geom] in [sim.model.body(n).id for n in ("cube_blue", "left_finger", "right_finger")]:
                sim.model.geom_friction[geom, 1] *= 5
    elif case == "noslip_10":
        sim.model.opt.noslip_iterations = 10
    mujoco.mj_forward(sim.model, sim.data)
    samples = [contact_stats(sim)]
    initial_z = samples[0]["cube_z_m"]
    next_sample = sim.data.time + 0.1

    def step(direction=None):
        nonlocal next_sample
        sim.control(direction)
        if sim.data.time >= next_sample:
            samples.append(contact_stats(sim))
            next_sample = sim.data.time + 0.1

    for move in moves:
        while not compact and sim.data.time + 0.005 < move["sim_time_s"]:
            step()
        direction = np.array([move.get(f"d{a}", 0) for a in "xyz"]) / (move["duration"] * sim.config["linear_speed_m_s"][1])
        for _ in range(round(move["duration"] / sim.config["control_dt_s"])):
            step(direction)
        for _ in range(35):
            step()
    for _ in range(round(hold_seconds / sim.config["control_dt_s"])):
        step()
    final = contact_stats(sim)
    peak = max(sample["cube_z_m"] for sample in samples)
    slipped = final["gripper_width_m"] < 0.01 or min(final["normal_per_finger_N"]) < 0.05
    summary = {"case": case, "compact": compact, "hold_seconds": hold_seconds, "slipped": bool(slipped), "peak_cube_lift_m": peak - initial_z,
               "final_cube_lift_m": final["cube_z_m"] - initial_z,
               "initial": {k: v for k, v in samples[0].items() if k != "contacts"},
               "final": {k: v for k, v in final.items() if k != "contacts"},
               "initial_effective_contact": samples[0]["contacts"][:2]}
    suffix = f"-compact-hold{hold_seconds:g}" if compact else ""
    (output / f"{case}{suffix}.json").write_text(json.dumps({"summary": summary, "samples": samples}, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="baseline", choices=["baseline", "sliding_friction_3x", "gripper_stiffness_4x", "torsional_friction_5x", "noslip_10"])
    parser.add_argument("--episode", type=Path, default=ROOT / "runs/20261019-220952-187535")
    parser.add_argument("--compact", action="store_true", help="Remove inter-action thinking waits from the reproduction")
    parser.add_argument("--hold-seconds", type=float, default=2)
    args = parser.parse_args()
    result = replay(args.episode, args.case, args.compact, args.hold_seconds)
    print(json.dumps(result, indent=2))
    assert not result["slipped"], "Recorded grasp slipped: sustained two-finger holding was lost"


if __name__ == "__main__":
    main()
