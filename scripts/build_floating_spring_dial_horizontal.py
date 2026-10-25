"""Build the lift-and-turn horizontal dial module with two Panda arms."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from robofrontier.cube_transport import ROOT, _camera, add_initial_keyframe, build_xml, initialize
from robofrontier.scene import vector
from scripts.build_floating_spring_dial import PARAMETERS as BASE_PARAMETERS, _make_face_texture, _write_json


DEFAULT_DIRECTORY = ROOT / "configs/environments/spring_dial/lifted-horizontal-01"


def parameters():
    p = json.loads(json.dumps(BASE_PARAMETERS))
    p["id"] = "spring-dial-lifted-horizontal-01"
    p["arms"] = [
        {"id": "A", "side": "south-west", "base_position_m": [-.25, -.65, .40],
         "yaw_rad": math.pi / 2, "home_tcp_m": [-.25, -.42, .82]},
        {"id": "B", "side": "north-east", "base_position_m": [.25, .65, .40],
         "yaw_rad": -math.pi / 2, "home_tcp_m": [.25, .42, .82]},
    ]
    p["module"].update({
        "layout": "horizontal_face_up",
        "stem_length_m": .22,
        "stem_width_m": [.016, .020],
        "stem_offset_y_m": 0.0,
        "stem_mass_kg": .55,
        "face_thickness_m": .008,
        "face_mass_kg": .30,
        "knob_mass_kg": .12,
    })
    return p


def environment(p):
    table = p["table"]
    stations = [{**arm, "pair": 1, "source_positions_m": [],
                 "goal_positions_m": [], "parking_positions_m": []}
                for arm in p["arms"]]
    return {
        "schema": "robofrontier.spring_dial.lifted_horizontal.v1",
        "id": p["id"],
        "collection": "spring_dial",
        "arm_count": 2,
        "topology": "opposed_diagonal_pair",
        "stations": stations,
        "module": {"id": "dial_module", "center_xy_m": [0.0, 0.0],
                   "initial_orientation": "horizontal_face_up"},
        "physics": p["physics"],
        "tables": [{"center_m": [0.0, 0.0, table["height_m"] - table["thickness_m"] / 2],
                    "half_size_m": [table["length_m"] / 2, table["width_m"] / 2,
                                    table["thickness_m"] / 2]}],
        "floor_rgba": [0.84, 0.89, 0.94, 1.0],
        "table_rgba": [0.64, 0.47, 0.35, 1.0],
        "edges": [], "objects": [], "initial_keyframe": "initial", "scene": "scene.xml",
    }


def add_module(root, directory, p):
    world, asset = root.find("worldbody"), root.find("asset")
    m = p["module"]
    table_z = p["table"]["height_m"]
    stem_len = m["stem_length_m"]
    face_t = m["face_thickness_m"]
    knob_h = m["knob_height_m"]
    root.find(".//geom[@name='table_0']").attrib.update(solref="0.001 1", solimp="0.95 0.99 0.001")

    ET.SubElement(asset, "material", name="module_blue", rgba=vector(m["stem_rgba"]),
                  specular=".22", shininess=".3", emission=".035")
    ET.SubElement(asset, "material", name="module_white", rgba=".96 .95 .91 1",
                  specular=".12", shininess=".2")
    ET.SubElement(asset, "material", name="module_red", rgba=vector(m["knob_rgba"]),
                  specular=".28", shininess=".35")
    _make_face_texture(directory / "assets" / "dial_face.png")
    ET.SubElement(asset, "texture", name="dial_face_tex", type="2d", file="assets/dial_face.png")
    ET.SubElement(asset, "material", name="dial_face_print", texture="dial_face_tex",
                  specular="0", emission=".05")

    module = ET.SubElement(world, "body", name="dial_module", pos=vector([0, 0, table_z]))
    ET.SubElement(module, "freejoint", name="dial_module_free")
    sx, sy = m["stem_width_m"]
    ET.SubElement(module, "geom", name="dial_module_stem", type="box",
                  pos=vector([m.get("stem_offset_x_m", 0.0), m["stem_offset_y_m"], stem_len / 2]),
                  size=vector([sx / 2, sy / 2, stem_len / 2]), mass=str(m["stem_mass_kg"]),
                  material="module_blue", friction=vector(m["friction"]), condim="4",
                  solref="0.001 1", solimp="0.95 0.99 0.001")
    face_center_z = stem_len + face_t / 2
    ET.SubElement(module, "geom", name="dial_module_disk", type="cylinder",
                  pos=vector([0, 0, face_center_z]),
                  size=vector([m["face_radius_m"], face_t / 2]),
                  mass=str(m["face_mass_kg"]), material="module_white",
                  friction=vector(m["friction"]), condim="4",
                  solref="0.001 1", solimp="0.95 0.99 0.001")
    ET.SubElement(module, "geom", name="dial_module_print", type="cylinder",
                  pos=vector([0, 0, stem_len + face_t + .0003]),
                  size=vector([m["face_radius_m"] * .985, .00025]),
                  material="dial_face_print", contype="0", conaffinity="0")

    rotor = ET.SubElement(module, "body", name="dial_rotor",
                          pos=vector([0, 0, stem_len + face_t / 2 + knob_h / 2]))
    ET.SubElement(rotor, "joint", name="dial_hinge", type="hinge", axis="0 0 1",
                  limited="true", range=vector(m["range_rad"]), springref="0",
                  stiffness=str(m["spring_nm_per_rad"]), damping=str(m["damping_nm_s_per_rad"]))
    ET.SubElement(rotor, "geom", name="dial_red_knob", type="cylinder",
                  size=vector([m["knob_radius_m"], knob_h / 2]),
                  mass=str(m["knob_mass_kg"] * .82), material="module_red",
                  friction=vector(m["friction"]), condim="4")
    for i in range(12):
        theta = i * math.pi / 6
        r = m["knob_radius_m"] * .91
        ET.SubElement(rotor, "geom", name=f"dial_red_knob_grip_{i}", type="box",
                      pos=vector([r * math.cos(theta), r * math.sin(theta), 0]),
                      size=vector([.0035, .0045, knob_h * .39]), euler=vector([0, 0, theta]),
                      mass=str(m["knob_mass_kg"] * .18 / 12), material="module_red",
                      friction=vector(m["friction"]), condim="4")
    pointer_len = min(.102, m["face_radius_m"] * .82)
    pointer_z = knob_h / 2 + .0012
    ET.SubElement(rotor, "geom", name="dial_orange_pointer", type="box",
                  pos=vector([0, pointer_len / 2, pointer_z]),
                  size=vector([.004, pointer_len / 2, .001]), rgba="1 .31 .035 1",
                  mass=".0001", contype="0", conaffinity="0")
    ET.SubElement(rotor, "geom", name="dial_pointer_tip", type="sphere",
                  pos=vector([0, pointer_len, pointer_z]), size=".007",
                  rgba="1 .31 .035 1", mass=".0001", contype="0", conaffinity="0")
    ET.SubElement(rotor, "site", name="dial_axis_site", size=".003", rgba="0 0 0 0")

    _camera(world, "module_global", [1.1, -1.55, 1.75], np.array([0, 0, .60]), 46)
    _camera(world, "module_side", [1.8, -.5, 1.0], np.array([0, 0, .60]), 43)
    _camera(world, "module_front", [0, -1.8, 1.15], np.array([0, 0, .60]), 42)
    ET.SubElement(world, "camera", name="module_top", pos="0 0 2.25",
                  xyaxes="1 0 0 0 1 0", fovy="43")
    root.find("statistic").attrib.update(center="0 0 .60", extent="2.6")


def build(directory=DEFAULT_DIRECTORY):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "assets").mkdir(exist_ok=True)
    p = parameters()
    env = environment(p)
    root = build_xml(env, directory)
    root.set("model", "RoboFrontier | Lifted Horizontal Spring Dial | two-arm preview")
    add_module(root, directory, p)
    scene_path = directory / "scene.xml"

    def save_scene():
        ET.indent(root)
        ET.ElementTree(root).write(scene_path, encoding="unicode")

    save_scene()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = initialize(model, env)
    module_id = model.body("dial_module").id
    initial_position = data.xpos[module_id].copy()
    initial_quat = data.xquat[module_id].copy()
    stem_geom = model.geom("dial_module_stem").id
    stem_size = model.geom_size[stem_geom].copy()
    corners = np.asarray([[x, y, z]
                          for x in (-stem_size[0], stem_size[0])
                          for y in (-stem_size[1], stem_size[1])
                          for z in (-stem_size[2], stem_size[2])])
    min_stem_clearance = float("inf")
    max_translation = 0.0
    for _ in range(round(5.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        center = data.geom(stem_geom).xpos
        rotation = data.geom(stem_geom).xmat.reshape(3, 3)
        min_stem_clearance = min(min_stem_clearance,
                                 float((corners @ rotation.T + center)[:, 2].min() - table_height(p)))
        max_translation = max(max_translation,
                              float(np.linalg.norm(data.xpos[module_id] - initial_position)))
    mujoco.mj_forward(model, data)
    deep_contacts = [{"a": model.geom(c.geom1).name, "b": model.geom(c.geom2).name,
                      "penetration_m": float(-c.dist)} for c in data.contact if c.dist < -.001]
    stem_table_contact = any(
        {model.geom(c.geom1).name, model.geom(c.geom2).name} == {"dial_module_stem", "table_0"}
        for c in data.contact
    )
    data.time = 0
    add_initial_keyframe(root, model, data)
    save_scene()

    for camera in ("module_global", "module_side", "module_front", "module_top"):
        with mujoco.Renderer(model, height=800, width=1200) as renderer:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(directory / f"preview-{camera.removeprefix('module_')}.png")

    final_quat = data.xquat[module_id].copy()
    angle = float(np.degrees(2 * np.arccos(np.clip(abs(float(initial_quat @ final_quat)), 0, 1))))
    report = {
        "passed": (not deep_contacts and stem_table_contact and min_stem_clearance >= -.001
                   and max_translation < .003 and angle < 1.0),
        "robot_count": 2, "module_count": 1, "module_is_free_body": True,
        "dial_hinge_actuators": 0, "total_actuators": model.nu,
        "disk_orientation": "horizontal_face_up",
        "stem_table_contact": stem_table_contact,
        "stem_minimum_table_clearance_m": min_stem_clearance,
        "module_max_translation_during_settle_m": max_translation,
        "module_orientation_change_during_settle_deg": angle,
        "initial_penetrations_over_1mm": deep_contacts,
        "settling_duration_s": 5.0,
        "scope": "Scene-load and passive settling only; no arm manipulation or LLM policy trial.",
    }
    _write_json(directory / "parameters.json", p)
    _write_json(directory / "environment.json", env)
    _write_json(directory / "validation.json", report)
    (directory / "README.md").write_text(
        "# Lifted Horizontal Spring-Dial Module · two-arm scene\n\n"
        "A red spring-return knob sits on top of a thin, horizontal white dial disk. A long, slim blue rectangular stem "
        "extends vertically down from the disk; its free tip rests on the tabletop. The entire module is a free body, "
        "not fixed to the table. The intended test is for one Panda to pinch the stem and lift the module clear, "
        "while the other grasps and turns the red knob.\n\n"
        "The dial rotates only through physical finger contact. It has no actuator or hidden angle command. "
        "This folder contains an environment preview, not a policy task. `validation.json` covers five seconds of passive settling only.\n\n"
        "Inspect `preview-global.png`, `preview-side.png`, `preview-front.png`, and `preview-top.png`.\n"
    )
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if not report["passed"]:
        raise SystemExit("Horizontal lifted-dial scene failed initial validation")
    return report


def table_height(p):
    return p["table"]["height_m"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY)
    args = parser.parse_args()
    build(args.directory)


if __name__ == "__main__":
    main()
