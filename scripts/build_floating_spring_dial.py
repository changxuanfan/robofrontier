"""Build a two-arm MuJoCo preview with one movable spring-dial module."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from robofrontier.cube_transport import ROOT, _camera, add_initial_keyframe, build_xml, initialize
from robofrontier.scene import vector


DEFAULT_DIRECTORY = ROOT / "configs/environments/spring_dial/floating-module-01"

PARAMETERS = {
    "id": "spring-dial-floating-module-01",
    "table": {"length_m": 1.90, "width_m": 2.06, "height_m": 0.40, "thickness_m": 0.05},
    "arms": [
        {"id": "A", "side": "south-west", "base_position_m": [-0.25, -0.65, 0.40],
         "yaw_rad": math.pi / 2, "home_tcp_m": [-0.25, -0.42, 0.82]},
        {"id": "B", "side": "north", "base_position_m": [0.0, 0.65, 0.40],
         "yaw_rad": -math.pi / 2, "home_tcp_m": [0.0, 0.42, 0.82]},
    ],
    "module": {
        "center_xy_m": [0.0, 0.0],
        "stem_length_m": 0.30,
        "stem_width_m": [0.016, 0.020],
        "stem_offset_y_m": -0.030,
        "stem_mass_kg": 0.75,
        "face_radius_m": 0.115,
        "face_thickness_m": 0.008,
        "face_mass_kg": 0.30,
        "knob_radius_m": 0.027,
        "knob_height_m": 0.040,
        "knob_mass_kg": 0.12,
        "spring_nm_per_rad": 0.03,
        "damping_nm_s_per_rad": 0.003,
        "range_rad": [-math.pi, math.pi],
        "friction": [0.8, 0.005, 0.0001],
        "stem_rgba": [0.045, 0.34, 0.92, 1.0],
        "knob_rgba": [0.78, 0.035, 0.025, 1.0],
    },
    "physics": {
        "timestep_s": 0.001,
        "gravity_m_s2": [0.0, 0.0, -9.81],
        "contact_friction": [1.0, 0.005, 0.0001],
        "noslip_iterations": 0,
    },
}


def _bold_font(size):
    try:
        return ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", size, index=1)
    except OSError:
        try:
            return ImageFont.truetype("DejaVuSans-Bold.ttf", size)
        except OSError:
            return ImageFont.load_default(size=size)


def _write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def _make_face_texture(path):
    size = 1024
    image = Image.new("RGB", (size, size), (248, 247, 241))
    draw = ImageDraw.Draw(image)
    cx = cy = size // 2
    draw.ellipse((18, 18, size-18, size-18), fill=(220, 216, 205))
    draw.ellipse((36, 36, size-36, size-36), fill=(249, 248, 242))
    for degrees in range(0, 360, 5):
        theta = math.radians(degrees)
        outer = 405
        inner = 354 if degrees % 90 else 325
        x1, y1 = cx + inner * math.sin(theta), cy - inner * math.cos(theta)
        x2, y2 = cx + outer * math.sin(theta), cy - outer * math.cos(theta)
        width = 12 if degrees % 90 == 0 else 5 if degrees % 15 == 0 else 2
        draw.line((x1, y1, x2, y2), fill=(20, 23, 27), width=width)
    labels = (("0", 0), ("90", 90), ("180", 180), ("270", 270))
    font = _bold_font(82)
    for text, degrees in labels:
        theta = math.radians(degrees)
        radius = 275
        x, y = cx + radius * math.sin(theta), cy - radius * math.cos(theta)
        draw.text((x, y), text, anchor="mm", font=font, fill=(12, 15, 19))
    image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(path)


def _environment(p):
    table = p["table"]
    stations = []
    for arm in p["arms"]:
        stations.append({
            **arm,
            "pair": 1,
            "source_positions_m": [],
            "goal_positions_m": [],
            "parking_positions_m": [],
        })
    return {
        "schema": "robofrontier.spring_dial.floating_module.v1",
        "id": p["id"],
        "collection": "spring_dial",
        "arm_count": 2,
        "topology": "one_facing_pair",
        "stations": stations,
        "module": {"id": "dial_module", "center_xy_m": p["module"]["center_xy_m"]},
        "physics": p["physics"],
        "tables": [{
            "center_m": [0.0, 0.0, table["height_m"] - table["thickness_m"] / 2],
            "half_size_m": [table["length_m"] / 2, table["width_m"] / 2,
                            table["thickness_m"] / 2],
        }],
        "floor_rgba": [0.84, 0.89, 0.94, 1.0],
        "table_rgba": [0.64, 0.47, 0.35, 1.0],
        "edges": [],
        "objects": [],
        "initial_keyframe": "initial",
        "scene": "scene.xml",
    }


def _add_module(root, directory, p):
    world, asset = root.find("worldbody"), root.find("asset")
    table_z = p["table"]["height_m"]
    m = p["module"]
    stem_len = m["stem_length_m"]
    face_t = m["face_thickness_m"]
    knob_h = m["knob_height_m"]
    table_geom = root.find(".//geom[@name='table_0']")
    table_geom.attrib.update(solref="0.001 1", solimp="0.95 0.99 0.001")

    ET.SubElement(asset, "material", name="module_blue",
                  rgba=vector(m["stem_rgba"]), specular=".22", shininess=".3", emission=".035")
    ET.SubElement(asset, "material", name="module_white",
                  rgba=".96 .95 .91 1", specular=".12", shininess=".2")
    ET.SubElement(asset, "material", name="module_red",
                  rgba=vector(m["knob_rgba"]), specular=".28", shininess=".35")
    _make_face_texture(directory / "assets" / "dial_face.png")
    ET.SubElement(asset, "texture", name="dial_face_tex", type="2d", file="assets/dial_face.png")
    ET.SubElement(asset, "material", name="dial_face_print", texture="dial_face_tex",
                  specular="0", emission=".05")

    # The upright disk rests on its circular rim. The slim, offset blue handle
    # projects from the back face, parallel to the disk axis and clear of the table.
    start_center = [*m["center_xy_m"], table_z + m["face_radius_m"]]
    half_turn = math.sqrt(.5)
    module = ET.SubElement(world, "body", name="dial_module", pos=vector(start_center),
                           quat=vector([half_turn, half_turn, 0, 0]))
    ET.SubElement(module, "freejoint", name="dial_module_free")
    sx, sy = m["stem_width_m"]
    ET.SubElement(module, "geom", name="dial_module_stem", type="box",
                  pos=vector([0, m["stem_offset_y_m"], -(face_t / 2 + stem_len / 2)]),
                  size=vector([sx / 2, sy / 2, stem_len / 2]),
                  mass=str(m["stem_mass_kg"]), material="module_blue",
                  friction=vector(m["friction"]), condim="4",
                  solref="0.001 1", solimp="0.95 0.99 0.001")
    ET.SubElement(module, "geom", name="dial_module_disk", type="cylinder",
                  pos="0 0 0",
                  size=vector([m["face_radius_m"], face_t / 2]),
                  mass=str(m["face_mass_kg"]), material="module_white",
                  friction=vector(m["friction"]), condim="4",
                  solref="0.001 1", solimp="0.95 0.99 0.001")
    ET.SubElement(module, "geom", name="dial_module_print", type="cylinder",
                  pos=vector([0, 0, face_t / 2 + 0.0003]),
                  size=vector([m["face_radius_m"] * .985, .00025]),
                  material="dial_face_print", contype="0", conaffinity="0")

    rotor = ET.SubElement(module, "body", name="dial_rotor",
                          pos=vector([0, 0, face_t / 2 + knob_h / 2]))
    ET.SubElement(rotor, "joint", name="dial_hinge", type="hinge", axis="0 0 1",
                  limited="true", range=vector(m["range_rad"]), springref="0",
                  stiffness=str(m["spring_nm_per_rad"]), damping=str(m["damping_nm_s_per_rad"]))
    ET.SubElement(rotor, "geom", name="dial_red_knob", type="cylinder",
                  size=vector([m["knob_radius_m"], knob_h / 2]),
                  mass=str(m["knob_mass_kg"] * .82), material="module_red",
                  friction=vector(m["friction"]), condim="4")
    for i in range(12):
        theta = i * math.pi / 6
        tooth_r = m["knob_radius_m"] * .91
        ET.SubElement(rotor, "geom", name=f"dial_red_knob_grip_{i}", type="box",
                      pos=vector([tooth_r * math.cos(theta), tooth_r * math.sin(theta), 0]),
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

    # Front shows the dial marks; side shows the rear handle and its clearance.
    _camera(world, "module_global", [1.25, -1.85, 1.25], np.array([0, 0, .52]), 44)
    _camera(world, "module_front", [.72, -1.65, .90], np.array([0, 0, .515]), 38)
    _camera(world, "module_side", [2.0, 0, .80], np.array([0, 0, .515]), 42)
    _camera(world, "module_close", [.65, -1.15, .78], np.array([0, 0, .515]), 34)
    ET.SubElement(world, "camera", name="module_top", pos="0 0 2.45",
                  xyaxes="1 0 0 0 1 0", fovy="43")
    root.find("statistic").attrib.update(center="0 0 .53", extent="2.5")


def build(directory=DEFAULT_DIRECTORY):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "assets").mkdir(exist_ok=True)
    p = json.loads(json.dumps(PARAMETERS))
    env = _environment(p)
    root = build_xml(env, directory)
    root.set("model", "RoboFrontier | Floating Spring Dial Module | two-arm preview")
    _add_module(root, directory, p)
    scene_path = directory / "scene.xml"

    def save_scene():
        ET.indent(root)
        ET.ElementTree(root).write(scene_path, encoding="unicode")

    save_scene()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = initialize(model, env)
    module_free_id = model.joint("dial_module_free").id
    module_qadr = model.jnt_qposadr[module_free_id]
    initial_quat = data.qpos[module_qadr + 3:module_qadr + 7].copy()
    initial_position = data.body("dial_module").xpos.copy()
    stem_geom_id = model.geom("dial_module_stem").id
    stem_half_size = model.geom_size[stem_geom_id].copy()
    stem_corners = np.asarray([
        [x, y, z]
        for x in (-stem_half_size[0], stem_half_size[0])
        for y in (-stem_half_size[1], stem_half_size[1])
        for z in (-stem_half_size[2], stem_half_size[2])
    ])

    def current_stem_clearance():
        center = data.geom(stem_geom_id).xpos.copy()
        rotation = data.geom(stem_geom_id).xmat.reshape(3, 3)
        lowest_z = float((stem_corners @ rotation.T + center)[..., 2].min())
        return lowest_z - p["table"]["height_m"]

    settling_duration_s = 60.0
    min_stem_clearance_m = float("inf")
    max_translation_m = 0.0
    max_rotation_deg = 0.0
    stem_end_contact_first_time_s = None
    stem_end_contact_seen = False
    stem_contact_local_z_m = None
    for _ in range(round(settling_duration_s / model.opt.timestep)):
        mujoco.mj_step(model, data)
        min_stem_clearance_m = min(min_stem_clearance_m, current_stem_clearance())
        max_translation_m = max(max_translation_m,
                                float(np.linalg.norm(data.body("dial_module").xpos - initial_position)))
        q = data.qpos[module_qadr + 3:module_qadr + 7]
        dot = float(np.clip(abs(np.dot(initial_quat, q)), 0.0, 1.0))
        max_rotation_deg = max(max_rotation_deg, float(np.degrees(2 * np.arccos(dot))))
        geom_center = data.geom(stem_geom_id).xpos.copy()
        geom_rotation = data.geom(stem_geom_id).xmat.reshape(3, 3)
        for contact in data.contact:
            names = {model.geom(contact.geom1).name, model.geom(contact.geom2).name}
            if names != {"dial_module_stem", "table_0"}:
                continue
            point_local = geom_rotation.T @ (contact.pos - geom_center)
            stem_contact_local_z_m = float(point_local[2])
            # The distal cap is local -Z because the handle projects from the disk along -Z.
            if abs(stem_contact_local_z_m + stem_half_size[2]) <= .025:
                stem_end_contact_seen = True
                if stem_end_contact_first_time_s is None:
                    stem_end_contact_first_time_s = float(data.time)
    mujoco.mj_forward(model, data)

    contacts = []
    for c in data.contact:
        if c.dist < -.001:
            contacts.append({
                "geom_a": model.geom(c.geom1).name,
                "geom_b": model.geom(c.geom2).name,
                "penetration_m": float(-c.dist),
            })
    settled_position = data.body("dial_module").xpos.copy()
    final_quat = data.qpos[module_qadr + 3:module_qadr + 7].copy()
    quat_dot = float(np.clip(abs(np.dot(initial_quat, final_quat)), 0.0, 1.0))
    orientation_change_deg = float(np.degrees(2 * np.arccos(quat_dot)))
    disk_table_contact = any(
        {model.geom(c.geom1).name, model.geom(c.geom2).name}
        == {"dial_module_disk", "table_0"}
        for c in data.contact
    )
    stem_table_contact = any(
        {model.geom(c.geom1).name, model.geom(c.geom2).name}
        == {"dial_module_stem", "table_0"}
        for c in data.contact
    )
    stem_clearance_m = min_stem_clearance_m
    stability = {
        "module_translation_from_start_m": float(np.linalg.norm(settled_position - initial_position)),
        "module_max_translation_during_settle_m": max_translation_m,
        "module_final_position_m": settled_position.tolist(),
        "module_orientation_change_deg": orientation_change_deg,
        "module_max_orientation_change_during_settle_deg": max_rotation_deg,
        "disk_rim_contacts_table": disk_table_contact,
        "stem_contacts_table": stem_table_contact,
        "stem_end_contact_seen": stem_end_contact_seen,
        "stem_end_contact_first_time_s": stem_end_contact_first_time_s,
        "stem_contact_local_z_m": stem_contact_local_z_m,
        "stem_minimum_table_clearance_m": float(stem_clearance_m),
        "dial_angle_after_settle_deg": float(np.degrees(data.joint("dial_hinge").qpos[0])),
    }
    data.time = 0
    add_initial_keyframe(root, model, data)
    save_scene()

    for camera in ("module_global", "module_front", "module_side", "module_close"):
        with mujoco.Renderer(model, height=900, width=1280) as renderer:
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(directory / f"preview-{camera.removeprefix('module_')}.png")

    report = {
        "passed": not contacts and bool(np.isfinite(data.qpos).all())
                  and stem_table_contact and stem_end_contact_seen and stem_clearance_m >= -.001,
        "robot_count": 2,
        "module_count": 1,
        "module_is_free_body": True,
        "dial_hinge_actuators": 0,
        "total_actuators": model.nu,
        "initial_penetrations_over_1mm": contacts,
        "settling_duration_s": settling_duration_s,
        "settling": stability,
        "scope": "Scene-load, contact and passive-settling check only; no arm manipulation or LLM policy trial.",
    }
    _write_json(directory / "parameters.json", p)
    _write_json(directory / "environment.json", env)
    _write_json(directory / "validation.json", report)
    (directory / "README.md").write_text(
        "# Floating Spring-Dial Module · two-arm scene\n\n"
        "One movable tabletop module: a red passive spring-return knob is mounted on the front of a thin white marked disk. "
        "The 8 mm disk starts on its rim. A slim 16 x 20 mm blue rectangular handle, 300 mm long, is attached off-center "
        "in the lower half of its back face. Its mass makes the free module tip under gravity until the distal end of the blue handle "
        "contacts the tabletop. The module is not welded or propped up; this motion comes from free-body dynamics and contact geometry.\n\n"
        "The scene includes two Panda arms facing each other across the disk. The intended manipulation concept is one arm "
        "stabilizing the blue rear handle while the other grasps and turns the red knob. That behavior has not yet "
        "been tested. No task policy, dial-angle command, or LLM trial is included.\n\n"
        "Inspect `preview-global.png`, `preview-front.png`, `preview-side.png`, and `preview-close.png`. "
        "`validation.json` records scene loading, distal handle contact, and 60 seconds of passive settling.\n"
    )
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if not report["passed"]:
        raise SystemExit("Floating dial module failed initial scene validation")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY)
    args = parser.parse_args()
    build(args.directory)


if __name__ == "__main__":
    main()
