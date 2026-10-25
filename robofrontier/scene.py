"""Import the unmodified upstream assets and assemble the tabletop scene."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def vector(values):
    return " ".join(str(float(x)) for x in values)


def build_scene(config):
    source = ROOT / "vendor/panda/panda.xml"
    if not source.exists():
        raise FileNotFoundError("Panda assets missing. Run: uv run python scripts/fetch_robot.py")
    root = ET.parse(source).getroot()
    root.set("model", "RoboFrontier Mini — Panda Tabletop")
    root.find("compiler").set("meshdir", str(source.parent / "assets"))
    root.remove(root.find("keyframe"))
    option = root.find("option")
    option.attrib.update(timestep=str(config["timestep_s"]), gravity="0 0 -9.81",
                         integrator="implicitfast", solver="Newton", iterations="100",
                         noslip_iterations=str(config.get("noslip_iterations", 0)))
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="800")
    ET.SubElement(visual, "quality", shadowsize="2048")
    ET.SubElement(visual, "headlight", diffuse="0.5 0.5 0.5", ambient="0.25 0.25 0.25")
    asset = root.find("asset")
    background_profile = config.get("background_profile", "plain")
    if background_profile == "plain":
        ET.SubElement(asset, "texture", name="floor_tex", type="2d", builtin="checker",
                      rgb1="0.13 0.16 0.20", rgb2="0.16 0.19 0.23", width="512", height="512")
    elif background_profile == "complex_texture":
        textures = ROOT / "assets/backgrounds"
        ET.SubElement(asset, "texture", name="skybox_tex", type="skybox", builtin="gradient",
                      rgb1="0.20 0.42 0.72", rgb2="0.90 0.66 0.42",
                      width="256", height="256")
        ET.SubElement(asset, "texture", name="floor_tex", type="2d",
                      file=str(textures / "floor-mottled.png"))
        ET.SubElement(asset, "texture", name="table_tex", type="2d",
                      file=str(textures / "table-terrazzo.png"))
        ET.SubElement(asset, "material", name="table_mat", texture="table_tex",
                      texrepeat="2 2", reflectance="0")
    else:
        raise ValueError(f"Unknown background_profile: {background_profile}")
    ET.SubElement(asset, "material", name="floor_mat", texture="floor_tex", texrepeat="4 4", reflectance="0")
    object_texture_profile = config.get("object_texture_profile", "solid")
    if object_texture_profile not in ("solid", "multicolor_dominant", "achromatic_patterns"):
        raise ValueError(f"Unknown object_texture_profile: {object_texture_profile}")
    if object_texture_profile in ("multicolor_dominant", "achromatic_patterns"):
        textures = ROOT / "assets/backgrounds"
        pattern_files = {"blue": "cube-blue.png", "red": "cube-red.png", "green": "cube-green.png"}
        if object_texture_profile == "achromatic_patterns":
            pattern_files = {"blue": "cube-dotted.png", "red": "cube-checkered.png",
                             "green": "cube-striped.png"}
        for name in config["cubes"]:
            ET.SubElement(asset, "texture", name=f"cube_{name}_tex", type="cube",
                          file=str(textures / pattern_files[name]))
            ET.SubElement(asset, "material", name=f"cube_{name}_mat",
                          texture=f"cube_{name}_tex", reflectance="0")
    world = root.find("worldbody")
    for light in list(world.findall("light")):
        world.remove(light)
    ET.SubElement(world, "light", pos="0.3 -0.5 2.2", dir="0 0 -1", diffuse="0.8 0.8 0.8", castshadow="true")
    ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1", material="floor_mat")
    height = config["table_height_m"]
    world.find("body[@name='link0']").set("pos", vector([0, 0, height]))
    table_attributes = {"name": "table", "type": "box", "size": "0.60 0.40 0.025",
                        "pos": vector([0.25, 0, height - 0.025]),
                        "friction": vector(config["contact_friction"]), "condim": "4"}
    if background_profile == "complex_texture":
        table_attributes.update(material="table_mat", rgba="1 1 1 1")
    else:
        table_attributes["rgba"] = "0.58 0.45 0.31 1"
    ET.SubElement(world, "geom", **table_attributes)
    for x in (-0.25, 0.75):
        for y in (-0.32, 0.32):
            ET.SubElement(world, "geom", type="box", size=vector([0.025, 0.025, (height - 0.05) / 2]),
                          pos=vector([x, y, (height - 0.05) / 2]), rgba="0.19 0.22 0.26 1")
    colors = {"blue": "0.08 0.35 0.95 1", "red": "0.90 0.10 0.10 1", "green": "0.08 0.70 0.22 1"}
    half = config["cube_size_m"] / 2
    shape_profile = config.get("object_shape_profile", "cube")
    if shape_profile not in ("cube", "straight_socket_pair"):
        raise ValueError(f"Unknown object_shape_profile: {shape_profile}")
    for name, xy in config["cubes"].items():
        if shape_profile == "straight_socket_pair":
            tray = name == "tray"
            bottom_z = 0.006 if tray else 0.0
            body = ET.SubElement(world, "body", name=f"cube_{name}",
                                 pos=vector([*xy, height + bottom_z]))
            ET.SubElement(body, "freejoint", name=f"cube_{name}_joint")
            rgba = "0.68 0.68 0.68 1"
            friction = vector(config["contact_friction"])
            if tray:
                # Square tray: 70 mm outside, 46 mm recess, straight walls.
                ET.SubElement(body, "geom", name="cube_tray_geom", type="box",
                              size="0.035 0.035 0.006", mass="0.030", rgba=rgba,
                              friction=friction, condim="4")
                for name_suffix, pos, size in (
                    ("wall_xn", "-0.029 0 0.018", "0.006 0.035 0.012"),
                    ("wall_xp", "0.029 0 0.018", "0.006 0.035 0.012"),
                    ("wall_yn", "0 -0.029 0.018", "0.023 0.006 0.012"),
                    ("wall_yp", "0 0.029 0.018", "0.023 0.006 0.012"),
                ):
                    ET.SubElement(body, "geom", name=f"cube_tray_{name_suffix}", type="box",
                                  pos=pos, size=size, mass="0.005", rgba=rgba,
                                  friction=friction, condim="4")
            else:
                # Cap with a square tenon; the tenon fits inside the tray recess.
                ET.SubElement(body, "geom", name="cube_peg_geom", type="box",
                              pos="0 0 0.030", size="0.027 0.027 0.008", mass="0.035",
                              rgba=rgba, friction=friction, condim="4")
                ET.SubElement(body, "geom", name="cube_peg_tenon", type="box",
                              pos="0 0 0.011", size="0.017 0.017 0.011", mass="0.015",
                              rgba=rgba, friction=friction, condim="4")
            continue
        body = ET.SubElement(world, "body", name=f"cube_{name}", pos=vector([*xy, height + half]))
        ET.SubElement(body, "freejoint", name=f"cube_{name}_joint")
        cube_geom = {"name": f"cube_{name}_geom", "type": "box", "size": vector([half] * 3),
                     "mass": str(config["cube_mass_kg"]),
                     "friction": vector(config["contact_friction"]), "condim": "4"}
        if object_texture_profile in ("multicolor_dominant", "achromatic_patterns"):
            cube_geom.update(material=f"cube_{name}_mat", rgba="1 1 1 1")
        else:
            cube_geom["rgba"] = colors[name]
        ET.SubElement(body, "geom", **cube_geom)
    hand = root.find(".//body[@name='hand']")
    # 58.4 mm hand→finger origin + 44.5 mm to fingertip pad center.
    ET.SubElement(hand, "site", name="tcp", pos="0 0 0.1029", size="0.004", rgba="1 0.85 0.1 0.7", group="1")
    position = np.array(config["camera_position_m"])
    z = position - np.array(config["camera_target_m"])
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(world, "camera", name="front", pos=vector(position), xyaxes=vector([*x, *y]), fovy="43")
    side_position = np.array([1.15, -1.1, 1.25])
    z = side_position - np.array([0.28, 0, 0.70])
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(world, "camera", name="side", pos=vector(side_position), xyaxes=vector([*x, *y]), fovy="43")
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml), xml
