#!/usr/bin/env python3
# Copyright 2026 jo0625
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Import simple_v6 CAD geometry while preserving the active ROS 2 sensors."""

import argparse
import copy
import math
import shutil
import struct
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

from split_flanged_wheels import split_flanged_wheels


SOURCE_URDF = Path("urdf/simple_v6.urdf")
SOURCE_LABEL = "feature/hardware:hardware/simple_v6@37ce04e"

JOINT_NAMES = {
    "w1": "w1_joint",
    "w2": "w2_joint",
    "w3": "w3_joint",
    "w4": "w4_joint",
    "yaw": "yaw_joint",
    "ph": "pitch_joint",
    "ld": "leida_joint",
    "xj1": "xj1_joint",
    "xj2": "xj2_joint",
    "xj3": "xj3_joint",
    "xj4": "xj4_joint",
}

HARDWARE_LINKS = {
    "base_link",
    "w1",
    "w2",
    "w3",
    "w4",
    "yuntai",
    "pitch",
    "leida",
    "xj1",
    "xj2",
    "xj3",
    "xj4",
}

# leida.STL is required as a CAD ownership reference, but it is not a runtime
# visual. The same housing is already assembled into base_link.STL.
RUNTIME_MESH_FILES = HARDWARE_LINKS - {"leida"}

EMBEDDED_MESH_MATCH_DECIMALS = 6
EMBEDDED_MESH_MIN_MATCH_RATIO = 0.99

# Axis-aligned bounds measured from the native simple_v6 binary STL vertices.
BOX_COLLISIONS = {
    "base_link": (
        "0.003428936005 0.032983805984 -0.038154035807",
        "0.475999981165 0.276014514267 0.464500010014",
    ),
    "yuntai": (
        "0 -0.064619481564 -0.077299998142",
        "0.140000000596 0.218761026859 0.191999999806",
    ),
    "pitch": (
        "-0.077299998142 0.038299999200 -0.021951824427",
        "0.193599997088 0.122999997810 0.234749644995",
    ),
    "leida": (
        "0 0.002000000328 0.016799999634",
        "0.100000001490 0.061999998987 0.046399998944",
    ),
    "xj1": (
        "0.000074910000 0.069820364937 0.019108943641",
        "0.037850178778 0.057438004762 0.120107173920",
    ),
    "xj2": (
        "0.000074910000 0.069820364937 0.019108943641",
        "0.037850178778 0.057438004762 0.120107173920",
    ),
    "xj3": (
        "0.000074910000 0 0.052000000142",
        "0.037850178778 0.037925016135 0.115999998525",
    ),
    "xj4": (
        "0.000074910000 0 0.052000000142",
        "0.037850178778 0.037925016135 0.115999998525",
    ),
}

# Primitive sections measured from the disconnected 520-triangle flanged-wheel
# components, expressed in base_link coordinates. The complete w1-w4 meshes also
# contain drive motors, so their overall bounds cannot describe rail contact.
# Each tread bears on the rail top; each larger flange runs just inside the rail.
WHEEL_CONTACTS = {
    "w1": {
        "tread": "0.254228939952 0.136999998856323 0.134095974152992",
        "flange": "0.238228939952 0.136999998856323 0.134095974152992",
    },
    "w2": {
        "tread": "-0.247371060048 0.136999998934387 0.134095970636500",
        "flange": "-0.231371060048 0.136999998934387 0.134095970636500",
    },
    "w3": {
        "tread": "-0.247371060048 0.136999999067289 -0.155904029760880",
        "flange": "-0.231371060048 0.136999999067289 -0.155904029760880",
    },
    "w4": {
        "tread": "0.254228939952 0.136999999516020 -0.155904029536466",
        "flange": "0.238228939952 0.136999999516020 -0.155904029536466",
    },
}
WHEEL_SECTION_GEOMETRY = {
    "tread": ("0.060", "0.019"),
    "flange": ("0.0775", "0.013"),
}
WHEEL_CYLINDER_RPY = "0 1.570796326795 0"

# The exported wheel joints are attached to their motor assemblies rather than
# the flange axes. Move each joint onto the tread axis and compensate the mesh
# origin so the zero-position assembly remains unchanged.
WHEEL_VISUAL_ORIGINS = {
    "w1": "-0.033518838253 0.005699772312 -0.101900000000",
    "w2": "0.033518837740 0.005699775792 -0.101900000000",
    "w3": "0.016381944458 0.029793150762 -0.101900000000",
    "w4": "-0.016381944871 0.029793151047 -0.101900000000",
}
WHEEL_AXIS_BY_NAME = {
    "w1_joint": "0 0 -1",
    "w2_joint": "0 0 1",
    "w3_joint": "0 0 1",
    "w4_joint": "0 0 -1",
}
LEFT_WHEEL_JOINTS = ("w2_joint", "w3_joint")
RIGHT_WHEEL_JOINTS = ("w1_joint", "w4_joint")
WHEEL_MASS = "0.5"
WHEEL_INERTIA_TRANSVERSE = "0.000812421666667"
WHEEL_INERTIA_AXIAL = "0.0015015625"

# The V6 export flipped three camera housings relative to the inspection layout.
# These rotations restore xj1/xj2 as outward-looking side cameras and xj3/xj4
# as downward-looking track cameras while retaining the V6 mounting positions.
CAMERA_HOUSING_RPY = {
    "xj1_joint": "3.14159265358979 -1.5707963267949 0",
    "xj2_joint": "3.14159265358979 1.5707963267949 0",
    "xj3_joint": "-1.5707963267949 -1.5707963267949 0",
    "xj4_joint": "-1.5707963267949 1.5707963267949 0",
}

# Camera-body frames use +X forward. These transforms place +X on each STL's
# optical axis and restore the previously validated lens-center offsets. The
# pitch camera is already part of pitch.STL, so its sensor frame is positioned
# directly at that mesh's lens face instead of using the obsolete detached link.
PRESERVED_JOINT_ORIGINS = {
    # The housing is already assembled into base_link.STL. Its front face is
    # local +Z; place the ray origin 1 mm beyond that face and map lidar +X to it.
    "lidar_joint": (
        "0 0 0.041",
        "1.5707963267949 -1.5707963267949 0",
    ),
    "xj1_camera_joint": (
        "0 0 0.018478",
        "-3.14159265358979 -1.13446401379631 1.5707963267949",
    ),
    "xj2_camera_joint": (
        "0 0 0.018478",
        "-3.14159265358979 -1.13446401379631 1.5707963267949",
    ),
    "xj3_camera_joint": (
        "0 0 0.077",
        "-1.5707963267949 -1.5707963267949 0",
    ),
    "xj4_camera_joint": (
        "0 0 0.077",
        "-1.5707963267949 -1.5707963267949 0",
    ),
    # Articulate the pitch assembly 75 degrees above the robot +X axis. Its
    # 34.5-degree vertical FOV then includes the tunnel crown at 90 degrees.
    "pitch_joint": (
        "0 0.01 0",
        "-1.83259571459394 1.5707963267949 0",
    ),
    "pitch_camera_joint": ("0 0 0", "0 0 0"),
    "pitch_camera_sensor_joint": (
        "-0.077300002798 0.003299999982 0.087818765",
        "1.5707963267949 -1.5707963267949 0",
    ),
}

# Rz(-90 deg) * Rx(-90 deg) maps CAD Y-up into REP-103 and makes the physical
# Odin1/nose end point along base_footprint +X. Translation centers the wheel
# pairs on the track and puts the lowest V6 tire vertex on base_footprint z=0.
ROOT_XYZ = "0.010904028627 0.003428939952 0.214499211212"
ROOT_RPY = "-1.570796326795 0 -1.570796326795"

PRESERVED_SENSOR_LINKS = (
    "lidar_link",
    "imu_link",
    "odin1_rgb_optical_frame",
    "xj1_camera_link",
    "xj1_optical_frame",
    "xj2_camera_link",
    "xj2_optical_frame",
    "xj3_camera_link",
    "xj3_optical_frame",
    "xj4_camera_link",
    "xj4_optical_frame",
    "pitch_camera",
    "pitch_camera_sensor_link",
    "pitch_camera_optical_frame",
)

PRESERVED_SENSOR_JOINTS = (
    "lidar_joint",
    "imu_joint",
    "odin1_rgb_optical_joint",
    "xj1_camera_joint",
    "xj1_optical_joint",
    "xj2_camera_joint",
    "xj2_optical_joint",
    "xj3_camera_joint",
    "xj3_optical_joint",
    "xj4_camera_joint",
    "xj4_optical_joint",
    "pitch_camera_joint",
    "pitch_camera_sensor_joint",
    "pitch_camera_optical_joint",
)

CAMERA_SENSOR_NAMES = {
    "odin1_rgb_camera",
    "xj1_camera_sensor",
    "xj2_camera_sensor",
    "xj3_camera_sensor",
    "xj4_camera_sensor",
    "pitch_camera_sensor",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge simple_v6 CAD geometry with the active ROS 2 sensors."
    )
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("sensor_template_urdf", type=Path)
    parser.add_argument("output_urdf", type=Path)
    parser.add_argument("output_mesh_dir", type=Path)
    return parser.parse_args()


def add_origin(parent: ET.Element, xyz: str, rpy: str = "0 0 0") -> None:
    ET.SubElement(parent, "origin", {"xyz": xyz, "rpy": rpy})


def parse_vector(text: str, expected: int, label: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split())
    if len(values) != expected or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain {expected} finite values: {text!r}")
    return values


def rotation_from_rpy(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transform_point(
    point: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rotation: tuple[tuple[float, ...], ...],
) -> tuple[float, float, float]:
    return tuple(
        xyz[row] + sum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    )


def read_binary_stl_triangles(
    path: Path,
) -> list[tuple[tuple[float, float, float], ...]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(
            f"Expected binary STL of {expected_size} bytes, found {len(data)}: {path}"
        )

    triangles = []
    for index in range(triangle_count):
        values = struct.unpack_from("<9f", data, 84 + index * 50 + 12)
        triangle = tuple(
            tuple(values[offset:offset + 3]) for offset in range(0, 9, 3)
        )
        triangles.append(triangle)
    return triangles


def triangle_key(
    triangle: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        sorted(
            tuple(round(value, EMBEDDED_MESH_MATCH_DECIMALS) for value in vertex)
            for vertex in triangle
        )
    )


def validate_embedded_leida(source_urdf: Path, source_mesh_dir: Path) -> None:
    """Prove that base_link owns the installed Odin1 housing before hiding leida."""
    source_root = ET.parse(source_urdf).getroot()
    candidates = [
        joint
        for joint in source_root.findall("joint")
        if (joint.find("child") is not None)
        and joint.find("child").get("link") == "leida"
    ]
    if len(candidates) != 1:
        raise ValueError(
            f"Expected exactly one source joint mounting leida, found {len(candidates)}"
        )
    origin = candidates[0].find("origin")
    if origin is None:
        raise ValueError("The source leida mounting joint has no origin")
    xyz = parse_vector(origin.get("xyz", "0 0 0"), 3, "leida joint xyz")
    rpy = parse_vector(origin.get("rpy", "0 0 0"), 3, "leida joint rpy")
    rotation = rotation_from_rpy(rpy)

    base_triangles = {
        triangle_key(triangle)
        for triangle in read_binary_stl_triangles(source_mesh_dir / "base_link.STL")
    }
    leida_triangles = {
        triangle_key(
            tuple(transform_point(vertex, xyz, rotation) for vertex in triangle)
        )
        for triangle in read_binary_stl_triangles(source_mesh_dir / "leida.STL")
    }
    if not leida_triangles:
        raise ValueError("leida.STL contains no triangles")
    matched = len(base_triangles & leida_triangles)
    ratio = matched / len(leida_triangles)
    if ratio < EMBEDDED_MESH_MIN_MATCH_RATIO:
        raise ValueError(
            "Refusing to hide leida.STL: base_link.STL does not contain the "
            f"installed Odin1 housing ({matched}/{len(leida_triangles)} triangles, "
            f"{ratio:.2%} matched; required {EMBEDDED_MESH_MIN_MATCH_RATIO:.0%}). "
            "Review the new CAD export ownership before importing it."
        )
    print(
        "Verified Odin1 visual ownership: base_link.STL contains "
        f"{matched}/{len(leida_triangles)} ({ratio:.2%}) leida triangles"
    )


def replace_collision(link: ET.Element) -> None:
    name = link.get("name", "")
    for collision in list(link.findall("collision")):
        link.remove(collision)

    if name in WHEEL_CONTACTS:
        for section_name, axial_offset in (("tread", "0"), ("flange", "-0.016")):
            radius, length = WHEEL_SECTION_GEOMETRY[section_name]
            collision = ET.SubElement(
                link, "collision", {"name": f"{name}_{section_name}_collision"}
            )
            add_origin(collision, f"0 0 {axial_offset}")
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                geometry, "cylinder", {"radius": radius, "length": length}
            )
        return

    collision = ET.SubElement(link, "collision", {"name": f"{name}_collision"})
    xyz, size = BOX_COLLISIONS[name]
    add_origin(collision, xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": size})


def configure_rotating_wheel(link: ET.Element) -> None:
    name = link.get("name", "")
    visual = link.find("visual")
    if visual is None:
        raise ValueError(f"Wheel {name} has no visual")
    mesh = visual.find("geometry/mesh")
    if mesh is None:
        raise ValueError(f"Wheel {name} visual has no mesh")
    mesh.set("filename", f"package://metro_description/meshes/{name}_wheel.STL")
    origin = visual.find("origin")
    if origin is None:
        origin = ET.SubElement(visual, "origin")
    origin.attrib.update({"xyz": WHEEL_VISUAL_ORIGINS[name], "rpy": "0 0 0"})

    old_inertial = link.find("inertial")
    if old_inertial is not None:
        link.remove(old_inertial)
    inertial = ET.Element("inertial")
    add_origin(inertial, "0 0 -0.00965")
    ET.SubElement(inertial, "mass", {"value": WHEEL_MASS})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": WHEEL_INERTIA_TRANSVERSE,
            "ixy": "0",
            "ixz": "0",
            "iyy": WHEEL_INERTIA_TRANSVERSE,
            "iyz": "0",
            "izz": WHEEL_INERTIA_AXIAL,
        },
    )
    link.insert(0, inertial)


def append_fixed_wheel_mount(
    base_link: ET.Element, wheel_name: str, joint_origin: ET.Element
) -> None:
    visual = ET.SubElement(
        base_link, "visual", {"name": f"{wheel_name}_fixed_mount_visual"}
    )
    add_origin(
        visual,
        joint_origin.get("xyz", "0 0 0"),
        joint_origin.get("rpy", "0 0 0"),
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {
            "filename":
                f"package://metro_description/meshes/{wheel_name}_mount.STL"
        },
    )
    material = ET.SubElement(visual, "material", {"name": ""})
    ET.SubElement(material, "color", {"rgba": "1 1 1 1"})


def clean_preserved_joint(joint: ET.Element) -> None:
    for tag in ("parent", "child"):
        elements = joint.findall(tag)
        if not elements:
            raise ValueError(f"Preserved joint {joint.get('name')} has no {tag}")
        for duplicate in elements[1:]:
            joint.remove(duplicate)


def set_joint_origin(joint: ET.Element, xyz: str, rpy: str) -> None:
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    origin.attrib.update({"xyz": xyz, "rpy": rpy})


def make_fixed_joint(joint: ET.Element) -> None:
    """Lock an unactuated CAD joint at its exported zero position."""
    joint.set("type", "fixed")
    for tag in ("axis", "limit", "dynamics", "calibration", "safety_controller"):
        element = joint.find(tag)
        if element is not None:
            joint.remove(element)


def append_preserved_sensors(root: ET.Element, template: ET.Element) -> None:
    for name in PRESERVED_SENSOR_LINKS:
        element = template.find(f"./link[@name='{name}']")
        if element is None:
            raise ValueError(f"Sensor template is missing link {name}")
        root.append(copy.deepcopy(element))

    for name in PRESERVED_SENSOR_JOINTS:
        element = template.find(f"./joint[@name='{name}']")
        if element is None:
            raise ValueError(f"Sensor template is missing joint {name}")
        copied = copy.deepcopy(element)
        clean_preserved_joint(copied)
        if name in PRESERVED_JOINT_ORIGINS:
            set_joint_origin(copied, *PRESERVED_JOINT_ORIGINS[name])
        root.append(copied)


def configure_hardware_tree(source_urdf: Path, template: ET.Element) -> ET.Element:
    root = ET.parse(source_urdf).getroot()
    root.set("name", "subway_v2")

    found_links = {link.get("name", "") for link in root.findall("link")}
    if found_links != HARDWARE_LINKS:
        raise ValueError(
            f"Unexpected simple_v6 links: expected {sorted(HARDWARE_LINKS)}, "
            f"found {sorted(found_links)}"
        )

    for link in root.findall("link"):
        name = link.get("name", "")
        meshes = link.findall(".//mesh")
        if not meshes:
            raise ValueError(f"Hardware link {name} has no mesh")
        for mesh in meshes:
            mesh.set(
                "filename", f"package://metro_description/meshes/{name}.STL"
            )
        if name == "leida":
            # simple_v6/base_link.STL already contains the correctly mounted
            # Odin1 housing. Keep this link for collision and sensor frames,
            # but do not render the separately exported duplicate mesh.
            for visual in list(link.findall("visual")):
                link.remove(visual)
        if name in WHEEL_CONTACTS:
            configure_rotating_wheel(link)
        replace_collision(link)

    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        raise ValueError("simple_v6 is missing base_link")
    found_joints = {joint.get("name", "") for joint in root.findall("joint")}
    if found_joints != set(JOINT_NAMES):
        raise ValueError(
            f"Unexpected simple_v6 joints: expected {sorted(JOINT_NAMES)}, "
            f"found {sorted(found_joints)}"
        )

    for joint in root.findall("joint"):
        joint.set("name", JOINT_NAMES[joint.get("name", "")])
        name = joint.get("name", "")
        if name in WHEEL_AXIS_BY_NAME:
            wheel_name = name.removesuffix("_joint")
            old_origin = joint.find("origin")
            if old_origin is None:
                raise ValueError(f"Wheel joint {name} has no origin")
            append_fixed_wheel_mount(base_link, wheel_name, old_origin)
            set_joint_origin(
                joint,
                WHEEL_CONTACTS[wheel_name]["tread"],
                old_origin.get("rpy", "0 0 0"),
            )
            axis = joint.find("axis")
            if axis is None:
                axis = ET.SubElement(joint, "axis")
            axis.set("xyz", WHEEL_AXIS_BY_NAME[name])
        if name in CAMERA_HOUSING_RPY:
            origin = joint.find("origin")
            if origin is None:
                raise ValueError(f"Camera housing joint {name} has no origin")
            origin.set("rpy", CAMERA_HOUSING_RPY[name])
        if name in {"yaw_joint", "pitch_joint"}:
            make_fixed_joint(joint)
        if joint.get("type") in {"continuous", "revolute"}:
            dynamics = joint.find("dynamics")
            if dynamics is None:
                dynamics = ET.SubElement(joint, "dynamics")
            dynamics.attrib.update({"damping": "0.2", "friction": "0.05"})

    root.insert(0, ET.Element("link", {"name": "base_footprint"}))
    base_joint = ET.Element(
        "joint", {"name": "base_footprint_to_base_link", "type": "fixed"}
    )
    add_origin(base_joint, ROOT_XYZ, ROOT_RPY)
    ET.SubElement(base_joint, "parent", {"link": "base_footprint"})
    ET.SubElement(base_joint, "child", {"link": "base_link"})
    root.insert(1, base_joint)

    append_preserved_sensors(root, template)
    return root


def copy_gazebo_configuration(root: ET.Element, template: ET.Element) -> None:
    for gazebo in template.findall("gazebo"):
        copied = copy.deepcopy(gazebo)
        for plugin in copied.findall(".//plugin"):
            if plugin.get("name") == "subway_v2_joint_state":
                for joint_name in list(plugin.findall("joint_name")):
                    if (joint_name.text or "").strip() == "pitch_joint":
                        plugin.remove(joint_name)
            if plugin.get("name") == "subway_v2_diff_drive":
                left = plugin.findall("left_joint")
                right = plugin.findall("right_joint")
                if len(left) != len(LEFT_WHEEL_JOINTS) or len(right) != len(
                    RIGHT_WHEEL_JOINTS
                ):
                    raise ValueError(
                        "subway_v2_diff_drive must define two left and two right wheels"
                    )
                for element, joint_name in zip(left, LEFT_WHEEL_JOINTS):
                    element.text = joint_name
                for element, joint_name in zip(right, RIGHT_WHEEL_JOINTS):
                    element.text = joint_name
        root.append(copied)

    camera_sensors = {
        sensor.get("name", ""): sensor
        for sensor in root.findall(".//gazebo/sensor[@type='camera']")
    }
    if set(camera_sensors) != CAMERA_SENSOR_NAMES:
        raise ValueError(
            f"Unexpected camera sensors: expected {sorted(CAMERA_SENSOR_NAMES)}, "
            f"found {sorted(camera_sensors)}"
        )
    for sensor in camera_sensors.values():
        always_on = sensor.find("always_on")
        if always_on is None:
            always_on = ET.SubElement(sensor, "always_on")
        always_on.text = "true"


def validate_tree(root: ET.Element) -> None:
    links = [link.get("name", "") for link in root.findall("link")]
    if len(links) != len(set(links)):
        raise ValueError("URDF contains duplicate link names")

    joints = [joint.get("name", "") for joint in root.findall("joint")]
    if len(joints) != len(set(joints)):
        raise ValueError("URDF contains duplicate joint names")

    parent_by_child = {}
    for joint in root.findall("joint"):
        parents = joint.findall("parent")
        children = joint.findall("child")
        if len(parents) != 1 or len(children) != 1:
            raise ValueError(
                f"Joint {joint.get('name')} must have exactly one parent and child"
            )
        parent = parents[0].get("link", "")
        child = children[0].get("link", "")
        if parent not in links or child not in links:
            raise ValueError(f"Joint {joint.get('name')} references a missing link")
        if child in parent_by_child:
            raise ValueError(f"Link {child} has more than one parent joint")
        parent_by_child[child] = parent

    roots = sorted(set(links) - set(parent_by_child))
    if roots != ["base_footprint"]:
        raise ValueError(f"Expected only base_footprint as root, found {roots}")

    for link in links:
        visited = set()
        current = link
        while current in parent_by_child:
            if current in visited:
                raise ValueError(f"Cycle detected at link {current}")
            visited.add(current)
            current = parent_by_child[current]
        if current != "base_footprint":
            raise ValueError(f"Link {link} is disconnected from base_footprint")

    valid_references = set(links) | set(joints)
    invalid = sorted(
        {
            gazebo.get("reference")
            for gazebo in root.findall("gazebo")
            if gazebo.get("reference")
            and gazebo.get("reference") not in valid_references
        }
    )
    if invalid:
        raise ValueError(f"Gazebo blocks reference missing links or joints: {invalid}")

    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        raise ValueError("URDF is missing base_link")
    misplaced_wheel_collisions = [
        collision.get("name", "")
        for collision in base_link.findall("collision")
        if collision.get("name", "").startswith(tuple(WHEEL_CONTACTS))
    ]
    if misplaced_wheel_collisions:
        raise ValueError(
            "Wheel collisions must rotate with their wheel links: "
            f"{misplaced_wheel_collisions}"
        )
    for wheel_name in WHEEL_CONTACTS:
        wheel = root.find(f"./link[@name='{wheel_name}']")
        if wheel is None:
            raise ValueError(f"URDF is missing wheel {wheel_name}")
        collision_names = {
            collision.get("name", "") for collision in wheel.findall("collision")
        }
        expected = {
            f"{wheel_name}_tread_collision",
            f"{wheel_name}_flange_collision",
        }
        if collision_names != expected:
            raise ValueError(
                f"{wheel_name} must own tread and flange collisions: "
                f"{sorted(collision_names)}"
            )

    leida_links = root.findall("./link[@name='leida']")
    if len(leida_links) != 1:
        raise ValueError(f"Expected exactly one leida link, found {len(leida_links)}")
    if leida_links[0].findall("visual"):
        raise ValueError("leida must not have a visual; base_link owns the Odin1 housing")
    if len(leida_links[0].findall("collision")) != 1:
        raise ValueError("leida must retain exactly one collision")

    required_unique = {
        "link": ("lidar_link", "imu_link", "odin1_rgb_optical_frame"),
        "joint": ("lidar_joint", "imu_joint", "odin1_rgb_optical_joint"),
        "sensor": ("odin1_lidar", "odin1_imu", "odin1_rgb_camera"),
        "plugin": (
            "odin1_pointcloud_plugin",
            "odin1_imu_plugin",
            "odin1_rgb_camera_plugin",
        ),
    }
    for tag, names in required_unique.items():
        counts = Counter(element.get("name", "") for element in root.findall(f".//{tag}"))
        for name in names:
            if counts[name] != 1:
                raise ValueError(
                    f"Expected exactly one {tag} named {name}, found {counts[name]}"
                )

    for sensor in root.findall(".//gazebo/sensor"):
        name = sensor.get("name", "")
        always_on = (sensor.findtext("always_on") or "").strip().lower()
        if name in CAMERA_SENSOR_NAMES and always_on not in {"true", "1"}:
            raise ValueError(f"Camera {name} must remain available in the full model")
        if name in {"odin1_lidar", "odin1_imu"} and always_on not in {"true", "1"}:
            raise ValueError(f"Sensor {name} must remain always on")


def synchronize_meshes(source_mesh_dir: Path, output_mesh_dir: Path) -> None:
    output_mesh_dir.mkdir(parents=True, exist_ok=True)
    expected_names = {f"{name}.STL" for name in RUNTIME_MESH_FILES}
    expected_names.update(
        f"{wheel_name}_{part}.STL"
        for wheel_name in WHEEL_CONTACTS
        for part in ("wheel", "mount")
    )
    for name in sorted(RUNTIME_MESH_FILES):
        shutil.copy2(source_mesh_dir / f"{name}.STL", output_mesh_dir / f"{name}.STL")
    split_flanged_wheels(output_mesh_dir)
    for existing in output_mesh_dir.iterdir():
        if existing.is_file() and existing.suffix.lower() == ".stl":
            if existing.name not in expected_names:
                existing.unlink()


def main() -> None:
    args = parse_args()
    source_urdf = args.source_dir / SOURCE_URDF
    source_mesh_dir = args.source_dir / "meshes"
    if not source_urdf.is_file():
        raise FileNotFoundError(source_urdf)
    missing_meshes = sorted(
        str(source_mesh_dir / f"{name}.STL")
        for name in HARDWARE_LINKS
        if not (source_mesh_dir / f"{name}.STL").is_file()
    )
    if missing_meshes:
        raise FileNotFoundError(f"Missing simple_v6 meshes: {missing_meshes}")

    template = ET.parse(args.sensor_template_urdf).getroot()
    validate_embedded_leida(source_urdf, source_mesh_dir)
    root = configure_hardware_tree(source_urdf, template)
    copy_gazebo_configuration(root, template)
    validate_tree(root)
    root.insert(
        0,
        ET.Comment(
            f" Hardware geometry imported from {SOURCE_LABEL}; ROS 2 sensor "
            "plugins and calibrated sensor frames are preserved. "
        ),
    )

    args.output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(
        args.output_urdf, encoding="utf-8", xml_declaration=True
    )
    synchronize_meshes(source_mesh_dir, args.output_mesh_dir)
    print(f"Imported {source_urdf} into {args.output_urdf}")
    print(
        f"Synchronized {len(RUNTIME_MESH_FILES)} runtime V6 meshes into "
        f"{args.output_mesh_dir}; leida.STL remains a source-only CAD reference"
    )


if __name__ == "__main__":
    main()
