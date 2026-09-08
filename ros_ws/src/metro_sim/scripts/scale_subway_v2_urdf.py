#!/usr/bin/env python3
"""Uniformly scale the active subway_v2 URDF for the tunnel preview."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


# These transforms are calibrated inside Odin1 and must remain metric.
UNSCALED_JOINT_TRANSLATIONS = {"imu_joint", "odin1_rgb_optical_joint"}
UNSCALED_SENSOR_TRANSLATIONS = {"odin1_rgb_camera"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scale", type=float, required=True)
    return parser.parse_args()


def format_number(value: float) -> str:
    return "0" if abs(value) < 1e-15 else format(value, ".15g")


def scale_values(text: str, factor: float, expected: int) -> str:
    values = [float(value) for value in text.split()]
    if len(values) != expected:
        raise ValueError(f"Expected {expected} values, found {len(values)}: {text}")
    return " ".join(format_number(value * factor) for value in values)


def scale_origin(origin: ET.Element | None, factor: float) -> bool:
    if origin is None or "xyz" not in origin.attrib:
        return False
    origin.set("xyz", scale_values(origin.get("xyz", ""), factor, 3))
    return True


def scale_geometry(geometry: ET.Element, factor: float) -> int:
    changes = 0
    mesh = geometry.find("mesh")
    if mesh is not None:
        mesh.set("scale", scale_values(mesh.get("scale", "1 1 1"), factor, 3))
        changes += 1
    box = geometry.find("box")
    if box is not None and "size" in box.attrib:
        box.set("size", scale_values(box.get("size", ""), factor, 3))
        changes += 1
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        for attribute in ("radius", "length"):
            if attribute in cylinder.attrib:
                cylinder.set(
                    attribute,
                    format_number(float(cylinder.get(attribute, "0")) * factor),
                )
                changes += 1
    sphere = geometry.find("sphere")
    if sphere is not None and "radius" in sphere.attrib:
        sphere.set(
            "radius", format_number(float(sphere.get("radius", "0")) * factor)
        )
        changes += 1
    return changes


def main() -> None:
    args = parse_args()
    if not math.isfinite(args.scale) or args.scale <= 0:
        raise ValueError(f"Scale must be positive and finite: {args.scale}")

    tree = ET.parse(args.input)
    root = tree.getroot()
    if root.tag != "robot" or root.get("name") != "subway_v2":
        raise ValueError("Input is not the subway_v2 URDF")

    for link in root.findall("link"):
        for section_name in ("visual", "collision", "inertial"):
            for section in link.findall(section_name):
                scale_origin(section.find("origin"), args.scale)
        inertial = link.find("inertial")
        if inertial is not None:
            inertia = inertial.find("inertia")
            if inertia is not None:
                for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    if attribute in inertia.attrib:
                        inertia.set(
                            attribute,
                            format_number(
                                float(inertia.get(attribute, "0")) * args.scale**2
                            ),
                        )

    for joint in root.findall("joint"):
        if joint.get("name") not in UNSCALED_JOINT_TRANSLATIONS:
            scale_origin(joint.find("origin"), args.scale)

    for geometry in root.findall(".//geometry"):
        scale_geometry(geometry, args.scale)

    for sensor in root.findall(".//gazebo/sensor"):
        if sensor.get("name") in UNSCALED_SENSOR_TRANSLATIONS:
            continue
        pose = sensor.find("pose")
        if pose is not None and pose.text:
            values = [float(value) for value in pose.text.split()]
            if len(values) != 6:
                raise ValueError(f"Expected six-value sensor pose: {pose.text}")
            values[:3] = [value * args.scale for value in values[:3]]
            pose.text = " ".join(format_number(value) for value in values)

    for plugin in root.findall(".//gazebo/plugin"):
        if plugin.get("filename") != "libgazebo_ros_diff_drive.so":
            continue
        for tag in ("wheel_separation", "wheel_diameter"):
            for value in plugin.findall(tag):
                if value.text:
                    value.text = format_number(float(value.text) * args.scale)

    root.insert(
        0,
        ET.Comment(
            f" Generated at scale {format_number(args.scale)}; mass is unchanged "
            "and inertia is scaled by s^2. "
        ),
    )
    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
