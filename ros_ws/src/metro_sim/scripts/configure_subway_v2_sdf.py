#!/usr/bin/env python3
"""Rewrite description-package mesh URIs for the Gazebo model database."""

import argparse
import math
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


SOURCE_PREFIXES = (
    "package://metro_description/meshes/",
    "model://metro_description/meshes/",
)
MODEL_PREFIX = "model://subway_v2/meshes/"
POSITION_TOLERANCE = 2e-5
AXIS_TOLERANCE = 2e-5
CALIBRATION_TOLERANCE = 1e-6
CAMERA_SENSOR_NAMES = {
    "odin1_rgb_camera",
    "xj1_camera_sensor",
    "xj2_camera_sensor",
    "xj3_camera_sensor",
    "xj4_camera_sensor",
    "pitch_camera_sensor",
}
ODIN1_CAMERA_CALIBRATION = {
    "width": 1600.0,
    "height": 1296.0,
    "fx": 736.9688,
    "fy": 737.0365,
    "cx": 766.6570,
    "cy": 642.9091,
    "s": 0.2058,
}
WHEEL_NAMES = ("w1", "w2", "w3", "w4")
LEFT_WHEEL_JOINTS = ("w2_joint", "w3_joint")
RIGHT_WHEEL_JOINTS = ("w1_joint", "w4_joint")


def parse_values(text: str, count: int, label: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split())
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain {count} finite values: {text!r}")
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


def matrix_multiply(
    left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def rotate_vector(
    rotation: tuple[tuple[float, ...], ...], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def compose_transform(
    parent: tuple[tuple[float, ...], tuple[tuple[float, ...], ...]],
    child: tuple[tuple[float, ...], tuple[tuple[float, ...], ...]],
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    parent_xyz, parent_rotation = parent
    child_xyz, child_rotation = child
    rotated_child = rotate_vector(parent_rotation, child_xyz)
    return (
        tuple(parent_xyz[index] + rotated_child[index] for index in range(3)),
        matrix_multiply(parent_rotation, child_rotation),
    )


def urdf_transform_to_link(
    root: ET.Element, target_link: str
) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...]]:
    joints_by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            joints_by_child[child.get("link")] = joint

    chain = []
    current = target_link
    while current != "base_footprint":
        joint = joints_by_child.get(current)
        if joint is None:
            raise ValueError(f"No URDF joint chain from base_footprint to {target_link}")
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None or not parent.get("link"):
            raise ValueError(f"Joint {joint.get('name')} has no parent link")
        current = parent.get("link")

    transform = (
        (0.0, 0.0, 0.0),
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    )
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = parse_values(
            origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            3,
            f"URDF joint {joint.get('name')} xyz",
        )
        rpy = parse_values(
            origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            3,
            f"URDF joint {joint.get('name')} rpy",
        )
        transform = compose_transform(transform, (xyz, rotation_from_rpy(rpy)))
    return transform


def validate_lidar_mount(urdf_root: ET.Element) -> None:
    lidar_joint = urdf_root.find("./joint[@name='lidar_joint']")
    leida = urdf_root.find("./link[@name='leida']")
    if lidar_joint is None or leida is None:
        raise ValueError("URDF is missing lidar_joint or leida")
    parent = lidar_joint.find("parent")
    child = lidar_joint.find("child")
    if (
        parent is None
        or parent.get("link") != "leida"
        or child is None
        or child.get("link") != "lidar_link"
    ):
        raise ValueError("lidar_joint must connect leida to lidar_link")

    origin = lidar_joint.find("origin")
    lidar_xyz = parse_values(
        origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
        3,
        "lidar_joint xyz",
    )
    collision_origin = leida.find("collision/origin")
    collision_box = leida.find("collision/geometry/box")
    if collision_origin is None or collision_box is None:
        raise ValueError("leida must retain its box collision for mount validation")
    box_xyz = parse_values(collision_origin.get("xyz", "0 0 0"), 3, "leida box xyz")
    box_size = parse_values(collision_box.get("size", ""), 3, "leida box size")
    front_z = box_xyz[2] + box_size[2] / 2
    front_gap = lidar_xyz[2] - front_z
    if (
        abs(lidar_xyz[0] - box_xyz[0]) > box_size[0] / 2
        or abs(lidar_xyz[1] - box_xyz[1]) > box_size[1] / 2
        or front_gap <= 0
        or front_gap > box_size[2] * 0.1
    ):
        raise ValueError(
            "lidar_joint is not on the Odin1 collision front face: "
            f"lidar={lidar_xyz}, face_z={front_z}, gap={front_gap}"
        )


def require_float(
    element: ET.Element, path: str, expected: float, label: str
) -> None:
    text = element.findtext(path)
    if text is None:
        raise ValueError(f"{label} is missing {path}")
    actual = float(text)
    if not math.isfinite(actual) or abs(actual - expected) > CALIBRATION_TOLERANCE:
        raise ValueError(
            f"{label} {path} must be {expected}, found {text.strip()}"
        )


def validate_odin1_camera(model: ET.Element) -> None:
    sensor = model.find(".//sensor[@name='odin1_rgb_camera']")
    if sensor is None:
        raise ValueError("SDF is missing Odin1 RGB camera sensor")
    camera = sensor.find("camera")
    plugin = sensor.find("plugin[@name='odin1_rgb_camera_plugin']")
    if camera is None or plugin is None:
        raise ValueError("Odin1 RGB camera is missing camera or ROS plugin settings")

    calibration = ODIN1_CAMERA_CALIBRATION
    require_float(camera, "image/width", calibration["width"], "Odin1 camera")
    require_float(camera, "image/height", calibration["height"], "Odin1 camera")
    for key in ("fx", "fy", "cx", "cy", "s"):
        require_float(
            camera,
            f"lens/intrinsics/{key}",
            calibration[key],
            "Odin1 camera",
        )

    # Gazebo Classic computes one plugin focal length from horizontal FOV and
    # uses it for both K[0] and K[4]. Do not set <focal_length>: sdformat's
    # internal HFOV serialization is lower precision and would cause a warning.
    # The calibrated relay publishes the authoritative K; renderer/P retain fy.
    expected_hfov = 2.0 * math.atan(
        calibration["width"] / (2.0 * calibration["fx"])
    )
    require_float(camera, "horizontal_fov", expected_hfov, "Odin1 camera")
    if plugin.find("focal_length") is not None:
        raise ValueError(
            "Odin1 ROS camera plugin must derive focal_length from horizontal_fov"
        )
    for path, key in (
        ("cx", "cx"),
        ("cy", "cy"),
        ("P_fx", "fx"),
        ("P_fy", "fy"),
        ("P_cx", "cx"),
        ("P_cy", "cy"),
    ):
        require_float(plugin, path, calibration[key], "Odin1 ROS camera plugin")


def validate_wheel_drive(model: ET.Element) -> None:
    if model.find("./plugin[@name='subway_v2_planar_move']") is not None:
        raise ValueError("subway_v2 must not use the pose-based planar_move plugin")
    plugins = model.findall("./plugin[@name='subway_v2_diff_drive']")
    if len(plugins) != 1:
        raise ValueError(
            f"Expected one subway_v2_diff_drive plugin, found {len(plugins)}"
        )
    plugin = plugins[0]
    if plugin.get("filename") != "libgazebo_ros_diff_drive.so":
        raise ValueError("subway_v2_diff_drive must use libgazebo_ros_diff_drive.so")
    if (plugin.findtext("num_wheel_pairs") or "").strip() != "2":
        raise ValueError("subway_v2_diff_drive must control two wheel pairs")
    left = tuple(
        (element.text or "").strip() for element in plugin.findall("left_joint")
    )
    right = tuple(
        (element.text or "").strip() for element in plugin.findall("right_joint")
    )
    if left != LEFT_WHEEL_JOINTS or right != RIGHT_WHEEL_JOINTS:
        raise ValueError(f"Unexpected drive pairs: left={left}, right={right}")

    separations = [float(value.text or "nan") for value in plugin.findall("wheel_separation")]
    diameters = [float(value.text or "nan") for value in plugin.findall("wheel_diameter")]
    if len(separations) != 2 or len(diameters) != 2:
        raise ValueError("Each drive pair needs one wheel separation and diameter")

    wheel_centers = {}
    tread_diameters = {}
    for wheel_name in WHEEL_NAMES:
        joint = model.find(f"./joint[@name='{wheel_name}_joint']")
        link = model.find(f"./link[@name='{wheel_name}']")
        if joint is None or link is None:
            raise ValueError(f"Generated SDF is missing wheel {wheel_name}")
        pose = joint.find("pose")
        if pose is None or not pose.text or pose.get("relative_to") != "base_footprint":
            raise ValueError(f"{wheel_name}_joint must be relative to base_footprint")
        wheel_centers[wheel_name] = parse_values(
            pose.text, 6, f"{wheel_name}_joint pose"
        )[:3]
        collisions = link.findall("collision")
        names = {collision.get("name", "") for collision in collisions}
        expected = {
            f"{wheel_name}_tread_collision_collision",
            f"{wheel_name}_flange_collision_collision_1",
        }
        if names != expected:
            raise ValueError(
                f"{wheel_name} must own tread and flange collisions: {sorted(names)}"
            )
        radius_text = link.findtext(
            f"collision[@name='{wheel_name}_tread_collision_collision']"
            "/geometry/cylinder/radius"
        )
        if radius_text is None:
            raise ValueError(f"{wheel_name} tread cylinder radius is missing")
        tread_diameters[wheel_name] = 2.0 * float(radius_text)

    base = model.find("./link[@name='base_footprint']")
    if base is None:
        raise ValueError("Generated SDF is missing base_footprint")
    misplaced = [
        collision.get("name", "")
        for collision in base.findall("collision")
        if any(f"__{wheel_name}_" in collision.get("name", "") for wheel_name in WHEEL_NAMES)
    ]
    if misplaced:
        raise ValueError(f"Wheel collisions must not be fixed to chassis: {misplaced}")

    expected_separations = (
        abs(wheel_centers["w2"][1] - wheel_centers["w1"][1]),
        abs(wheel_centers["w3"][1] - wheel_centers["w4"][1]),
    )
    expected_diameters = (tread_diameters["w2"], tread_diameters["w3"])
    for actual, expected in zip(separations, expected_separations):
        if not math.isfinite(actual) or abs(actual - expected) > POSITION_TOLERANCE:
            raise ValueError(
                f"Drive wheel separation {actual} does not match joint geometry {expected}"
            )
    for actual, expected in zip(diameters, expected_diameters):
        if not math.isfinite(actual) or abs(actual - expected) > POSITION_TOLERANCE:
            raise ValueError(
                f"Drive wheel diameter {actual} does not match tread geometry {expected}"
            )


def validate_generated_model(model: ET.Element, urdf_root: ET.Element) -> None:
    sensor_counts = Counter(
        sensor.get("name", "") for sensor in model.findall(".//sensor")
    )
    plugin_counts = Counter(
        plugin.get("name", "") for plugin in model.findall(".//plugin")
    )
    for name in ("odin1_lidar", "odin1_imu", "odin1_rgb_camera"):
        if sensor_counts[name] != 1:
            raise ValueError(
                f"Expected exactly one SDF sensor {name}, found {sensor_counts[name]}"
            )
    for name in (
        "odin1_pointcloud_plugin",
        "odin1_imu_plugin",
        "odin1_rgb_camera_plugin",
    ):
        if plugin_counts[name] != 1:
            raise ValueError(
                f"Expected exactly one SDF plugin {name}, found {plugin_counts[name]}"
            )

    duplicate_sensors = sorted(name for name, count in sensor_counts.items() if count > 1)
    duplicate_plugins = sorted(name for name, count in plugin_counts.items() if count > 1)
    if duplicate_sensors or duplicate_plugins:
        raise ValueError(
            f"Duplicate SDF names: sensors={duplicate_sensors}, plugins={duplicate_plugins}"
        )

    camera_sensors = {
        sensor.get("name", ""): sensor
        for sensor in model.findall(".//sensor[@type='camera']")
    }
    if set(camera_sensors) != CAMERA_SENSOR_NAMES:
        raise ValueError(
            f"Unexpected SDF camera sensors: expected {sorted(CAMERA_SENSOR_NAMES)}, "
            f"found {sorted(camera_sensors)}"
        )
    for name, sensor in camera_sensors.items():
        always_on = (sensor.findtext("always_on") or "").strip().lower()
        if always_on not in {"true", "1"}:
            raise ValueError(f"SDF camera {name} must remain available in the full model")
    for name in ("odin1_lidar", "odin1_imu"):
        sensor = model.find(f".//sensor[@name='{name}']")
        always_on = (sensor.findtext("always_on") or "").strip().lower()
        if always_on not in {"true", "1"}:
            raise ValueError(f"SDF sensor {name} must remain always on")

    validate_odin1_camera(model)
    validate_wheel_drive(model)

    _, pitch_camera_rotation = urdf_transform_to_link(
        urdf_root, "pitch_camera_sensor_link"
    )
    pitch_camera_forward = rotate_vector(
        pitch_camera_rotation, (1.0, 0.0, 0.0)
    )
    pitch_forward_error = max(
        abs(pitch_camera_forward[index] - expected)
        for index, expected in enumerate((-1.0, 0.0, 0.0))
    )
    if pitch_forward_error > 0.002:
        raise ValueError(
            "Pitch camera optical axis must point rearward through its lens "
            "(base_footprint -X), "
            f"found {pitch_camera_forward}"
        )
    pitch_camera_up = rotate_vector(pitch_camera_rotation, (0.0, 0.0, 1.0))
    pitch_up_error = max(
        abs(pitch_camera_up[index] - expected)
        for index, expected in enumerate((0.0, 0.0, 1.0))
    )
    if pitch_up_error > 0.002:
        raise ValueError(
            "Pitch camera image must be upright (camera +Z toward "
            "base_footprint +Z), "
            f"found {pitch_camera_up}"
        )

    leida_uris = [
        (uri.text or "").strip()
        for uri in model.findall(".//uri")
        if Path((uri.text or "").strip()).name.lower() == "leida.stl"
    ]
    if leida_uris:
        raise ValueError(f"SDF must not render the duplicate leida.STL: {leida_uris}")

    lidar_joints = model.findall("./joint[@name='lidar_joint']")
    lidar_links = model.findall("./link[@name='lidar_link']")
    if len(lidar_joints) != 1 or len(lidar_links) != 1:
        raise ValueError(
            "SDF must contain exactly one lidar_joint and lidar_link: "
            f"joints={len(lidar_joints)}, links={len(lidar_links)}"
        )
    pose = lidar_joints[0].find("pose")
    if pose is None or not pose.text or pose.get("relative_to") != "base_footprint":
        raise ValueError("SDF lidar_joint pose must be relative to base_footprint")
    sdf_values = parse_values(pose.text, 6, "SDF lidar_joint pose")
    sdf_xyz = sdf_values[:3]
    sdf_rotation = rotation_from_rpy(sdf_values[3:])
    expected_xyz, expected_rotation = urdf_transform_to_link(urdf_root, "lidar_link")
    position_error = max(abs(sdf_xyz[index] - expected_xyz[index]) for index in range(3))
    if position_error > POSITION_TOLERANCE:
        raise ValueError(
            f"SDF lidar optical center differs from URDF by {position_error}: "
            f"SDF={sdf_xyz}, URDF={expected_xyz}"
        )
    sdf_forward = rotate_vector(sdf_rotation, (1.0, 0.0, 0.0))
    expected_forward = rotate_vector(expected_rotation, (1.0, 0.0, 0.0))
    axis_error = max(
        abs(sdf_forward[index] - expected_forward[index]) for index in range(3)
    )
    if axis_error > AXIS_TOLERANCE:
        raise ValueError(
            f"SDF lidar axis differs from URDF: SDF={sdf_forward}, "
            f"URDF={expected_forward}"
        )
    base_forward_error = max(
        abs(sdf_forward[index] - expected)
        for index, expected in enumerate((1.0, 0.0, 0.0))
    )
    if base_forward_error > AXIS_TOLERANCE:
        raise ValueError(
            "Odin1 scan axis must point toward robot front (base_footprint +X), "
            f"found {sdf_forward}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--urdf", type=Path, required=True)
    args = parser.parse_args()

    tree = ET.parse(args.input)
    model = tree.getroot().find("model")
    if model is None or model.get("name") != "subway_v2":
        raise ValueError("Generated SDF does not contain model subway_v2")

    rewritten = 0
    for uri in model.findall(".//uri"):
        if not uri.text:
            continue
        for prefix in SOURCE_PREFIXES:
            if uri.text.startswith(prefix):
                uri.text = MODEL_PREFIX + uri.text[len(prefix):]
                rewritten += 1
                break

    unresolved = [
        uri.text
        for uri in model.findall(".//uri")
        if uri.text and "metro_description/meshes" in uri.text
    ]
    if unresolved:
        raise ValueError(f"Unresolved mesh URIs: {unresolved}")

    # `gz sdf -p` prints standard SDF floating-point values with about six
    # significant digits. Restore camera calibration values from the scaled
    # URDF so true intrinsics do not acquire conversion-rounding warnings.
    urdf_root = ET.parse(args.urdf).getroot()
    validate_lidar_mount(urdf_root)
    urdf_cameras = {
        sensor.get("name"): sensor.find("camera")
        for sensor in urdf_root.findall(".//gazebo/sensor[@type='camera']")
    }
    calibration_paths = (
        "horizontal_fov",
        "distortion/k1",
        "distortion/k2",
        "distortion/k3",
        "distortion/p1",
        "distortion/p2",
        "distortion/center",
        "lens/intrinsics/fx",
        "lens/intrinsics/fy",
        "lens/intrinsics/cx",
        "lens/intrinsics/cy",
        "lens/intrinsics/s",
    )
    restored = 0
    for sensor in model.findall(".//sensor[@type='camera']"):
        source_camera = urdf_cameras.get(sensor.get("name"))
        target_camera = sensor.find("camera")
        if source_camera is None or target_camera is None:
            continue
        for path in calibration_paths:
            source_value = source_camera.find(path)
            target_value = target_camera.find(path)
            if (
                source_value is not None
                and target_value is not None
                and source_value.text
            ):
                target_value.text = source_value.text.strip()
                restored += 1

    validate_generated_model(model, urdf_root)

    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Rewrote {rewritten} mesh URIs")
    print(f"Restored {restored} camera calibration values from {args.urdf}")


if __name__ == "__main__":
    main()
