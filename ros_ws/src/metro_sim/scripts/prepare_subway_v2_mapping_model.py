#!/usr/bin/env python3
"""Create a temporary camera-free subway_v2 model for lidar mapping."""

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERA_SENSOR_NAMES = {
    "odin1_rgb_camera",
    "xj1_camera_sensor",
    "xj2_camera_sensor",
    "xj3_camera_sensor",
    "xj4_camera_sensor",
    "pitch_camera_sensor",
}
REQUIRED_SENSOR_NAMES = {"odin1_lidar", "odin1_imu"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Derive a camera-free mapping model without copying meshes."
    )
    parser.add_argument("source_sdf", type=Path)
    parser.add_argument("output_model_dir", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    tree = ET.parse(args.source_sdf)
    model = tree.getroot().find("model")
    if model is None or model.get("name") != "subway_v2":
        raise ValueError("Source SDF does not contain model subway_v2")

    sensor_names = {
        sensor.get("name", "") for sensor in model.findall(".//sensor")
    }
    missing_cameras = sorted(CAMERA_SENSOR_NAMES - sensor_names)
    if missing_cameras:
        raise ValueError(f"Full model is missing cameras: {missing_cameras}")
    missing_required = sorted(REQUIRED_SENSOR_NAMES - sensor_names)
    if missing_required:
        raise ValueError(f"Full model is missing mapping sensors: {missing_required}")

    removed = []
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.get("name", "") in CAMERA_SENSOR_NAMES:
                removed.append(sensor.get("name", ""))
                link.remove(sensor)

    if set(removed) != CAMERA_SENSOR_NAMES:
        raise ValueError(f"Unexpected removed camera set: {sorted(removed)}")
    remaining_sensors = {
        sensor.get("name", "") for sensor in model.findall(".//sensor")
    }
    if remaining_sensors != REQUIRED_SENSOR_NAMES:
        raise ValueError(
            f"Mapping model must contain only lidar and IMU, found {sorted(remaining_sensors)}"
        )
    model.set("name", "subway_v2_mapping")

    output_dir = args.output_model_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_dir / "model.sdf", encoding="utf-8", xml_declaration=True)

    config = ET.Element("model")
    ET.SubElement(config, "name").text = "subway_v2_mapping"
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", {"version": "1.7"}).text = "model.sdf"
    author = ET.SubElement(config, "author")
    ET.SubElement(author, "name").text = "metro-inspection"
    ET.SubElement(config, "description").text = (
        "Temporary lidar/IMU mapping profile derived from subway_v2."
    )
    config_tree = ET.ElementTree(copy.deepcopy(config))
    ET.indent(config_tree, space="  ")
    config_tree.write(output_dir / "model.config", encoding="utf-8", xml_declaration=True)

    print(
        f"Prepared {output_dir}: removed {len(removed)} cameras; "
        "retained odin1_lidar and odin1_imu"
    )


if __name__ == "__main__":
    main()
