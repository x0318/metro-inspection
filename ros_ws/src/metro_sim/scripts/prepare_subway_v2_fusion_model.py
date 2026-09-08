#!/usr/bin/env python3
"""Create a temporary subway_v2 model for front/ceiling damage fusion."""

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


REMOVED_CAMERA_SENSOR_NAMES = {
    "xj1_camera_sensor",
    "xj2_camera_sensor",
    "xj3_camera_sensor",
    "xj4_camera_sensor",
}
RETAINED_SENSOR_NAMES = {
    "odin1_lidar",
    "odin1_imu",
    "odin1_rgb_camera",
    "pitch_camera_sensor",
}
EXPECTED_FULL_SENSOR_NAMES = REMOVED_CAMERA_SENSOR_NAMES | RETAINED_SENSOR_NAMES


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Derive an Odin1/pitch-camera fusion model without copying runtime meshes."
        )
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
    if sensor_names != EXPECTED_FULL_SENSOR_NAMES:
        raise ValueError(
            "Full subway_v2 sensor set changed; review the fusion profile before "
            f"continuing. Expected {sorted(EXPECTED_FULL_SENSOR_NAMES)}, "
            f"found {sorted(sensor_names)}"
        )

    removed = []
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.get("name", "") in REMOVED_CAMERA_SENSOR_NAMES:
                removed.append(sensor.get("name", ""))
                link.remove(sensor)

    if set(removed) != REMOVED_CAMERA_SENSOR_NAMES:
        raise ValueError(f"Unexpected removed camera set: {sorted(removed)}")

    remaining_sensors = {
        sensor.get("name", "") for sensor in model.findall(".//sensor")
    }
    if remaining_sensors != RETAINED_SENSOR_NAMES:
        raise ValueError(
            "Fusion model retained an unexpected sensor set: "
            f"{sorted(remaining_sensors)}"
        )

    model.set("name", "subway_v2_fusion")
    output_dir = args.output_model_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_dir / "model.sdf", encoding="utf-8", xml_declaration=True)

    config = ET.Element("model")
    ET.SubElement(config, "name").text = "subway_v2_fusion"
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", {"version": "1.7"}).text = "model.sdf"
    author = ET.SubElement(config, "author")
    ET.SubElement(author, "name").text = "metro-inspection"
    ET.SubElement(config, "description").text = (
        "Temporary front and ceiling camera fusion profile derived from subway_v2."
    )
    config_tree = ET.ElementTree(copy.deepcopy(config))
    ET.indent(config_tree, space="  ")
    config_tree.write(
        output_dir / "model.config", encoding="utf-8", xml_declaration=True
    )

    print(
        f"Prepared {output_dir}: removed {len(removed)} non-fusion cameras; "
        "retained Odin1 RGB/lidar/IMU and the ceiling Pitch camera"
    )


if __name__ == "__main__":
    main()
