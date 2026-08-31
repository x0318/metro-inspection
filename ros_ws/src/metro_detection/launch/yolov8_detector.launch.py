"""Launch YOLOv8 against one Metro simulation camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_MODEL_PATH = "/home/jo/incoming/yolov8n_sim_demo_best(1).pt"


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("metro_detection"),
        "config",
        "yolov8_sim.yaml",
    )

    yolo_node = Node(
        package="metro_detection",
        executable="yolo_detector",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "model_path": LaunchConfiguration("model_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": LaunchConfiguration("detections_topic"),
                "annotated_image_topic": LaunchConfiguration(
                    "annotated_image_topic"
                ),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"),
                    value_type=float,
                ),
                "device": LaunchConfiguration("device"),
                "max_inference_rate_hz": ParameterValue(
                    LaunchConfiguration("max_inference_rate_hz"),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("node_name", default_value="yolo_detector"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "model_path",
                default_value=EnvironmentVariable(
                    "METRO_YOLO_MODEL_PATH",
                    default_value=DEFAULT_MODEL_PATH,
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/odin1/rgb/image_raw"
            ),
            DeclareLaunchArgument(
                "detections_topic", default_value="/damage_detections"
            ),
            DeclareLaunchArgument(
                "annotated_image_topic",
                default_value="/damage_detection/annotated_image",
            ),
            DeclareLaunchArgument(
                "confidence_threshold", default_value="0.35"
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "max_inference_rate_hz", default_value="10.0"
            ),
            yolo_node,
        ]
    )
