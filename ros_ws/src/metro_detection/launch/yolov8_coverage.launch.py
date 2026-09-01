"""Launch shared five-camera YOLO inference and simulation coverage checks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_MODEL_PATH = "/home/jo/incoming/yolov8n_sim_demo_best(1).pt"


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("metro_detection"),
        "config",
        "yolov8_coverage.yaml",
    )

    detector = Node(
        package="metro_detection",
        executable="yolo_detector",
        name="yolo_coverage_detector",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "model_path": LaunchConfiguration("model_path"),
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
    evaluator = Node(
        package="metro_detection",
        executable="simulation_coverage_evaluator",
        name="simulation_coverage_evaluator",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "publish_events": ParameterValue(
                    LaunchConfiguration("publish_events"), value_type=bool
                ),
            },
        ],
    )
    driver = Node(
        package="metro_detection",
        executable="simulation_coverage_driver",
        name="simulation_coverage_driver",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_drive")),
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "model_path",
                default_value=EnvironmentVariable(
                    "METRO_YOLO_MODEL_PATH",
                    default_value=DEFAULT_MODEL_PATH,
                ),
            ),
            DeclareLaunchArgument(
                "confidence_threshold", default_value="0.35"
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "max_inference_rate_hz", default_value="3.0"
            ),
            DeclareLaunchArgument("publish_events", default_value="true"),
            DeclareLaunchArgument("auto_drive", default_value="false"),
            detector,
            evaluator,
            driver,
        ]
    )
