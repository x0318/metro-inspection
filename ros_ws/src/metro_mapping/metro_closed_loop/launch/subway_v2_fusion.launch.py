"""Connect damage localization to an already running subway_v2 simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_config = os.path.join(
        get_package_share_directory("metro_localization"),
        "config",
        "localization.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    sync_slop_sec = LaunchConfiguration("sync_slop_sec")
    run_camera_info_calibrator = LaunchConfiguration(
        "run_camera_info_calibrator"
    )
    run_placeholder_detector = LaunchConfiguration("run_placeholder_detector")

    calibrated_camera_info = Node(
        package="metro_closed_loop",
        executable="camera_info_calibrator",
        name="odin1_camera_info_calibrator",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/odin1/rgb/camera_info_gazebo",
                "output_topic": camera_info_topic,
            }
        ],
        condition=IfCondition(run_camera_info_calibrator),
        output="screen",
    )

    detector = Node(
        package="metro_closed_loop",
        executable="damage_detector",
        name="damage_detector",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": image_topic,
                "detections_topic": "/damage_detections",
                "mask_topic": "/damage_mask",
            }
        ],
        condition=IfCondition(run_placeholder_detector),
        output="screen",
    )

    localizer = Node(
        package="metro_localization",
        executable="damage_localizer",
        name="damage_localizer",
        parameters=[
            localization_config,
            {
                "use_sim_time": use_sim_time,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "cloud_topic": cloud_topic,
                # Bound latency under large PointCloud2/image load. Old frames
                # are less useful than fresh frames for moving-platform fusion.
                "sync_queue_size": 5,
                "sync_slop_sec": sync_slop_sec,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo clock.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/odin1/rgb/image_raw",
                description="Image from the camera selected for lidar fusion.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/odin1/rgb/camera_info",
                description="CameraInfo paired with image_topic.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/odin1/cloud_raw",
                description="Odin1 PointCloud2 input.",
            ),
            DeclareLaunchArgument(
                "sync_slop_sec",
                default_value="0.08",
                description=(
                    "Maximum image/cloud timestamp difference. Keep this small "
                    "for moving-platform accuracy."
                ),
            ),
            DeclareLaunchArgument(
                "run_camera_info_calibrator",
                default_value="true",
                description=(
                    "Publish calibrated Odin1 CameraInfo. Set false when the "
                    "full sensor simulation already runs camera_info_calibrator."
                ),
            ),
            DeclareLaunchArgument(
                "run_placeholder_detector",
                default_value="false",
                description=(
                    "Run the red-color wiring-test detector. Enable it only for "
                    "an explicit pipeline test; keep it disabled for normal use "
                    "and when a real detector publishes /damage_detections."
                ),
            ),
            calibrated_camera_info,
            detector,
            localizer,
        ]
    )
