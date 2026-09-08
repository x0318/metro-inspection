# Copyright 2026 jo0625
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run lidar odometry, fused local odometry, and loop-closing 3D SLAM."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _is_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


class _RestartBudget:
    def __init__(self, limit):
        if limit < 0:
            raise ValueError("restart limit must be non-negative")
        self.limit = limit
        self.used = 0

    def take(self, return_code):
        if return_code == 0 or self.used >= self.limit:
            return False
        self.used += 1
        return True


def _non_negative_int(context, name):
    value = LaunchConfiguration(name).perform(context)
    try:
        parsed = int(value)
    except ValueError as error:
        raise ValueError(f"{name} must be a non-negative integer") from error
    if parsed < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return parsed


def _non_negative_float(context, name):
    value = LaunchConfiguration(name).perform(context)
    try:
        parsed = float(value)
    except ValueError as error:
        raise ValueError(f"{name} must be a non-negative number") from error
    if parsed < 0.0:
        raise ValueError(f"{name} must be a non-negative number")
    return parsed


def _launch_mapping(context):
    config_file = LaunchConfiguration("config_file")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    validated_cloud_topic = LaunchConfiguration("validated_cloud_topic")
    keyframe_cloud_topic = LaunchConfiguration("keyframe_cloud_topic")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    lidar_odom_topic = LaunchConfiguration("lidar_odom_topic")
    filtered_odom_topic = LaunchConfiguration("filtered_odom_topic")
    database_path = LaunchConfiguration("database_path")
    pcd_path = LaunchConfiguration("pcd_path")
    map_assembler_restart_delay = _non_negative_float(
        context, "map_assembler_restart_delay"
    )
    map_assembler_restart_budget = _RestartBudget(
        _non_negative_int(context, "map_assembler_max_restarts")
    )

    rtabmap_arguments = []
    if _is_true(LaunchConfiguration("reset_database").perform(context)):
        rtabmap_arguments.append("--delete_db_on_start")

    odometry_fusion = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("metro_localization"),
                            "launch",
                            "odometry_fusion.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_file": ekf_config_file,
                    "use_sim_time": use_sim_time,
                    "frequency": LaunchConfiguration("ekf_frequency"),
                    "transform_time_offset": LaunchConfiguration(
                        "ekf_transform_time_offset"
                    ),
                    "raw_odom_topic": raw_odom_topic,
                    "imu_topic": imu_topic,
                    "lidar_odom_topic": lidar_odom_topic,
                    "filtered_odom_topic": filtered_odom_topic,
                    "fuse_lidar_odometry": "true",
                }.items(),
            )
        ],
    )

    def make_map_assembler():
        return Node(
            package="rtabmap_util",
            executable="map_assembler",
            namespace="mapping",
            name="map_assembler",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[("mapData", "mapData_significant")],
        )

    def on_map_assembler_exit(event, _context):
        if not map_assembler_restart_budget.take(event.returncode):
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg=(
                            "map_assembler restart budget exhausted; "
                            "leaving the rest of the SLAM pipeline online"
                        )
                    )
                ]
            return []

        replacement = make_map_assembler()
        return [
            LogInfo(
                msg=(
                    "map_assembler exited with code "
                    f"{event.returncode}; restart "
                    f"{map_assembler_restart_budget.used}/"
                    f"{map_assembler_restart_budget.limit} after "
                    f"{map_assembler_restart_delay:.1f} seconds"
                )
            ),
            TimerAction(
                period=map_assembler_restart_delay,
                actions=[
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=replacement,
                            on_exit=on_map_assembler_exit,
                        )
                    ),
                    replacement,
                ],
            ),
        ]

    map_assembler = make_map_assembler()

    return [
        Node(
            package="metro_pointcloud_mapping",
            executable="cloud_gate",
            namespace="mapping",
            name="cloud_gate",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("cloud_raw", cloud_topic),
                ("cloud_valid", validated_cloud_topic),
            ],
        ),
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            namespace="mapping",
            name="icp_odometry",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("scan_cloud", validated_cloud_topic),
                ("odom", lidar_odom_topic),
                ("imu", "/mapping/imu_not_used"),
            ],
        ),
        odometry_fusion,
        Node(
            package="metro_pointcloud_mapping",
            executable="motion_cloud_gate",
            namespace="mapping",
            name="motion_cloud_gate",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("cloud_in", validated_cloud_topic),
                ("odom", filtered_odom_topic),
                ("cloud_out", keyframe_cloud_topic),
            ],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            namespace="mapping",
            name="rtabmap",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": use_sim_time,
                    "database_path": database_path,
                },
            ],
            remappings=[
                ("scan_cloud", keyframe_cloud_topic),
                ("odom", filtered_odom_topic),
                ("imu", "/mapping/imu_not_used"),
            ],
            arguments=rtabmap_arguments,
        ),
        Node(
            package="metro_pointcloud_mapping",
            executable="map_data_gate",
            namespace="mapping",
            name="map_data_gate",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            remappings=[
                ("map_data_in", "mapData"),
                ("map_data_out", "mapData_significant"),
            ],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=map_assembler,
                on_exit=on_map_assembler_exit,
            )
        ),
        map_assembler,
        Node(
            package="metro_pointcloud_mapping",
            executable="optimized_cloud_saver",
            namespace="mapping",
            name="optimized_cloud_saver",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": use_sim_time, "pcd_path": pcd_path},
            ],
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("metro_pointcloud_mapping"),
            "config",
            "graph_slam.yaml",
        ]
    )
    default_ekf_config = PathJoinSubstitution(
        [FindPackageShare("metro_localization"), "config", "ekf_odom.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "ekf_config_file", default_value=default_ekf_config
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("cloud_topic", default_value="/odin1/cloud_raw"),
            DeclareLaunchArgument(
                "validated_cloud_topic", default_value="/mapping/cloud_valid"
            ),
            DeclareLaunchArgument(
                "keyframe_cloud_topic", default_value="/mapping/cloud_keyframe"
            ),
            DeclareLaunchArgument(
                "raw_odom_topic", default_value="/wheel/odom_raw"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/odin1/imu"),
            DeclareLaunchArgument(
                "lidar_odom_topic", default_value="/lidar/odom"
            ),
            DeclareLaunchArgument(
                "filtered_odom_topic", default_value="/odometry/filtered"
            ),
            DeclareLaunchArgument(
                "database_path", default_value="/tmp/metro_rtabmap.db"
            ),
            DeclareLaunchArgument(
                "pcd_path", default_value="/tmp/metro_optimized_cloud.pcd"
            ),
            DeclareLaunchArgument(
                "reset_database",
                default_value="true",
                description="Delete an existing database before a new mapping run.",
            ),
            DeclareLaunchArgument(
                "ekf_frequency",
                default_value="10.0",
                description="Fused local odometry update rate.",
            ),
            DeclareLaunchArgument(
                "ekf_transform_time_offset",
                default_value="0.05",
                description="Future offset for odom TF in simulation.",
            ),
            DeclareLaunchArgument(
                "map_assembler_max_restarts",
                default_value="1",
                description="Maximum automatic restarts after an abnormal exit.",
            ),
            DeclareLaunchArgument(
                "map_assembler_restart_delay",
                default_value="15.0",
                description="Delay before the bounded map assembler restart.",
            ),
            OpaqueFunction(function=_launch_mapping),
        ]
    )
