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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    target_frame = LaunchConfiguration("target_frame")
    pcd_path = LaunchConfiguration("pcd_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_config = PathJoinSubstitution(
        [FindPackageShare("metro_pointcloud_mapping"), "config", "accumulated_map.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("cloud_topic", default_value="/odin1/cloud_raw"),
            DeclareLaunchArgument("target_frame", default_value="odom"),
            DeclareLaunchArgument(
                "pcd_path", default_value="/tmp/metro_accumulated_cloud.pcd"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="metro_pointcloud_mapping",
                executable="cloud_accumulator",
                namespace="mapping",
                name="cloud_accumulator",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "target_frame": target_frame,
                        "pcd_path": pcd_path,
                        "use_sim_time": use_sim_time,
                    },
                ],
                remappings=[("cloud_in", cloud_topic)],
            ),
        ]
    )
