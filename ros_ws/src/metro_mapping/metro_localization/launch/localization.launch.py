import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("metro_localization")  # 查找功能包
    config = os.path.join(package_share, "config", "localization.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_evaluator = LaunchConfiguration("run_evaluator")
    run_semantic_mapper = LaunchConfiguration("run_semantic_mapper")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("run_evaluator", default_value="true"),
            DeclareLaunchArgument("run_semantic_mapper", default_value="true"),
            Node(
                package="metro_localization",
                executable="damage_localizer",
                name="damage_localizer",
                parameters=[config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="metro_localization",
                executable="damage_semantic_mapper",
                name="damage_semantic_mapper",
                parameters=[config, {"use_sim_time": use_sim_time}],
                condition=IfCondition(run_semantic_mapper),
                output="screen",
            ),
            Node(
                package="metro_localization",
                executable="localization_evaluator",
                name="localization_evaluator",
                parameters=[config, {"use_sim_time": use_sim_time}],
                condition=IfCondition(run_evaluator),
                output="screen",
            ),
        ]
    )
