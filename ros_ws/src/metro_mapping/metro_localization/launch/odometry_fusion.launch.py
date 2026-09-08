"""Fuse raw wheel odometry and Odin1 IMU into a local odometry estimate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIDAR_ODOMETRY_PARAMETERS = {
    "odom1": "lidar/odom",
    "odom1_config": [
        True,
        True,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
    ],
    "odom1_differential": False,
    "odom1_relative": True,
    "odom1_queue_size": 10,
    "odom1_pose_rejection_threshold": 5.0,
}


def _is_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_ekf(context):
    fuse_lidar_odometry = _is_true(
        LaunchConfiguration("fuse_lidar_odometry").perform(context)
    )

    parameter_overrides = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "frequency": LaunchConfiguration("frequency"),
        "transform_time_offset": LaunchConfiguration("transform_time_offset"),
    }
    if fuse_lidar_odometry:
        parameter_overrides.update(LIDAR_ODOMETRY_PARAMETERS)

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="odometry_ekf",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), parameter_overrides],
            remappings=[
                ("wheel/odom_raw", LaunchConfiguration("raw_odom_topic")),
                ("odin1/imu", LaunchConfiguration("imu_topic")),
                ("lidar/odom", LaunchConfiguration("lidar_odom_topic")),
                ("odometry/filtered", LaunchConfiguration("filtered_odom_topic")),
            ],
        )
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("metro_localization"), "config", "ekf_odom.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "frequency",
                default_value="10.0",
                description=(
                    "EKF update rate. The Gazebo profile publishes /clock at "
                    "10 Hz; use 50.0 with use_sim_time:=false on hardware."
                ),
            ),
            DeclareLaunchArgument(
                "transform_time_offset",
                default_value="0.05",
                description=(
                    "Seconds to future-date EKF TF. Use 0.0 on hardware unless "
                    "timestamped consumers demonstrate a measured need."
                ),
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
                "fuse_lidar_odometry",
                default_value="false",
                description=(
                    "Fuse relative-origin absolute x/y/yaw from lidar odometry. "
                    "Enable only when a scan-matching odometry node is running."
                ),
            ),
            OpaqueFunction(function=_launch_ekf),
        ]
    )
