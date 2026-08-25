"""Fuse raw wheel odometry and Odin1 IMU into a local odometry estimate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    frequency = LaunchConfiguration("frequency")
    transform_time_offset = LaunchConfiguration("transform_time_offset")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    filtered_odom_topic = LaunchConfiguration("filtered_odom_topic")

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
                "filtered_odom_topic", default_value="/odometry/filtered"
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="odometry_ekf",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "frequency": frequency,
                        "transform_time_offset": transform_time_offset,
                    },
                ],
                remappings=[
                    ("wheel/odom_raw", raw_odom_topic),
                    ("odin1/imu", imu_topic),
                    ("odometry/filtered", filtered_odom_topic),
                ],
            ),
        ]
    )
