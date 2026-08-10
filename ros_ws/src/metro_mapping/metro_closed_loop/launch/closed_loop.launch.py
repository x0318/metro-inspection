import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("metro_closed_loop")#找到总的功能包
    localization_pkg = get_package_share_directory("metro_localization")#找三维定位算法包
    gazebo_pkg = get_package_share_directory("gazebo_ros")#找gazebo的功能包
    default_world = os.path.join(pkg, "worlds", "metro_sim_tunnel_damage.world")#默认加载 metro_sim 隧道+假病害验证场景
    metro_sim_model_path = "/home/yang/metro-inspection/ros_ws/src/metro_sim/models"#让 Gazebo 能找到 model://subway_tunnel
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_model_path = (
        metro_sim_model_path
        if not existing_model_path
        else metro_sim_model_path + os.pathsep + existing_model_path
    )
    urdf = os.path.join(pkg, "urdf", "hardware_car.urdf.xacro")  # hardware car model wrapper
    rviz_config = os.path.join(pkg, "rviz", "closed_loop.rviz")#加载rviz
    #三维定位参数
    localization_config = os.path.join(
        localization_pkg, "config", "localization.yaml"
    )
    #解析Xacro
    robot_description = xacro.process_file(urdf).toxml()

    #读取启动参数
    use_sim_time = LaunchConfiguration("use_sim_time")#是否使用Gazebo仿真时间
    auto_drive = LaunchConfiguration("auto_drive")#是否自动驾驶
    auto_drive_delay = LaunchConfiguration("auto_drive_delay")#延迟多久启动自动驾驶
    auto_linear_x = LaunchConfiguration("auto_linear_x")#小车前进速度
    auto_angular_z = LaunchConfiguration("auto_angular_z")#小车旋转角速度
    auto_stop_distance = LaunchConfiguration("auto_stop_distance")#病害点距离小车前方多远时停车
    auto_slow_distance = LaunchConfiguration("auto_slow_distance")#接近病害前开始减速的距离
    start_rviz = LaunchConfiguration("rviz")#是否启动RViz
    gazebo_gui = LaunchConfiguration("gui")#是否显示Gazebo图形界面
    world = LaunchConfiguration("world")#允许启动时临时切换 world 文件
    #启动gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "gui": gazebo_gui}.items(),
    )
    #启动小车 并发布/tf和同步时间
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )
    #把小车生成到gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "inspection_car",
            # Start on the metro_sim rail/sleeper centerline and stop beside the red wall damage.
            # x=3.0 aligns with the red patch; y=0.0 is the track centerline; z=0.315 is rail-head/drive-surface height.
            # Y=0 keeps the car body facing along tunnel +X; the side-looking camera/lidar point to the right wall.
            "-x", "3.0", "-y", "0.0", "-z", "0.315", "-Y", "0.0",
        ],
        output="screen",
    )
    #算法节点列表
    algorithm_nodes = [
        Node(#假病害检测器
            package="metro_closed_loop",
            executable="damage_detector",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(#三维定位节点->病害三维坐标
            package="metro_localization",
            executable="damage_localizer",
            parameters=[localization_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(#定位误差评价节点
            package="metro_localization",
            executable="localization_evaluator",
            parameters=[localization_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(#空间语义映射节点：三维坐标 -> 里程/区段/时钟方位/JSON
            package="metro_localization",
            executable="damage_semantic_mapper",
            parameters=[localization_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_rviz),
            output="screen",
        ),
    ]

    auto_driver = Node(
        package="metro_closed_loop",
        executable="auto_driver",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "linear_x": ParameterValue(auto_linear_x, value_type=float),
                "angular_z": ParameterValue(auto_angular_z, value_type=float),
                "stop_distance_m": ParameterValue(auto_stop_distance, value_type=float),
                "slow_distance_m": ParameterValue(auto_slow_distance, value_type=float),
            }
        ],
        condition=IfCondition(auto_drive),
        output="screen",
    )
    #返回所有的启动动作
    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            SetEnvironmentVariable("ALSOFT_DRIVERS", "null"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world file. Default uses metro_sim subway_tunnel plus the placeholder red damage target.",
            ),
            DeclareLaunchArgument(
                "auto_drive",
                default_value="false",
                description="Move the robot automatically after static validation succeeds.",
            ),
            DeclareLaunchArgument(
                "auto_drive_delay",
                default_value="9.0",
                description="Wall-clock delay before auto driving starts.",
            ),
            DeclareLaunchArgument(
                "auto_linear_x",
                default_value="0.08",
                description="Auto-drive forward speed in m/s.",
            ),
            DeclareLaunchArgument(
                "auto_angular_z",
                default_value="0.0",
                description="Auto-drive yaw-rate amplitude in rad/s.",
            ),
            DeclareLaunchArgument(
                "auto_stop_distance",
                default_value="0.80",
                description="Stop when the localized damage point is this far in front of base_footprint.",
            ),
            DeclareLaunchArgument(
                "auto_slow_distance",
                default_value="1.60",
                description="Start slowing down when the localized damage point is closer than this forward distance.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            gazebo,
            state_publisher,
            TimerAction(period=2.0, actions=[spawn]),
            TimerAction(period=4.0, actions=algorithm_nodes),
            TimerAction(period=auto_drive_delay, actions=[auto_driver]),
        ]
    )



