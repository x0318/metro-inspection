"""Existing fork training world, Nav2 and its matching web interface."""

import os
from pathlib import Path
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('metro_navigation_demo'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    attach = LaunchConfiguration('attach').perform(context).lower() == 'true'
    domain = int(LaunchConfiguration('domain').perform(context))
    port = int(LaunchConfiguration('port').perform(context))
    gazebo_port = int(LaunchConfiguration('gazebo_port').perform(context))
    if not 0 <= domain <= 101 or not all(1024 <= p <= 65535 for p in (port, gazebo_port)) or port == gazebo_port:
        raise ValueError('Invalid ROS domain or conflicting ports')
    for checked_port in ([port] if attach else [port, gazebo_port]):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(('127.0.0.1', checked_port))
            except OSError as error:
                raise RuntimeError(f'端口 {checked_port} 已占用，请停止原实例或选择其他端口。') from error

    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', f'http://127.0.0.1:{gazebo_port}'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(share / 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', '')),
    ]
    critical = []
    if not attach:
        server = ExecuteProcess(cmd=['gzserver', str(share / 'worlds/route_choice_training.world'),
                                     '-slibgazebo_ros_init.so', '-slibgazebo_ros_factory.so',
                                     '-slibgazebo_ros_force_system.so'], output='screen')
        rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
                   parameters=[{'use_sim_time': True,
                                'robot_description': (share / 'urdf/gazebo_train_tf.urdf').read_text()}])
        watchdog = Node(package='metro_navigation_demo', executable='drive_watchdog',
                        parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/cmd_vel_drive', 'timeout': 0.5}],
                        output='screen')
        actions += [server, rsp, watchdog]
        critical += [server, rsp, watchdog]
        nav_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_launch)),
                       launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                                         'params_file': str(share / 'config/nav2_odom_params.yaml'),
                                         'use_composition': 'False'}.items()))
        if gui:
            actions.append(ExecuteProcess(cmd=['gzclient'], output='screen'))
        if rviz:
            actions.append(Node(package='rviz2', executable='rviz2',
                                arguments=['-d', str(share / 'config/gazebo_lidar.rviz')],
                                parameters=[{'use_sim_time': True}]))

    bridge = Node(package='metro_navigation_demo', executable='inspection_dashboard_bridge', output='screen',
                  additional_env={
                      'PLATFORM_PORT': str(port), 'PLATFORM_BIND_ADDRESS': '127.0.0.1',
                      'PLATFORM_CAMERA_PROFILE': 'route_training',
                      'PLATFORM_DASHBOARD_DIR': str(share / 'web/inspection_dashboard'),
                      'PLATFORM_ROUTE_CONFIG': str(share / 'config/route_choice_training_routes.json'),
                      'PLATFORM_NAV_BEHAVIOR_TREE': str(share / 'config/navigate_through_poses_odom.xml'),
                  })
    actions.append(bridge)
    critical.append(bridge)
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
                       on_exit=[EmitEvent(event=Shutdown(reason='岔轨测试关键进程已退出'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain', default_value='74'),
        DeclareLaunchArgument('port', default_value='8090'),
        DeclareLaunchArgument('gazebo_port', default_value='11374'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('attach', default_value='false'),
        OpaqueFunction(function=start),
    ])
