"""Start the sensor simulation, RViz, and inspection dashboard together."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_project_dir() -> str:
    configured = os.environ.get("METRO_INSPECTION_PROJECT_DIR", "").strip()
    if configured:
        return str(Path(configured).expanduser().resolve())

    for parent in Path(__file__).resolve().parents:
        if (parent / "ros_ws/src/metro_sim").is_dir():
            return str(parent)

    return "/home/jo/my-project/metro-inspection"


def _as_bool(value: str, argument_name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{argument_name} must be true or false, got: {value!r}")


def _launch_platform(context):
    project_dir = Path(
        LaunchConfiguration("project_dir").perform(context)
    ).expanduser().resolve()
    simulation_script = (
        project_dir
        / "ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh"
    )
    dashboard_dir = project_dir / "dashboard"

    configured_rviz = LaunchConfiguration("rviz_config").perform(context).strip()
    rviz_config = (
        Path(configured_rviz).expanduser().resolve()
        if configured_rviz
        else project_dir / "ros_ws/src/metro_sim/config/odin1_pointcloud.rviz"
    )
    configured_database = LaunchConfiguration("database_path").perform(context).strip()
    database_path = (
        Path(configured_database).expanduser().resolve()
        if configured_database
        else Path.home() / ".local/share/metro-inspection/defects.sqlite3"
    )

    gui = _as_bool(LaunchConfiguration("gui").perform(context), "gui")
    dashboard_enabled = _as_bool(
        LaunchConfiguration("dashboard").perform(context), "dashboard"
    )
    rviz_enabled = _as_bool(LaunchConfiguration("rviz").perform(context), "rviz")
    browser_enabled = _as_bool(
        LaunchConfiguration("open_browser").perform(context), "open_browser"
    )
    qt_enabled = _as_bool(LaunchConfiguration("qt").perform(context), "qt")
    if qt_enabled and not dashboard_enabled:
        raise ValueError("qt:=true requires dashboard:=true")

    required_paths = {"simulation script": simulation_script}
    if dashboard_enabled:
        required_paths["dashboard directory"] = dashboard_dir
    if rviz_enabled:
        required_paths["RViz configuration"] = rviz_config
    for label, path in required_paths.items():
        if not path.exists():
            raise FileNotFoundError(f"{label} not found: {path}")

    dashboard_port_text = LaunchConfiguration("dashboard_port").perform(context)
    try:
        dashboard_port = int(dashboard_port_text)
    except ValueError as error:
        raise ValueError("dashboard_port must be an integer") from error
    if not 1 <= dashboard_port <= 65535:
        raise ValueError("dashboard_port must be between 1 and 65535")

    simulation = ExecuteProcess(
        cmd=[str(simulation_script), f"gui:={'true' if gui else 'false'}"],
        output="screen",
        # Gazebo Classic may use 15 seconds to stop its server cleanly.
        sigterm_timeout="25.0",
        sigkill_timeout="5.0",
    )
    actions = [
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
        ),
        SetEnvironmentVariable(
            "SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI",
            LaunchConfiguration("gazebo_master_uri"),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=simulation,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="The Gazebo simulation process exited"
                        )
                    )
                ],
            )
        ),
        simulation,
    ]

    if rviz_enabled:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="inspection_rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": True}],
            )
        )

    if dashboard_enabled:
        actions.append(
            Node(
                package="metro_dashboard_bridge",
                executable="defect_event_bridge",
                name="defect_event_bridge",
                output="screen",
                additional_env={
                    "METRO_DASHBOARD_DIR": str(dashboard_dir),
                    "METRO_DASHBOARD_BIND_ADDRESS": LaunchConfiguration(
                        "dashboard_bind_address"
                    ),
                    "METRO_DASHBOARD_PORT": str(dashboard_port),
                },
                parameters=[
                    {
                        "defect_topic": LaunchConfiguration("defect_topic"),
                        "database_path": str(database_path),
                        "inspection_session_id": LaunchConfiguration(
                            "inspection_session_id"
                        ),
                    }
                ],
            )
        )
        if browser_enabled:
            actions.append(
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "python3",
                                "-m",
                                "webbrowser",
                                f"http://127.0.0.1:{dashboard_port}",
                            ],
                            output="screen",
                        )
                    ],
                )
            )
        if qt_enabled:
            actions.append(
                TimerAction(
                    period=1.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2",
                                "run",
                                "metro_dashboard_bridge",
                                "dashboard_qt",
                                "--url",
                                f"http://127.0.0.1:{dashboard_port}",
                                "--startup-timeout",
                                "30.0",
                            ],
                            output="screen",
                        )
                    ],
                )
            )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_dir",
                default_value=_default_project_dir(),
                description="Metro Inspection repository root.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the Odin1 point-cloud configuration.",
            ),
            DeclareLaunchArgument(
                "dashboard",
                default_value="true",
                description="Start the dashboard ROS bridge and HTTP server.",
            ),
            DeclareLaunchArgument(
                "qt",
                default_value="true",
                description="Open the dashboard in an embedded Qt window.",
            ),
            DeclareLaunchArgument(
                "open_browser",
                default_value="false",
                description="Try to open the dashboard in the default browser.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="RViz config path; empty uses metro_sim/config/odin1_pointcloud.rviz.",
            ),
            DeclareLaunchArgument(
                "defect_topic",
                default_value="/simulation/defect_events",
                description="DefectEvent topic consumed by the dashboard.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="",
                description="SQLite path; empty uses ~/.local/share/metro-inspection/defects.sqlite3.",
            ),
            DeclareLaunchArgument(
                "inspection_session_id",
                default_value="simulation",
                description="Dashboard event session stored in SQLite.",
            ),
            DeclareLaunchArgument(
                "dashboard_bind_address",
                default_value="127.0.0.1",
                description="HTTP bind address; use 0.0.0.0 only when remote access is required.",
            ),
            DeclareLaunchArgument(
                "dashboard_port",
                default_value="8088",
                description="Dashboard HTTP port.",
            ),
            DeclareLaunchArgument(
                "gazebo_master_uri",
                default_value="http://127.0.0.1:11370",
                description="Gazebo Classic master URI for this simulation.",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="70",
                description="ROS 2 DDS domain shared by all launched components.",
            ),
            OpaqueFunction(function=_launch_platform),
        ]
    )
