"""Start simulation, YOLO, 3D localization, RViz, and the dashboard."""

import json
import os
from datetime import datetime
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


def _shutdown_after_exit(component):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        status_path = os.environ.get("METRO_RUNTIME_STATUS_PATH", "")
        if status_path:
            try:
                path = Path(status_path)
                temporary = path.with_suffix(".tmp")
                temporary.write_text(
                    json.dumps(
                        {
                            "component": component,
                            "exit_code": event.returncode,
                            "timestamp": datetime.now().isoformat(),
                        }
                    )
                    + "\n",
                    encoding="utf-8",
                )
                temporary.replace(path)
            except OSError as error:
                print(f"Could not write runtime status: {error}", flush=True)
        return [EmitEvent(event=Shutdown(reason=f"The {component} process exited"))]

    return on_exit


def _launch_platform(context):
    project_dir = (
        Path(LaunchConfiguration("project_dir").perform(context)).expanduser().resolve()
    )
    simulation_script = (
        project_dir / "ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh"
    )
    yolo_script = project_dir / "scripts/open_yolo_coverage.sh"
    dashboard_dir = project_dir / "dashboard"
    localization_config = (
        project_dir
        / "ros_ws/src/metro_mapping/metro_localization/config/localization.yaml"
    )

    detection_enabled = _as_bool(
        LaunchConfiguration("detection").perform(context), "detection"
    )
    configured_rviz = LaunchConfiguration("rviz_config").perform(context).strip()
    rviz_config = (
        Path(configured_rviz).expanduser().resolve()
        if configured_rviz
        else project_dir
        / "ros_ws/src/metro_sim/config"
        / ("yolo_coverage.rviz" if detection_enabled else "odin1_pointcloud.rviz")
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
    yolo_auto_drive = _as_bool(
        LaunchConfiguration("yolo_auto_drive").perform(context), "yolo_auto_drive"
    )
    localization_enabled = detection_enabled and _as_bool(
        LaunchConfiguration("localization").perform(context), "localization"
    )
    if qt_enabled and not dashboard_enabled:
        raise ValueError("qt:=true requires dashboard:=true")

    required_paths = {"simulation script": simulation_script}
    if dashboard_enabled:
        required_paths["dashboard directory"] = dashboard_dir
    if rviz_enabled:
        required_paths["RViz configuration"] = rviz_config
    if detection_enabled:
        configured_model = LaunchConfiguration("yolo_model_path").perform(context)
        yolo_model_path = Path(configured_model).expanduser().resolve()
        required_paths.update(
            {
                "YOLO launch script": yolo_script,
                "YOLO model": yolo_model_path,
                "YOLO Python environment": project_dir / ".venv-yolo/bin/python",
                "YOLO NumPy compatibility overlay": project_dir
                / ".yolo-ros-compat/numpy",
                "YOLO OpenCV compatibility overlay": project_dir
                / ".yolo-ros-compat/cv2",
            }
        )
    if localization_enabled:
        required_paths["3D localization configuration"] = localization_config
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

    configured_defect_topic = (
        LaunchConfiguration("defect_topic").perform(context).strip()
    )
    defect_topic = configured_defect_topic or (
        "/localized/defect_events"
        if localization_enabled
        else "/simulation/defect_events"
    )

    simulation = ExecuteProcess(
        cmd=[str(simulation_script), f"gui:={'true' if gui else 'false'}"],
        output="screen",
        additional_env={
            "SUBWAY_V2_INITIAL_PITCH_DEG": LaunchConfiguration("initial_pitch_deg")
        },
        # Gazebo Classic may use 15 seconds to stop its server cleanly.
        sigterm_timeout="25.0",
        sigkill_timeout="5.0",
    )
    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable(
            "SUBWAY_TUNNEL_V2_GAZEBO_MASTER_URI",
            LaunchConfiguration("gazebo_master_uri"),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=simulation,
                on_exit=_shutdown_after_exit("simulation"),
            )
        ),
        simulation,
    ]

    if detection_enabled:
        detection = ExecuteProcess(
            cmd=[str(yolo_script)],
            output="screen",
            additional_env={
                "METRO_YOLO_MODEL_PATH": str(yolo_model_path),
                "METRO_COVERAGE_AUTO_DRIVE": ("true" if yolo_auto_drive else "false"),
                "METRO_YOLO_SKIP_BUILD": "1",
                "ROS_DOMAIN_ID": LaunchConfiguration("ros_domain_id"),
            },
            sigterm_timeout="10.0",
            sigkill_timeout="5.0",
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=detection,
                        on_exit=_shutdown_after_exit("detection"),
                    )
                ),
                detection,
            ]
        )

    if localization_enabled:
        localizer = Node(
            package="metro_localization",
            executable="damage_localizer",
            name="odin1_damage_localizer",
            output="screen",
            parameters=[
                str(localization_config),
                {
                    "use_sim_time": True,
                    "detections_topic": "/damage_detections/odin1",
                    "cloud_topic": "/odin1/cloud_raw",
                    "image_topic": "/odin1/rgb/image_raw",
                    "camera_info_topic": "/odin1/rgb/camera_info",
                    "use_mask": False,
                    "sync_queue_size": 5,
                    "sync_slop_sec": 0.12,
                    "publish_events": True,
                    "event_topic": defect_topic,
                    "camera_name": "odin1",
                    "model_name": str(yolo_model_path),
                },
            ],
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=localizer,
                        on_exit=_shutdown_after_exit("localization"),
                    )
                ),
                localizer,
            ]
        )

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
        configured_session_id = (
            LaunchConfiguration("inspection_session_id").perform(context).strip()
        )
        inspection_session_id = configured_session_id or datetime.now().strftime(
            "simulation-%Y%m%d-%H%M%S"
        )
        dashboard_bridge = Node(
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
                    "defect_topic": defect_topic,
                    "database_path": str(database_path),
                    "inspection_session_id": inspection_session_id,
                }
            ],
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=dashboard_bridge,
                        on_exit=_shutdown_after_exit("dashboard"),
                    )
                ),
                dashboard_bridge,
            ]
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
                "initial_pitch_deg",
                default_value="45",
                description=(
                    "Initial Pitch joint angle in degrees (-15 to +45); "
                    "+45 aims forward and upward along the tunnel wall."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the configuration selected for this mode.",
            ),
            DeclareLaunchArgument(
                "detection",
                default_value="true",
                description="Start five-camera YOLO and use its RViz configuration.",
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=os.environ.get(
                    "METRO_YOLO_MODEL_PATH",
                    "/home/jo/incoming/yolov8n_sim_demo_best(1).pt",
                ),
                description=(
                    "YOLO checkpoint used when detection is enabled; defaults "
                    "to the simulation-trained model."
                ),
            ),
            DeclareLaunchArgument(
                "yolo_auto_drive",
                default_value="false",
                description="Drive the bounded ten-site coverage pass automatically.",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="true",
                description=(
                    "Fuse Odin1 YOLO boxes with its calibrated point cloud and "
                    "publish 3D engineering-location events. Effective only "
                    "when detection is enabled."
                ),
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
                description="RViz config path; empty selects the sensor or YOLO profile.",
            ),
            DeclareLaunchArgument(
                "defect_topic",
                default_value="",
                description=(
                    "DefectEvent topic used by localization and dashboard; empty "
                    "selects localized events or simulation coverage automatically."
                ),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="",
                description=(
                    "SQLite path; empty uses "
                    "~/.local/share/metro-inspection/defects.sqlite3."
                ),
            ),
            DeclareLaunchArgument(
                "inspection_session_id",
                default_value="",
                description=(
                    "Dashboard event session stored in SQLite; empty creates a "
                    "fresh timestamped simulation session for each launch."
                ),
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
