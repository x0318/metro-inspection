import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "inspection_platform.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "inspection_platform_launch", LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_launch_exposes_expected_controls() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert set(arguments) == {
        "project_dir",
        "gui",
        "initial_pitch_deg",
        "rviz",
        "detection",
        "yolo_model_path",
        "yolo_auto_drive",
        "localization",
        "dashboard",
        "qt",
        "open_browser",
        "rviz_config",
        "defect_topic",
        "database_path",
        "inspection_session_id",
        "dashboard_bind_address",
        "dashboard_port",
        "gazebo_master_uri",
        "ros_domain_id",
    }
    assert any(isinstance(entity, OpaqueFunction) for entity in description.entities)


def test_boolean_launch_arguments_are_strict() -> None:
    module = _load_launch_module()

    assert module._as_bool("true", "gui") is True
    assert module._as_bool("OFF", "gui") is False

    try:
        module._as_bool("sometimes", "gui")
    except ValueError as error:
        assert "gui must be true or false" in str(error)
    else:
        raise AssertionError("invalid boolean value was accepted")


def test_fault_status_records_component_and_ignores_requested_shutdown(
    tmp_path, monkeypatch
):
    module = _load_launch_module()
    status = tmp_path / "runtime.status.json"
    monkeypatch.setenv("METRO_RUNTIME_STATUS_PATH", str(status))
    handler = module._shutdown_after_exit("detection")
    assert handler(SimpleNamespace(returncode=7), SimpleNamespace(is_shutdown=False))
    assert json.loads(status.read_text())["component"] == "detection"
    assert json.loads(status.read_text())["exit_code"] == 7
    assert (
        handler(SimpleNamespace(returncode=-2), SimpleNamespace(is_shutdown=True)) == []
    )
    assert json.loads(status.read_text())["exit_code"] == 7


def test_default_project_directory_contains_runtime_assets() -> None:
    module = _load_launch_module()
    project_dir = Path(module._default_project_dir())

    assert (
        project_dir / "ros_ws/src/metro_sim/scripts/open_subway_tunnel_v2_sensors.sh"
    ).is_file()
    assert (project_dir / "scripts/open_yolo_coverage.sh").is_file()
    assert (project_dir / "ros_ws/src/metro_sim/config/yolo_coverage.rviz").is_file()
    assert (project_dir / "dashboard/index.html").is_file()


def test_rviz_profiles_only_reference_topics_from_their_runtime_mode() -> None:
    module = _load_launch_module()
    project_dir = Path(module._default_project_dir())
    sensor_config = (
        project_dir / "ros_ws/src/metro_sim/config/odin1_pointcloud.rviz"
    ).read_text(encoding="utf-8")
    yolo_config = (
        project_dir / "ros_ws/src/metro_sim/config/yolo_coverage.rviz"
    ).read_text(encoding="utf-8")

    assert "/odin1/rgb/image_raw" in sensor_config
    assert "/localization/debug_projection" not in sensor_config
    assert "/damage_mask" not in sensor_config
    assert "Name: Selected YOLO Camera" in yolo_config
    assert "/damage_detection/xj4/annotated_image" in yolo_config
    assert yolo_config.count("Class: rviz_default_plugins/Image") == 1
    assert yolo_config.count("Reliability Policy: Best Effort") == 1


def test_runtime_actions_can_be_constructed_for_headless_mode() -> None:
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "project_dir": module._default_project_dir(),
            "gui": "false",
            "initial_pitch_deg": "45",
            "rviz": "false",
            "detection": "false",
            "yolo_auto_drive": "false",
            "localization": "true",
            "dashboard": "false",
            "qt": "false",
            "open_browser": "false",
            "rviz_config": "",
            "defect_topic": "",
            "database_path": "",
            "dashboard_port": "8088",
            "ros_domain_id": "70",
            "gazebo_master_uri": "http://127.0.0.1:11370",
        }
    )

    assert module._launch_platform(context)
