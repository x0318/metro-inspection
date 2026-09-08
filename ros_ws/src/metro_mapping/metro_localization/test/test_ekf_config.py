from pathlib import Path
import runpy

import yaml


CONFIG_PATH = Path(__file__).parents[1] / "config" / "ekf_odom.yaml"
LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "odometry_fusion.launch.py"


def load_parameters():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["odometry_ekf"][
        "ros__parameters"
    ]


def test_local_ekf_owns_only_local_odometry_transform():
    params = load_parameters()

    assert params["world_frame"] == "odom"
    assert params["odom_frame"] == "odom"
    assert params["base_link_frame"] == "base_footprint"
    assert params["two_d_mode"] is True
    assert params["publish_tf"] is True


def test_ekf_uses_only_valid_baseline_measurements():
    params = load_parameters()

    assert params["odom0"] == "wheel/odom_raw"
    assert params["imu0"] == "odin1/imu"
    assert len(params["odom0_config"]) == 15
    assert len(params["imu0_config"]) == 15
    assert [index for index, enabled in enumerate(params["odom0_config"]) if enabled] == [
        6,
        11,
    ]
    assert [index for index, enabled in enumerate(params["imu0_config"]) if enabled] == [
        11
    ]
    assert len(params["process_noise_covariance"]) == 225


def test_lidar_pose_anchors_planar_drift_without_differentiating_noise():
    launch_globals = runpy.run_path(str(LAUNCH_PATH))
    params = launch_globals["LIDAR_ODOMETRY_PARAMETERS"]

    assert params["odom1"] == "lidar/odom"
    assert [index for index, enabled in enumerate(params["odom1_config"]) if enabled] == [
        0,
        1,
        5,
    ]
    assert params["odom1_differential"] is False
    assert params["odom1_relative"] is True
    assert params["odom1_pose_rejection_threshold"] == 5.0
