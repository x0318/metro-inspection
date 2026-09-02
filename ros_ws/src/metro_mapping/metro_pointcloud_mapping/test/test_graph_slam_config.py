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

import importlib.util
from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).parents[1] / "config" / "graph_slam.yaml"
LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "graph_slam.launch.py"
SLAM_CHECK_PATH = (
    Path(__file__).parents[3]
    / "metro_sim"
    / "scripts"
    / "check_subway_v2_slam.sh"
)


def load_node(name):
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))[name][
        "ros__parameters"
    ]


def load_launch_module():
    spec = importlib.util.spec_from_file_location("graph_slam_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cloud_gate_rejects_sparse_and_out_of_order_lidar_frames():
    params = load_node("/mapping/cloud_gate")

    assert params["minimum_points"] >= 100
    assert params["reject_out_of_order"] is True


def test_icp_odometry_never_owns_the_odom_transform():
    params = load_node("/mapping/icp_odometry")

    assert params["frame_id"] == "base_footprint"
    assert params["odom_frame_id"] == "odom"
    assert params["publish_tf"] is False
    assert params["publish_null_when_lost"] is False
    assert params["guess_frame_id"] == "odom"
    assert params["expected_update_rate"] == 0.0
    assert params["Reg/Force3DoF"] == "true"
    assert params["Icp/PointToPlaneLowComplexityStrategy"] == "1"


def test_graph_slam_owns_only_map_to_odom_and_enables_loop_constraints():
    params = load_node("/mapping/rtabmap")

    assert params["subscribe_scan_cloud"] is True
    assert params["approx_sync"] is True
    assert params["publish_tf"] is True
    assert params["map_frame_id"] == "map"
    assert params["odom_frame_id"] == ""
    assert params["RGBD/NeighborLinkRefining"] == "true"
    assert params["RGBD/ProximityBySpace"] == "true"
    assert int(params["RGBD/ProximityPathMaxNeighbors"]) > 0
    assert params["Optimizer/Robust"] == "true"


def test_graph_nodes_limit_scan_density_without_reducing_map_resolution():
    params = load_node("/mapping/rtabmap")
    motion_gate = load_node("/mapping/motion_cloud_gate")

    scan_voxel_size = float(params["Mem/LaserScanVoxelSize"])
    map_cell_size = float(params["Grid/CellSize"])
    linear_update = float(params["RGBD/LinearUpdate"])
    motion_translation = motion_gate["translation_threshold_m"]
    motion_rotation = motion_gate["rotation_threshold_rad"]

    assert scan_voxel_size == map_cell_size == 0.05
    assert linear_update == 0.30
    assert linear_update < motion_translation <= 0.5
    assert float(params["RGBD/AngularUpdate"]) < motion_rotation <= 0.2
    assert 0.0 < motion_gate["maximum_odom_age_s"] <= 0.5


def test_optimized_map_reuses_keyframe_grids_and_is_saved():
    assembler = load_node("/mapping/map_assembler")
    saver = load_node("/mapping/optimized_cloud_saver")

    assert assembler["regenerate_local_grids"] is False
    assert assembler["map_always_update"] is False
    assert assembler["map_cleanup"] is True
    assert assembler["Grid/Sensor"] == "0"
    assert assembler["Grid/3D"] == "true"
    assert saver["save_on_shutdown"] is True
    assert saver["minimum_points"] > 0


def test_map_data_gate_uses_sub_voxel_change_thresholds():
    gate = load_node("/mapping/map_data_gate")
    assembler = load_node("/mapping/map_assembler")

    assert 0.0 < gate["translation_threshold_m"] < 0.05
    assert 0.0 < gate["rotation_threshold_rad"] < 0.05
    assert assembler["map_always_update"] is False


def test_map_assembler_restart_budget_is_bounded():
    launch_module = load_launch_module()

    budget = launch_module._RestartBudget(1)
    assert budget.take(-9) is True
    assert budget.take(-9) is False
    assert budget.used == 1


def test_map_assembler_clean_exit_is_not_restarted():
    launch_module = load_launch_module()

    budget = launch_module._RestartBudget(1)
    assert budget.take(0) is False
    assert budget.used == 0


def test_stationary_acceptance_does_not_require_a_new_rtabmap_info_message():
    script = SLAM_CHECK_PATH.read_text(encoding="utf-8")

    assert "require_topic_publisher /mapping/info" in script
    assert "require_topic_message /mapping/info" not in script
    assert "/mapping/rtabmap/publish_map" in script
    assert "--qos-durability transient_local" in script
