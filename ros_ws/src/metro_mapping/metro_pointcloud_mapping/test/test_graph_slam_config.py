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

from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).parents[1] / "config" / "graph_slam.yaml"


def load_node(name):
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))[name][
        "ros__parameters"
    ]


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


def test_optimized_map_is_rebuilt_from_lidar_nodes_and_saved():
    assembler = load_node("/mapping/map_assembler")
    saver = load_node("/mapping/optimized_cloud_saver")

    assert assembler["regenerate_local_grids"] is True
    assert assembler["Grid/Sensor"] == "0"
    assert assembler["Grid/3D"] == "true"
    assert saver["save_on_shutdown"] is True
    assert saver["minimum_points"] > 0
