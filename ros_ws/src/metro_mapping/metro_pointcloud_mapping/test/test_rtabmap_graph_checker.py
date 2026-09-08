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

import pytest


CHECKER_PATH = (
    Path(__file__).parents[1] / "scripts" / "check_rtabmap_graph.py"
)
SPEC = importlib.util.spec_from_file_location("check_rtabmap_graph", CHECKER_PATH)
checker = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(checker)


def pose(x, y=0.0, z=0.0):
    return {"position": {"x": x, "y": y, "z": z}}


def link(link_type, translation_x):
    return {
        "from_id": 1,
        "to_id": 100,
        "type": link_type,
        "transform": {
            "translation": {"x": translation_x, "y": 0.0, "z": 0.0}
        },
    }


def validate(graph, max_revisit_translation_m=0.5):
    metrics = checker.analyze_graph(graph, max_revisit_translation_m)
    checker.validate_metrics(
        metrics,
        min_proximity_links=1,
        min_revisit_links=1,
        min_trajectory_extent_m=5.0,
    )
    return metrics


def test_accepts_long_trajectory_with_near_revisit_closure():
    graph = {
        "poses": [pose(0.0), pose(3.0), pose(6.0), pose(0.1)],
        "links": [link(0, 3.0), link(2, 0.1)],
    }

    metrics = validate(graph)

    assert metrics.pose_count == 4
    assert metrics.trajectory_extent_m == pytest.approx(6.0)
    assert metrics.proximity_link_count == 1
    assert metrics.revisit_link_count == 1


def test_rejects_graph_without_spatial_loop_closure():
    graph = {
        "poses": [pose(0.0), pose(6.0)],
        "links": [link(0, 6.0)],
    }

    with pytest.raises(checker.GraphCheckError, match="proximity links"):
        validate(graph)


def test_rejects_proximity_link_that_does_not_revisit_same_area():
    graph = {
        "poses": [pose(0.0), pose(6.0)],
        "links": [link(2, 2.0)],
    }

    with pytest.raises(checker.GraphCheckError, match="near-revisit links"):
        validate(graph)


def test_rejects_trivial_stationary_graph():
    graph = {
        "poses": [pose(0.0), pose(0.2), pose(0.0)],
        "links": [link(2, 0.0)],
    }

    with pytest.raises(checker.GraphCheckError, match="trajectory extent"):
        validate(graph)


def test_rejects_ros_cli_truncated_link_array():
    graph = {
        "poses": [pose(0.0), pose(6.0)],
        "links": [link(2, 0.0), "..."],
    }

    with pytest.raises(checker.GraphCheckError, match="link 1 is not a mapping"):
        validate(graph)
