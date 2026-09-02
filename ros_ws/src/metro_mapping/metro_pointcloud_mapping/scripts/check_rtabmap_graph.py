#!/usr/bin/env python3
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

"""Validate cumulative loop-closure evidence in an RTAB-Map MapGraph YAML message."""

import argparse
from dataclasses import dataclass
import math
import sys
from typing import Any, Mapping, Sequence

import yaml


LOCAL_SPACE_CLOSURE_TYPE = 2


class GraphCheckError(ValueError):
    """Raised when a graph is malformed or fails an acceptance condition."""


@dataclass(frozen=True)
class GraphMetrics:
    pose_count: int
    trajectory_extent_m: float
    proximity_link_count: int
    revisit_link_count: int
    minimum_proximity_translation_m: float | None


def _position(pose: Mapping[str, Any], index: int) -> tuple[float, float, float]:
    try:
        position = pose["position"]
        return (
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise GraphCheckError(f"pose {index} has no valid XYZ position") from error


def _translation_norm(link: Mapping[str, Any], index: int) -> float:
    try:
        translation = link["transform"]["translation"]
        values = (
            float(translation["x"]),
            float(translation["y"]),
            float(translation["z"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise GraphCheckError(
            f"proximity link {index} has no valid XYZ translation"
        ) from error
    return math.sqrt(sum(value * value for value in values))


def analyze_graph(
    graph: Mapping[str, Any], max_revisit_translation_m: float
) -> GraphMetrics:
    """Extract acceptance metrics from one rtabmap_msgs/MapGraph document."""
    poses = graph.get("poses")
    links = graph.get("links")
    if not isinstance(poses, Sequence) or isinstance(poses, (str, bytes)):
        raise GraphCheckError("MapGraph poses must be a sequence")
    if not isinstance(links, Sequence) or isinstance(links, (str, bytes)):
        raise GraphCheckError("MapGraph links must be a sequence")
    if len(poses) < 2:
        raise GraphCheckError("MapGraph must contain at least two poses")

    positions = []
    for index, pose in enumerate(poses):
        if not isinstance(pose, Mapping):
            raise GraphCheckError(f"pose {index} is not a mapping")
        positions.append(_position(pose, index))

    axis_extents = [
        max(position[axis] for position in positions)
        - min(position[axis] for position in positions)
        for axis in range(3)
    ]
    trajectory_extent_m = max(axis_extents)

    proximity_translations = []
    for index, link in enumerate(links):
        if not isinstance(link, Mapping):
            raise GraphCheckError(f"link {index} is not a mapping")
        try:
            link_type = int(link["type"])
        except (KeyError, TypeError, ValueError) as error:
            raise GraphCheckError(f"link {index} has no valid type") from error
        if link_type == LOCAL_SPACE_CLOSURE_TYPE:
            proximity_translations.append(_translation_norm(link, index))

    revisit_link_count = sum(
        translation <= max_revisit_translation_m
        for translation in proximity_translations
    )
    minimum_translation = (
        min(proximity_translations) if proximity_translations else None
    )
    return GraphMetrics(
        pose_count=len(positions),
        trajectory_extent_m=trajectory_extent_m,
        proximity_link_count=len(proximity_translations),
        revisit_link_count=revisit_link_count,
        minimum_proximity_translation_m=minimum_translation,
    )


def validate_metrics(
    metrics: GraphMetrics,
    min_proximity_links: int,
    min_revisit_links: int,
    min_trajectory_extent_m: float,
) -> None:
    """Raise GraphCheckError when cumulative graph evidence is insufficient."""
    failures = []
    if metrics.proximity_link_count < min_proximity_links:
        failures.append(
            "proximity links "
            f"{metrics.proximity_link_count} < required {min_proximity_links}"
        )
    if metrics.revisit_link_count < min_revisit_links:
        failures.append(
            "near-revisit links "
            f"{metrics.revisit_link_count} < required {min_revisit_links}"
        )
    if metrics.trajectory_extent_m < min_trajectory_extent_m:
        failures.append(
            "trajectory extent "
            f"{metrics.trajectory_extent_m:.3f} m < required "
            f"{min_trajectory_extent_m:.3f} m"
        )
    if failures:
        raise GraphCheckError("; ".join(failures))


def _non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return parsed


def _non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("must be a finite non-negative number")
    return parsed


def _load_graph(stream: Any) -> Mapping[str, Any]:
    try:
        documents = yaml.safe_load_all(stream)
        graph = next(
            (
                document
                for document in documents
                if isinstance(document, Mapping) and "poses" in document
            ),
            None,
        )
    except yaml.YAMLError as error:
        raise GraphCheckError(f"invalid MapGraph YAML: {error}") from error
    if graph is None:
        raise GraphCheckError("no MapGraph YAML document received")
    return graph


def _format_metrics(metrics: GraphMetrics) -> str:
    minimum_translation = metrics.minimum_proximity_translation_m
    minimum_text = "none" if minimum_translation is None else f"{minimum_translation:.3f}"
    return (
        f"poses={metrics.pose_count} "
        f"trajectory_extent_m={metrics.trajectory_extent_m:.3f} "
        f"proximity_links={metrics.proximity_link_count} "
        f"near_revisit_links={metrics.revisit_link_count} "
        f"min_proximity_translation_m={minimum_text}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check RTAB-Map spatial loop closures from MapGraph YAML on stdin."
    )
    parser.add_argument("--min-proximity-links", type=_non_negative_int, default=1)
    parser.add_argument("--min-revisit-links", type=_non_negative_int, default=1)
    parser.add_argument(
        "--max-revisit-translation-m", type=_non_negative_float, default=0.5
    )
    parser.add_argument(
        "--min-trajectory-extent-m", type=_non_negative_float, default=5.0
    )
    arguments = parser.parse_args()

    try:
        graph = _load_graph(sys.stdin)
        metrics = analyze_graph(graph, arguments.max_revisit_translation_m)
        validate_metrics(
            metrics,
            arguments.min_proximity_links,
            arguments.min_revisit_links,
            arguments.min_trajectory_extent_m,
        )
    except GraphCheckError as error:
        print(f"RTABMAP_GRAPH_CHECK=FAIL {error}", file=sys.stderr)
        return 1

    print(f"RTABMAP_GRAPH_CHECK=PASS {_format_metrics(metrics)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
