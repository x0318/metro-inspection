#!/usr/bin/env python3
# Copyright 2026 jo0625
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import struct
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Component:
    triangles: list[int]
    center: tuple[float, float, float]


class BinaryStl:
    def __init__(self, path: Path):
        self.path = path
        data = path.read_bytes()
        self.triangle_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + self.triangle_count * 50
        if len(data) != expected_size:
            raise ValueError(f"{path} is not a supported binary STL")
        self.records = [
            data[84 + index * 50: 84 + (index + 1) * 50]
            for index in range(self.triangle_count)
        ]
        self.components = self._find_components()

    def _find_components(self) -> list[Component]:
        vertices: list[tuple[float, float, float]] = []
        vertex_ids: dict[tuple[float, float, float], int] = {}
        parents: list[int] = []
        ranks: list[int] = []
        triangles: list[tuple[int, int, int]] = []

        def add_vertex(vertex: tuple[float, float, float]) -> int:
            key = tuple(round(value, 7) for value in vertex)
            if key not in vertex_ids:
                vertex_ids[key] = len(vertices)
                vertices.append(vertex)
                parents.append(len(parents))
                ranks.append(0)
            return vertex_ids[key]

        def find(vertex_id: int) -> int:
            while parents[vertex_id] != vertex_id:
                parents[vertex_id] = parents[parents[vertex_id]]
                vertex_id = parents[vertex_id]
            return vertex_id

        def union(left: int, right: int) -> None:
            left_root = find(left)
            right_root = find(right)
            if left_root == right_root:
                return
            if ranks[left_root] < ranks[right_root]:
                left_root, right_root = right_root, left_root
            parents[right_root] = left_root
            if ranks[left_root] == ranks[right_root]:
                ranks[left_root] += 1

        for record in self.records:
            values = struct.unpack("<12fH", record)
            triangle = tuple(
                add_vertex(tuple(values[offset: offset + 3]))
                for offset in (3, 6, 9)
            )
            union(triangle[0], triangle[1])
            union(triangle[1], triangle[2])
            triangles.append(triangle)

        groups: dict[int, list[int]] = defaultdict(list)
        for triangle_id, triangle in enumerate(triangles):
            groups[find(triangle[0])].append(triangle_id)

        components = []
        for triangle_ids in groups.values():
            used_vertices = {
                vertex_id
                for triangle_id in triangle_ids
                for vertex_id in triangles[triangle_id]
            }
            points = [vertices[vertex_id] for vertex_id in used_vertices]
            lower = tuple(min(point[axis] for point in points) for axis in range(3))
            upper = tuple(max(point[axis] for point in points) for axis in range(3))
            center = tuple((lower[axis] + upper[axis]) / 2 for axis in range(3))
            components.append(Component(triangle_ids, center))
        return components

    def write(self, path: Path, selected_triangles: set[int]) -> None:
        ordered = sorted(selected_triangles)
        header = f"Generated from {self.path.name}".encode("ascii")[:80].ljust(80, b" ")
        with path.open("wb") as output:
            output.write(header)
            output.write(struct.pack("<I", len(ordered)))
            for triangle_id in ordered:
                output.write(self.records[triangle_id])


def triangle_ids(components: list[Component]) -> set[int]:
    return {
        triangle_id
        for component in components
        for triangle_id in component.triangles
    }


def main() -> None:
    meshes = Path(__file__).resolve().parents[1] / "meshes"

    base = BinaryStl(meshes / "base_link.STL")
    base_cameras = [
        component for component in base.components if len(component.triangles) == 614
    ]
    if len(base_cameras) not in (0, 4):
        raise ValueError(
            f"Expected either 0 or 4 base cameras, found {len(base_cameras)}"
        )
    base_camera_triangles = triangle_ids(base_cameras)
    base.write(
        meshes / "base_link_body.STL",
        set(range(base.triangle_count)) - base_camera_triangles,
    )

    pitch = BinaryStl(meshes / "pitch.STL")
    pitch_cameras = [
        component for component in pitch.components if len(component.triangles) == 614
    ]
    if len(pitch_cameras) != 1:
        raise ValueError(f"Expected 1 pitch camera, found {len(pitch_cameras)}")
    pitch_camera_triangles = triangle_ids(pitch_cameras)
    pitch.write(meshes / "pitch_camera.STL", pitch_camera_triangles)
    pitch.write(
        meshes / "pitch_body.STL",
        set(range(pitch.triangle_count)) - pitch_camera_triangles,
    )

    for wheel_name in ("w1", "w2", "w3", "w4"):
        wheel = BinaryStl(meshes / f"{wheel_name}.STL")
        tire_components = [
            component
            for component in wheel.components
            if math.hypot(component.center[0], component.center[1]) > 0.05
        ]
        if len(tire_components) != 56:
            raise ValueError(
                f"Expected 56 tire components in {wheel_name}, "
                f"found {len(tire_components)}"
            )
        tire_triangles = triangle_ids(tire_components)
        wheel.write(meshes / f"{wheel_name}_tire.STL", tire_triangles)
        wheel.write(
            meshes / f"{wheel_name}_hub.STL",
            set(range(wheel.triangle_count)) - tire_triangles,
        )

    base_camera_status = (
        "removed 4 embedded cameras" if base_cameras else "no embedded cameras"
    )
    print(
        "Generated split visual meshes for the chassis, pitch, and four wheels "
        f"({base_camera_status})."
    )


if __name__ == "__main__":
    main()
