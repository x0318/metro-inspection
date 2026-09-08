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

"""Split each V6 wheel assembly into rotating wheel and fixed mount meshes."""

import argparse
import struct
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


ROTATING_TRIANGLE_COUNT = 520


@dataclass
class Component:
    triangles: list[int]
    center: tuple[float, float, float]
    size: tuple[float, float, float]


class BinaryStl:
    def __init__(self, path: Path):
        self.path = path
        data = path.read_bytes()
        if len(data) < 84:
            raise ValueError(f"STL is too short: {path}")
        self.triangle_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + self.triangle_count * 50
        if len(data) != expected_size:
            raise ValueError(
                f"Expected binary STL of {expected_size} bytes, "
                f"found {len(data)}: {path}"
            )
        self.records = [
            data[84 + index * 50:84 + (index + 1) * 50]
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
                add_vertex(tuple(values[offset:offset + 3]))
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
            size = tuple(upper[axis] - lower[axis] for axis in range(3))
            components.append(Component(triangle_ids, center, size))
        return components

    def write(self, path: Path, selected_triangles: set[int]) -> None:
        ordered = sorted(selected_triangles)
        header = f"Generated from {self.path.name}".encode("ascii")[:80]
        with path.open("wb") as output:
            output.write(header.ljust(80, b" "))
            output.write(struct.pack("<I", len(ordered)))
            for triangle_id in ordered:
                output.write(self.records[triangle_id])


def split_flanged_wheels(mesh_dir: Path) -> None:
    for wheel_name in ("w1", "w2", "w3", "w4"):
        source = BinaryStl(mesh_dir / f"{wheel_name}.STL")
        candidates = [
            component
            for component in source.components
            if len(component.triangles) == ROTATING_TRIANGLE_COUNT
        ]
        if len(candidates) != 1:
            raise ValueError(
                f"Expected one {ROTATING_TRIANGLE_COUNT}-triangle flanged wheel "
                f"in {source.path}, found {len(candidates)}"
            )
        rotating = candidates[0]
        if not (
            0.14 < rotating.size[0] < 0.17
            and 0.14 < rotating.size[1] < 0.17
            and 0.03 < rotating.size[2] < 0.05
        ):
            raise ValueError(
                f"Unexpected rotating-wheel bounds in {source.path}: {rotating.size}"
            )
        rotating_triangles = set(rotating.triangles)
        source.write(mesh_dir / f"{wheel_name}_wheel.STL", rotating_triangles)
        source.write(
            mesh_dir / f"{wheel_name}_mount.STL",
            set(range(source.triangle_count)) - rotating_triangles,
        )
        print(
            f"Split {source.path.name}: {len(rotating_triangles)} rotating and "
            f"{source.triangle_count - len(rotating_triangles)} fixed triangles"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_dir", type=Path)
    args = parser.parse_args()
    split_flanged_wheels(args.mesh_dir)


if __name__ == "__main__":
    main()
