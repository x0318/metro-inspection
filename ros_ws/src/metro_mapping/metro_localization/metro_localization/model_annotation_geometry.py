"""Project explicit tunnel material regions, with static-mesh occlusion checks."""

from dataclasses import dataclass
from pathlib import Path

import collada
import cv2
import numpy as np
import trimesh


@dataclass
class ModelRegion:
    site_id: str
    class_name: str
    points: np.ndarray
    reference: np.ndarray


def sample_surface(triangles, count=384):
    """Deterministic area sampling avoids weighting tessellated details too much."""
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                    triangles[:, 2] - triangles[:, 0]), axis=1)
    if not np.any(areas > 0):
        raise ValueError("Model region has no non-degenerate surface")
    rng = np.random.default_rng(42)
    selected = triangles[rng.choice(len(triangles), count, p=areas / areas.sum())]
    uv = rng.random((count, 2))
    uv[uv.sum(axis=1) > 1] = 1 - uv[uv.sum(axis=1) > 1]
    return selected[:, 0] + uv[:, :1] * (selected[:, 1] - selected[:, 0]) + uv[:, 1:] * (selected[:, 2] - selected[:, 0])


def load_tunnel_regions(path: Path):
    if not trimesh.ray.has_embree:
        raise RuntimeError("Model annotation requires trimesh>=4.5 and embreex; see docs/model_annotation_demo.md")
    scene = collada.Collada(str(path))
    if scene.assetInfo.upaxis != "Z_UP" or scene.assetInfo.unitmeter != 1.0:
        raise ValueError("Model annotation requires a metre, Z-up tunnel mesh")
    vertices, faces, selected = [], [], {}
    offset = 0
    # The same dark material is used for different defects. Object 6 is the
    # crack confirmed in the starting XJ2 view; material alone is not a class.
    surface_regions = {
        "6": ("mat_0", "crack_01", "crack"),
        "7": ("mat_0", "leak_02", "water_leakage"),
        "8": ("mat_0", "leak_03", "water_leakage"),
        "9": ("mat_0", "leak_04", "water_leakage"),
        "5": ("mat_9", "segment_damage_01", "segment_damage"),
    }
    # One loosened fastening assembly includes the displaced clip and its
    # detached hardware, as confirmed in the stationary XJ3 reference view.
    object_regions = {
        "板手": ("foreign_01", "foreign_object"),
        "2": ("fastener_broken_01", "fastener_broken"),
        "W型弹条.008": ("fastener_loose_01", "fastener_loose"),
        "平垫圈 GB_T 97.1 42.008": ("fastener_loose_01", "fastener_loose"),
        "螺栓 GB_T 5782 M42 x 160.008": ("fastener_loose_01", "fastener_loose"),
        # Lower pipe clamp with its loosened nut/washer at x~10.8 m.
        "细圆管连接件.004": ("bracket_loose_01", "bracket_loose"),
        "六角螺栓 GB_T 5780 M36 x 140.027": ("bracket_loose_01", "bracket_loose"),
        "平垫圈 GB_T 97.1 36.069": ("bracket_loose_01", "bracket_loose"),
        "螺母 GB_T 6170 M36.029": ("bracket_loose_01", "bracket_loose"),
    }
    for geometry in scene.scene.objects("geometry"):
        name = geometry.original.name
        for primitive in geometry.primitives():
            primitive = primitive.triangleset() if hasattr(primitive, "triangleset") else primitive
            xyz = np.asarray(primitive.vertex, dtype=np.float64)
            indices = np.asarray(primitive.vertex_index, dtype=np.int64)
            vertices.append(xyz)
            faces.append(indices + offset)
            offset += len(xyz)
            key = None
            if name in surface_regions:
                material_prefix, site_id, class_name = surface_regions[name]
                if primitive.material.id.startswith(material_prefix):
                    key = (site_id, class_name)
            elif name in object_regions:
                key = object_regions[name]
            elif name == "铁垫板.035":
                # The outer fastening assembly at x=21.31, y~0.854 is absent.
                # Use its exposed seat, not the intact assembly across the rail.
                seat = xyz[indices]
                seat = seat[np.all(seat[:, :, 1] >= 0.79, axis=1)]
                if len(seat):
                    selected.setdefault(("fastener_missing_01", "fastener_missing"), []).append(seat)
            if key:
                selected.setdefault(key, []).append(xyz[indices])
    mesh = trimesh.Trimesh(vertices=np.concatenate(vertices), faces=np.concatenate(faces), process=False)
    regions = []
    for (site_id, class_name), parts in sorted(selected.items()):
        points = sample_surface(np.concatenate(parts))
        # Select an actual surface sample, never the centre of the tunnel arch.
        reference = points[np.argmin(np.linalg.norm(points - np.median(points, axis=0), axis=1))]
        regions.append(ModelRegion(site_id, class_name, points, reference))
    if len(regions) != 10:
        raise ValueError(
            "Expected one crack, three leakage regions, one segment damage, "
            f"one loose fastener, one missing fastener, one broken fastener, one loose pipe bracket "
            f"and one wrench, found {len(regions)} regions")
    return mesh, regions


def project_region(region, world_from_camera, k, distortion, width, height, mesh,
                   max_distance=12.0, min_visible_fraction=0.10):
    # Samples are uniform by surface area. A few surviving edge/occlusion
    # samples do not provide enough visible surface to confirm the region.
    # Count against the whole region, before clipping to this camera's view.
    required_visible = max(4, int(np.ceil(len(region.points) * min_visible_fraction)))
    rotation = world_from_camera[:3, :3]
    origin = world_from_camera[:3, 3]
    local = (region.points - origin) @ rotation
    distances = np.linalg.norm(local, axis=1)
    mask = (local[:, 2] > 0.05) & (distances < max_distance)
    if mask.sum() < required_visible:
        return None
    points, local, distances = region.points[mask], local[mask], distances[mask]
    uv, _ = cv2.projectPoints(local, np.zeros(3), np.zeros(3), k, distortion)
    uv = uv.reshape(-1, 2)
    inside = ((uv[:, 0] >= 0) & (uv[:, 0] < width) &
              (uv[:, 1] >= 0) & (uv[:, 1] < height))
    points, uv, distances = points[inside], uv[inside], distances[inside]
    if len(points) < required_visible:
        return None
    directions = (points - origin) / distances[:, None]
    locations, rays, _ = mesh.ray.intersects_location(
        ray_origins=np.broadcast_to(origin, directions.shape),
        ray_directions=directions, multiple_hits=False,
    )
    visible = np.zeros(len(points), dtype=bool)
    if len(rays):
        hit_distances = np.linalg.norm(locations - origin, axis=1)
        visible[rays] = np.abs(hit_distances - distances[rays]) <= 0.025
    if visible.sum() < required_visible:
        return None
    pixels = uv[visible]
    lower = np.maximum(0, pixels.min(axis=0) - 4)
    upper = np.minimum([width - 1, height - 1], pixels.max(axis=0) + 4)
    if np.any(upper - lower < 8):
        return None
    return (*lower, *upper)
