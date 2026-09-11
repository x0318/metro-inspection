from types import SimpleNamespace

import numpy as np
import pytest
import trimesh
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from scipy.spatial.transform import Rotation

from metro_localization import model_annotation_geometry
from metro_localization.model_annotation_geometry import ModelRegion, project_region, sample_surface
from metro_localization.model_annotation_node import ModelAnnotationNode, transform_matrix
from metro_localization.semantic_geometry import TunnelSemanticProjector


def plane(z):
    return trimesh.Trimesh(vertices=[[-1, -1, z], [1, -1, z], [1, 1, z], [-1, 1, z]],
                           faces=[[0, 1, 2], [0, 2, 3]], process=False)


def test_marker_world_frame_aligns_with_odom_without_assuming_equal_origins():
    world_base, odom_base = TransformStamped(), TransformStamped()
    world_base.transform.translation.x = 8.0
    world_base.transform.translation.z = .52
    odom_base.transform.translation.x = 2.0
    odom_base.transform.translation.y = 1.0
    for message, yaw in [(world_base, .2), (odom_base, .5)]:
        q = Rotation.from_euler("z", yaw).as_quat()
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = map(float, q)
    sent = []
    node = SimpleNamespace(last_marker_frame_stamp=-1,
                           tf=SimpleNamespace(lookup_transform=lambda *args: odom_base),
                           marker_frame_broadcaster=SimpleNamespace(sendTransform=sent.append))
    ModelAnnotationNode.update_marker_frame(node, Time(seconds=12), world_base)
    ModelAnnotationNode.update_marker_frame(node, Time(seconds=11), world_base)
    assert len(sent) == 1
    assert sent[0].header.frame_id == "odom"
    assert sent[0].child_frame_id == "model_annotation_world"
    # A point relative to the robot must land at the same odom coordinate by
    # either path: directly through odom/base, or via world and the new frame.
    robot_point = np.array([3., 1., 2., 1.])
    assert (transform_matrix(sent[0].transform) @ transform_matrix(world_base.transform)
            @ robot_point) == pytest.approx(transform_matrix(odom_base.transform) @ robot_point)


def test_shared_dark_material_does_not_turn_starting_crack_into_leakage(monkeypatch):
    surface = plane(4)

    def geometry(name):
        if name.startswith("铁垫板"):
            outer, inner = plane(.4), plane(.4)
            outer.vertices = outer.vertices * [.06, .05, 1] + [21.31, .88, 0]
            inner.vertices = inner.vertices * [.06, .05, 1] + [21.31, .64, 0]
            seat = trimesh.util.concatenate([outer, inner])
            primitive = SimpleNamespace(vertex=seat.vertices, vertex_index=seat.faces,
                                        material=SimpleNamespace(id="mat_7-material"))
            return SimpleNamespace(original=SimpleNamespace(name=name), primitives=lambda: iter([primitive]))
        if name in ("6", "7", "8", "9"):
            materials = ["mat_0-material", "mat_8-material"]
        elif name == "5":
            materials = ["mat_9_003-material", "mat_8_003-material"]
        else:
            materials = ["mat_7-material"]
        primitives = [SimpleNamespace(vertex=surface.vertices, vertex_index=surface.faces,
                                      material=SimpleNamespace(id=material)) for material in materials]
        return SimpleNamespace(original=SimpleNamespace(name=name), primitives=lambda: iter(primitives))

    geometries = [geometry(name) for name in (
        "6", "7", "8", "9", "5", "板手", "W型弹条.008",
        "平垫圈 GB_T 97.1 42.008", "螺栓 GB_T 5782 M42 x 160.008",
        "W型弹条.007",  # The neighbouring normal assembly must not become a defect.
        "铁垫板.035", "铁垫板.036",
        "细圆管连接件.004", "六角螺栓 GB_T 5780 M36 x 140.027",
        "平垫圈 GB_T 97.1 36.069", "螺母 GB_T 6170 M36.029",
        "细圆管连接件.003", "螺母 GB_T 6170 M36.023",  # Normal neighbouring bracket.
        "2", "W型弹条.028",  # Broken inner clip and the intact outer clip.
    )]
    scene = SimpleNamespace(assetInfo=SimpleNamespace(upaxis="Z_UP", unitmeter=1.0),
                            scene=SimpleNamespace(objects=lambda _: iter(geometries)))
    monkeypatch.setattr(model_annotation_geometry.collada, "Collada", lambda _: scene)
    _, regions = model_annotation_geometry.load_tunnel_regions("unused.dae")
    assert {region.site_id: region.class_name for region in regions} == {
        "crack_01": "crack", "leak_02": "water_leakage",
        "leak_03": "water_leakage", "leak_04": "water_leakage",
        "foreign_01": "foreign_object",
        "segment_damage_01": "segment_damage", "fastener_loose_01": "fastener_loose",
        "fastener_missing_01": "fastener_missing",
        "bracket_loose_01": "bracket_loose",
        "fastener_broken_01": "fastener_broken",
    }
    missing = next(r for r in regions if r.site_id == "fastener_missing_01")
    assert np.all(missing.points[:, 1] >= .79)
    assert missing.reference[1] >= .79  # Never use the intact inner seat.


def test_projection_uses_visible_surface_and_rejects_occlusion_and_behind_camera():
    surface = plane(4)
    region = ModelRegion("leak_01", "water_leakage", sample_surface(surface.triangles), np.array([0, 0, 4]))
    k = np.array([[100., 0, 100], [0, 100, 100], [0, 0, 1]])
    args = (region, np.eye(4), k, np.zeros(5), 200, 200)
    box = project_region(*args, surface)
    assert box is not None
    assert box == pytest.approx([71, 71, 129, 129], abs=2)
    occluded = trimesh.util.concatenate([surface, plane(2)])
    assert project_region(*args, occluded) is None
    pose = np.eye(4)
    pose[:3, 3] = [0, 0, 6]
    assert project_region(region, pose, k, np.zeros(5), 200, 200, surface) is None


def test_outside_view_and_too_distant_region_produces_no_box():
    surface = plane(4)
    region = ModelRegion("leak_01", "water_leakage", sample_surface(surface.triangles), np.array([0, 0, 4]))
    k = np.array([[100., 0, 100], [0, 100, 100], [0, 0, 1]])
    assert project_region(region, np.eye(4), k, np.zeros(5), 200, 200, surface, 2) is None
    pose = np.eye(4)
    pose[0, 3] = 10
    assert project_region(region, pose, k, np.zeros(5), 200, 200, surface, 20) is None


def test_edge_sliver_is_not_confirmed_but_half_visible_region_is():
    surface = plane(4)
    region = ModelRegion("leak_02", "water_leakage", sample_surface(surface.triangles),
                         np.array([0, 0, 4]))
    k = np.array([[1028., 0, 720], [0, 1028., 540], [0, 0, 1]])
    pose = np.eye(4)
    # Only a ten-pixel strip reaches the left edge of the image. The old
    # four-point check produced a box even though almost all surface was lost.
    pose[0, 3] = 1 + (720 - 10) * 4 / 1028
    args = (region, pose, k, np.zeros(5), 1440, 1080, surface)
    assert project_region(*args, min_visible_fraction=0) is not None
    assert project_region(*args) is None
    pose[0, 3] = 720 * 4 / 1028
    assert project_region(*args) is not None


def test_visible_leak_at_eleven_metres_is_within_demo_range():
    surface = plane(11)
    region = ModelRegion("leak_04", "water_leakage", sample_surface(surface.triangles),
                         np.array([0, 0, 11]))
    k = np.array([[1383., 0, 720], [0, 1383., 540], [0, 0, 1]])
    args = (region, np.eye(4), k, np.zeros(5), 1440, 1080, surface)
    assert project_region(*args, max_distance=8) is None
    assert project_region(*args) is not None
    assert project_region(*args[:-1], trimesh.util.concatenate([surface, plane(5)])) is None


def test_occlusion_sliver_is_not_confirmed():
    surface = plane(4)
    region = ModelRegion("leak_02", "water_leakage", sample_surface(surface.triangles),
                         np.array([0, 0, 4]))
    # At half the distance this cover hides all but a narrow strip at x > .94.
    cover = trimesh.Trimesh(vertices=[[-1, -1, 2], [.47, -1, 2],
                                     [.47, 1, 2], [-1, 1, 2]],
                            faces=[[0, 1, 2], [0, 2, 3]], process=False)
    mesh = trimesh.util.concatenate([surface, cover])
    k = np.array([[1028., 0, 720], [0, 1028., 540], [0, 0, 1]])
    args = (region, np.eye(4), k, np.zeros(5), 1440, 1080, mesh)
    assert project_region(*args, min_visible_fraction=0) is not None
    assert project_region(*args) is None


@pytest.mark.parametrize("site_id,class_name", [
    ("crack_01", "crack"), ("leak_02", "water_leakage"), ("foreign_01", "foreign_object"),
    ("segment_damage_01", "segment_damage"), ("fastener_loose_01", "fastener_loose"),
    ("fastener_missing_01", "fastener_missing"),
    ("bracket_loose_01", "bracket_loose"),
    ("fastener_broken_01", "fastener_broken"),
])
def test_repeated_views_share_site_id_and_preserve_explicit_model_provenance(site_id, class_name):
    events = []
    node = SimpleNamespace(events=SimpleNamespace(publish=events.append), semantic=TunnelSemanticProjector(
        chainage_start_m=12000, chainage_axis="x", chainage_sign=1,
        segment_start_id=1000, segment_length_m=1.2, segment_name="simulation",
        clock_center_y_m=0, clock_center_z_m=1.75))
    region = ModelRegion(site_id, class_name, np.zeros((4, 3)), np.array([2., -2., 3.]))
    for camera, sec in [("odin1", 1), ("xj1", 2), ("odin1", 3)]:
        header = Header()
        header.stamp.sec = sec
        header.frame_id = f"{camera}_optical_frame"
        ModelAnnotationNode.publish_event(node, region, camera, header, (10, 20, 40, 60), 100, 100)
    assert len({event.event_id for event in events}) == 1
    assert len({event.detection_id for event in events}) == 3
    for event in events:
        assert event.class_name == class_name
        assert event.event_id == f"model-demo:{site_id}"
        assert event.localization_method == event.LOCALIZATION_MODEL_REFERENCE
        assert event.position.header.frame_id == "world"
        assert event.confidence == 0.0
        assert event.position.point.x == 2.0
