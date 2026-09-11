"""Exercise the real localization callback with synthetic ROS sensor messages."""

from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformException
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

from metro_localization import damage_localizer as module


@pytest.fixture
def make_node(monkeypatch):
    rclpy.init(args=["--ros-args", "-p", "publish_events:=true"])
    nodes = []

    def create():
        node = module.DamageLocalizer()
        nodes.append(node)
        for name in ("camera_point_pub", "global_point_pub", "event_pub",
                     "debug_pub", "marker_pub"):
            publisher = Mock()
            publisher.messages = []
            publisher.publish.side_effect = lambda msg, p=publisher: p.messages.append(
                deepcopy(msg)
            )
            monkeypatch.setattr(node, name, publisher)
        info = CameraInfo()
        info.header.frame_id = "camera_optical"
        info.k = [100., 0., 100., 0., 100., 80., 0., 0., 1.]
        node.on_camera_info(info)

        def lookup(target, source, stamp):
            transform = TransformStamped()
            transform.transform.rotation.w = 1.0
            if target == "odom":
                transform.transform.translation.x = 10.0
            return transform

        monkeypatch.setattr(node, "lookup_transform", Mock(side_effect=lookup))
        return node

    yield create
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def detection(center_x, score=0.9):
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = "crack"
    result.hypothesis.score = score
    msg = Detection2D()
    msg.results = [result]
    msg.bbox.center.position.x = float(center_x)
    msg.bbox.center.position.y = 80.0
    msg.bbox.size_x = 12.0
    msg.bbox.size_y = 12.0
    return msg


def frame(sec, targets=None):
    header = Header()
    header.stamp.sec = sec
    header.frame_id = "camera_optical"
    image = CvBridge().cv2_to_imgmsg(np.zeros((160, 200, 3), np.uint8), "bgr8")
    image.header = header
    points = [(x + dx, dy, 2.) for x in (-0.2, 0.2)
              for dx, dy in ((-.01, -.01), (-.01, .01), (.01, -.01), (.01, .01))]
    cloud = create_cloud_xyz32(header, points)
    detections = Detection2DArray()
    detections.header = header
    detections.detections = targets if targets is not None else [
        detection(90, 0.99), detection(150, 0.95), detection(110, 0.6)
    ]
    return detections, cloud, image


def test_multi_target_local_failure_duplicate_frames_and_debug(make_node, monkeypatch):
    node = make_node()
    conversion = Mock(wraps=module.pointcloud2_to_xyz)
    projection = Mock(wraps=module.project_camera_points)
    monkeypatch.setattr(module, "pointcloud2_to_xyz", conversion)
    monkeypatch.setattr(module, "project_camera_points", projection)
    debug = Mock(wraps=node.publish_debug_image)
    monkeypatch.setattr(node, "publish_debug_image", debug)

    for sec in (1, 2, 3):
        node.on_synced_data(*frame(sec))
        node.on_synced_data(*frame(sec))
    node.on_synced_data(*frame(1))
    assert conversion.call_count == projection.call_count == 3
    assert node.lookup_transform.call_count == 6
    assert debug.call_count == 3
    assert node.roi_failure_count == 3
    assert len(node.camera_point_pub.messages) == len(node.global_point_pub.messages) == 6
    events = node.event_pub.messages
    assert len(events) == 2
    assert len({event.event_id for event in events}) == 2
    assert [event.confidence for event in events] == pytest.approx([.99, .6])
    assert [event.bbox.center.position.x for event in events] == [90., 110.]
    np.testing.assert_allclose(
        [(e.position.point.x, e.position.point.z) for e in events],
        [(9.8, 2.), (10.2, 2.)], atol=1e-6,
    )
    assert all(e.has_3d_position and e.has_semantic_location for e in events)
    assert all(e.header.stamp.sec == 3 for e in events)
    assert all(e.position.header.frame_id == "odom" for e in events)
    assert all(track.hit_count == 3 for track in node.event_tracker.tracks)
    targets = debug.call_args.kwargs["targets"]
    assert [t.global_point is not None for t in targets] == [True, False, True]
    pixels = node.bridge.imgmsg_to_cv2(node.debug_pub.messages[-1], "bgr8")
    assert np.any(pixels[74:87, 84:97]) and np.any(pixels[74:87, 104:117])
    markers = [m for m in node.marker_pub.messages if m.action == Marker.ADD]
    assert {m.id for m in markers[-4:]} == {0, 1, 4, 5}
    assert markers[-4].color != markers[-2].color
    assert all(m.pose.position.z == pytest.approx(2.) for m in markers if m.type == Marker.SPHERE)

    # Changed order must preserve the ID assigned to each spatial target.
    node.on_synced_data(*frame(5, [detection(110), detection(90)]))
    assert [e.event_id for e in node.event_pub.messages[-2:]] == [
        events[1].event_id, events[0].event_id
    ]
    node.on_synced_data(*frame(6, []))
    assert node.marker_ids == set()
    assert node.marker_pub.messages[-1].action == Marker.DELETE


@pytest.mark.parametrize("failure", ["estimator", "invalid_bbox", "global_point"])
def test_target_exception_does_not_interrupt_other_targets(make_node, monkeypatch, failure):
    node = make_node()
    targets = [detection(90), detection(110)]
    if failure == "invalid_bbox":
        targets[0].bbox.center.position.x = float("nan")
    elif failure == "estimator":
        original = node.estimate_damage_point

        def estimate(points):
            if points[0][0] < 0:
                raise ValueError("injected estimator failure")
            return original(points)

        monkeypatch.setattr(node, "estimate_damage_point", estimate)
    else:
        original = module.transform_point

        def transform(point, tf):
            if point[0] < 0:
                raise ValueError("injected transform failure")
            return original(point, tf)

        monkeypatch.setattr(module, "transform_point", transform)
    for sec in (1, 2, 3):
        node.on_synced_data(*frame(sec, targets))
    assert len(node.event_pub.messages) == 1
    assert node.event_pub.messages[0].bbox.center.position.x == 110.
    assert node.roi_failure_count == 3
    assert len(node.debug_pub.messages) == 3


def test_mask_is_converted_once_and_restricted_to_each_bbox(make_node, monkeypatch):
    node = make_node()
    inputs = frame(1, [detection(90), detection(110)])
    mask = node.bridge.cv2_to_imgmsg(np.full((160, 200), 255, np.uint8), "mono8")
    mask.header = inputs[2].header
    node.on_mask(mask)
    conversion = Mock(wraps=node.mask_msg_to_array)
    monkeypatch.setattr(node, "mask_msg_to_array", conversion)
    node.on_synced_data(*inputs)
    assert conversion.call_count == 1
    assert node.mask_used_count == 2
    assert [track.position[0] for track in node.event_tracker.tracks] == pytest.approx([9.8, 10.2])


@pytest.mark.parametrize("fallback", [True, False])
def test_bad_mask_respects_bbox_fallback(make_node, monkeypatch, fallback):
    node = make_node()
    node.set_parameters([Parameter("mask_fallback_to_bbox", value=fallback)])
    inputs = frame(1, [detection(90), detection(110)])
    node.on_mask(inputs[2])
    conversion = Mock(side_effect=ValueError("bad mask"))
    monkeypatch.setattr(node, "mask_msg_to_array", conversion)
    node.on_synced_data(*inputs)
    assert conversion.call_count == 1
    assert node.global_point_count == (2 if fallback else 0)
    assert len(node.debug_pub.messages) == 1


def test_missing_global_tf_preserves_all_camera_points(make_node, monkeypatch):
    node = make_node()
    original = node.lookup_transform

    def lookup(target, source, stamp):
        if target == "odom":
            raise TransformException("no odom")
        return original(target, source, stamp)

    monkeypatch.setattr(node, "lookup_transform", lookup)
    node.on_synced_data(*frame(1))
    assert node.camera_point_count == 2
    assert node.global_point_count == node.event_count == 0
    assert node.global_tf_failure_count == 1
    assert len(node.debug_pub.messages) == 1


def test_node_recreation_uses_new_identity_even_at_same_coordinates(make_node):
    first = make_node()
    for sec in (1, 2, 3):
        first.on_synced_data(*frame(sec))
    old_ids = {event.event_id for event in first.event_pub.messages}
    restarted = make_node()
    for sec in (1, 2, 3):
        restarted.on_synced_data(*frame(sec))
    new_ids = {event.event_id for event in restarted.event_pub.messages}
    assert len(old_ids) == len(new_ids) == 2
    assert old_ids.isdisjoint(new_ids)
