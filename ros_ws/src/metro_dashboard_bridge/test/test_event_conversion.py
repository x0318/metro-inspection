import pytest
from metro_inspection_interfaces.msg import DefectEvent

from metro_dashboard_bridge.event_conversion import defect_event_to_record


def make_event() -> DefectEvent:
    message = DefectEvent()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.header.frame_id = "xj1_optical_frame"
    message.event_id = "defect-001"
    message.detection_id = "frame-12-box-0"
    message.camera_name = "xj1"
    message.class_name = "crack"
    message.confidence = 0.93
    message.severity = DefectEvent.SEVERITY_MODERATE
    message.bbox.center.position.x = 320.0
    message.bbox.center.position.y = 180.0
    message.bbox.size_x = 120.0
    message.bbox.size_y = 80.0
    message.image_width = 640
    message.image_height = 360
    message.has_3d_position = True
    message.position.header.stamp = message.header.stamp
    message.position.header.frame_id = "odom"
    message.position.point.x = 10.2
    message.position.point.y = -1.1
    message.position.point.z = 2.4
    message.localization_method = DefectEvent.LOCALIZATION_CURRENT_CLOUD
    message.localization_confidence = 0.82
    message.model_name = "simulation-yolo-v1"
    message.has_semantic_location = True
    message.chainage_m = 12010.2
    message.chainage = "K12+010.200"
    message.segment_name = "ring"
    message.segment_id = 1008
    message.segment_offset_m = 0.6
    message.clock_position_hours = 2.5
    message.structure_area = "右侧边墙"
    return message


def test_converts_complete_event_to_dashboard_record() -> None:
    record = defect_event_to_record(make_event())

    assert record["event_id"] == "defect-001"
    assert record["type"] == "crack"
    assert record["level"] == "II级 (中度)"
    assert record["bbox"]["x"] == 260.0
    assert record["position"] == {"x": 10.2, "y": -1.1, "z": 2.4}
    assert record["localization"]["method_name"] == "current_cloud"
    assert record["mileage"] == "K12+010.200"
    assert record["ring_number"] == 1008


def test_allows_unlocalized_event_with_unknown_severity() -> None:
    message = make_event()
    message.severity = DefectEvent.SEVERITY_UNKNOWN
    message.has_3d_position = False
    message.localization_method = DefectEvent.LOCALIZATION_NONE
    message.has_semantic_location = False

    record = defect_event_to_record(message)

    assert record["level"] == "未评估"
    assert record["position"] is None
    assert record["semantic_location"] is None


def test_rejects_event_without_stable_id() -> None:
    message = make_event()
    message.event_id = ""

    with pytest.raises(ValueError, match="event_id"):
        defect_event_to_record(message)


def test_rejects_bbox_outside_source_image() -> None:
    message = make_event()
    message.bbox.center.position.x = 10.0

    with pytest.raises(ValueError, match="bbox"):
        defect_event_to_record(message)
