from std_msgs.msg import Header

from metro_detection.detection_conversion import (
    DetectionBox,
    class_name,
    to_detection_array,
)


def test_class_name_supports_list_dict_and_unknown_index():
    assert class_name(["crack", "leak"], 1) == "leak"
    assert class_name({0: "crack", 4: "fastener_missing"}, 4) == (
        "fastener_missing"
    )
    assert class_name(["crack"], 9) == "9"


def test_detection_array_preserves_header_and_box_geometry():
    header = Header()
    header.frame_id = "odin1_rgb_optical_frame"
    header.stamp.sec = 12
    header.stamp.nanosec = 345

    output = to_detection_array(
        header,
        [DetectionBox(10.0, 20.0, 50.0, 80.0, 0.875, 1)],
        {0: "crack", 1: "water_leakage"},
    )

    assert output.header == header
    assert len(output.detections) == 1
    detection = output.detections[0]
    assert detection.header == header
    assert detection.bbox.center.position.x == 30.0
    assert detection.bbox.center.position.y == 50.0
    assert detection.bbox.size_x == 40.0
    assert detection.bbox.size_y == 60.0
    assert detection.results[0].hypothesis.class_id == "water_leakage"
    assert detection.results[0].hypothesis.score == 0.875


def test_invalid_zero_width_box_is_dropped():
    output = to_detection_array(
        Header(),
        [DetectionBox(10.0, 20.0, 10.0, 80.0, 0.9, 0)],
        ["crack"],
    )

    assert output.detections == []
