import cv2
import numpy as np
from types import SimpleNamespace
from std_msgs.msg import Header
from rclpy.qos import ReliabilityPolicy

from metro_detection.detection_conversion import (
    DetectionBox,
    class_name,
    display_class_names,
    normalize_class_names,
    to_detection_array,
)
from metro_detection.yolo_detector import (
    CameraStream,
    YoloDetector,
    image_qos_profile,
    to_compressed_image,
)


def test_class_name_supports_list_dict_and_unknown_index():
    assert class_name(["crack", "leak"], 1) == "leak"
    assert class_name({0: "crack", 4: "fastener_missing"}, 4) == (
        "fastener_missing"
    )
    assert class_name(["crack"], 9) == "9"


def test_normalize_class_names_preserves_indices_and_project_taxonomy():
    assert normalize_class_names(
        {0: "裂缝", 3: "扣件断裂", 7: "异物入侵"}
    ) == {0: "crack", 3: "fastener_broken", 7: "foreign_object"}
    assert normalize_class_names(["crack", "渗漏水"]) == [
        "crack",
        "water_leakage",
    ]
    assert normalize_class_names({1: "shenloushui", 7: "yiwuruqin"}) == {
        1: "water_leakage", 7: "foreign_object"
    }


def test_display_class_names_uses_full_ascii_pinyin_and_preserves_indices():
    assert display_class_names(
        {0: "crack", 3: "fastener_broken", 7: "foreign_object"}
    ) == {0: "liefeng", 3: "koujianduanlie", 7: "yiwuruqin"}
    assert display_class_names(
        ["water_leakage", "segment_damage", "unknown"]
    ) == ["shenloushui", "guanpianposundiaokuai", "unknown"]


def test_image_qos_can_match_simulation_and_hardware_publishers():
    assert (
        image_qos_profile("reliable").reliability
        == ReliabilityPolicy.RELIABLE
    )
    assert (
        image_qos_profile("best_effort").reliability
        == ReliabilityPolicy.BEST_EFFORT
    )


def test_annotated_image_is_published_as_jpeg_with_original_header():
    header = Header()
    header.frame_id = "xj1_optical_frame"
    image = np.zeros((24, 32, 3), dtype=np.uint8)

    output = to_compressed_image(image, header, jpeg_quality=75)
    decoded = cv2.imdecode(np.frombuffer(output.data, np.uint8), cv2.IMREAD_COLOR)

    assert output.header == header
    assert output.format == "jpeg"
    assert decoded.shape == image.shape


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


def test_image_callback_keeps_canonical_labels_after_backend_names_change():
    published = []
    stream = CameraStream("xj1", "/image", "/detections", "/annotated")
    stream.detection_pub = SimpleNamespace(publish=published.append)
    node = SimpleNamespace(
        model=SimpleNamespace(names={1: "shenloushui"}),
        class_names={1: "water_leakage"},
        should_process_frame=lambda _: True,
        bridge=SimpleNamespace(imgmsg_to_cv2=lambda *args, **kwargs: None),
        predict=lambda _: None,
        result_boxes=lambda _: [DetectionBox(10, 20, 50, 80, 0.9, 1)],
        get_parameter=lambda _: SimpleNamespace(value=False),
        ready=True,
    )

    YoloDetector.on_image(node, stream, SimpleNamespace(header=Header()))

    assert published[0].detections[0].results[0].hypothesis.class_id == "water_leakage"
