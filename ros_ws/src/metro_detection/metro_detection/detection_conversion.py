"""Convert backend-agnostic YOLO boxes to ROS 2 vision messages."""

from dataclasses import dataclass
from typing import Mapping, Sequence, Union

from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


@dataclass(frozen=True)
class DetectionBox:
    """One axis-aligned detector result in source-image pixel coordinates."""

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_index: int


ClassNames = Union[Sequence[str], Mapping[int, str]]

CLASS_NAME_ALIASES = {
    "裂缝": "crack",
    "渗漏水": "water_leakage",
    "管片破损掉块": "segment_damage",
    "异物入侵": "foreign_object",
    "扣件缺失": "fastener_missing",
    "扣件断裂": "fastener_broken",
    "扣件松动歪斜": "fastener_loose",
    "管线支架松脱": "bracket_loose",
}

DISPLAY_CLASS_NAMES = {
    "crack": "liefeng",
    "water_leakage": "shenloushui",
    "segment_damage": "guanpianposundiaokuai",
    "fastener_broken": "koujianduanlie",
    "fastener_missing": "koujianqueshi",
    "fastener_loose": "koujiansongdongwaixie",
    "bracket_loose": "guanxianzhijiasongtuo",
    "foreign_object": "yiwuruqin",
}
CLASS_NAME_ALIASES.update({label: name for name, label in DISPLAY_CLASS_NAMES.items()})


def normalize_class_names(class_names: ClassNames) -> ClassNames:
    """Map checkpoint-specific labels onto the stable project taxonomy."""
    if isinstance(class_names, Mapping):
        return {
            index: CLASS_NAME_ALIASES.get(str(name), str(name))
            for index, name in class_names.items()
        }
    return [CLASS_NAME_ALIASES.get(str(name), str(name)) for name in class_names]


def display_class_names(class_names: ClassNames) -> ClassNames:
    """Map stable taxonomy names to ASCII pinyin for annotated images."""
    if isinstance(class_names, Mapping):
        return {
            index: DISPLAY_CLASS_NAMES.get(str(name), str(name))
            for index, name in class_names.items()
        }
    return [
        DISPLAY_CLASS_NAMES.get(str(name), str(name))
        for name in class_names
    ]


def class_name(class_names: ClassNames, index: int) -> str:
    """Return a stable class label for list- or dict-based model metadata."""
    if isinstance(class_names, Mapping):
        return str(class_names.get(index, index))
    if 0 <= index < len(class_names):
        return str(class_names[index])
    return str(index)


def to_detection_array(
    header: Header,
    boxes: Sequence[DetectionBox],
    class_names: ClassNames,
) -> Detection2DArray:
    """Create a standard Detection2DArray while preserving the image header."""
    output = Detection2DArray()
    output.header = header

    for box in boxes:
        width = max(0.0, float(box.x2) - float(box.x1))
        height = max(0.0, float(box.y2) - float(box.y1))
        if width <= 0.0 or height <= 0.0:
            continue

        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = (
            float(box.x1) + float(box.x2)
        ) / 2.0
        detection.bbox.center.position.y = (
            float(box.y1) + float(box.y2)
        ) / 2.0
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = width
        detection.bbox.size_y = height

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_name(
            class_names, int(box.class_index)
        )
        hypothesis.hypothesis.score = float(box.confidence)
        detection.results.append(hypothesis)
        output.detections.append(detection)

    return output
