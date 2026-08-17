import math
from typing import Dict

from metro_inspection_interfaces.msg import DefectEvent


SEVERITY_LABELS = {
    DefectEvent.SEVERITY_UNKNOWN: "未评估",
    DefectEvent.SEVERITY_MINOR: "I级 (轻微)",
    DefectEvent.SEVERITY_MODERATE: "II级 (中度)",
    DefectEvent.SEVERITY_SEVERE: "III级 (严重)",
}

LOCALIZATION_LABELS = {
    DefectEvent.LOCALIZATION_NONE: "none",
    DefectEvent.LOCALIZATION_CURRENT_CLOUD: "current_cloud",
    DefectEvent.LOCALIZATION_ACCUMULATED_MAP: "accumulated_map",
    DefectEvent.LOCALIZATION_TUNNEL_MODEL: "tunnel_model",
}


def _finite(value: float, field_name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")
    return value


def _confidence(value: float, field_name: str) -> float:
    value = _finite(value, field_name)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{field_name} must be between 0 and 1")
    return value


def _stamp(stamp) -> Dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _bbox(message: DefectEvent) -> Dict[str, object]:
    image_width = int(message.image_width)
    image_height = int(message.image_height)
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")

    center_x = _finite(message.bbox.center.position.x, "bbox center x")
    center_y = _finite(message.bbox.center.position.y, "bbox center y")
    width = _finite(message.bbox.size_x, "bbox width")
    height = _finite(message.bbox.size_y, "bbox height")
    theta = _finite(message.bbox.center.theta, "bbox theta")
    if width <= 0.0 or height <= 0.0:
        raise ValueError("bbox size must be positive")

    x = center_x - width / 2.0
    y = center_y - height / 2.0
    tolerance = 1e-6
    if (
        x < -tolerance
        or y < -tolerance
        or x + width > image_width + tolerance
        or y + height > image_height + tolerance
    ):
        raise ValueError("bbox must remain inside the source image")

    return {
        "x": max(0.0, x),
        "y": max(0.0, y),
        "width": width,
        "height": height,
        "center_x": center_x,
        "center_y": center_y,
        "theta": theta,
        "image_width": image_width,
        "image_height": image_height,
    }


def defect_event_to_record(message: DefectEvent) -> Dict[str, object]:
    """Validate and convert one DefectEvent to the dashboard JSON contract."""

    event_id = message.event_id.strip()
    if not event_id:
        raise ValueError("event_id must not be empty")
    class_name = message.class_name.strip()
    if not class_name:
        raise ValueError("class_name must not be empty")
    camera_name = message.camera_name.strip()
    if not camera_name:
        raise ValueError("camera_name must not be empty")

    severity = int(message.severity)
    if severity not in SEVERITY_LABELS:
        raise ValueError(f"unknown severity value: {severity}")
    confidence = _confidence(message.confidence, "confidence")

    localization_method = int(message.localization_method)
    if localization_method not in LOCALIZATION_LABELS:
        raise ValueError(f"unknown localization_method value: {localization_method}")

    position = None
    localization = {
        "available": bool(message.has_3d_position),
        "method": localization_method,
        "method_name": LOCALIZATION_LABELS[localization_method],
        "confidence": None,
        "frame_id": None,
        "stamp": None,
    }
    if message.has_3d_position:
        if localization_method == DefectEvent.LOCALIZATION_NONE:
            raise ValueError("localized event cannot use LOCALIZATION_NONE")
        frame_id = message.position.header.frame_id.strip()
        if not frame_id:
            raise ValueError("localized event position frame_id must not be empty")
        localization_confidence = _confidence(
            message.localization_confidence, "localization_confidence"
        )
        position = {
            "x": _finite(message.position.point.x, "position x"),
            "y": _finite(message.position.point.y, "position y"),
            "z": _finite(message.position.point.z, "position z"),
        }
        localization.update(
            {
                "confidence": localization_confidence,
                "frame_id": frame_id,
                "stamp": _stamp(message.position.header.stamp),
            }
        )
    elif localization_method != DefectEvent.LOCALIZATION_NONE:
        raise ValueError("event without 3D position must use LOCALIZATION_NONE")

    semantic_location = None
    if message.has_semantic_location:
        clock_position = _finite(
            message.clock_position_hours, "clock_position_hours"
        )
        if not 0.0 <= clock_position < 12.0:
            raise ValueError("clock_position_hours must be in [0, 12)")
        semantic_location = {
            "chainage_m": _finite(message.chainage_m, "chainage_m"),
            "chainage": message.chainage.strip(),
            "segment_name": message.segment_name.strip(),
            "segment_id": int(message.segment_id),
            "segment_offset_m": _finite(
                message.segment_offset_m, "segment_offset_m"
            ),
            "clock_position_hours": clock_position,
            "structure_area": message.structure_area.strip(),
        }

    return {
        "event_id": event_id,
        "detection_id": message.detection_id.strip(),
        "source_stamp": _stamp(message.header.stamp),
        "source_frame_id": message.header.frame_id.strip(),
        "camera_name": camera_name,
        "type": class_name,
        "class_name": class_name,
        "confidence": confidence,
        "severity": severity,
        "level": SEVERITY_LABELS[severity],
        "bbox": _bbox(message),
        "has_3d_position": bool(message.has_3d_position),
        "position": position,
        "localization": localization,
        "model_name": message.model_name.strip(),
        "snapshot_uri": message.snapshot_uri.strip(),
        "has_semantic_location": bool(message.has_semantic_location),
        "semantic_location": semantic_location,
        # Compatibility aliases used by the existing dashboard.
        "mileage": semantic_location["chainage"] if semantic_location else None,
        "ring_number": semantic_location["segment_id"] if semantic_location else None,
        "clock_position": (
            semantic_location["clock_position_hours"] if semantic_location else None
        ),
    }
