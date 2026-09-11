"""Render the report snapshot: one red point per localized defect record."""

from visualization_msgs.msg import Marker, MarkerArray


def report_markers(records, session_id):
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers = [clear]
    # Stable namespaces also make repeated observations update the same marker.
    records_by_id = {record["event_id"]: record for record in records}
    for event_id, record in sorted(records_by_id.items()):
        location = record.get("localization") or {}
        position = record.get("position")
        if not record.get("has_3d_position") or not position or not location.get("frame_id"):
            continue
        point = Marker()
        point.header.frame_id = location["frame_id"]
        if location.get("method_name") == "model_reference" and point.header.frame_id == "world":
            point.header.frame_id = "model_annotation_world"
        # Resolve the latest frame transform, including after a simulation pause.
        point.frame_locked = True
        point.ns = f"{session_id}/{event_id}"
        point.id = 0
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose.orientation.w = 1.0
        point.pose.position.x = float(position["x"])
        point.pose.position.y = float(position["y"])
        point.pose.position.z = float(position["z"])
        point.scale.x = point.scale.y = point.scale.z = 0.18
        point.color.r = 1.0
        point.color.a = 1.0
        markers.append(point)
    return MarkerArray(markers=markers)
