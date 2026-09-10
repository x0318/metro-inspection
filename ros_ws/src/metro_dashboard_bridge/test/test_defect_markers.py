from visualization_msgs.msg import Marker

from metro_dashboard_bridge.defect_markers import report_markers
from metro_dashboard_bridge.defect_store import DefectStore


def record(site="leak_04", camera="pitch_camera"):
    return {"event_id": f"model-demo:{site}", "camera_name": camera,
            "has_3d_position": True, "position": {"x": 18.5, "y": 2.7, "z": 2.4},
            "localization": {"frame_id": "world", "method_name": "model_reference"}}


def test_multiple_cameras_and_repeated_frames_keep_one_record_and_one_red_point(tmp_path):
    path = tmp_path / "defects.sqlite3"
    store = DefectStore(database_path=path, session_id="patrol")
    for camera in ["pitch_camera", "odin1", "pitch_camera"]:
        store.upsert(record(camera=camera))
    assert store.count() == 1
    snapshot = report_markers(store.list(), store.session_id)
    assert snapshot.markers[0].action == Marker.DELETEALL
    points = [m for m in snapshot.markers if m.action == Marker.ADD]
    assert len(points) == 1
    point = points[0]
    assert point.header.frame_id == "model_annotation_world"
    assert point.pose.position.x == 18.5
    assert point.color.r == point.color.a == 1.0
    assert point.color.g == point.color.b == 0.0
    assert point.frame_locked
    store.close()
    reopened = DefectStore(database_path=path, session_id="patrol")
    assert report_markers(reopened.list(), "patrol") == snapshot
    reopened.close()


def test_distinct_defects_survive_and_unlocalized_records_have_no_point():
    first, second = record(), record("leak_02")
    missing = {"event_id": "no-depth", "has_3d_position": False}
    snapshot = report_markers([first, second, first, missing], "patrol")
    assert len(snapshot.markers) == 3  # clear plus two distinct sites
    assert len({m.ns for m in snapshot.markers[1:]}) == 2
    # An empty/pruned snapshot also clears points retained by RViz.
    assert [m.action for m in report_markers([], "patrol").markers] == [Marker.DELETEALL]


def test_measured_position_retains_its_coordinate_frame():
    measured = record()
    measured["localization"] = {"frame_id": "map", "method_name": "current_cloud"}
    assert report_markers([measured], "patrol").markers[1].header.frame_id == "map"
