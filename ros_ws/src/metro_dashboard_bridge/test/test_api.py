import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from metro_dashboard_bridge.api import create_app
from metro_dashboard_bridge.camera_store import CameraDefinition, CameraFrameStore
from metro_dashboard_bridge.camera_stream import CameraStreamConfig
from metro_dashboard_bridge.defect_store import DefectStore


def test_health_and_defect_queries() -> None:
    store = DefectStore()
    store.upsert({"event_id": "event-1", "type": "crack"})
    client = TestClient(
        create_app(
            store,
            lambda: {
                "accepted_messages": 1,
                "rejected_messages": 0,
                "stored_events": 1,
            },
        )
    )

    health = client.get("/api/health")
    assert health.status_code == 200
    assert health.json()["defects"]["stored_events"] == 1

    records = client.get("/api/defects")
    assert records.status_code == 200
    assert records.json()["records"][0]["event_id"] == "event-1"

    assert client.get("/api/defects/event-1").status_code == 200
    assert client.get("/api/defects/missing").status_code == 404


def test_inspection_session_history_queries(tmp_path) -> None:
    database_path = tmp_path / "defects.sqlite3"
    previous_store = DefectStore(
        database_path=database_path,
        session_id="inspection-previous",
    )
    previous_store.upsert({"event_id": "old-event", "type": "crack"})
    previous_store.close()

    active_store = DefectStore(
        database_path=database_path,
        session_id="inspection-active",
    )
    active_store.upsert({"event_id": "new-event", "type": "water_leakage"})
    client = TestClient(create_app(active_store))

    sessions = client.get("/api/sessions")
    assert sessions.status_code == 200
    assert sessions.headers["cache-control"] == "no-store"
    assert sessions.json()["active_session_id"] == "inspection-active"
    assert {
        session["session_id"]: session["record_count"]
        for session in sessions.json()["sessions"]
    } == {"inspection-active": 1, "inspection-previous": 1}

    previous_records = client.get(
        "/api/defects", params={"session_id": "inspection-previous"}
    )
    assert previous_records.status_code == 200
    assert previous_records.json()["session_id"] == "inspection-previous"
    assert previous_records.json()["records"][0]["event_id"] == "old-event"
    assert client.get(
        "/api/defects/old-event",
        params={"session_id": "inspection-previous"},
    ).status_code == 200
    assert client.get("/api/defects/old-event").status_code == 404
    active_store.close()


def test_camera_status_and_allowlisted_stream_route() -> None:
    camera_store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")]
    )
    client = TestClient(
        create_app(
            DefectStore(),
            camera_store=camera_store,
            camera_stream_config=CameraStreamConfig(default_fps=5.0),
        )
    )

    cameras = client.get("/api/cameras")
    assert cameras.status_code == 200
    assert cameras.json()["cameras"][0]["id"] == "xj1"
    assert cameras.json()["preview"]["default_fps"] == 5.0
    assert client.get("/api/health").json()["cameras"] == {
        "online": 0,
        "total": 1,
    }
    assert client.get("/api/cameras/xj2/stream.mjpg").status_code == 404
    assert client.get("/api/cameras/xj2/frame.jpg").status_code == 404
    assert client.get("/api/cameras/xj1/frame.jpg").status_code == 503


def test_camera_snapshot_returns_latest_jpeg() -> None:
    camera_store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")]
    )
    source = np.full((240, 320, 3), (30, 120, 210), dtype=np.uint8)
    success, encoded = cv2.imencode(".jpg", source)
    assert success
    camera_store.update("xj1", encoded.tobytes(), image_format="jpeg")
    client = TestClient(
        create_app(
            DefectStore(),
            camera_store=camera_store,
            camera_stream_config=CameraStreamConfig(),
        )
    )

    response = client.get("/api/cameras/xj1/frame.jpg")

    assert response.status_code == 200
    assert response.headers["content-type"] == "image/jpeg"
    assert response.headers["cache-control"] == (
        "no-store, no-cache, must-revalidate"
    )
    assert response.headers["x-camera-sequence"] == "1"
    decoded = cv2.imdecode(
        np.frombuffer(response.content, np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape[:2] == (240, 320)


def test_dashboard_is_served_without_shadowing_api(tmp_path) -> None:
    (tmp_path / "index.html").write_text(
        "<!DOCTYPE html><title>Metro dashboard</title>", encoding="utf-8"
    )
    store = DefectStore()
    client = TestClient(create_app(store, dashboard_dir=tmp_path))

    page = client.get("/")
    assert page.status_code == 200
    assert "Metro dashboard" in page.text
    assert page.headers["cache-control"] == "no-store"
    assert client.get("/api/health").status_code == 200


def test_unknown_websocket_is_rejected_before_static_mount(tmp_path) -> None:
    (tmp_path / "index.html").write_text(
        "<!DOCTYPE html><title>Metro dashboard</title>", encoding="utf-8"
    )
    client = TestClient(create_app(DefectStore(), dashboard_dir=tmp_path))

    with pytest.raises(WebSocketDisconnect) as error:
        with client.websocket_connect("/ws/legacy"):
            pass

    assert error.value.code == 1008
