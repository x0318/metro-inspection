from datetime import datetime, timezone

import pytest

from metro_dashboard_bridge.camera_store import CameraDefinition, CameraFrameStore


def test_camera_store_tracks_latest_frame_fps_and_staleness() -> None:
    monotonic = [0.0]
    store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")],
        monotonic_clock=lambda: monotonic[0],
        utc_now=lambda: datetime(2026, 8, 17, tzinfo=timezone.utc),
    )

    for index in range(3):
        monotonic[0] = index * 0.1
        store.update(
            "xj1",
            data=f"frame-{index}".encode("ascii"),
            image_format="jpeg",
            frame_id="xj1_optical_frame",
        )

    status = store.status("xj1", timeout_seconds=1.0)
    assert status["available"] is True
    assert status["source_fps"] == pytest.approx(10.0)
    assert status["frames_received"] == 3
    assert status["frame_id"] == "xj1_optical_frame"
    assert store.get("xj1").data == b"frame-2"

    monotonic[0] = 1.3
    assert store.status("xj1", timeout_seconds=1.0)["available"] is False


def test_camera_store_rejects_unknown_camera_and_empty_frame() -> None:
    store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")]
    )

    with pytest.raises(KeyError, match="unknown camera"):
        store.get("xj2")
    with pytest.raises(ValueError, match="must not be empty"):
        store.update("xj1", b"")


def test_wait_for_frame_returns_current_newer_frame_without_blocking() -> None:
    store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")]
    )
    frame = store.update("xj1", b"jpeg")

    assert store.wait_for_frame("xj1", 0, 0.0) == frame
    assert store.wait_for_frame("xj1", frame.sequence, 0.0) is None
