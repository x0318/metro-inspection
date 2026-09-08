from datetime import datetime, timezone

import cv2
import numpy as np
import pytest

from metro_dashboard_bridge.camera_store import (
    CameraDefinition,
    CameraFrame,
    CameraFrameStore,
)
from metro_dashboard_bridge.camera_stream import (
    CameraPreviewEncoder,
    CameraStreamConfig,
    iter_mjpeg,
)


def make_jpeg(width: int = 1440, height: int = 1080) -> bytes:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (30, 120, 210)
    success, encoded = cv2.imencode(".jpg", image)
    assert success
    return encoded.tobytes()


def make_frame(data: bytes, sequence: int = 1) -> CameraFrame:
    return CameraFrame(
        camera_id="xj1",
        sequence=sequence,
        data=data,
        format="jpeg",
        frame_id="xj1_optical_frame",
        source_timestamp=1.0,
        received_at=datetime(2026, 8, 17, tzinfo=timezone.utc).isoformat(),
        received_monotonic=1.0,
    )


def test_preview_encoder_preserves_aspect_ratio_and_caches_result() -> None:
    encoder = CameraPreviewEncoder(
        ["xj1"],
        CameraStreamConfig(max_width=640, max_height=360, jpeg_quality=55),
    )
    frame = make_frame(make_jpeg())

    preview = encoder.encode(frame)
    cached = encoder.encode(frame)

    assert preview is cached
    decoded = cv2.imdecode(np.frombuffer(preview.data, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (360, 480)


def test_preview_encoder_rejects_invalid_compressed_bytes() -> None:
    encoder = CameraPreviewEncoder(["xj1"], CameraStreamConfig())
    assert encoder.encode(make_frame(b"not-an-image")) is None


def test_mjpeg_iterator_yields_encoded_multipart_frame() -> None:
    store = CameraFrameStore(
        [CameraDefinition("xj1", "XJ1", "/xj1/compressed")]
    )
    store.update("xj1", make_jpeg(320, 240), image_format="jpeg")
    encoder = CameraPreviewEncoder(["xj1"], CameraStreamConfig(default_fps=6.0))

    chunk = next(iter_mjpeg(store, encoder, "xj1", requested_fps=6.0))

    assert chunk.startswith(b"--frame\r\nContent-Type: image/jpeg\r\n")
    assert b"Content-Length:" in chunk
    assert chunk.endswith(b"\r\n")


def test_stream_config_rejects_invalid_rate() -> None:
    with pytest.raises(ValueError, match="must not exceed"):
        CameraStreamConfig(default_fps=12.0, max_fps=10.0)
