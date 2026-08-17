import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, Optional

import cv2
import numpy as np

from .camera_store import CameraFrame, CameraFrameStore


@dataclass(frozen=True)
class CameraStreamConfig:
    max_width: int = 640
    max_height: int = 360
    jpeg_quality: int = 55
    default_fps: float = 6.0
    max_fps: float = 10.0

    def __post_init__(self) -> None:
        if self.max_width <= 0 or self.max_height <= 0:
            raise ValueError("camera preview dimensions must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        if self.default_fps <= 0.0 or self.max_fps <= 0.0:
            raise ValueError("camera stream FPS must be positive")
        if self.default_fps > self.max_fps:
            raise ValueError("default_fps must not exceed max_fps")


@dataclass(frozen=True)
class EncodedPreview:
    sequence: int
    data: bytes
    width: int
    height: int


class CameraPreviewEncoder:
    """Resize compressed source frames once and share the result across viewers."""

    def __init__(
        self,
        camera_ids: Iterable[str],
        config: CameraStreamConfig,
    ) -> None:
        self.config = config
        self._locks = {camera_id: threading.Lock() for camera_id in camera_ids}
        self._cache: Dict[str, EncodedPreview] = {}

    def encode(self, frame: CameraFrame) -> Optional[EncodedPreview]:
        try:
            camera_lock = self._locks[frame.camera_id]
        except KeyError as error:
            raise KeyError(f"unknown camera: {frame.camera_id}") from error

        with camera_lock:
            cached = self._cache.get(frame.camera_id)
            if cached is not None and cached.sequence == frame.sequence:
                return cached

            source = cv2.imdecode(
                np.frombuffer(frame.data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if source is None or source.size == 0:
                return None

            source_height, source_width = source.shape[:2]
            scale = min(
                1.0,
                self.config.max_width / source_width,
                self.config.max_height / source_height,
            )
            width = max(1, int(round(source_width * scale)))
            height = max(1, int(round(source_height * scale)))
            if (width, height) != (source_width, source_height):
                source = cv2.resize(
                    source,
                    (width, height),
                    interpolation=cv2.INTER_AREA,
                )

            success, encoded = cv2.imencode(
                ".jpg",
                source,
                [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality],
            )
            if not success:
                return None

            preview = EncodedPreview(
                sequence=frame.sequence,
                data=encoded.tobytes(),
                width=width,
                height=height,
            )
            self._cache[frame.camera_id] = preview
            return preview


def iter_mjpeg(
    store: CameraFrameStore,
    encoder: CameraPreviewEncoder,
    camera_id: str,
    requested_fps: float,
) -> Iterator[bytes]:
    fps = min(requested_fps, encoder.config.max_fps)
    if fps <= 0.0:
        raise ValueError("requested_fps must be positive")

    minimum_interval = 1.0 / fps
    last_sequence = 0
    last_sent = 0.0

    while True:
        frame = store.wait_for_frame(
            camera_id,
            after_sequence=last_sequence,
            timeout_seconds=1.0,
        )
        if frame is None:
            continue

        remaining = minimum_interval - (time.monotonic() - last_sent)
        if remaining > 0.0:
            time.sleep(remaining)
            latest = store.get(camera_id)
            if latest is not None:
                frame = latest

        preview = encoder.encode(frame)
        last_sequence = frame.sequence
        if preview is None:
            continue

        last_sent = time.monotonic()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: "
            + str(len(preview.data)).encode("ascii")
            + b"\r\n\r\n"
            + preview.data
            + b"\r\n"
        )
