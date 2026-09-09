import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Deque, Dict, Iterable, List, Optional


@dataclass(frozen=True)
class CameraDefinition:
    camera_id: str
    label: str
    topic: str


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    sequence: int
    data: bytes
    format: str
    frame_id: str
    source_timestamp: Optional[float]
    received_at: str
    received_monotonic: float


@dataclass
class _CameraState:
    definition: CameraDefinition
    frame: Optional[CameraFrame]
    frames_received: int
    arrival_times: Deque[float]


class CameraFrameStore:
    """Keep only the latest compressed frame and recent timing for each camera."""

    def __init__(
        self,
        definitions: Iterable[CameraDefinition],
        monotonic_clock: Callable[[], float] = time.monotonic,
        utc_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    ) -> None:
        states: Dict[str, _CameraState] = {}
        for definition in definitions:
            camera_id = definition.camera_id.strip()
            if not camera_id:
                raise ValueError("camera_id must not be empty")
            if camera_id in states:
                raise ValueError(f"duplicate camera_id: {camera_id}")
            topic = definition.topic.strip()
            if not topic:
                raise ValueError(f"camera topic must not be empty: {camera_id}")
            normalized = CameraDefinition(
                camera_id=camera_id,
                label=definition.label.strip() or camera_id,
                topic=topic,
            )
            states[camera_id] = _CameraState(
                definition=normalized,
                frame=None,
                frames_received=0,
                arrival_times=deque(maxlen=240),
            )

        self._states = states
        self._monotonic_clock = monotonic_clock
        self._utc_now = utc_now
        self._condition = threading.Condition()

    @property
    def camera_ids(self) -> List[str]:
        return list(self._states)

    @property
    def definitions(self) -> List[CameraDefinition]:
        return [state.definition for state in self._states.values()]

    def contains(self, camera_id: str) -> bool:
        return camera_id in self._states

    def update(
        self,
        camera_id: str,
        data: bytes,
        image_format: str = "",
        frame_id: str = "",
        source_timestamp: Optional[float] = None,
    ) -> CameraFrame:
        payload = bytes(data)
        if not payload:
            raise ValueError("compressed camera frame must not be empty")

        with self._condition:
            state = self._require_state(camera_id)
            now = self._monotonic_clock()
            state.frames_received += 1
            state.arrival_times.append(now)
            frame = CameraFrame(
                camera_id=camera_id,
                sequence=state.frames_received,
                data=payload,
                format=image_format.strip(),
                frame_id=frame_id.strip(),
                source_timestamp=source_timestamp,
                received_at=self._utc_now().isoformat(),
                received_monotonic=now,
            )
            state.frame = frame
            self._condition.notify_all()
            return frame

    def get(self, camera_id: str) -> Optional[CameraFrame]:
        with self._condition:
            return self._require_state(camera_id).frame

    def wait_for_frame(
        self,
        camera_id: str,
        after_sequence: int,
        timeout_seconds: float,
    ) -> Optional[CameraFrame]:
        if timeout_seconds < 0.0:
            raise ValueError("timeout_seconds must not be negative")

        with self._condition:
            state = self._require_state(camera_id)
            self._condition.wait_for(
                lambda: state.frame is not None
                and state.frame.sequence > after_sequence,
                timeout=timeout_seconds,
            )
            if state.frame is None or state.frame.sequence <= after_sequence:
                return None
            return state.frame

    def status(self, camera_id: str, timeout_seconds: float) -> Dict[str, object]:
        if timeout_seconds <= 0.0:
            raise ValueError("timeout_seconds must be positive")

        with self._condition:
            state = self._require_state(camera_id)
            return self._status_locked(state, self._monotonic_clock(), timeout_seconds)

    def list_status(self, timeout_seconds: float) -> List[Dict[str, object]]:
        if timeout_seconds <= 0.0:
            raise ValueError("timeout_seconds must be positive")

        with self._condition:
            now = self._monotonic_clock()
            return [
                self._status_locked(state, now, timeout_seconds)
                for state in self._states.values()
            ]

    def summary(self, timeout_seconds: float) -> Dict[str, int]:
        statuses = self.list_status(timeout_seconds)
        return {
            "online": sum(1 for item in statuses if item["available"]),
            "total": len(statuses),
        }

    def _require_state(self, camera_id: str) -> _CameraState:
        try:
            return self._states[camera_id]
        except KeyError as error:
            raise KeyError(f"unknown camera: {camera_id}") from error

    @staticmethod
    def _source_fps(arrival_times: Deque[float], now: float) -> float:
        recent = [timestamp for timestamp in arrival_times if now - timestamp <= 2.0]
        if len(recent) < 2:
            return 0.0
        duration = recent[-1] - recent[0]
        return 0.0 if duration <= 0.0 else (len(recent) - 1) / duration

    def _status_locked(
        self,
        state: _CameraState,
        now: float,
        timeout_seconds: float,
    ) -> Dict[str, object]:
        frame = state.frame
        age_seconds = None if frame is None else max(0.0, now - frame.received_monotonic)
        available = age_seconds is not None and age_seconds <= timeout_seconds
        return {
            "id": state.definition.camera_id,
            "label": state.definition.label,
            "topic": state.definition.topic,
            "available": available,
            "source_fps": round(self._source_fps(state.arrival_times, now), 1),
            "age_seconds": None if age_seconds is None else round(age_seconds, 3),
            "frames_received": state.frames_received,
            "format": frame.format if frame is not None else None,
            "frame_id": frame.frame_id if frame is not None else None,
            "received_at": frame.received_at if frame is not None else None,
            "stream_url": (
                f"/api/cameras/{state.definition.camera_id}/stream.mjpg"
            ),
            "frame_url": (
                f"/api/cameras/{state.definition.camera_id}/frame.jpg"
            ),
        }
