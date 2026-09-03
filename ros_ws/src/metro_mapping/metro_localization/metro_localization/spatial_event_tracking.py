"""Small spatial tracker used to avoid one dashboard event per video frame."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


FrameKey = Tuple[int, int]
Position3D = Tuple[float, float, float]


@dataclass
class SpatialTrack:
    event_id: str
    class_name: str
    position: Position3D
    hit_count: int
    last_frame_key: FrameKey
    last_publish_sec: Optional[float] = None


class SpatialEventTracker:
    """Associate same-class 3D observations and confirm them over time."""

    def __init__(
        self,
        *,
        distance_threshold_m: float,
        confirmation_hits: int,
        republish_period_sec: float,
        event_prefix: str,
    ) -> None:
        if distance_threshold_m <= 0.0:
            raise ValueError("distance_threshold_m must be positive")
        if confirmation_hits <= 0:
            raise ValueError("confirmation_hits must be positive")
        if republish_period_sec < 0.0:
            raise ValueError("republish_period_sec must be non-negative")
        if not event_prefix.strip():
            raise ValueError("event_prefix must not be empty")

        self.distance_threshold_m = float(distance_threshold_m)
        self.confirmation_hits = int(confirmation_hits)
        self.republish_period_sec = float(republish_period_sec)
        self.event_prefix = event_prefix.strip()
        self.tracks: list[SpatialTrack] = []

    @staticmethod
    def _distance(a: Position3D, b: Position3D) -> float:
        return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))

    def _nearest(self, class_name: str, position: Position3D):
        candidates = [
            track for track in self.tracks if track.class_name == class_name
        ]
        if not candidates:
            return None
        nearest = min(
            candidates,
            key=lambda track: self._distance(track.position, position),
        )
        if self._distance(nearest.position, position) > self.distance_threshold_m:
            return None
        return nearest

    def observe(
        self,
        *,
        class_name: str,
        position: Position3D,
        frame_key: FrameKey,
        stamp_sec: float,
    ) -> tuple[SpatialTrack, bool]:
        """Return the associated track and whether a platform event is due."""

        normalized_class = str(class_name).strip()
        if not normalized_class:
            raise ValueError("class_name must not be empty")
        point = tuple(float(value) for value in position)
        track = self._nearest(normalized_class, point)
        if track is None:
            track = SpatialTrack(
                event_id=f"{self.event_prefix}-{len(self.tracks) + 1:04d}",
                class_name=normalized_class,
                position=point,
                hit_count=1,
                last_frame_key=frame_key,
            )
            self.tracks.append(track)
        elif track.last_frame_key != frame_key:
            previous_hits = track.hit_count
            track.hit_count += 1
            track.position = tuple(
                (old * previous_hits + new) / track.hit_count
                for old, new in zip(track.position, point)
            )
            track.last_frame_key = frame_key

        if track.hit_count < self.confirmation_hits:
            return track, False
        if track.last_publish_sec is None:
            track.last_publish_sec = float(stamp_sec)
            return track, True
        if float(stamp_sec) - track.last_publish_sec >= self.republish_period_sec:
            track.last_publish_sec = float(stamp_sec)
            return track, True
        return track, False
