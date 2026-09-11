"""Small spatial tracker used to avoid one dashboard event per video frame."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple
from uuid import uuid4

import numpy as np
from scipy.optimize import linear_sum_assignment


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
        self._last_frame_key: Optional[FrameKey] = None

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
        """Single-target convenience API; use observe_frame for multiple targets."""

        results = self.observe_frame(
            observations=[(class_name, position)],
            frame_key=frame_key,
            stamp_sec=stamp_sec,
        )
        if results:
            return results[0]
        track = self._nearest(str(class_name).strip(), position)
        if track is None:
            raise ValueError("Cannot associate an obsolete single-target frame")
        return track, False

    def observe_frame(
        self,
        *,
        observations: list[tuple[str, Position3D]],
        frame_key: FrameKey,
        stamp_sec: float,
    ) -> list[tuple[SpatialTrack, bool]]:
        """Associate a whole frame one-to-one, returning results in input order.

        Duplicate and out-of-order frames return no results and change no state.
        Tracks are process-local; UUIDs prevent overwrite, not restart deduplication.
        """
        if self._last_frame_key is not None and frame_key <= self._last_frame_key:
            return []

        normalized = []
        for class_name, position in observations:
            name = str(class_name).strip()
            point = tuple(float(value) for value in position)
            if not name:
                raise ValueError("class_name must not be empty")
            if len(point) != 3 or not all(math.isfinite(value) for value in point):
                raise ValueError("position must contain three finite coordinates")
            normalized.append((name, point))

        # Dummy columns permit unmatched observations. Their cost makes maximum
        # valid match count take priority over minimum total spatial distance.
        count = len(normalized)
        track_count = len(self.tracks)
        assignments = {}
        if count and track_count:
            unmatched_cost = (count + 1) * self.distance_threshold_m
            costs = np.full((count, track_count + count), unmatched_cost)
            costs[:, :track_count] = np.inf
            for row, (name, point) in enumerate(normalized):
                for col, track in enumerate(self.tracks):
                    distance = self._distance(track.position, point)
                    if name == track.class_name and distance <= self.distance_threshold_m:
                        costs[row, col] = distance
            rows, columns = linear_sum_assignment(costs)
            assignments = {
                int(row): self.tracks[col]
                for row, col in zip(rows, columns) if col < track_count
            }

        self._last_frame_key = frame_key
        results = []
        for index, (name, point) in enumerate(normalized):
            track = assignments.get(index)
            if track is None:
                track = SpatialTrack(
                    event_id=f"{self.event_prefix}-{uuid4().hex}",
                    class_name=name,
                    position=point,
                    hit_count=1,
                    last_frame_key=frame_key,
                )
                self.tracks.append(track)
            else:
                previous_hits = track.hit_count
                track.hit_count += 1
                track.position = tuple(
                    (old * previous_hits + new) / track.hit_count
                    for old, new in zip(track.position, point)
                )
                track.last_frame_key = frame_key
            results.append((track, self._publication_due(track, stamp_sec)))
        return results

    def _publication_due(self, track: SpatialTrack, stamp_sec: float) -> bool:
        if track.hit_count < self.confirmation_hits:
            return False
        if track.last_publish_sec is None:
            track.last_publish_sec = float(stamp_sec)
            return True
        if float(stamp_sec) - track.last_publish_sec >= self.republish_period_sec:
            track.last_publish_sec = float(stamp_sec)
            return True
        return False
