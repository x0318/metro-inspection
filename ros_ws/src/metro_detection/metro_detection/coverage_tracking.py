"""Pure tracking logic for truth-assisted simulation coverage evaluation."""

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Sequence, Tuple


@dataclass(frozen=True)
class DefectSite:
    """One simulation-only reference position along the tunnel."""

    site_id: str
    chainage_m: float


@dataclass(frozen=True)
class DetectionObservation:
    """Best YOLO result from one camera frame."""

    camera_name: str
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    class_name: str
    confidence: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float

    @property
    def frame_key(self) -> Tuple[str, int, int]:
        return self.camera_name, self.stamp_sec, self.stamp_nanosec


@dataclass(frozen=True)
class SiteConfirmation:
    """A site confirmed by detections from multiple unique image frames."""

    site: DefectSite
    observation: DetectionObservation
    hit_count: int


@dataclass
class _SiteState:
    hit_frames: set[Tuple[str, int, int]] = field(default_factory=set)
    class_scores: Dict[str, float] = field(default_factory=dict)
    best_by_class: Dict[str, DetectionObservation] = field(default_factory=dict)
    confirmed: bool = False


class CoverageTracker:
    """Match real YOLO boxes to reference positions without creating boxes."""

    def __init__(
        self,
        sites: Sequence[DefectSite],
        camera_offsets_m: Dict[str, float],
        match_tolerance_m: float,
        confirmation_hits: int,
    ) -> None:
        if not sites:
            raise ValueError("at least one defect site is required")
        if len({site.site_id for site in sites}) != len(sites):
            raise ValueError("defect site IDs must be unique")
        if match_tolerance_m <= 0.0:
            raise ValueError("match_tolerance_m must be positive")
        if confirmation_hits <= 0:
            raise ValueError("confirmation_hits must be positive")

        self.sites = tuple(sorted(sites, key=lambda site: site.chainage_m))
        self.camera_offsets_m = dict(camera_offsets_m)
        self.match_tolerance_m = float(match_tolerance_m)
        self.confirmation_hits = int(confirmation_hits)
        self._states = {site.site_id: _SiteState() for site in self.sites}

    def observe(
        self,
        robot_chainage_m: float,
        camera_name: str,
        observations: Iterable[DetectionObservation],
    ) -> Optional[SiteConfirmation]:
        """Record one frame and return a newly confirmed site, if any."""
        if camera_name not in self.camera_offsets_m:
            raise ValueError(f"unknown camera: {camera_name}")
        observations = tuple(observations)
        if not observations:
            return None

        camera_chainage = (
            float(robot_chainage_m) + self.camera_offsets_m[camera_name]
        )
        site = min(
            self.sites,
            key=lambda candidate: abs(candidate.chainage_m - camera_chainage),
        )
        if abs(site.chainage_m - camera_chainage) > self.match_tolerance_m:
            return None

        state = self._states[site.site_id]
        if state.confirmed:
            return None
        best = max(observations, key=lambda observation: observation.confidence)
        if best.frame_key in state.hit_frames:
            return None
        state.hit_frames.add(best.frame_key)
        state.class_scores[best.class_name] = (
            state.class_scores.get(best.class_name, 0.0) + best.confidence
        )
        previous_best = state.best_by_class.get(best.class_name)
        if previous_best is None or best.confidence > previous_best.confidence:
            state.best_by_class[best.class_name] = best

        if len(state.hit_frames) < self.confirmation_hits:
            return None

        state.confirmed = True
        stable_class = max(state.class_scores, key=state.class_scores.get)
        return SiteConfirmation(
            site=site,
            observation=state.best_by_class[stable_class],
            hit_count=len(state.hit_frames),
        )

    def confirmed_site_ids(self) -> Tuple[str, ...]:
        return tuple(
            site.site_id
            for site in self.sites
            if self._states[site.site_id].confirmed
        )

    def status(self) -> Tuple[dict, ...]:
        """Return serializable per-site coverage state in chainage order."""
        return tuple(
            {
                "id": site.site_id,
                "chainage_m": site.chainage_m,
                "hits": len(self._states[site.site_id].hit_frames),
                "confirmed": self._states[site.site_id].confirmed,
            }
            for site in self.sites
        )
