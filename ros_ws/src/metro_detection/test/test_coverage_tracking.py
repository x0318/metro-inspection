from metro_detection.coverage_tracking import (
    CoverageTracker,
    DefectSite,
    DetectionObservation,
)


def observation(stamp: int, class_name: str = "crack", score: float = 0.8):
    return DetectionObservation(
        camera_name="xj1",
        stamp_sec=stamp,
        stamp_nanosec=0,
        frame_id="xj1_optical_frame",
        class_name=class_name,
        confidence=score,
        center_x=100.0,
        center_y=200.0,
        size_x=40.0,
        size_y=60.0,
    )


def tracker(confirmation_hits: int = 3):
    return CoverageTracker(
        sites=[
            DefectSite("sim_defect_01", 1.8),
            DefectSite("sim_defect_02", 6.6),
        ],
        camera_offsets_m={"xj1": 0.0},
        match_tolerance_m=0.9,
        confirmation_hits=confirmation_hits,
    )


def test_location_without_yolo_box_never_confirms_site():
    coverage = tracker(confirmation_hits=1)

    assert coverage.observe(1.8, "xj1", []) is None
    assert coverage.confirmed_site_ids() == ()


def test_three_unique_yolo_frames_confirm_one_site():
    coverage = tracker()

    assert coverage.observe(1.7, "xj1", [observation(1)]) is None
    assert coverage.observe(1.8, "xj1", [observation(2)]) is None
    confirmation = coverage.observe(1.9, "xj1", [observation(3)])

    assert confirmation is not None
    assert confirmation.site.site_id == "sim_defect_01"
    assert confirmation.hit_count == 3
    assert coverage.confirmed_site_ids() == ("sim_defect_01",)


def test_duplicate_camera_timestamp_is_not_counted_twice():
    coverage = tracker(confirmation_hits=2)
    same_frame = observation(10)

    assert coverage.observe(1.8, "xj1", [same_frame]) is None
    assert coverage.observe(1.8, "xj1", [same_frame]) is None
    assert coverage.confirmed_site_ids() == ()


def test_detection_outside_reference_window_is_not_counted():
    coverage = tracker(confirmation_hits=1)

    assert coverage.observe(4.0, "xj1", [observation(1)]) is None
    assert coverage.confirmed_site_ids() == ()


def test_confirmation_uses_class_with_highest_accumulated_confidence():
    coverage = tracker()

    coverage.observe(1.8, "xj1", [observation(1, "crack", 0.6)])
    coverage.observe(1.8, "xj1", [observation(2, "water_leakage", 0.9)])
    result = coverage.observe(1.8, "xj1", [observation(3, "crack", 0.7)])

    assert result is not None
    assert result.observation.class_name == "crack"
