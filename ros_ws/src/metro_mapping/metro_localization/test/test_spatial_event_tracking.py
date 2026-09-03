from metro_localization.spatial_event_tracking import SpatialEventTracker


def make_tracker():
    return SpatialEventTracker(
        distance_threshold_m=0.5,
        confirmation_hits=3,
        republish_period_sec=2.0,
        event_prefix="odin1",
    )


def test_requires_unique_frames_before_publishing():
    tracker = make_tracker()

    first, due = tracker.observe(
        class_name="crack", position=(1.0, 2.0, 3.0), frame_key=(1, 0), stamp_sec=1.0
    )
    assert due is False
    duplicate, due = tracker.observe(
        class_name="crack", position=(1.01, 2.0, 3.0), frame_key=(1, 0), stamp_sec=1.0
    )
    assert duplicate.event_id == first.event_id
    assert duplicate.hit_count == 1
    assert due is False

    tracker.observe(
        class_name="crack", position=(1.02, 2.0, 3.0), frame_key=(2, 0), stamp_sec=2.0
    )
    confirmed, due = tracker.observe(
        class_name="crack", position=(1.03, 2.0, 3.0), frame_key=(3, 0), stamp_sec=3.0
    )
    assert confirmed.hit_count == 3
    assert due is True


def test_separates_distant_observations_and_throttles_updates():
    tracker = SpatialEventTracker(
        distance_threshold_m=0.2,
        confirmation_hits=1,
        republish_period_sec=2.0,
        event_prefix="odin1",
    )

    first, due = tracker.observe(
        class_name="crack", position=(0.0, 0.0, 0.0), frame_key=(1, 0), stamp_sec=1.0
    )
    assert due is True
    same, due = tracker.observe(
        class_name="crack", position=(0.05, 0.0, 0.0), frame_key=(2, 0), stamp_sec=2.0
    )
    assert same.event_id == first.event_id
    assert due is False
    _, due = tracker.observe(
        class_name="crack", position=(0.05, 0.0, 0.0), frame_key=(3, 0), stamp_sec=3.0
    )
    assert due is True

    distant, due = tracker.observe(
        class_name="crack", position=(1.0, 0.0, 0.0), frame_key=(4, 0), stamp_sec=4.0
    )
    assert distant.event_id != first.event_id
    assert due is True
