import pytest

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


def observe_frame(tracker, positions, frame, classes=None):
    return tracker.observe_frame(
        observations=list(zip(classes or ["crack"] * len(positions), positions)),
        frame_key=(frame, 0), stamp_sec=float(frame),
    )


def test_nearby_same_class_targets_remain_distinct_when_order_changes():
    tracker = make_tracker()
    first = observe_frame(tracker, [(0., 0., 0.), (0.3, 0., 0.)], 1)
    ids = [track.event_id for track, _ in first]
    assert len(set(ids)) == 2
    second = observe_frame(tracker, [(0.31, 0., 0.), (0.01, 0., 0.)], 2)
    assert [track.event_id for track, _ in second] == ids[::-1]
    third = observe_frame(tracker, [(0.02, 0., 0.), (0.32, 0., 0.)], 3)
    assert [track.event_id for track, _ in third] == ids
    assert all(track.hit_count == 3 and due for track, due in third)


def test_assignment_preserves_matches_when_nearest_greedy_would_steal_track():
    tracker = make_tracker()
    first = observe_frame(tracker, [(0., 0., 0.), (0.6, 0., 0.)], 1)
    # First observation can use either track; the second can only use the left.
    second = observe_frame(tracker, [(0.2, 0., 0.), (-0.4, 0., 0.)], 2)
    assert second[0][0] is first[1][0]
    assert second[1][0] is first[0][0]
    assert len(tracker.tracks) == 2


def test_existing_track_cannot_be_used_twice_and_classes_are_gated():
    tracker = make_tracker()
    original = observe_frame(tracker, [(0., 0., 0.)], 1)[0][0]
    results = observe_frame(
        tracker, [(0.1, 0., 0.), (0.2, 0., 0.), (0., 0., 0.)], 2,
        ["crack", "crack", "leak"],
    )
    assert len({track.event_id for track, _ in results}) == 3
    assert results[0][0] is original
    assert [track.hit_count for track, _ in results] == [2, 1, 1]


def test_duplicate_and_delayed_frames_do_not_change_tracks_or_republish():
    tracker = make_tracker()
    tracker.republish_period_sec = 0.0
    positions = [(0., 0., 0.), (0.3, 0., 0.)]
    for frame in (1, 2, 3):
        observe_frame(tracker, positions, frame)
    before = [vars(track).copy() for track in tracker.tracks]
    for frame in (3, 1, 2, 3):
        assert observe_frame(tracker, positions + [(99., 0., 0.)], frame) == []
        assert [vars(track).copy() for track in tracker.tracks] == before


@pytest.mark.parametrize("position", [(float("nan"), 0., 0.), (1., 2.)])
def test_invalid_batch_does_not_partially_update_tracker(position):
    tracker = make_tracker()
    with pytest.raises(ValueError, match="coordinates"):
        observe_frame(tracker, [(0., 0., 0.), position], 1)
    assert tracker.tracks == []
    assert len(observe_frame(tracker, [(0., 0., 0.)], 1)) == 1
