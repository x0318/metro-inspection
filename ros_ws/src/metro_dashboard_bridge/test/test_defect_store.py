from metro_dashboard_bridge.defect_store import DefectStore


def test_upsert_updates_existing_event_and_preserves_creation_time() -> None:
    store = DefectStore(maximum_records=2)
    first = store.upsert({"event_id": "event-1", "value": 1})
    updated = store.upsert({"event_id": "event-1", "value": 2})

    assert store.count() == 1
    assert store.get("event-1")["value"] == 2
    assert updated["created_at"] == first["created_at"]


def test_store_discards_oldest_event_when_bounded() -> None:
    store = DefectStore(maximum_records=2)
    store.upsert({"event_id": "event-1"})
    store.upsert({"event_id": "event-2"})
    store.upsert({"event_id": "event-3"})

    assert [record["event_id"] for record in store.list()] == [
        "event-3",
        "event-2",
    ]
    assert store.get("event-1") is None
