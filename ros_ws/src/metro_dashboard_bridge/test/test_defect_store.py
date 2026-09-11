import pytest

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


def test_records_persist_after_database_is_reopened(tmp_path) -> None:
    database_path = tmp_path / "defects.sqlite3"
    first_store = DefectStore(database_path=database_path, session_id="run-1")
    first_store.upsert(
        {
            "event_id": "event-1",
            "type": "crack",
            "position": {"x": 1.25, "tags": ["rail", "left"]},
        }
    )
    first_store.close()

    reopened_store = DefectStore(database_path=database_path, session_id="run-1")
    assert reopened_store.count() == 1
    assert reopened_store.get("event-1")["position"] == {
        "x": 1.25,
        "tags": ["rail", "left"],
    }
    reopened_store.close()


def test_sessions_are_isolated_in_the_same_database(tmp_path) -> None:
    database_path = tmp_path / "defects.sqlite3"
    first_session = DefectStore(database_path=database_path, session_id="run-1")
    first_session.upsert({"event_id": "event-1", "value": "first"})
    first_session.close()

    second_session = DefectStore(database_path=database_path, session_id="run-2")
    assert second_session.count() == 0
    second_session.upsert({"event_id": "event-1", "value": "second"})
    assert second_session.list(session_id="run-1")[0]["value"] == "first"
    assert second_session.get("event-1", session_id="run-1")["value"] == "first"
    assert second_session.count(session_id="run-1") == 1
    sessions = second_session.list_sessions()
    assert [session["session_id"] for session in sessions] == ["run-2", "run-1"]
    assert sessions[0]["active"] is True
    assert sessions[1]["active"] is False
    assert sessions[1]["record_count"] == 1
    second_session.close()

    reopened_first = DefectStore(database_path=database_path, session_id="run-1")
    assert reopened_first.get("event-1")["value"] == "first"
    reopened_first.close()


def test_import_records_preserves_existing_timestamps(tmp_path) -> None:
    store = DefectStore(database_path=tmp_path / "defects.sqlite3")
    record = {
        "event_id": "event-1",
        "created_at": "2026-08-01T01:02:03+00:00",
        "updated_at": "2026-08-01T02:03:04+00:00",
    }

    assert store.import_records([record]) == 1
    imported = store.get("event-1")
    assert imported["created_at"] == record["created_at"]
    assert imported["updated_at"] == record["updated_at"]
    store.close()


@pytest.mark.parametrize("event_id", ["", "   ", None])
def test_upsert_rejects_empty_event_id(event_id) -> None:
    store = DefectStore()
    with pytest.raises(ValueError, match="event_id"):
        store.upsert({"event_id": event_id})
    store.close()
