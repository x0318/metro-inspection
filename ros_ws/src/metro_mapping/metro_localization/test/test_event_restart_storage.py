"""Verify event identity across separate processes against the dashboard store."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys


def test_restarted_producer_does_not_overwrite_same_session_database(tmp_path):
    source_root = Path(__file__).resolve().parents[3]
    store_path = (source_root / "metro_dashboard_bridge" /
                  "metro_dashboard_bridge" / "defect_store.py")
    spec = importlib.util.spec_from_file_location("restart_test_store", store_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    database = tmp_path / "events.sqlite3"
    store = module.DefectStore(database_path=database, session_id="same-inspection")
    store.upsert({"event_id": "odin1-0001", "value": "legacy record"})
    store.close()

    script = """
import json
from metro_localization.spatial_event_tracking import SpatialEventTracker
tracker = SpatialEventTracker(distance_threshold_m=0.5, confirmation_hits=3,
                              republish_period_sec=2.0, event_prefix="odin1")
records = []
for stamp in (1, 2, 3, 5):
    results = tracker.observe_frame(
        observations=[("crack", (0., 0., 2.)), ("crack", (0.3, 0., 2.))],
        frame_key=(stamp, 0), stamp_sec=float(stamp))
    records.extend({"event_id": track.event_id, "value": stamp}
                   for track, due in results if due)
print(json.dumps(records))
"""
    env = os.environ.copy()
    package_root = str(Path(__file__).resolve().parents[1])
    env["PYTHONPATH"] = os.pathsep.join([package_root, env.get("PYTHONPATH", "")])
    previous_ids = set()
    for generation in range(2):
        result = subprocess.run(
            [sys.executable, "-c", script], env=env,
            capture_output=True, text=True, check=True, timeout=20,
        )
        records = json.loads(result.stdout)
        assert len(records) == 4
        ids = {record["event_id"] for record in records}
        assert len(ids) == 2
        assert ids.isdisjoint(previous_ids)
        store = module.DefectStore(database_path=database, session_id="same-inspection")
        try:
            for record in records:
                store.upsert(record)
            assert store.count() == 1 + 2 * (generation + 1)
            assert store.get("odin1-0001")["value"] == "legacy record"
            assert all(store.get(event_id)["value"] == 5 for event_id in ids | previous_ids)
        finally:
            store.close()
        previous_ids.update(ids)
