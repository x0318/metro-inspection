import copy
import threading
from collections import OrderedDict
from datetime import datetime, timezone
from typing import Dict, List, Optional


class DefectStore:
    """Thread-safe, bounded event store with event_id based updates."""

    def __init__(self, maximum_records: int = 500) -> None:
        if maximum_records <= 0:
            raise ValueError("maximum_records must be positive")
        self._maximum_records = maximum_records
        self._lock = threading.Lock()
        self._records: "OrderedDict[str, Dict[str, object]]" = OrderedDict()

    def upsert(self, record: Dict[str, object]) -> Dict[str, object]:
        event_id = str(record.get("event_id", "")).strip()
        if not event_id:
            raise ValueError("record event_id must not be empty")

        now = datetime.now(timezone.utc).isoformat()
        stored = copy.deepcopy(record)
        stored["id"] = event_id
        stored["event_id"] = event_id

        with self._lock:
            previous = self._records.get(event_id)
            stored["created_at"] = (
                previous["created_at"] if previous is not None else now
            )
            stored["updated_at"] = now
            self._records[event_id] = stored
            self._records.move_to_end(event_id, last=False)
            while len(self._records) > self._maximum_records:
                self._records.popitem(last=True)
            return copy.deepcopy(stored)

    def list(self) -> List[Dict[str, object]]:
        with self._lock:
            return copy.deepcopy(list(self._records.values()))

    def get(self, event_id: str) -> Optional[Dict[str, object]]:
        with self._lock:
            record = self._records.get(event_id)
            return copy.deepcopy(record) if record is not None else None

    def count(self) -> int:
        with self._lock:
            return len(self._records)
