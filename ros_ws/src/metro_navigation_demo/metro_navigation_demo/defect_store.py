import copy
import threading
import uuid
from datetime import datetime, timezone
from typing import Dict, List


class DefectStore:
    """Bounded in-memory store for real detector submissions."""

    def __init__(self, maximum_records: int = 500) -> None:
        if maximum_records <= 0:
            raise ValueError('maximum_records must be positive')
        self._maximum_records = maximum_records
        self._lock = threading.Lock()
        self._records: List[Dict[str, object]] = []

    def add(self, record: Dict[str, object]) -> Dict[str, object]:
        stored = copy.deepcopy(record)
        stored['id'] = str(uuid.uuid4())
        stored['created_at'] = datetime.now(timezone.utc).isoformat()
        with self._lock:
            self._records.insert(0, stored)
            del self._records[self._maximum_records:]
        return copy.deepcopy(stored)

    def list(self) -> List[Dict[str, object]]:
        with self._lock:
            return copy.deepcopy(self._records)
