import copy
import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Union


DatabasePath = Union[str, Path]


class DefectStore:
    """Thread-safe, bounded SQLite event store with event_id based updates."""

    def __init__(
        self,
        maximum_records: int = 500,
        database_path: DatabasePath = ":memory:",
        session_id: str = "simulation",
    ) -> None:
        if maximum_records <= 0:
            raise ValueError("maximum_records must be positive")

        normalized_session_id = str(session_id).strip()
        if not normalized_session_id:
            raise ValueError("session_id must not be empty")

        normalized_path = str(database_path).strip()
        if not normalized_path:
            raise ValueError("database_path must not be empty")
        if normalized_path != ":memory:":
            path = Path(normalized_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            normalized_path = str(path)

        self._maximum_records = maximum_records
        self._database_path = normalized_path
        self._session_id = normalized_session_id
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            self._database_path,
            timeout=5.0,
            check_same_thread=False,
        )
        self._connection.row_factory = sqlite3.Row
        self._configure_database()
        self._initialize_schema()

    @property
    def database_path(self) -> str:
        return self._database_path

    @property
    def session_id(self) -> str:
        return self._session_id

    def _configure_database(self) -> None:
        with self._lock:
            self._connection.execute("PRAGMA foreign_keys = ON")
            self._connection.execute("PRAGMA busy_timeout = 5000")
            self._connection.execute("PRAGMA journal_mode = WAL")
            self._connection.execute("PRAGMA synchronous = NORMAL")

    def _initialize_schema(self) -> None:
        now = self._timestamp()
        with self._lock, self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS inspection_sessions (
                    session_id TEXT PRIMARY KEY,
                    started_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS defect_events (
                    session_id TEXT NOT NULL,
                    event_id TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    PRIMARY KEY (session_id, event_id),
                    FOREIGN KEY (session_id)
                        REFERENCES inspection_sessions(session_id)
                        ON DELETE CASCADE
                );

                CREATE INDEX IF NOT EXISTS defect_events_session_updated_idx
                    ON defect_events(session_id, updated_at DESC);
                """
            )
            self._connection.execute(
                """
                INSERT INTO inspection_sessions(session_id, started_at, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(session_id) DO UPDATE SET updated_at = excluded.updated_at
                """,
                (self._session_id, now, now),
            )

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _event_id(record: Dict[str, object]) -> str:
        raw_event_id = record.get("event_id")
        event_id = "" if raw_event_id is None else str(raw_event_id).strip()
        if not event_id:
            raise ValueError("record event_id must not be empty")
        return event_id

    @staticmethod
    def _serialize(record: Dict[str, object]) -> str:
        try:
            return json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            )
        except (TypeError, ValueError) as error:
            raise ValueError(f"record must be valid JSON: {error}") from error

    @staticmethod
    def _deserialize(payload: str) -> Dict[str, object]:
        record = json.loads(payload)
        if not isinstance(record, dict):
            raise ValueError("stored defect payload must be a JSON object")
        return record

    def _previous_created_at(self, event_id: str) -> Optional[str]:
        row = self._connection.execute(
            """
            SELECT created_at
            FROM defect_events
            WHERE session_id = ? AND event_id = ?
            """,
            (self._session_id, event_id),
        ).fetchone()
        return str(row["created_at"]) if row is not None else None

    def _write_record(
        self,
        record: Dict[str, object],
        *,
        preserve_timestamps: bool,
    ) -> Dict[str, object]:
        event_id = self._event_id(record)
        now = self._timestamp()
        stored = copy.deepcopy(record)
        stored["id"] = event_id
        stored["event_id"] = event_id

        previous_created_at = self._previous_created_at(event_id)
        imported_created_at = str(stored.get("created_at", "")).strip()
        imported_updated_at = str(stored.get("updated_at", "")).strip()
        stored["created_at"] = (
            previous_created_at
            or (imported_created_at if preserve_timestamps else "")
            or now
        )
        stored["updated_at"] = (
            imported_updated_at if preserve_timestamps and imported_updated_at else now
        )
        payload = self._serialize(stored)

        self._connection.execute(
            """
            INSERT INTO defect_events(
                session_id, event_id, created_at, updated_at, payload_json
            )
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(session_id, event_id) DO UPDATE SET
                updated_at = excluded.updated_at,
                payload_json = excluded.payload_json
            """,
            (
                self._session_id,
                event_id,
                stored["created_at"],
                stored["updated_at"],
                payload,
            ),
        )
        self._connection.execute(
            "UPDATE inspection_sessions SET updated_at = ? WHERE session_id = ?",
            (now, self._session_id),
        )
        self._prune()
        return copy.deepcopy(stored)

    def _prune(self) -> None:
        self._connection.execute(
            """
            DELETE FROM defect_events
            WHERE rowid IN (
                SELECT rowid
                FROM defect_events
                WHERE session_id = ?
                ORDER BY updated_at DESC, rowid DESC
                LIMIT -1 OFFSET ?
            )
            """,
            (self._session_id, self._maximum_records),
        )

    def upsert(self, record: Dict[str, object]) -> Dict[str, object]:
        with self._lock, self._connection:
            return self._write_record(record, preserve_timestamps=False)

    def import_records(self, records: Iterable[Dict[str, object]]) -> int:
        """Import an existing snapshot while retaining its timestamps."""
        imported = 0
        with self._lock, self._connection:
            for record in records:
                self._write_record(record, preserve_timestamps=True)
                imported += 1
        return imported

    def list(self) -> List[Dict[str, object]]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT payload_json
                FROM defect_events
                WHERE session_id = ?
                ORDER BY updated_at DESC, rowid DESC
                """,
                (self._session_id,),
            ).fetchall()
            return [self._deserialize(str(row["payload_json"])) for row in rows]

    def get(self, event_id: str) -> Optional[Dict[str, object]]:
        normalized_event_id = str(event_id).strip()
        if not normalized_event_id:
            return None
        with self._lock:
            row = self._connection.execute(
                """
                SELECT payload_json
                FROM defect_events
                WHERE session_id = ? AND event_id = ?
                """,
                (self._session_id, normalized_event_id),
            ).fetchone()
            return (
                self._deserialize(str(row["payload_json"]))
                if row is not None
                else None
            )

    def count(self) -> int:
        with self._lock:
            row = self._connection.execute(
                "SELECT COUNT(*) AS count FROM defect_events WHERE session_id = ?",
                (self._session_id,),
            ).fetchone()
            return int(row["count"])

    def close(self) -> None:
        with self._lock:
            if self._connection is not None:
                self._connection.close()
                self._connection = None
