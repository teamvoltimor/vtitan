"""Storage backends for telemetry recording (JSONL, CSV, in-memory, etc).

Separates the Recorder (what to record) from the Backend (how/where to store it).
Allows swapping storage formats without touching TelemetryRecorder.

Usage:
    recorder = TelemetryRecorder(base_dir, backend=JSONLBackend(base_dir))
    recorder.record(snapshot)  # Recorder doesn't care about format
"""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from src.telemetry.exceptions import RecorderError, SessionNotFoundError
from src.telemetry.models import RobotSnapshot

if TYPE_CHECKING:
    from collections.abc import Sequence

logger = logging.getLogger(__name__)


class RecordBackend(Protocol):
    """Protocol for telemetry storage backends.

    Implementations handle serialization, file naming, persistence,
    session management. Recorder delegates all I/O to the backend.
    """

    def write(self, snapshot: RobotSnapshot) -> None:
        """Persist a snapshot to storage.

        Args:
            snapshot: RobotSnapshot to store.

        Raises:
            RecorderError: If write fails.
        """
        ...

    def read_session(self, session_id: str) -> Sequence[RobotSnapshot]:
        """Load all snapshots from a session.

        Args:
            session_id: Session identifier.

        Returns:
            Sequence of RobotSnapshot objects.

        Raises:
            SessionNotFoundError: If session doesn't exist.
            RecorderError: If read fails.
        """
        ...

    def list_sessions(self) -> Sequence[tuple[str, float, int]]:
        """List all available sessions.

        Returns:
            Sequence of (session_id, created_at, entry_count).
        """
        ...

    def evict_old_sessions(self, max_count: int) -> None:
        """Delete oldest sessions exceeding max_count.

        Args:
            max_count: Maximum sessions to retain.
        """
        ...

    def close(self) -> None:
        """Close any open resources."""
        ...


class JSONLBackend:
    """JSONL file-based storage backend (default, current implementation).

    Stores snapshots as newline-delimited JSON, one per line, in session files.
    Session filename: `{session_id}.jsonl`
    """

    def __init__(
        self,
        base_dir: Path | str,
        session_id: str,
        max_sessions: int = 20,
    ) -> None:
        """Initialize JSONL backend.

        Args:
            base_dir: Directory for session files.
            session_id: Current session identifier.
            max_sessions: Maximum sessions to retain (for eviction).
        """
        self._base_dir = Path(base_dir)
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._session_id = session_id
        self._file_path = self._base_dir / f"{session_id}.jsonl"
        self._entry_count = 0
        self._file = None
        self._max_sessions = max_sessions

    @property
    def entry_count(self) -> int:
        """Return the number of entries recorded in current session."""
        return self._entry_count

    @property
    def file(self) -> object:
        """Access the underlying file handle."""
        return self._file

    def ensure_file_open(self) -> None:
        """Lazily open the recording file on first write."""
        if self._file is None:
            self._file_path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self._file_path.open("a", encoding="utf-8")
            self.evict_old_sessions(self._max_sessions)

    def write(self, snapshot: RobotSnapshot) -> None:
        """Write snapshot as JSON line.

        Args:
            snapshot: RobotSnapshot to persist.

        Raises:
            RecorderError: If write fails.
        """
        self.ensure_file_open()
        try:
            json.dump(snapshot.model_dump(by_alias=True), self._file)
            self._file.write("\n")
            self._file.flush()
            self._entry_count += 1
        except OSError as exc:
            logger.exception(
                "Failed to record snapshot to disk",
                exc_info=exc,
                extra={
                    "session_id": self._session_id,
                    "path": str(self._file_path),
                    "errno": exc.errno,
                },
            )
            msg = f"Failed to persist snapshot: {exc}"
            raise RecorderError(msg) from exc
        except (json.JSONDecodeError, ValueError) as exc:
            logger.exception("Snapshot serialization failed", exc_info=exc)
            msg = f"Cannot serialize snapshot: {exc}"
            raise RecorderError(msg) from exc

    def read_session(self, session_id: str) -> Sequence[RobotSnapshot]:
        """Load all snapshots from JSONL session file.

        Args:
            session_id: Session identifier.

        Returns:
            Sequence of RobotSnapshot objects.

        Raises:
            SessionNotFoundError: If session file doesn't exist.
            RecorderError: If file can't be read or contains corrupted data.
        """
        path = self._base_dir / f"{session_id}.jsonl"
        if not path.exists():
            msg = f"Session not found: {session_id}"
            raise SessionNotFoundError(msg)

        snapshots: list[RobotSnapshot] = []
        try:
            with path.open("r", encoding="utf-8") as fh:
                for line_num, line in enumerate(fh, start=1):
                    text = line.strip()
                    if not text:
                        continue
                    try:
                        snapshots.append(RobotSnapshot.model_validate_json(text))
                    except (json.JSONDecodeError, ValueError) as exc:
                        logger.warning(
                            "Skipping corrupted line %s in session %s",
                            line_num,
                            session_id,
                            exc_info=exc,
                            extra={"path": str(path), "line_num": line_num},
                        )
                        continue
        except OSError as exc:
            logger.exception(
                "Failed to read session file %s",
                session_id,
                exc_info=exc,
                extra={"path": str(path)},
            )
            msg = f"Cannot read session file: {exc}"
            raise RecorderError(msg) from exc

        return snapshots

    def list_sessions(self) -> Sequence[tuple[str, float, int]]:
        """List all JSONL session files with metadata.

        Returns:
            Sequence of (session_id, created_at, entry_count).
        """
        sessions: list[tuple[str, float, int]] = []
        for path in sorted(self._base_dir.glob("session_*.jsonl"), reverse=True):
            stat = path.stat()
            entry_count = path.read_bytes().count(b"\n")
            sessions.append((path.stem, stat.st_ctime, entry_count))
        return sessions

    def evict_old_sessions(self, max_count: int) -> None:
        """Delete oldest JSONL files when exceeding max_count.

        Args:
            max_count: Maximum sessions to retain.
        """
        try:
            sessions = sorted(self._base_dir.glob("session_*.jsonl"))
            to_delete = sessions[: max(0, len(sessions) - max_count)]

            for path in to_delete:
                try:
                    path.unlink()
                    logger.info("Evicted old session: %s", path.stem)
                except OSError as exc:
                    logger.warning(
                        "Failed to evict session file",
                        exc_info=exc,
                        extra={"path": str(path)},
                    )
        except Exception as exc:
            logger.exception("Session eviction failed", exc_info=exc)

    def close(self) -> None:
        """Close the open file handle (keep reference for testing)."""
        if self.file is not None:
            self._file.close()


class InMemoryBackend:
    """In-memory storage backend for unit testing.

    Stores snapshots in RAM. Useful for tests that can't use disk I/O.
    """

    def __init__(self) -> None:
        """Initialize in-memory backend."""
        self._sessions: dict[str, list[RobotSnapshot]] = {}
        self._session_created: dict[str, float] = {}

    def write(self, snapshot: RobotSnapshot, session_id: str = "default") -> None:
        """Store snapshot in memory under session_id.

        Args:
            snapshot: RobotSnapshot to store.
            session_id: Session identifier (default: "default").
        """
        if session_id not in self._sessions:
            self._sessions[session_id] = []
            self._session_created[session_id] = time.time()
        self._sessions[session_id].append(snapshot)

    def read_session(self, session_id: str) -> Sequence[RobotSnapshot]:
        """Retrieve all snapshots from a session.

        Args:
            session_id: Session identifier.

        Returns:
            Sequence of RobotSnapshot objects.

        Raises:
            SessionNotFoundError: If session doesn't exist.
        """
        if session_id not in self._sessions:
            msg = f"Session not found: {session_id}"
            raise SessionNotFoundError(msg)
        return self._sessions[session_id]

    def list_sessions(self) -> Sequence[tuple[str, float, int]]:
        """List all in-memory sessions.

        Returns:
            Sequence of (session_id, created_at, entry_count).
        """
        return [
            (sid, self._session_created[sid], len(self._sessions[sid]))
            for sid in sorted(self._sessions.keys(), reverse=True)
        ]

    def evict_old_sessions(self, max_count: int) -> None:
        """Delete oldest sessions exceeding max_count.

        Args:
            max_count: Maximum sessions to retain.
        """
        if len(self._sessions) > max_count:
            to_delete = sorted(self._sessions.keys())[: len(self._sessions) - max_count]
            for sid in to_delete:
                del self._sessions[sid]
                del self._session_created[sid]

    def close(self) -> None:
        """No-op for in-memory backend."""

    def clear(self) -> None:
        """Clear all sessions (test utility)."""
        self._sessions.clear()
        self._session_created.clear()
