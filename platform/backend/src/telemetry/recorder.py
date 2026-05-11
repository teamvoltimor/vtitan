"""Persist telemetry snapshots to disk for replay sessions.

TelemetryRecorder delegates storage to pluggable backends (JSONL, CSV, in-memory, etc).
Recorder only manages the active session; backend handles serialization and I/O.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel

from src.telemetry.config import RecorderConfig
from src.telemetry.record_backend import JSONLBackend, RecordBackend

if TYPE_CHECKING:
    from collections.abc import Sequence
    from typing import Self

    from src.telemetry.models import RobotSnapshot

logger = logging.getLogger(__name__)


class ReplaySessionInfo(BaseModel):
    """Metadata exposed for each recorder replay session."""

    session_id: str
    created_at: float
    entry_count: int

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class TelemetryRecorder:
    """Persist telemetry frames using pluggable storage backends.

    Delegates all I/O to backend (RecordBackend protocol).
    Recorder only manages sessions and delegates writes/reads.
    """

    def __init__(
        self,
        base_dir: Path | str,
        session_id: str | None = None,
        max_sessions: int = RecorderConfig.DEFAULT_MAX_SESSIONS,
        backend: RecordBackend | None = None,
    ) -> None:
        """Initialize recorder with optional backend.

        Args:
            base_dir: Directory for session storage.
            session_id: Session identifier (auto-generated if not provided).
            max_sessions: Maximum sessions to retain.
            backend: Storage backend (defaults to JSONLBackend).
        """
        base_dir = Path(base_dir)
        base_dir.mkdir(parents=True, exist_ok=True)
        self._base_dir = base_dir
        self._max_sessions = max_sessions
        self._session_id = session_id or f"session_{int(time.time())}"
        if backend is None:
            backend = JSONLBackend(base_dir, self._session_id, max_sessions)
        self._backend = backend

    def __enter__(self) -> Self:
        """Return the recorder while entering the context manager."""
        return self

    def __exit__(self, *_: object) -> None:
        """Close backend resources when exiting context."""
        self.close()

    @property
    def session_id(self) -> str:
        """Return the active replay session identifier."""
        return self._session_id

    @property
    def max_sessions(self) -> int:
        """Return the maximum number of sessions to retain."""
        return self._max_sessions

    @property
    def sessions_dir(self) -> Path:
        """Return the base directory for session storage."""
        return self._base_dir

    @property
    def entry_count(self) -> int:
        """Return the number of entries recorded in current session.

        Note: For JSONL backend, this is tracked locally. Other backends
        may calculate it dynamically.
        """
        if isinstance(self._backend, JSONLBackend):
            return self._backend.entry_count
        return 0

    @property
    def file(self) -> object:
        """Access backend file for test compatibility.

        Test utility property for checking file closed state.
        """
        if isinstance(self._backend, JSONLBackend):
            return self._backend.file
        return None

    def ensure_file_open(self) -> None:
        """Ensure backend is ready (for test compatibility).

        For JSONL backend, this opens the file. Other backends may be no-op.
        """
        if isinstance(self._backend, JSONLBackend):
            self._backend.ensure_file_open()

    def record(self, snapshot: RobotSnapshot) -> None:
        """Delegate snapshot write to backend.

        Args:
            snapshot: RobotSnapshot to persist.

        Raises:
            RecorderError: If write operation fails.
        """
        self._backend.write(snapshot)

    def close(self) -> None:
        """Close backend resources."""
        self._backend.close()

    def list_sessions(self) -> Sequence[ReplaySessionInfo]:
        """Return metadata for all sessions from backend.

        Returns:
            Sequence of ReplaySessionInfo sorted by creation time (newest first).
        """
        sessions: list[ReplaySessionInfo] = []
        for session_id, created_at, entry_count in self._backend.list_sessions():
            sessions.append(
                ReplaySessionInfo(
                    session_id=session_id,
                    created_at=created_at,
                    entry_count=entry_count,
                ),
            )
        return sessions

    def load_session(self, session_id: str) -> Sequence[RobotSnapshot]:
        """Load snapshots from backend.

        Args:
            session_id: Session identifier to load.

        Returns:
            Sequence of RobotSnapshot objects from the session.

        Raises:
            SessionNotFoundError: If session does not exist.
            RecorderError: If session cannot be read or is corrupted.
        """
        return self._backend.read_session(session_id)

    @classmethod
    def enumerate_sessions(cls, base_dir: Path | str) -> Sequence[ReplaySessionInfo]:
        """Enumerate all sessions in a directory using JSONL backend.

        Utility method for listing sessions without creating a recorder instance.

        Args:
            base_dir: Directory containing session files.

        Returns:
            Sequence of ReplaySessionInfo sorted by creation time (newest first).
        """
        backend = JSONLBackend(
            base_dir,
            "",
            max_sessions=RecorderConfig.DEFAULT_MAX_SESSIONS,
        )
        sessions: list[ReplaySessionInfo] = []
        for session_id, created_at, entry_count in backend.list_sessions():
            sessions.append(
                ReplaySessionInfo(
                    session_id=session_id,
                    created_at=created_at,
                    entry_count=entry_count,
                ),
            )
        return sessions
