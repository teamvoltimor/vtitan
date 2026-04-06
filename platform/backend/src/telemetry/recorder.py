"""Persist telemetry snapshots to disk for replay sessions."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import IO, TYPE_CHECKING

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel

from src.telemetry.exceptions import RecorderError, SessionNotFoundError
from src.telemetry.models import RobotSnapshot

if TYPE_CHECKING:
    from collections.abc import Sequence
    from typing import Self

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


_DEFAULT_MAX_SESSIONS = 20


class TelemetryRecorder:
    """Persist telemetry frames to disk and enumerate replay history."""

    def __init__(
        self,
        base_dir: Path | str,
        session_id: str | None = None,
        max_sessions: int = _DEFAULT_MAX_SESSIONS,
    ) -> None:
        base_dir = Path(base_dir)
        base_dir.mkdir(parents=True, exist_ok=True)
        self._base_dir = base_dir
        self._max_sessions = max_sessions
        self._session_id = session_id or f"session_{int(time.time())}"
        self._file_path = self._base_dir / f"{self._session_id}.jsonl"
        self._entry_count = 0
        self._file: IO[str] | None = None

    def __enter__(self) -> Self:
        """Return the recorder while entering the context manager."""
        return self

    def __exit__(self, *_: object) -> None:
        """Ensure the file handle closes when exiting context."""
        self.close()

    @property
    def session_id(self) -> str:
        """Return the active replay session identifier."""
        return self._session_id

    @property
    def entry_count(self) -> int:
        """Return the number of entries recorded so far."""
        return self._entry_count

    def _ensure_file_open(self) -> None:
        """Lazily open the recording file on first write.

        This delays file creation until actually needed, so empty
        sessions are not created if nothing is ever recorded.
        """
        if self._file is None:
            self._file_path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self._file_path.open("a", encoding="utf-8")
            self._evict_old_sessions()

    def record(self, snapshot: RobotSnapshot) -> None:
        """Append a serialized snapshot to the open recording file.

        Args:
            snapshot: RobotSnapshot to persist.

        Raises:
            RecorderError: If write operation fails.
        """
        self._ensure_file_open()
        try:
            json.dump(snapshot.model_dump(by_alias=True), self._file)
            self._file.write("\n")
            self._file.flush()
            self._entry_count += 1
        except OSError as exc:
            logger.error(
                "Failed to record snapshot to disk",
                exc_info=exc,
                extra={
                    "session_id": self._session_id,
                    "path": str(self._file_path),
                    "errno": exc.errno,
                },
            )
            raise RecorderError(f"Failed to persist snapshot: {exc}") from exc
        except (json.JSONDecodeError, ValueError) as exc:
            logger.error("Snapshot serialization failed", exc_info=exc)
            raise RecorderError(f"Cannot serialize snapshot: {exc}") from exc

    def close(self) -> None:
        """Close the associated file handle if it's open."""
        if self._file is not None:
            self._file.close()

    def _evict_old_sessions(self) -> None:
        """Delete the oldest sessions when the total exceeds max_sessions.

        Logs warnings for individual file failures but continues eviction.
        """
        try:
            sessions = sorted(self._base_dir.glob("session_*.jsonl"))
            to_delete = sessions[: max(0, len(sessions) - self._max_sessions)]

            for path in to_delete:
                try:
                    path.unlink()
                    logger.info(f"Evicted old session: {path.stem}")
                except OSError as exc:
                    logger.warning(
                        "Failed to evict session file",
                        exc_info=exc,
                        extra={"path": str(path)},
                    )
        except Exception as exc:
            logger.error("Session eviction failed", exc_info=exc)

    def list_sessions(self) -> Sequence[ReplaySessionInfo]:
        """Return metadata for every recorded session in the base directory."""
        return self.enumerate_sessions(self._base_dir)

    @classmethod
    def enumerate_sessions(cls, base_dir: Path | str) -> Sequence[ReplaySessionInfo]:
        """Return metadata for sessions in *base_dir*, sorted newest first."""
        directory = Path(base_dir)
        directory.mkdir(parents=True, exist_ok=True)
        sessions: list[ReplaySessionInfo] = []
        for path in sorted(directory.glob("session_*.jsonl"), reverse=True):
            stat = path.stat()
            sessions.append(
                ReplaySessionInfo(
                    session_id=path.stem,
                    created_at=stat.st_ctime,
                    entry_count=cls._count_lines(path),
                ),
            )
        return sessions

    def load_session(self, session_id: str) -> Sequence[RobotSnapshot]:
        """Return snapshots stored in session_id.

        Args:
            session_id: Session identifier to load.

        Returns:
            Sequence of RobotSnapshot objects from the session.

        Raises:
            SessionNotFoundError: If session file does not exist.
            RecorderError: If session file cannot be read or contains corrupted data.
        """
        path = self._base_dir / f"{session_id}.jsonl"
        if not path.exists():
            raise SessionNotFoundError(f"Session not found: {session_id}")

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
                            f"Skipping corrupted line {line_num} in session {session_id}",
                            exc_info=exc,
                            extra={"path": str(path), "line_num": line_num},
                        )
                        # Continue on corrupted lines to maximize recovery
                        continue
        except OSError as exc:
            logger.error(
                f"Failed to read session file {session_id}",
                exc_info=exc,
                extra={"path": str(path)},
            )
            raise RecorderError(f"Cannot read session file: {exc}") from exc

        return snapshots

    @staticmethod
    def _count_lines(path: Path) -> int:
        """Return the number of non-empty lines in *path* if it exists."""
        if not path.exists():
            return 0
        return path.read_bytes().count(b"\n")
