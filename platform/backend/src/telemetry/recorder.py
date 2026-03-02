"""Persist telemetry snapshots to disk for replay sessions."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import IO, TYPE_CHECKING

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel

from src.telemetry.models import RobotSnapshot

if TYPE_CHECKING:
    from collections.abc import Sequence
    from typing import Self


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
    """Persist telemetry frames to disk and enumerate replay history."""

    def __init__(self, base_dir: Path | str, session_id: str | None = None) -> None:
        base_dir = Path(base_dir)
        base_dir.mkdir(parents=True, exist_ok=True)
        self._base_dir = base_dir
        self._session_id = session_id or f"session_{int(time.time())}"
        self._file_path = self._base_dir / f"{self._session_id}.jsonl"
        self._entry_count = self._count_lines(self._file_path)
        self._file: IO[str] = self._file_path.open("a", encoding="utf-8")

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

    def record(self, snapshot: RobotSnapshot) -> None:
        """Append a serialized snapshot to the open recording file."""
        json.dump(snapshot.model_dump(by_alias=True), self._file)
        self._file.write("\n")
        self._file.flush()
        self._entry_count += 1

    def close(self) -> None:
        """Close the associated file handle."""
        self._file.close()

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
        """Return snapshots stored in *session_id*."""
        path = self._base_dir / f"{session_id}.jsonl"
        if not path.exists():
            raise FileNotFoundError(session_id)
        snapshots: list[RobotSnapshot] = []
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                text = line.strip()
                if not text:
                    continue
                snapshots.append(RobotSnapshot.model_validate_json(text))
        return snapshots

    @staticmethod
    def _count_lines(path: Path) -> int:
        """Return the number of lines in *path* if it exists."""
        if not path.exists():
            return 0
        with path.open("r", encoding="utf-8") as fh:
            return sum(1 for _ in fh)
