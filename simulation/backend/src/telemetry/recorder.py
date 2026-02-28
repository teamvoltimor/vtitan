"""Persist telemetry snapshots to disk for replay sessions."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import IO, Sequence

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel

from src.telemetry.models import SimulationSnapshot


class ReplaySessionInfo(BaseModel):
    session_id: str
    created_at: float
    entry_count: int

    model_config = ConfigDict(
        frozen=True,
        alias_generator=to_camel,
        populate_by_name=True,
    )


class TelemetryRecorder:
    """Appends telemetry frames to a JSONL file and surfaces stored sessions."""

    def __init__(self, base_dir: Path | str, session_id: str | None = None) -> None:
        base_dir = Path(base_dir)
        base_dir.mkdir(parents=True, exist_ok=True)
        self._base_dir = base_dir
        self._session_id = session_id or f"session_{int(time.time())}"
        self._file_path = self._base_dir / f"{self._session_id}.jsonl"
        self._entry_count = self._count_lines(self._file_path)
        self._file: IO[str] = self._file_path.open("a", encoding="utf-8")

    def __enter__(self) -> TelemetryRecorder:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def entry_count(self) -> int:
        return self._entry_count

    def record(self, snapshot: SimulationSnapshot) -> None:
        json.dump(snapshot.model_dump(by_alias=True), self._file)
        self._file.write("\n")
        self._file.flush()
        self._entry_count += 1

    def close(self) -> None:
        self._file.close()

    def list_sessions(self) -> Sequence[ReplaySessionInfo]:
        return self.enumerate_sessions(self._base_dir)

    @classmethod
    def enumerate_sessions(cls, base_dir: Path | str) -> Sequence[ReplaySessionInfo]:
        """Return metadata for all recorded sessions in *base_dir*, newest first."""
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
                )
            )
        return sessions

    def load_session(self, session_id: str) -> Sequence[SimulationSnapshot]:
        path = self._base_dir / f"{session_id}.jsonl"
        if not path.exists():
            raise FileNotFoundError(session_id)
        snapshots: list[SimulationSnapshot] = []
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                text = line.strip()
                if not text:
                    continue
                snapshots.append(SimulationSnapshot.model_validate_json(text))
        return snapshots

    @staticmethod
    def _count_lines(path: Path) -> int:
        if not path.exists():
            return 0
        with path.open("r", encoding="utf-8") as fh:
            return sum(1 for _ in fh)
