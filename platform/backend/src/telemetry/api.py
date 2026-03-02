"""HTTP helpers for exposing open telemetry snapshots."""

from __future__ import annotations

import os

from fastapi import APIRouter, Depends, HTTPException, Query

from src.telemetry.generator import TelemetryGenerator
from src.telemetry.models import RobotSnapshot
from src.telemetry.recorder import ReplaySessionInfo, TelemetryRecorder

router = APIRouter(prefix="/telemetry", tags=["telemetry"])

_sessions_dir = os.environ.get("TELEMETRY_SESSIONS_DIR", "./telemetry_sessions")
_recorder = TelemetryRecorder(base_dir=_sessions_dir)
_generator = TelemetryGenerator(recorder=_recorder)


def get_recorder() -> TelemetryRecorder:
    return _recorder


def shutdown() -> None:
    """Release recorder resources — called by the FastAPI lifespan hook."""
    _recorder.close()


@router.get("/latest", response_model=RobotSnapshot)
def get_latest_snapshot() -> RobotSnapshot:
    """Return the freshest telemetry snapshot."""
    return _generator.latest_snapshot()


@router.get("/history", response_model=list[RobotSnapshot])
def get_history(limit: int = Query(60, ge=10, le=360)) -> list[RobotSnapshot]:
    """Return a bounded window of recent telemetry snapshots."""
    return list(_generator.history(limit))


@router.post("/record", response_model=None)
def record_snapshot(
    snapshot: RobotSnapshot,
    recorder: TelemetryRecorder = Depends(get_recorder),
) -> None:
    recorder.record(snapshot)


@router.get("/sessions", response_model=list[ReplaySessionInfo])
def list_sessions(recorder: TelemetryRecorder = Depends(get_recorder)) -> list[ReplaySessionInfo]:
    return list(recorder.list_sessions())


@router.get("/sessions/{session_id}", response_model=list[RobotSnapshot])
def load_session(
    session_id: str,
    recorder: TelemetryRecorder = Depends(get_recorder),
) -> list[RobotSnapshot]:
    try:
        return list(recorder.load_session(session_id))
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
