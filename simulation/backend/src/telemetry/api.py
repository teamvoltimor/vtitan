"""HTTP helpers for exposing open telemetry snapshots."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query

from src.telemetry.generator import TelemetryGenerator
from src.telemetry.models import SimulationSnapshot
from src.telemetry.recorder import ReplaySessionInfo, TelemetryRecorder

router = APIRouter(prefix="/telemetry", tags=["telemetry"])

_recorder = TelemetryRecorder(base_dir="./telemetry_sessions")
_generator = TelemetryGenerator(recorder=_recorder)


def get_recorder() -> TelemetryRecorder:
    return _recorder


@router.get("/latest", response_model=SimulationSnapshot)
def get_latest_snapshot() -> SimulationSnapshot:
    """Return the freshest telemetry snapshot."""
    return _generator.latest_snapshot()


@router.get("/history", response_model=list[SimulationSnapshot])
def get_history(limit: int = Query(60, ge=10, le=360)) -> list[SimulationSnapshot]:
    """Return a bounded window of recent telemetry snapshots."""
    return list(_generator.history(limit))


@router.post("/record", response_model=None)
def record_snapshot(
    snapshot: SimulationSnapshot,
    recorder: TelemetryRecorder = Depends(get_recorder),
) -> None:
    recorder.record(snapshot)


@router.get("/sessions", response_model=list[ReplaySessionInfo])
def list_sessions(recorder: TelemetryRecorder = Depends(get_recorder)) -> list[ReplaySessionInfo]:
    return list(recorder.list_sessions())


@router.get("/sessions/{session_id}", response_model=list[SimulationSnapshot])
def load_session(
    session_id: str,
    recorder: TelemetryRecorder = Depends(get_recorder),
) -> list[SimulationSnapshot]:
    try:
        return list(recorder.load_session(session_id))
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
