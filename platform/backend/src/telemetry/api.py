"""HTTP helpers for exposing open telemetry snapshots."""

from __future__ import annotations

import asyncio
import logging
import os
import time
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

logger = logging.getLogger(__name__)

from src.telemetry.generator_sim import TelemetryGenerator
from src.telemetry.models import RobotSnapshot, TopicsSnapshot
from src.telemetry.recorder import ReplaySessionInfo, TelemetryRecorder

router = APIRouter(prefix="/telemetry", tags=["telemetry"])


class ConnectionManager:
    def __init__(self) -> None:
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, data: str) -> None:
        for connection in list(self.active_connections):
            try:
                await connection.send_text(data)
            except Exception:
                self.disconnect(connection)


manager = ConnectionManager()

_BACKEND_ROOT = Path(__file__).parent.parent.parent
_sessions_dir = Path(
    os.environ.get(
        "TELEMETRY_SESSIONS_DIR",
        str(_BACKEND_ROOT / "telemetry_sessions"),
    ),
)
_max_sessions = int(os.environ.get("TELEMETRY_MAX_SESSIONS", "20"))
_recorder = TelemetryRecorder(base_dir=_sessions_dir, max_sessions=_max_sessions)
_generator = TelemetryGenerator(recorder=_recorder)
_start_time = time.time()


def get_recorder() -> TelemetryRecorder:
    """Return the singleton recorder that services the router."""
    return _recorder


RECORDER_DEPENDENCY = Depends(get_recorder)


def shutdown() -> None:
    """Release recorder resources — called by the FastAPI lifespan hook."""
    _recorder.close()


@router.get("/health", response_model=dict)
def health_check() -> dict:
    """Backend health check endpoint."""
    return {
        "status": "ok",
        "version": "0.3.0",
        "uptime_seconds": time.time() - _start_time,
    }


@router.get("/config", response_model=dict)
def get_config() -> dict:
    """Return current backend configuration."""
    return {
        "max_sessions": _max_sessions,
        "sessions_dir": str(_sessions_dir),
        "poll_interval_ms": 2500,
    }


@router.get("/latest", response_model=RobotSnapshot)
def get_latest_snapshot() -> RobotSnapshot:
    """Return the freshest telemetry snapshot."""
    return _generator.latest_snapshot()


@router.get("/history", response_model=list[RobotSnapshot])
def get_history(limit: int = Query(60, ge=10, le=360)) -> list[RobotSnapshot]:
    """Return a bounded window of recent telemetry snapshots."""
    return list(_generator.history(limit))


@router.post("/record", response_model=None)
async def record_snapshot(
    snapshot: RobotSnapshot,
    recorder: TelemetryRecorder = RECORDER_DEPENDENCY,
) -> None:
    """Persist a snapshot for later replay and broadcast it to WebSockets."""
    recorder.record(snapshot)
    await manager.broadcast(snapshot.model_dump_json())


_topics_data: TopicsSnapshot | None = None


@router.post("/topics/update", response_model=None)
async def update_topics(snapshot: TopicsSnapshot) -> None:
    """Receive raw topics update from robot and broadcast to WebSockets."""
    global _topics_data
    _topics_data = snapshot
    await manager.broadcast(snapshot.model_dump_json())


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    """WebSocket endpoint for streaming telemetry data."""
    await manager.connect(websocket)
    try:
        while True:
            # Keep connection open and listen for close
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# Start a background task to drive the simulation and broadcast
async def run_simulation_loop() -> None:
    """Drive the simulation and broadcast frames to connected clients."""
    try:
        while True:
            if manager.active_connections:
                snapshot = _generator.latest_snapshot()
                await manager.broadcast(snapshot.model_dump_json())
            await asyncio.sleep(2.5)
    except asyncio.CancelledError:
        pass


class SpeedConfigUpdate(BaseModel):
    max_linear_speed: float


@router.post("/robot/config/speed")
async def update_robot_speed(config: SpeedConfigUpdate) -> dict:
    """Update robot's maximum linear speed configuration."""
    logger.info(f"Received speed config update: {config.max_linear_speed}")
    # In a real system, you would broadcast this to the robot via ROS/WebSocket
    # For now, just log and return a success confirmation
    return {"status": "success", "max_linear_speed": config.max_linear_speed}


@router.get("/topics", response_model=TopicsSnapshot)
def get_raw_topics() -> TopicsSnapshot:
    """Return raw ROS2 topic data for debugging."""
    if _topics_data is None:
        # Return empty snapshot if no data yet
        return TopicsSnapshot(timestamp=time.time(), topics=[])
    return _topics_data


@router.get("/sessions", response_model=list[ReplaySessionInfo])
def list_sessions(
    recorder: TelemetryRecorder = RECORDER_DEPENDENCY,
) -> list[ReplaySessionInfo]:
    """List replay sessions currently stored on disk."""
    return list(recorder.list_sessions())


@router.get("/sessions/{session_id}", response_model=list[RobotSnapshot])
def load_session(
    session_id: str,
    recorder: TelemetryRecorder = RECORDER_DEPENDENCY,
) -> list[RobotSnapshot]:
    """Return the snapshots recorded under ``session_id``."""
    try:
        return list(recorder.load_session(session_id))
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
