"""HTTP and WebSocket endpoints for telemetry API.

Single Responsibility: Only HTTP/WebSocket route handlers.
All other concerns (connection pooling, simulation, state) handled elsewhere.
"""

from __future__ import annotations

import asyncio
import logging
import re
import time
from typing import TYPE_CHECKING, Annotated

from fastapi import APIRouter, Depends, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from src.telemetry.config import ServerConfig
from src.telemetry.dependencies import (
    get_connection_manager,
    get_error_handler,
    get_generator,
    get_recorder,
)
from src.telemetry.exceptions import SessionNotFoundError, TelemetryError
from src.telemetry.models import RobotSnapshot, TopicsSnapshot
from src.telemetry.recorder import ReplaySessionInfo, TelemetryRecorder

if TYPE_CHECKING:
    from shared.config.types import (
        ConfigResponseDict,
        HealthCheckResponseDict,
        SpeedUpdateResponseDict,
    )

    from src.telemetry.error_handler import ErrorHandler
    from src.telemetry.generator_sim import TelemetryGenerator
    from src.telemetry.ws.manager import ConnectionManager

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/telemetry", tags=["telemetry"])

_SESSION_ID_PATTERN = re.compile(r"^[a-zA-Z0-9_-]+$")

_API_STATUS_OK = "ok"
_SPEED_PROFILES = ["normal", "fast", "slow"]
_DEFAULT_PROFILE = "normal"


@router.get("/health", response_model=dict)
def health_check(request: Request) -> HealthCheckResponseDict:
    """Backend health check endpoint.

    Returns:
        HealthCheckResponseDict with status, uptime, and node count.
    """
    try:
        uptime = time.time() - request.app.state.telemetry.server_start_time
    except AttributeError:
        uptime = 0.0

    return {
        "status": _API_STATUS_OK,
        "uptime_seconds": uptime,
        "active_nodes": 1,
        "version": "0.3.0",
    }


@router.get("/config", response_model=dict)
def get_config(request: Request) -> ConfigResponseDict:
    """Return current backend configuration.

    Returns:
        ConfigResponseDict with speed profiles and parameters.
    """
    state = request.app.state.telemetry
    return {
        "speed_profiles": _SPEED_PROFILES,
        "current_profile": _DEFAULT_PROFILE,
        "max_sessions": state.recorder.max_sessions,
        "sessions_dir": str(state.recorder.sessions_dir),
        "poll_interval_ms": ServerConfig().poll_interval_ms,
    }


@router.get("/latest", response_model=RobotSnapshot)
def get_latest_snapshot(
    generator: TelemetryGenerator = Depends(get_generator),
) -> RobotSnapshot:
    """Return the freshest telemetry snapshot.

    Args:
        generator: Injected TelemetryGenerator dependency.

    Returns:
        Most recent RobotSnapshot.
    """
    return generator.latest_snapshot()


@router.get("/history", response_model=list[RobotSnapshot])
def get_history(
    limit: Annotated[int, Query(ge=10, le=360)] = 60,
    generator: TelemetryGenerator = Depends(get_generator),
) -> list[RobotSnapshot]:
    """Return a bounded window of recent telemetry snapshots.

    Args:
        limit: Number of snapshots to return (10-360).
        generator: Injected TelemetryGenerator dependency.

    Returns:
        List of recent RobotSnapshot objects.
    """
    return list(generator.history(limit))


@router.post("/record", response_model=None)
async def record_snapshot(
    snapshot: RobotSnapshot,
    recorder: TelemetryRecorder = Depends(get_recorder),
    manager: ConnectionManager = Depends(get_connection_manager),
    error_handler: ErrorHandler[TelemetryError] = Depends(get_error_handler),
) -> None:
    """Persist a snapshot for later replay and broadcast it to WebSockets.

    Args:
        snapshot: RobotSnapshot to record and broadcast.
        recorder: Injected TelemetryRecorder dependency.
        manager: Injected ConnectionManager dependency.
        error_handler: Injected ErrorHandler dependency.

    Raises:
        HTTPException: If recording or broadcast fails.
    """
    try:
        await asyncio.to_thread(recorder.record, snapshot)
    except TelemetryError as exc:
        raise error_handler.to_http_response(exc) from exc

    try:
        await manager.broadcast(snapshot.model_dump_json())
    except TelemetryError as exc:
        raise error_handler.to_http_response(exc) from exc


class SpeedConfigUpdate(BaseModel):
    """Speed configuration update payload."""

    max_linear_speed: float


@router.post("/robot/config/speed")
async def update_robot_speed(config: SpeedConfigUpdate) -> SpeedUpdateResponseDict:
    """Update robot's maximum linear speed configuration.

    Args:
        config: New speed configuration.

    Returns:
        SpeedUpdateResponseDict confirming the update.
    """
    logger.info("Received speed config update: %s", config.max_linear_speed)
    # In a real system, you would broadcast this to the robot via ROS/WebSocket
    # For now, just log and return a success confirmation
    return {"status": "success", "max_linear_speed": config.max_linear_speed}


@router.get("/topics", response_model=TopicsSnapshot)
def get_raw_topics(_request: Request) -> TopicsSnapshot:
    """Return raw ROS2 topic data for debugging.

    Returns:
        Current TopicsSnapshot or empty snapshot if no data available.
    """
    # For now, return empty snapshot (this would be populated by robot in production)
    return TopicsSnapshot(timestamp=time.time(), topics=[])


@router.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket,
) -> None:
    """WebSocket endpoint for streaming telemetry data.

    Implements timeout protection and error handling (Error Handling Pillar).

    Args:
        websocket: FastAPI WebSocket instance.
    """
    manager = websocket.app.state.telemetry.connection_manager
    await manager.connect(websocket)
    try:
        while True:
            try:
                # Guard clause: timeout prevents hanging (Logic-Cleaner Rule 1)
                data = await asyncio.wait_for(
                    websocket.receive_text(),
                    timeout=30.0,  # 30 second idle timeout
                )
                # Optionally echo back as heartbeat response
                if data == "ping":
                    await websocket.send_text("pong")

            except TimeoutError:
                logger.debug(
                    "WebSocket idle timeout",
                    extra={"client": websocket.client},
                )
                await websocket.close(code=1000, reason="Idle timeout")
                break

    except WebSocketDisconnect:
        logger.debug("Client disconnected", extra={"client": websocket.client})
        manager.disconnect(websocket)

    except Exception as exc:
        logger.exception(
            "Unexpected error in WebSocket handler",
            exc_info=exc,
            extra={"client": websocket.client},
        )
        manager.disconnect(websocket)


@router.get("/sessions", response_model=list[ReplaySessionInfo])
def list_sessions(
    recorder: TelemetryRecorder = Depends(get_recorder),
) -> list[ReplaySessionInfo]:
    """List replay sessions currently stored on disk.

    Args:
        recorder: Injected TelemetryRecorder dependency.

    Returns:
        List of available replay sessions sorted by creation time.
    """
    return list(recorder.list_sessions())


@router.get("/sessions/{session_id}", response_model=list[RobotSnapshot])
def load_session(
    session_id: str,
    recorder: TelemetryRecorder = Depends(get_recorder),
    error_handler: ErrorHandler[TelemetryError] = Depends(get_error_handler),
) -> list[RobotSnapshot]:
    """Return the snapshots recorded under the given session_id.

    Args:
        session_id: Session identifier to load.
        recorder: Injected TelemetryRecorder dependency.
        error_handler: Injected ErrorHandler dependency.

    Returns:
        List of RobotSnapshot objects from the session.

    Raises:
        HTTPException: If session not found (404) or invalid session_id format.
    """
    if not _SESSION_ID_PATTERN.match(session_id):
        raise HTTPException(
            status_code=400,
            detail="Invalid session_id format. Use only alphanumeric, underscore, or hyphen.",
        )

    try:
        return list(recorder.load_session(session_id))
    except SessionNotFoundError as exc:
        raise error_handler.to_http_response(exc) from exc
