"""Dependency injection setup for FastAPI application.

This module implements the factory pattern for creating and configuring
all application dependencies (Python-Architect Rule 3: Dependency Injection).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from fastapi import Request

from src.telemetry.config import ServerConfig, SimulationProfile
from src.telemetry.error_handler import ErrorHandler
from src.telemetry.exceptions import TelemetryError
from src.telemetry.generator_sim import TelemetryGenerator
from src.telemetry.recorder import TelemetryRecorder

if TYPE_CHECKING:
    from fastapi import FastAPI

    from src.telemetry.ws.manager import ConnectionManager

logger = logging.getLogger(__name__)


@dataclass(slots=True, frozen=True)
class TelemetryAppState:
    """Immutable application state container.

    This frozen dataclass serves as the single source of truth for all
    application dependencies. Frozen=True prevents runtime mutations and
    uses slots to minimize memory overhead (Python-Architect Rule 2).
    """

    generator: TelemetryGenerator
    recorder: TelemetryRecorder
    connection_manager: ConnectionManager
    server_start_time: float = field(default_factory=time.time)


def create_app_state(config: ServerConfig) -> TelemetryAppState:
    """Factory function to create immutable application state.

    This function initializes all dependencies and returns a frozen dataclass
    containing the complete application state (Python-Architect Rule 2).

    Args:
        config: Server configuration loaded from environment.

    Returns:
        Fully initialized, immutable application state.

    Raises:
        OSError: If session directory cannot be created.
        FileNotFoundError: If simulation profile cannot be loaded.
    """
    # Import here to avoid circular imports
    from src.telemetry.ws.manager import ConnectionManager

    # Initialize session directory
    base_dir = Path(config.sessions_dir)
    base_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Session directory: {base_dir}")

    # Load simulation profile from environment or default to 'normal'
    import os
    profile_name = os.getenv('SIMULATION_PROFILE', 'normal')
    profile = SimulationProfile.load(profile_name)
    logger.info(f"Loaded simulation profile: {profile_name}")

    # Initialize core dependencies
    recorder = TelemetryRecorder(
        base_dir=base_dir,
        max_sessions=config.max_sessions,
    )
    generator = TelemetryGenerator(profile=profile)
    connection_manager = ConnectionManager()

    return TelemetryAppState(
        generator=generator,
        recorder=recorder,
        connection_manager=connection_manager,
    )


def setup_dependencies(app: FastAPI, config: ServerConfig) -> None:
    """Initialize FastAPI with application state.

    Args:
        app: FastAPI application instance.
        config: Server configuration.
    """
    app.state.telemetry = create_app_state(config)


# Dependency injection functions for route handlers


def get_state(request: Request) -> TelemetryAppState:
    """Retrieve application state from FastAPI context.

    Args:
        request: FastAPI request object.

    Returns:
        Application state.

    Raises:
        RuntimeError: If state was not initialized.
    """
    if not hasattr(request.app.state, "telemetry"):
        raise RuntimeError("Application state not initialized")
    return request.app.state.telemetry


def get_recorder(request: Request) -> TelemetryRecorder:
    """Route dependency: Retrieve recorder from app state."""
    return get_state(request).recorder


def get_generator(request: Request) -> TelemetryGenerator:
    """Route dependency: Retrieve generator from app state."""
    return get_state(request).generator


def get_connection_manager(request: Request) -> ConnectionManager:
    """Route dependency: Retrieve connection manager from app state."""
    return get_state(request).connection_manager


def get_error_handler() -> ErrorHandler[TelemetryError]:
    """Route dependency: Create an ErrorHandler for this request.

    Each request gets a fresh handler instance for error tracking.

    Returns:
        ErrorHandler configured for TelemetryError and subclasses.
    """
    return ErrorHandler(logger, TelemetryError, max_retries=3)
