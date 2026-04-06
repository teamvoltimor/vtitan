"""FastAPI application factory with proper dependency injection.

Replaces server.py with improved structure following SOLID principles.
"""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING, AsyncIterator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.telemetry.api import router as telemetry_router
from src.telemetry.config import ServerConfig
from src.telemetry.dependencies import setup_dependencies
from src.telemetry.simulation.orchestrator import SimulationOrchestrator

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """FastAPI lifespan context manager.

    Manages startup (create simulation task) and shutdown (cancel task, close resources).

    Args:
        app: FastAPI application instance.

    Yields:
        During application lifetime.
    """
    # Startup: Create and start simulation task
    config = ServerConfig.from_env()
    orchestrator = SimulationOrchestrator(
        generator=app.state.telemetry.generator,
        connection_manager=app.state.telemetry.connection_manager,
        poll_interval_ms=config.poll_interval_ms,
    )
    sim_task = asyncio.create_task(orchestrator.run_loop())
    logger.info("Simulation loop started")

    yield

    # Shutdown: Cancel task and close resources
    sim_task.cancel()
    try:
        await sim_task
    except asyncio.CancelledError:
        logger.info("Simulation loop cancelled")

    # Close recorder file handles
    app.state.telemetry.recorder.close()
    logger.info("Recorder closed, application shutdown complete")


def create_app(config: ServerConfig | None = None) -> FastAPI:
    """Factory function to create and configure FastAPI application.

    Args:
        config: Server configuration. If None, loaded from environment.

    Returns:
        Configured FastAPI application instance.
    """
    if config is None:
        config = ServerConfig.from_env()

    app = FastAPI(
        title="Klevor Telemetry",
        version="0.3.0",
        docs_url="/docs",
        redoc_url="/redoc",
        lifespan=lifespan,
    )

    # CORS middleware (allow all origins for development)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["GET", "POST", "WebSocket"],
        allow_headers=["*"],
    )

    # Initialize dependencies
    setup_dependencies(app, config)

    # Include routers
    app.include_router(telemetry_router)

    return app


# Module-level app instance for Uvicorn
app = create_app()
