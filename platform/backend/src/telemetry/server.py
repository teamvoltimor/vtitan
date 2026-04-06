"""FastAPI application and ASGI app instance for the telemetry service."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.telemetry import api as telemetry_api
from src.telemetry.api import router as telemetry_router

if TYPE_CHECKING:
    from collections.abc import AsyncIterator


@asynccontextmanager
async def _lifespan(_: FastAPI) -> AsyncIterator[None]:
    """Yield until shutdown and then stop the recorder."""
    sim_task = asyncio.create_task(telemetry_api.run_simulation_loop())
    yield
    sim_task.cancel()
    telemetry_api.shutdown()


app = FastAPI(
    title="Klevor Telemetry",
    version="0.2.0",
    docs_url="/docs",
    redoc_url="/redoc",
    lifespan=_lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

app.include_router(telemetry_router)
