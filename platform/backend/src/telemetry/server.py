"""Entrypoint hosting the telemetry FastAPI service."""

from __future__ import annotations

import os
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.telemetry import api as telemetry_api
from src.telemetry.api import router as telemetry_router

if TYPE_CHECKING:
    from collections.abc import AsyncIterator


@asynccontextmanager
async def _lifespan(_: FastAPI) -> AsyncIterator[None]:
    """Yield until shutdown and then stop the recorder."""
    yield
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


def main() -> None:
    """Bootstrap the FastAPI server that exposes telemetry APIs."""
    port = int(os.environ.get("TELEMETRY_PORT", "8010"))
    uvicorn.run(
        "src.telemetry.server:app",
        host="0.0.0.0",  # noqa: S104
        port=port,
        log_level="info",
        reload=os.environ.get("TELEMETRY_RELOAD", "0") == "1",
    )


if __name__ == "__main__":
    main()
