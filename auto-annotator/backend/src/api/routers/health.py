"""Health, liveness, and readiness endpoints."""

from __future__ import annotations

import sqlite3
from typing import TYPE_CHECKING

from fastapi import APIRouter

from src.api.schemas import HealthStatus

if TYPE_CHECKING:
    from src.api.dependencies import AppContextDep, RepositoryDep

router = APIRouter()


def _readiness(app_context: AppContextDep, repository: RepositoryDep) -> HealthStatus:
    try:
        repository.stats.get_stats()
    except (OSError, sqlite3.Error):
        return HealthStatus(
            ready=False,
            inference_available=False,
            database_accessible=False,
            message="Database error",
        )

    inference_ok = bool(app_context.inference and app_context.client)
    ready = inference_ok
    return HealthStatus(
        ready=ready,
        inference_available=inference_ok,
        database_accessible=True,
        message="System ready" if ready else "System degraded",
    )


@router.get("/healthz")
def liveness() -> dict:
    """Liveness probe — returns 200 if the process is running."""
    return {"status": "ok"}


@router.get("/readyz", response_model=HealthStatus)
def readiness(app_context: AppContextDep, repository: RepositoryDep) -> HealthStatus:
    """Readiness probe — checks DB connectivity and inference availability."""
    return _readiness(app_context, repository)


@router.get("/health", response_model=HealthStatus)
def health_check(app_context: AppContextDep, repository: RepositoryDep) -> HealthStatus:
    """Full health check (alias for /readyz, kept for backward compatibility)."""
    return _readiness(app_context, repository)
