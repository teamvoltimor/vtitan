"""Health check and diagnostics endpoints.

Verify inference availability and system status.
"""

from __future__ import annotations

from dataclasses import dataclass

from fastapi import APIRouter

from src.api.dependencies import AppContextDep, RepositoryDep

router = APIRouter()


@dataclass
class HealthStatus:
    """System health status."""

    ready: bool
    inference_available: bool
    database_accessible: bool
    message: str = ""


@router.get("/health", response_model=HealthStatus)
def health_check(app_context: AppContextDep, repository: RepositoryDep) -> HealthStatus:
    """Check system health: database connectivity and inference availability.

    Returns:
        HealthStatus with ready=true if system is operational.

    Use /health for monitoring and readiness checks.
    """
    # Check database connectivity
    db_ok = True
    try:
        # Try a simple database query
        repository.stats.get_stats()
    except Exception as e:
        db_ok = False
        return HealthStatus(
            ready=False,
            inference_available=False,
            database_accessible=False,
            message=f"Database error: {e}",
        )

    # Check inference availability
    inference_ok = True
    if not app_context.inference or not app_context.client:
        inference_ok = False

    ready = db_ok and inference_ok
    return HealthStatus(
        ready=ready,
        inference_available=inference_ok,
        database_accessible=db_ok,
        message="System ready" if ready else "System degraded",
    )


@router.get("/health/startup")
def startup_check(app_context: AppContextDep) -> dict:
    """Liveness check for startup/readiness probes.

    Use this for container orchestration (Kubernetes, Docker Compose).
    """
    return {"status": "ok"}
