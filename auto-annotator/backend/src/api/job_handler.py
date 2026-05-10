"""Unified job handler pattern for async tasks (training, augmentation, etc.).

Eliminates duplication between training and augmentation routers.
Provides common job lifecycle, error handling, and SSE streaming.
"""

from __future__ import annotations

import asyncio
import json
from typing import Callable

from fastapi import HTTPException
from fastapi.responses import StreamingResponse

from src.job_manager import JobEvent, JobManager, JobStatus
from src.job_progress import JobProgressReporter, ProgressUpdate
from src.utils import get_logger

logger = get_logger(__name__)


async def run_job(
    job_manager: JobManager,
    job_func: Callable[..., None],
    job_name: str,
    *job_args,
    progress_callback: Callable[[ProgressUpdate], None] | None = None,
    **job_kwargs,
) -> dict:
    """Start an async job and return immediate response.

    Args:
        job_manager: JobManager for lifecycle management.
        job_func: Synchronous function to run in thread (e.g., run_training_job).
        job_name: Human-readable job name for logging.
        *job_args: Arguments to pass to job_func.
        progress_callback: Optional callback to receive ProgressUpdate events.
        **job_kwargs: Keyword arguments to pass to job_func.

    Returns:
        {"running": True, "message": "...started"}.
    """
    job_id = job_manager.create()
    job_manager.set_status(job_id, JobStatus.RUNNING)

    # Create progress reporter that wraps callback
    def progress_handler(update: ProgressUpdate) -> None:
        """Handle progress updates by emitting to JobManager."""
        if progress_callback:
            progress_callback(update)
        # Also emit as job event for streaming
        job_manager.emit(
            job_id,
            JobEvent(
                job_id=job_id,
                status=JobStatus.RUNNING,
                message=update.message or f"{job_name}: {update.stage}",
                data={"stage": update.stage, "progress": update.progress, "details": update.details},
            ),
        )

    reporter = JobProgressReporter(job_id, progress_handler)

    async def run_job_async() -> None:
        try:
            await asyncio.to_thread(job_func, *job_args, reporter=reporter, **job_kwargs)
            job_manager.set_status(job_id, JobStatus.COMPLETED)
            job_manager.emit(
                job_id,
                JobEvent(
                    job_id=job_id,
                    status=JobStatus.COMPLETED,
                    message=f"{job_name} completed",
                    data={"finished": True},
                ),
            )
        except Exception as e:
            logger.error(f"{job_name} job failed", extra={"_extra": {"error": str(e)}})
            job_manager.set_status(job_id, JobStatus.FAILED)
            job_manager.emit(
                job_id,
                JobEvent(
                    job_id=job_id,
                    status=JobStatus.FAILED,
                    message=str(e),
                    data={"error": str(e)},
                ),
            )
        finally:
            job_manager.cleanup(job_id)

    asyncio.create_task(run_job_async())
    return {"running": True, "message": f"{job_name} started"}


def get_job_status(job_manager: JobManager) -> dict:
    """Get status of any running job.

    Args:
        job_manager: JobManager.

    Returns:
        {"running": bool, "message": "...status"}.
    """
    running = any(j.status == JobStatus.RUNNING for j in job_manager.jobs.values())
    return {"running": running, "message": ""}


async def stream_job_events(job_manager: JobManager, job_name: str) -> StreamingResponse:
    """Create SSE stream for job progress.

    Args:
        job_manager: JobManager.
        job_name: Human-readable name for error messages.

    Returns:
        StreamingResponse with event stream.

    Raises:
        HTTPException 404 if no job is running.
    """
    q: asyncio.Queue[JobEvent] = asyncio.Queue()

    jobs_to_subscribe = [j for j in job_manager.jobs.values() if j.status == JobStatus.RUNNING]
    if not jobs_to_subscribe:
        raise HTTPException(status_code=404, detail=f"No {job_name} job running")

    active_job = jobs_to_subscribe[0]
    job_manager.subscribe(active_job.job_id, q)

    async def generator():
        try:
            while True:
                evt = await asyncio.wait_for(q.get(), timeout=30.0)
                yield f"data: {json.dumps(evt.to_dict())}\n\n"
                if evt.status in (JobStatus.COMPLETED, JobStatus.FAILED):
                    break
        except asyncio.TimeoutError:
            yield "data: {\"heartbeat\": true}\n\n"

    return StreamingResponse(
        generator(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )
