"""src.job_manager – Async job lifecycle management with SSE streaming.

Replaces module-level _jobs dict with typed job state, cancellation support,
and proper cleanup. Jobs are queued, executed, and subscribers are notified
of progress.
"""

from __future__ import annotations

import asyncio
import uuid
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from src.utils import get_logger

logger = get_logger(__name__)


class JobStatus(StrEnum):
    """Job execution status."""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class JobEvent:
    """A single event in job execution (progress, error, etc.)."""

    job_id: str
    status: JobStatus
    message: str
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Convert to JSON-serializable dict."""
        return {
            "job_id": self.job_id,
            "status": self.status.value,
            "message": self.message,
            "data": self.data,
        }


@dataclass
class JobState:
    """Mutable state of an active job."""

    job_id: str
    status: JobStatus = JobStatus.PENDING
    subscribers: list[asyncio.Queue[JobEvent]] = field(default_factory=list)
    cancelled: bool = False

    def emit(self, event: JobEvent) -> None:
        """Send event to all subscribers (thread-safe via asyncio)."""
        for queue in list(self.subscribers):
            try:
                queue.put_nowait(event)
            except asyncio.QueueFull:
                logger.warning("SSE queue full, dropping event", extra={"_extra": {"job_id": self.job_id}})


class JobManager:
    """Manages async job lifecycle (augmentation, training, etc).

    Replaces module-level state dict with typed jobs, proper cancellation,
    and cleanup.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop):
        """Initialize job manager.

        Args:
            loop: Event loop for asyncio operations.
        """
        self.loop = loop
        self.jobs: dict[str, JobState] = {}

    def create(self) -> str:
        """Create a new job and return its ID.

        Returns:
            Unique job ID.
        """
        job_id = str(uuid.uuid4())
        self.jobs[job_id] = JobState(job_id=job_id)
        logger.info("Job created", extra={"_extra": {"job_id": job_id}})
        return job_id

    def get(self, job_id: str) -> JobState | None:
        """Get job state by ID."""
        return self.jobs.get(job_id)

    def subscribe(self, job_id: str, queue: asyncio.Queue[JobEvent]) -> bool:
        """Subscribe to job events.

        Args:
            job_id: Job to subscribe to.
            queue: Async queue to receive events.

        Returns:
            True if subscription succeeded, False if job not found.
        """
        job = self.jobs.get(job_id)
        if job is None:
            return False
        job.subscribers.append(queue)
        return True

    def emit(self, job_id: str, event: JobEvent) -> bool:
        """Send an event to all subscribers.

        Args:
            job_id: Job ID.
            event: Event to emit.

        Returns:
            True if job found and event sent, False otherwise.
        """
        job = self.jobs.get(job_id)
        if job is None:
            return False
        job.emit(event)
        return True

    def set_status(self, job_id: str, status: JobStatus) -> bool:
        """Update job status.

        Args:
            job_id: Job ID.
            status: New status.

        Returns:
            True if job found, False otherwise.
        """
        job = self.jobs.get(job_id)
        if job is None:
            return False
        job.status = status
        return True

    def cancel(self, job_id: str) -> bool:
        """Request job cancellation.

        Args:
            job_id: Job ID.

        Returns:
            True if job found, False otherwise.
        """
        job = self.jobs.get(job_id)
        if job is None:
            return False
        job.cancelled = True
        logger.info("Job cancelled", extra={"_extra": {"job_id": job_id}})
        return True

    def cleanup(self, job_id: str) -> None:
        """Remove completed job from tracking.

        Args:
            job_id: Job ID.
        """
        self.jobs.pop(job_id, None)

    def __repr__(self) -> str:
        pending = sum(1 for j in self.jobs.values() if j.status == JobStatus.PENDING)
        running = sum(1 for j in self.jobs.values() if j.status == JobStatus.RUNNING)
        return f"JobManager({pending} pending, {running} running, {len(self.jobs)} total)"
