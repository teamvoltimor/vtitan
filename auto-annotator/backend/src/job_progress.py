"""Progress reporting for async jobs.

Converts progress callbacks from no-ops to first-class citizen events
that flow through JobManager to SSE streams and future progress queries.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ProgressUpdate:
    """Progress update from a running job.

    Encapsulates the current state of work.
    """

    stage: str
    """Current stage name (e.g., 'augmenting', 'training', 'validation')."""

    progress: float
    """Progress percentage [0.0, 1.0]."""

    message: str = ""
    """Optional human-readable message."""

    details: dict | None = None
    """Optional details (items processed, ETA, etc.)."""


class ProgressReporter(Protocol):
    """Interface for reporting job progress.

    Implementations emit progress updates that can be observed via SSE,
    stored in job state, or used for monitoring/debugging.
    """

    def update(self, stage: str, progress: float, message: str = "", details: dict | None = None) -> None:
        """Report progress update.

        Args:
            stage: Current stage name.
            progress: Progress as percentage [0.0, 1.0].
            message: Optional status message.
            details: Optional extra details.
        """
        ...


class JobProgressReporter:
    """Progress reporter that emits JobEvents to JobManager.

    Used by job functions (training, augmentation) to report progress
    without knowing about JobManager directly.
    """

    def __init__(self, job_id: str, on_progress: callable) -> None:
        """Initialize with job ID and callback.

        Args:
            job_id: Unique job identifier.
            on_progress: Callback to emit progress (receives ProgressUpdate).
        """
        self.job_id = job_id
        self.on_progress = on_progress

    def update(self, stage: str, progress: float, message: str = "", details: dict | None = None) -> None:
        """Emit progress update via callback."""
        update = ProgressUpdate(stage=stage, progress=progress, message=message, details=details)
        self.on_progress(update)
