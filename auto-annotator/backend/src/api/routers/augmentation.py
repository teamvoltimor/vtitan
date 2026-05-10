"""Augmentation job endpoints."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request

from src.api.dependencies import JobManagerDep, RepositoryDep
from src.api.job_handler import get_job_status, run_job, stream_job_events
from src.api.schemas import AugmentRequest, JobStatusResponse
from src.augment import run_augmentation_job
from src.job_manager import JobStatus

router = APIRouter()


@router.get("/status", response_model=JobStatusResponse)
async def augment_status(job_manager: JobManagerDep) -> JobStatusResponse:
    """Get augmentation job status."""
    status = get_job_status(job_manager)
    return JobStatusResponse(**status)


@router.post("/start", response_model=JobStatusResponse)
async def start_augment(
    request: Request,
    payload: AugmentRequest,
    job_manager: JobManagerDep,
    repository: RepositoryDep,
) -> JobStatusResponse:
    """Start an augmentation job in the background using asyncio."""
    if not payload.imageIds:
        raise HTTPException(status_code=400, detail="No images specified")

    if any(j.status == JobStatus.RUNNING for j in job_manager.jobs.values()):
        raise HTTPException(status_code=409, detail="Augmentation already running")

    app_state = request.app.state.app_state
    result = await run_job(
        job_manager,
        run_augmentation_job,
        "Augmentation",
        payload.imageIds,
        payload.numAugmentations,
        lambda evt: None,  # Progress callback (optional)
        repository=repository,
        cache=app_state.annotation_cache,
    )
    return JobStatusResponse(**result)


@router.get("/stream")
async def augment_stream(job_manager: JobManagerDep):
    """SSE stream for augmentation progress."""
    return await stream_job_events(job_manager, "augmentation")
