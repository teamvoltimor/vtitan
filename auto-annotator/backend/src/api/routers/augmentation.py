"""Augmentation job endpoints."""

from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException

from src.api.job_handler import get_job_status, run_job, stream_job_events
from src.api.schemas import AugmentRequest, JobStatusResponse
from src.augment import run_augmentation_job

if TYPE_CHECKING:
    from fastapi.responses import StreamingResponse

    from src.api.dependencies import JobManagerDep, LabelStoreDep, RepositoryDep

router = APIRouter()


@router.get("/status", response_model=JobStatusResponse)
async def augment_status(job_manager: JobManagerDep) -> JobStatusResponse:
    """Get augmentation job status."""
    status = get_job_status(job_manager)
    return JobStatusResponse(**status)


@router.post("/start", response_model=JobStatusResponse, status_code=202)
async def start_augment(
    payload: AugmentRequest,
    job_manager: JobManagerDep,
    repository: RepositoryDep,
    label_store: LabelStoreDep,
) -> JobStatusResponse:
    """Start an augmentation job in the background using asyncio."""
    if not payload.image_ids:
        raise HTTPException(status_code=400, detail="No images specified")

    if job_manager.has_running():
        raise HTTPException(status_code=409, detail="Augmentation already running")

    result = await run_job(
        job_manager,
        run_augmentation_job,
        "Augmentation",
        payload.image_ids,
        payload.num_augmentations,
        repository=repository,
        label_store=label_store,
    )
    return JobStatusResponse(**result)


@router.get("/stream")
async def augment_stream(job_manager: JobManagerDep) -> StreamingResponse:
    """SSE stream for augmentation progress."""
    return await stream_job_events(job_manager, "augmentation")
