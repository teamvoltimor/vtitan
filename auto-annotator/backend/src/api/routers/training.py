"""Training job endpoints."""

from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException

from src.api.job_handler import get_job_status, run_job, stream_job_events
from src.api.schemas import JobStatusResponse, TrainRequest
from src.train_service import run_training_job

if TYPE_CHECKING:
    from fastapi.responses import StreamingResponse

    from src.api.dependencies import JobManagerDep

router = APIRouter()


@router.get("/status", response_model=JobStatusResponse)
async def train_status(job_manager: JobManagerDep) -> JobStatusResponse:
    """Get training job status."""
    status = get_job_status(job_manager)
    return JobStatusResponse(**status)


@router.post("/start", response_model=JobStatusResponse, status_code=202)
async def start_train(
    payload: TrainRequest,
    job_manager: JobManagerDep,
) -> JobStatusResponse:
    """Start a YOLO training job in the background using asyncio."""
    if job_manager.has_running():
        raise HTTPException(status_code=409, detail="Training already running")

    result = await run_job(
        job_manager,
        run_training_job,
        "Training",
        model_name=payload.model_name,
        epochs=payload.epochs,
        batch=payload.batch,
        imgsz=payload.imgsz,
    )
    return JobStatusResponse(**result)


@router.get("/stream")
async def train_stream(job_manager: JobManagerDep) -> StreamingResponse:
    """SSE stream for training progress."""
    return await stream_job_events(job_manager, "training")
