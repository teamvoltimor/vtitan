"""Training job endpoints."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request

from src.api.dependencies import JobManagerDep, RepositoryDep
from src.api.job_handler import get_job_status, run_job, stream_job_events
from src.api.schemas import JobStatusResponse, TrainRequest
from src.job_manager import JobStatus
from src.train_service import run_training_job

router = APIRouter()


@router.get("/status", response_model=JobStatusResponse)
async def train_status(job_manager: JobManagerDep) -> JobStatusResponse:
    """Get training job status."""
    status = get_job_status(job_manager)
    return JobStatusResponse(**status)


@router.post("/start", response_model=JobStatusResponse)
async def start_train(
    request: Request,
    payload: TrainRequest,
    job_manager: JobManagerDep,
    repository: RepositoryDep,
) -> JobStatusResponse:
    """Start a YOLO training job in the background using asyncio."""
    if any(j.status == JobStatus.RUNNING for j in job_manager.jobs.values()):
        raise HTTPException(status_code=409, detail="Training already running")

    app_state = request.app.state.app_state
    result = await run_job(
        job_manager,
        run_training_job,
        "Training",
        model_name=payload.modelName,
        epochs=payload.epochs,
        batch=payload.batch,
        imgsz=payload.imgsz,
        on_progress=lambda evt: None,  # Progress callback (optional)
        repository=repository,
        cache=app_state.annotation_cache,
    )
    return JobStatusResponse(**result)


@router.get("/stream")
async def train_stream(job_manager: JobManagerDep):
    """SSE stream for training progress."""
    return await stream_job_events(job_manager, "training")
