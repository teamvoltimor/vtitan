"""Segmentation inference endpoint."""

from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException

from src.api.schemas import SegmentationRequest, SegmentationResponse

if TYPE_CHECKING:
    from src.api.dependencies import AppContextDep, RepositoryDep, SegmentationServiceDep, ValidatorDep

router = APIRouter()


@router.post("/segment", response_model=SegmentationResponse)
def run_segmentation(
    payload: SegmentationRequest,
    app_context: AppContextDep,
    repository: RepositoryDep,
    segmentation_service: SegmentationServiceDep,
    validator: ValidatorDep,
) -> SegmentationResponse:
    """Run SAM inference for the provided click points and return the best mask as a polygon."""
    try:
        classes = repository.classes.get_all()
        # Validate request before calling service
        validator.validate_segmentation_request(payload.imageId, payload.points, classes)

        shape = segmentation_service.segment(payload.imageId, [p.dict() for p in payload.points], app_context, classes)

        if shape is None:
            return SegmentationResponse(state="error", message="Mask too small to render", shapes=[])

        return SegmentationResponse(state="ready", message="Model mask ready", shapes=[shape])

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e
