"""src.inference_backends – Pluggable inference adapters behind a protocol.

Defines InferenceBackend interface and concrete adapters for SAM2 and ultralytics,
with explicit exception types instead of bare exception catching.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

import contextlib
import numpy as np

from src.constants import (
    DEVICE_CPU,
    DEVICE_CUDA,
    INFERENCE_DEFAULT_MASK_SCORE,
    MODELS_DIR,
    SAM2_DEFAULT_HF_REPO,
    SAM2_DEFAULT_HIERA_CONFIG,
    SAM2_LOCAL_CHECKPOINT_FILENAME,
)
from src.exceptions import InferenceBackendError, InferenceGPUMemory
from src.utils import get_logger

if TYPE_CHECKING:
    from src.models import InferenceContext

logger = get_logger(__name__)


class InferenceBackend(Protocol):
    """Interface for SAM inference adapters.

    All backends operate on a loaded image set via ``set_image()``, then
    accept point-prompted inference via ``predict()``.
    """

    def set_image(self, image: np.ndarray) -> None:
        """Load image into the backend predictor.

        Args:
            image: RGB image array of shape (H, W, 3).

        Raises:
            InferenceBackendError: If image loading fails.
        """
        ...

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray | None]:
        """Run inference on points.

        Args:
            coords: Float32 array of shape (N, 2) with point coordinates.
            labels: Int32 array of shape (N,) with point labels (1=foreground, 0=background).
            mask_input: Optional logit mask for iterative refinement.

        Returns:
            Three-tuple:
              - masks: List of boolean arrays, one per output.
              - scores: 1D array of confidence scores for each mask.
              - logits: Optional logit array for iterative refinement, or None.

        Raises:
            InferenceBackendError: If inference fails.
            InferenceGPUMemory: If CUDA runs out of memory.
        """
        ...


class SAM2Backend:
    """Native SAM2 inference adapter."""

    def __init__(self, context: InferenceContext):
        """Initialize SAM2 backend from context.

        Args:
            context: InferenceContext with torch_module and predictor loaded.

        Raises:
            InferenceBackendError: If predictor is not available.
        """
        if context.predictor is None or not context.use_native:
            msg = "SAM2 backend requires native predictor in context"
            raise InferenceBackendError(msg)
        self.predictor = context.predictor
        self.torch_module = context.torch_module
        self.oom_error = context.oom_error

    def set_image(self, image: np.ndarray) -> None:
        try:
            with self.torch_module.inference_mode(), self._autocast_ctx():
                self.predictor.set_image(image)
        except Exception as e:
            raise InferenceBackendError(f"SAM2 set_image failed: {e}") from e

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray | None]:
        try:
            with self.torch_module.inference_mode(), self._autocast_ctx():
                masks, scores, logits = self.predictor.predict(
                    point_coords=coords,
                    point_labels=labels,
                    mask_input=mask_input,
                    multimask_output=True,
                )
            all_masks = [masks[i].astype(bool) for i in range(len(masks))]
            return all_masks, scores.flatten(), logits
        except self.oom_error:
            raise InferenceGPUMemory("CUDA out of memory") from None
        except Exception as e:
            raise InferenceBackendError(f"SAM2 predict failed: {e}") from e

    def _autocast_ctx(self) -> contextlib.AbstractContextManager:
        if self.torch_module.cuda.is_available():
            return self.torch_module.autocast(DEVICE_CUDA, dtype=self.torch_module.bfloat16)
        return contextlib.nullcontext()


class UltralyticsBackend:
    """Ultralytics SAM wrapper adapter (positive points only, no logits)."""

    def __init__(self, context: InferenceContext):
        """Initialize ultralytics backend from context.

        Args:
            context: InferenceContext with ultralytics predictor loaded.

        Raises:
            InferenceBackendError: If predictor is not available.
        """
        if context.predictor is None or context.use_native:
            msg = "Ultralytics backend requires ultralytics predictor in context"
            raise InferenceBackendError(msg)
        self.predictor = context.predictor
        self.current_image = None

    def set_image(self, image: np.ndarray) -> None:
        self.current_image = image

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray | None]:
        if self.current_image is None:
            msg = "Image must be set before calling predict"
            raise InferenceBackendError(msg)

        try:
            positive_coords = coords[labels == 1]
            if len(positive_coords) == 0:
                positive_coords = coords
            positive_labels = [1] * len(positive_coords)

            results = self.predictor(self.current_image, points=[positive_coords.tolist()], labels=[positive_labels])
            if not results or results[0].masks is None:
                msg = "Ultralytics returned no mask"
                raise InferenceBackendError(msg)

            single_mask = results[0].masks.data[0].cpu().numpy().astype(bool)
            return [single_mask], np.array([INFERENCE_DEFAULT_MASK_SCORE]), None
        except InferenceBackendError:
            raise
        except Exception as e:
            raise InferenceBackendError(f"Ultralytics predict failed: {e}") from e


def load_native_sam2(context: InferenceContext) -> None:
    """Load SAM2 native backend into context.

    Tries local checkpoint first, then falls back to HuggingFace.

    Args:
        context: InferenceContext to populate in-place.

    Raises:
        InferenceBackendError: If both local and HuggingFace loading fail.
    """
    try:
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore[import-untyped]

        import torch

        context.torch_module = torch
        context.oom_error = torch.cuda.OutOfMemoryError

        device = DEVICE_CUDA if torch.cuda.is_available() else DEVICE_CPU
        local_checkpoint = MODELS_DIR / SAM2_LOCAL_CHECKPOINT_FILENAME

        try:
            if local_checkpoint.exists():
                from sam2.build_sam import build_sam2  # type: ignore[import-untyped]

                model = build_sam2(SAM2_DEFAULT_HIERA_CONFIG, str(local_checkpoint), device=device)
                context.predictor = SAM2ImagePredictor(model)
            else:
                context.predictor = SAM2ImagePredictor.from_pretrained(SAM2_DEFAULT_HF_REPO)
            context.use_native = True
            logger.info("Loaded native SAM2 backend")
        except Exception as e:
            msg = f"Failed to load SAM2 (local: {local_checkpoint.exists()}, HF: fallback): {e}"
            raise InferenceBackendError(msg) from e
    except InferenceBackendError:
        raise
    except Exception as e:
        raise InferenceBackendError(f"SAM2 import failed: {e}") from e


def load_ultralytics_fallback(context: InferenceContext) -> None:
    """Load Ultralytics SAM backend as fallback.

    Args:
        context: InferenceContext to populate in-place.

    Raises:
        InferenceBackendError: If loading fails.
    """
    try:
        from ultralytics import SAM as UltralyticsSAM  # type: ignore[import-untyped]

        context.predictor = UltralyticsSAM(SAM2_LOCAL_CHECKPOINT_FILENAME)
        context.use_native = False
        logger.info("Loaded Ultralytics SAM fallback")
    except Exception as e:
        raise InferenceBackendError(f"Ultralytics fallback failed: {e}") from e


def get_backend(context: InferenceContext) -> InferenceBackend:
    """Get the appropriate backend for the loaded predictor.

    Args:
        context: InferenceContext with predictor loaded.

    Returns:
        InferenceBackend adapter (SAM2Backend or UltralyticsBackend).

    Raises:
        InferenceBackendError: If predictor is not loaded or type is unknown.
    """
    if context.predictor is None:
        msg = "No predictor loaded"
        raise InferenceBackendError(msg)

    if context.use_native:
        return SAM2Backend(context)
    return UltralyticsBackend(context)
