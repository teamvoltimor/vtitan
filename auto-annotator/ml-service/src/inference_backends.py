"""src.inference_backends – Pluggable inference adapters behind a protocol.

Defines InferenceBackend interface and concrete adapters for SAM2 and ultralytics,
with explicit exception types instead of bare exception catching.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

import numpy as np

from src.enums import ComputeDevice
from src.exceptions import InferenceBackendError, InferenceGPUMemory
from src.utils import get_logger

if TYPE_CHECKING:
    from src.config import InferenceConfig, PathConfig
    from src.models import InferenceContext

logger = get_logger(__name__)


@dataclass
class InferencePrediction:
    """Result of SAM inference on a set of points.

    Bundles the masks, confidence scores, and optional logits from a single predict call.
    """

    masks: list[np.ndarray]
    scores: np.ndarray
    logits: np.ndarray | None


@dataclass
class InferenceConfigPaths:
    """Configuration and paths for SAM inference.

    Bundles inference settings with filesystem paths, loaded from AppConfig.
    """

    inference: InferenceConfig
    paths: PathConfig


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
    ) -> InferencePrediction:
        """Run inference on points.

        Args:
            coords: Float32 array of shape (N, 2) with point coordinates.
            labels: Int32 array of shape (N,) with point labels (1=foreground, 0=background).
            mask_input: Optional logit mask for iterative refinement.

        Returns:
            InferencePrediction with masks, scores, and optional logits.

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
        """Set image for SAM2 inference."""
        try:
            with self.torch_module.inference_mode(), self._autocast_ctx():
                self.predictor.set_image(image)
        except Exception as e:
            err_msg = f"SAM2 set_image failed: {e}"
            raise InferenceBackendError(err_msg) from e

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> InferencePrediction:
        """Run SAM2 predict."""
        try:
            with self.torch_module.inference_mode(), self._autocast_ctx():
                masks, scores, logits = self.predictor.predict(
                    point_coords=coords,
                    point_labels=labels,
                    mask_input=mask_input,
                    multimask_output=True,
                )
            all_masks = [masks[i].astype(bool) for i in range(len(masks))]
            return InferencePrediction(
                masks=all_masks,
                scores=scores.flatten(),
                logits=logits,
            )
        except self.oom_error:
            err_msg = "CUDA out of memory"
            raise InferenceGPUMemory(err_msg) from None
        except Exception as e:
            err_msg = f"SAM2 predict failed: {e}"
            raise InferenceBackendError(err_msg) from e

    def _autocast_ctx(self) -> contextlib.AbstractContextManager:
        if self.torch_module.cuda.is_available():
            return self.torch_module.autocast(ComputeDevice.CUDA, dtype=self.torch_module.bfloat16)
        return contextlib.nullcontext()


class UltralyticsBackend:
    """Ultralytics SAM wrapper adapter (positive points only, no logits)."""

    def __init__(self, context: InferenceContext, default_mask_score: float = 1.0):
        """Initialize ultralytics backend from context.

        Args:
            context:            InferenceContext with ultralytics predictor loaded.
            default_mask_score: Fallback confidence score when backend returns none.

        Raises:
            InferenceBackendError: If predictor is not available.
        """
        if context.predictor is None or context.use_native:
            msg = "Ultralytics backend requires ultralytics predictor in context"
            raise InferenceBackendError(msg)
        self.predictor = context.predictor
        self.default_mask_score = default_mask_score
        self.current_image: np.ndarray | None = None

    def set_image(self, image: np.ndarray) -> None:
        """Store image for Ultralytics inference."""
        self.current_image = image

    def _ultralytics_predict(
        self, coords: np.ndarray, labels: np.ndarray,
    ) -> InferencePrediction:
        """Run actual Ultralytics prediction."""
        if self.current_image is None:
            err_msg = "Image must be set before calling predict"
            raise InferenceBackendError(err_msg)

        positive_coords = coords[labels == 1]
        if len(positive_coords) == 0:
            positive_coords = coords
        positive_labels = [1] * len(positive_coords)

        results = self.predictor(self.current_image, points=[positive_coords.tolist()], labels=[positive_labels])
        if not results or results[0].masks is None:
            err_msg = "Ultralytics returned no mask"
            raise InferenceBackendError(err_msg)

        single_mask = results[0].masks.data[0].cpu().numpy().astype(bool)
        return InferencePrediction(
            masks=[single_mask],
            scores=np.array([self.default_mask_score]),
            logits=None,
        )

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        _mask_input: np.ndarray | None = None,
    ) -> InferencePrediction:
        """Run Ultralytics predict (positive points only, no logits)."""
        try:
            return self._ultralytics_predict(coords, labels)
        except Exception as e:
            err_msg = f"Ultralytics predict failed: {e}"
            raise InferenceBackendError(err_msg) from e


def _get_inference_config() -> InferenceConfigPaths:
    """Return inference config and paths from AppConfig (lazy load)."""
    from src.config import AppConfig  # noqa: PLC0415
    cfg = AppConfig.load()
    return InferenceConfigPaths(inference=cfg.inference, paths=cfg.paths)


def load_native_sam2(
    context: InferenceContext,
    inference_cfg: InferenceConfig | None = None,
    paths_cfg: PathConfig | None = None,
) -> None:
    """Load SAM2 native backend into context.

    Tries local checkpoint first, then falls back to HuggingFace.

    Args:
        context:       InferenceContext to populate in-place.
        inference_cfg: SAM2 inference settings. Falls back to AppConfig when omitted.
        paths_cfg:     Filesystem path settings. Falls back to AppConfig when omitted.

    Raises:
        InferenceBackendError: If both local and HuggingFace loading fail.
    """
    if inference_cfg is None or paths_cfg is None:
        cfg_paths = _get_inference_config()
        inference_cfg = inference_cfg or cfg_paths.inference
        paths_cfg = paths_cfg or cfg_paths.paths

    try:
        import torch  # noqa: PLC0415
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore[import-untyped]  # noqa: PLC0415

        context.torch_module = torch
        context.oom_error = torch.cuda.OutOfMemoryError

        device = ComputeDevice.CUDA if torch.cuda.is_available() else ComputeDevice.CPU
        local_checkpoint = paths_cfg.models_dir / inference_cfg.sam2_checkpoint_filename

        try:
            if local_checkpoint.exists():
                from sam2.build_sam import build_sam2  # type: ignore[import-untyped]  # noqa: PLC0415

                model = build_sam2(inference_cfg.sam2_config_path, str(local_checkpoint), device=device)
                context.predictor = SAM2ImagePredictor(model)
            else:
                context.predictor = SAM2ImagePredictor.from_pretrained(inference_cfg.sam2_hf_repo)
            context.use_native = True
            logger.info("Loaded native SAM2 backend")
        except Exception as e:
            err_msg = f"Failed to load SAM2 (local: {local_checkpoint.exists()}, HF: fallback): {e}"
            raise InferenceBackendError(err_msg) from e
    except InferenceBackendError:
        raise
    except Exception as e:
        err_msg = f"SAM2 import failed: {e}"
        raise InferenceBackendError(err_msg) from e


def load_ultralytics_fallback(
    context: InferenceContext,
    inference_cfg: InferenceConfig | None = None,
    paths_cfg: PathConfig | None = None,
) -> None:
    """Load Ultralytics SAM backend as fallback.

    Args:
        context:       InferenceContext to populate in-place.
        inference_cfg: SAM2 inference settings. Falls back to AppConfig when omitted.
        paths_cfg:     Filesystem path settings. Falls back to AppConfig when omitted.

    Raises:
        InferenceBackendError: If loading fails.
    """
    if inference_cfg is None or paths_cfg is None:
        cfg_paths = _get_inference_config()
        inference_cfg = inference_cfg or cfg_paths.inference
        paths_cfg = paths_cfg or cfg_paths.paths

    try:
        from ultralytics import SAM as UltralyticsSAM  # type: ignore[import-untyped]  # noqa: PLC0415

        context.predictor = UltralyticsSAM(inference_cfg.sam2_checkpoint_filename)
        context.use_native = False
        logger.info("Loaded Ultralytics SAM fallback")
    except Exception as e:
        err_msg = f"Ultralytics fallback failed: {e}"
        raise InferenceBackendError(err_msg) from e


def get_backend(
    context: InferenceContext,
    inference_cfg: InferenceConfig | None = None,
) -> InferenceBackend:
    """Get the appropriate backend for the loaded predictor.

    Args:
        context:       InferenceContext with predictor loaded.
        inference_cfg: Inference settings for score defaults. Falls back to AppConfig when omitted.

    Returns:
        InferenceBackend adapter (SAM2Backend or UltralyticsBackend).

    Raises:
        InferenceBackendError: If predictor is not loaded or type is unknown.
    """
    if context.predictor is None:
        err_msg = "No predictor loaded"
        raise InferenceBackendError(err_msg)

    if context.use_native:
        return SAM2Backend(context)

    default_score: float = 1.0
    if inference_cfg is not None:
        default_score = inference_cfg.default_mask_score
    return UltralyticsBackend(context, default_mask_score=default_score)
