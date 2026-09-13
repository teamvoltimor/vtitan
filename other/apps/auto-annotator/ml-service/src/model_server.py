"""src.model_server – In-process model context, replacing the TCP model-server.

The gRPC compute server and the SAM/YOLO model state used to live in separate
"servers" (a TCP socket loop and a gRPC service) that only ever talked to
each other over localhost, in the same process
(see docs/internal/audits/2026-07-08-auto-annotator-ml-service.md, finding M6).
That socket/pickle boundary bought no isolation and is gone: :class:`LocalModelClient`
wraps a :class:`~src.server.context.ServerContext` directly and satisfies the
same :class:`~src.models.SAMClientProtocol` the deleted TCP client did.
"""

from __future__ import annotations

import os
import tomllib
from pathlib import Path  # noqa: TC003
from typing import TYPE_CHECKING

import torch

from src.core.enums import ComputeDevice
from src.server import dispatch
from src.server.context import ServerContext
from src.server.loader import initial_load
from src.server.registry import ModelConfig
from src.utils import get_logger

if TYPE_CHECKING:
    import threading

    import numpy as np

    from src.core.config import AppConfig
    from src.server.context import TextSegmentationResult
    from src.server.responses import ModelDescriptor, SetModelResponse

logger = get_logger(__name__)


def _load_models_config(config_file: Path) -> list[ModelConfig]:
    if not config_file.exists():
        logger.warning("Config not found, no models configured: %s", config_file)
        return []
    with config_file.open("rb") as f:
        data: dict[str, object] = tomllib.load(f)
    raw = data.get("models", [])
    return [ModelConfig.model_validate(m) for m in raw] if isinstance(raw, list) else []


class LocalModelClient:
    """In-process stand-in for the deleted TCP ``ModelServerClient``.

    Every method wraps the corresponding :mod:`src.server.dispatch` call in
    *lock* so concurrent gRPC requests never touch ``ctx.predictor``
    simultaneously — the same guarantee the old TCP server's single dispatch
    lock provided.
    """

    def __init__(self, ctx: ServerContext, lock: threading.Lock) -> None:
        self._ctx = ctx
        self._lock = lock

    def ping(self) -> bool:
        """Return ``True`` once a model has finished loading."""
        return self._ctx.predictor is not None

    def set_image(self, image: np.ndarray) -> None:
        """Encode *image* with the active predictor.

        Raises:
            ModelNotAvailable: If no model has been loaded yet.
        """
        with self._lock:
            dispatch.set_image(self._ctx, image)

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], list[float], np.ndarray | None]:
        """Run point-prompted mask prediction on the current image."""
        with self._lock:
            resp = dispatch.predict(self._ctx, coords, labels, mask_input)
        return resp.masks, resp.scores, resp.logits

    def list_models(self) -> list[ModelDescriptor]:
        """Return descriptors for all configured models."""
        with self._lock:
            return dispatch.list_models(self._ctx).models

    def set_model(self, model_id: str) -> SetModelResponse:
        """Load a different model by *model_id*."""
        with self._lock:
            return dispatch.set_model(self._ctx, model_id)

    def predict_text(self, image: np.ndarray, class_names: list[str]) -> list[TextSegmentationResult]:
        """Run text-prompted segmentation for each class name.

        Raises:
            RuntimeError: When the active model does not support text prompts.
        """
        with self._lock:
            resp = dispatch.predict_text(self._ctx, image, class_names)
        if resp.error:
            raise RuntimeError(resp.error)
        return resp.results


def build_model_client(config: AppConfig, lock: threading.Lock) -> LocalModelClient:
    """Load the default model into a fresh :class:`ServerContext` and wrap it.

    Blocks until :func:`~src.server.loader.initial_load` finishes (or gives
    up); callers run this on a background thread so it doesn't stall gRPC
    server startup.

    Args:
        config: Application configuration (models dir, config file, default model).
        lock:   Shared lock the returned client uses to guard every predictor call.

    Returns:
        A :class:`LocalModelClient` wrapping the loaded (or load-failed) context.
    """
    os.environ.setdefault("HF_HUB_CACHE", str(config.paths.models_dir))
    device = ComputeDevice.CUDA if torch.cuda.is_available() else ComputeDevice.CPU
    models_config = _load_models_config(config.paths.config_file)
    ctx = ServerContext(models_config=models_config, device=device)
    initial_load(ctx, models_config, config.inference.default_model or "sam2.1-large")
    return LocalModelClient(ctx, lock)
