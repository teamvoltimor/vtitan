"""src.sam_client – TCP client and request dataclasses for the model-server microservice.

:class:`ModelServerClient` sends length-prefixed pickle messages to the server and
returns decoded responses.  Each public method constructs a typed request dataclass,
converts it to a dict via ``.to_dict()``, serialises it, and deserialises the response.

All command and response key strings come from :mod:`src.server.constants` so no
magic strings appear here.
"""

from __future__ import annotations

import socket
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np

from src.constants import SERVER_HOST, SERVER_PORT
from src.server import wire
from src.server.constants import (
    CMD_LIST_MODELS,
    CMD_PING,
    CMD_PREDICT,
    CMD_PREDICT_TEXT,
    CMD_SET_IMAGE,
    CMD_SET_MODEL,
    MSG_KEY_CLASS_NAMES,
    MSG_KEY_CMD,
    MSG_KEY_COORDS,
    MSG_KEY_IMAGE,
    MSG_KEY_LABELS,
    MSG_KEY_MASK_INPUT,
    MSG_KEY_MODEL_ID,
    RESP_KEY_ERROR,
    RESP_KEY_LOGITS,
    RESP_KEY_MASKS,
    RESP_KEY_MODELS,
    RESP_KEY_OK,
    RESP_KEY_RESULTS,
    RESP_KEY_SCORES,
)
from src.utils import get_logger

logger = get_logger(__name__)


# Request dataclasses – one per TCP command.
# Each provides a to_dict() method that produces the plain dict expected by the server.


@dataclass(frozen=True)
class PingRequest:
    """Liveness probe request; no payload required."""

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {MSG_KEY_CMD: CMD_PING}


@dataclass
class SetImageRequest:
    """Request to encode *image* with SAM's image encoder on the server.

    Attributes:
        image: RGB uint8 numpy array to send.
    """

    image: np.ndarray

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {MSG_KEY_CMD: CMD_SET_IMAGE, MSG_KEY_IMAGE: self.image}


@dataclass
class PredictRequest:
    """Request to run point-prompted mask prediction on the server.

    Attributes:
        coords:     Float32 array of shape ``(N, 2)`` with (x, y) pixel coords.
        labels:     Int32 array of shape ``(N,)`` with 1=positive, 0=negative.
        mask_input: Optional logit mask from a previous call for iterative refinement.
    """

    coords: np.ndarray
    labels: np.ndarray
    mask_input: np.ndarray | None = None

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {
            MSG_KEY_CMD: CMD_PREDICT,
            MSG_KEY_COORDS: self.coords,
            MSG_KEY_LABELS: self.labels,
            MSG_KEY_MASK_INPUT: self.mask_input,
        }


@dataclass(frozen=True)
class ListModelsRequest:
    """Request to retrieve descriptors for all configured models."""

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {MSG_KEY_CMD: CMD_LIST_MODELS}


@dataclass(frozen=True)
class SetModelRequest:
    """Request to load a different SAM model on the server.

    Attributes:
        model_id: Unique model identifier string matching a config entry.
    """

    model_id: str

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {MSG_KEY_CMD: CMD_SET_MODEL, MSG_KEY_MODEL_ID: self.model_id}


@dataclass
class PredictTextRequest:
    """Request to run text-prompted segmentation for a list of class names (SAM 3 only).

    Attributes:
        image:       RGB uint8 numpy array to segment.
        class_names: List of class-name strings used as text prompts.
    """

    image: np.ndarray
    class_names: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        """Serialise to the wire-protocol request dict."""
        return {
            MSG_KEY_CMD: CMD_PREDICT_TEXT,
            MSG_KEY_IMAGE: self.image,
            MSG_KEY_CLASS_NAMES: self.class_names,
        }


class ModelServerClient:
    """Thin TCP client that forwards inference requests to the model server.

    Each public method creates a new connection, sends one request, and
    returns the decoded response.  The server handles requests concurrently
    so multiple clients can connect simultaneously.

    Args:
        host: TCP host of the model server (default: ``SERVER_HOST``).
        port: TCP port of the model server (default: ``SERVER_PORT``).
    """

    def __init__(self, host: str = SERVER_HOST, port: int = SERVER_PORT) -> None:
        self.addr = (host, port)

    def _call(self, req: Any) -> dict:
        """Serialise *req* via ``.to_dict()``, send it, and return the decoded response.

        Args:
            req: A request dataclass instance with a ``to_dict()`` method.

        Returns:
            Decoded response dict from the server.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(120)
            s.connect(self.addr)
            wire.send(s, req.to_dict())
            return wire.recv(s)

    def ping(self) -> bool:
        """Check whether the model server is reachable and responding.

        Returns:
            ``True`` when the server replies with ``ok=True``.
        """
        try:
            return self._call(PingRequest()).get(RESP_KEY_OK, False)
        except Exception as e:  # noqa: BLE001
            logger.debug("Model server probe failed", extra={"_extra": {"err": str(e)}})
            return False

    def set_image(self, image: np.ndarray) -> None:
        """Send *image* to the server so SAM can encode it for subsequent predictions.

        Args:
            image: RGB uint8 numpy array.

        Raises:
            RuntimeError: When the server returns an error response.
        """
        resp = self._call(SetImageRequest(image=image))
        if RESP_KEY_ERROR in resp:
            raise RuntimeError(resp[RESP_KEY_ERROR])

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list, list, object]:
        """Run point-prompted mask prediction on the server.

        Args:
            coords:     Float32 point-coordinates array of shape ``(N, 2)``.
            labels:     Int32 point-labels array of shape ``(N,)``.
            mask_input: Optional logit mask for iterative refinement.

        Returns:
            Three-tuple ``(masks, scores, logits)``.

        Raises:
            RuntimeError: When the server returns an error response.
        """
        resp = self._call(PredictRequest(coords=coords, labels=labels, mask_input=mask_input))
        if RESP_KEY_ERROR in resp:
            raise RuntimeError(resp[RESP_KEY_ERROR])
        return resp[RESP_KEY_MASKS], resp[RESP_KEY_SCORES], resp.get(RESP_KEY_LOGITS)

    def list_models(self) -> list[dict]:
        """Retrieve the list of all configured models and their availability.

        Returns:
            List of model descriptor dicts from the server.
        """
        return self._call(ListModelsRequest()).get(RESP_KEY_MODELS, [])

    def set_model(self, model_id: str) -> dict:
        """Ask the server to load a different SAM model by *model_id*.

        Args:
            model_id: Unique model identifier string matching a server config entry.

        Returns:
            Response dict from the server (contains ``"ok"`` or ``"error"``).
        """
        return self._call(SetModelRequest(model_id=model_id))

    def predict_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run text-prompted segmentation for every class name (SAM 3 only).

        Args:
            image:       RGB uint8 numpy array of the image to segment.
            class_names: List of class-name strings used as text prompts.

        Returns:
            List of per-class result dicts from the server.

        Raises:
            RuntimeError: When the server returns a top-level error response.
        """
        resp = self._call(PredictTextRequest(image=image, class_names=class_names))
        if RESP_KEY_ERROR in resp:
            raise RuntimeError(resp[RESP_KEY_ERROR])
        return resp[RESP_KEY_RESULTS]
