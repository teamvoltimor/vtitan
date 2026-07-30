"""src.sam_client – TCP client and request Pydantic models for the model-server microservice.

:class:`ModelServerClient` sends length-prefixed pickle messages to the server and
returns decoded responses.  Each public method constructs a typed request model,
converts it to a dict via ``.model_dump(by_alias=True)``, serialises it, and
deserialises the response.

All command and response key strings come from :mod:`src.server.constants` so no
magic strings appear here.
"""

from __future__ import annotations

import socket
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, ConfigDict, Field

from src.config import SERVER_DEFAULT_HOST, SERVER_DEFAULT_PORT, SERVER_DEFAULT_RECV_CHUNK_SIZE
from src.enums import ServerCommand

if TYPE_CHECKING:
    import numpy as np

from src.server import wire
from src.server.constants import (
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


class PingRequest(BaseModel):
    """Liveness probe request; no payload required."""

    model_config = ConfigDict(frozen=True)
    cmd: str = ServerCommand.PING


class SetImageRequest(BaseModel):
    """Request to encode *image* with SAM's image encoder on the server.

    Attributes:
        image: RGB uint8 numpy array to send.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)
    cmd: str = ServerCommand.SET_IMAGE
    image: Any = None


class PredictRequest(BaseModel):
    """Request to run point-prompted mask prediction on the server.

    Attributes:
        coords:     Float32 array of shape ``(N, 2)`` with (x, y) pixel coords.
        labels:     Int32 array of shape ``(N,)`` with 1=positive, 0=negative.
        mask_input: Optional logit mask from a previous call for iterative refinement.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)
    cmd: str = ServerCommand.PREDICT
    coords: Any = None
    labels: Any = None
    mask_input: Any = None


class ListModelsRequest(BaseModel):
    """Request to retrieve descriptors for all configured models."""

    model_config = ConfigDict(frozen=True)
    cmd: str = ServerCommand.LIST_MODELS


class SetModelRequest(BaseModel):
    """Request to load a different SAM model on the server.

    Attributes:
        model_id: Unique model identifier string matching a config entry.
    """

    model_config = ConfigDict(frozen=True)
    cmd: str = ServerCommand.SET_MODEL
    model_id: str = ""


class PredictTextRequest(BaseModel):
    """Request to run text-prompted segmentation for a list of class names (SAM 3 only).

    Attributes:
        image:       RGB uint8 numpy array to segment.
        class_names: List of class-name strings used as text prompts.
    """

    model_config = ConfigDict(arbitrary_types_allowed=True)
    cmd: str = ServerCommand.PREDICT_TEXT
    image: Any = None
    class_names: list[str] = Field(default_factory=list)


class ModelServerClient:
    """Thin TCP client that forwards inference requests to the model server.

    Each public method creates a new connection, sends one request, and
    returns the decoded response.  The server handles requests concurrently
    so multiple clients can connect simultaneously.

    Args:
        host: TCP host of the model server (default: ``SERVER_HOST``).
        port: TCP port of the model server (default: ``SERVER_PORT``).
    """

    def __init__(self, host: str = SERVER_DEFAULT_HOST, port: int = SERVER_DEFAULT_PORT, recv_chunk_size: int = SERVER_DEFAULT_RECV_CHUNK_SIZE) -> None:
        self.addr = (host, port)
        self._recv_chunk_size = recv_chunk_size

    def _call(self, req: BaseModel) -> dict:
        """Serialise *req* via ``.model_dump(by_alias=True)``, send it, and return the decoded response.

        Args:
            req: A request Pydantic model instance.

        Returns:
            Decoded response dict from the server.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(120)
            s.connect(self.addr)
            wire.send(s, req.model_dump(by_alias=True))
            return wire.recv(s, self._recv_chunk_size)

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
