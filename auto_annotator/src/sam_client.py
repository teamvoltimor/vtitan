"""src.sam_client – TCP client for the model-server microservice."""

from __future__ import annotations

import pickle
import socket
import struct

import numpy as np

from src.constants import SERVER_HOST, SERVER_PORT
from src.utils import get_logger

logger = get_logger("sam_client")


def _recv_all(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(65536, n - len(buf)))
        if not chunk:
            msg = "Model server disconnected"
            raise ConnectionError(msg)
        buf += chunk
    return buf


class ModelServerClient:
    """Thin TCP client that forwards inference requests to model_server.py."""

    def __init__(self, host: str = SERVER_HOST, port: int = SERVER_PORT) -> None:
        self.addr = (host, port)

    def _call(self, msg: dict) -> dict:
        data = pickle.dumps(msg)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(120)
            s.connect(self.addr)
            s.sendall(struct.pack(">I", len(data)) + data)
            resp_len = struct.unpack(">I", _recv_all(s, 4))[0]
            return pickle.loads(_recv_all(s, resp_len))  # noqa: S301

    def ping(self) -> bool:
        try:
            return self._call({"cmd": "ping"}).get("ok", False)
        except Exception as e:  # noqa: BLE001
            logger.info("Model server probe failed", extra={"_extra": {"err": str(e)}})
            return False

    def set_image(self, image: np.ndarray) -> None:
        resp = self._call({"cmd": "set_image", "image": image})
        if "error" in resp:
            raise RuntimeError(resp["error"])

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list, list, object]:
        resp = self._call(
            {"cmd": "predict", "coords": coords, "labels": labels, "mask_input": mask_input}
        )
        if "error" in resp:
            raise RuntimeError(resp["error"])
        return resp["masks"], resp["scores"], resp.get("logits")

    def list_models(self) -> list[dict]:
        return self._call({"cmd": "list_models"}).get("models", [])

    def set_model(self, model_id: str) -> dict:
        return self._call({"cmd": "set_model", "model_id": model_id})

    def predict_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        resp = self._call({"cmd": "predict_text", "image": image, "class_names": class_names})
        if "error" in resp:
            raise RuntimeError(resp["error"])
        return resp["results"]
