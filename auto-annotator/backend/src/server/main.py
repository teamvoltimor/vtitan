"""src.server.main – SAM inference microserver runner.

Loads one SAM model at a time and keeps running so the FastAPI API can
connect while the model is still loading.  The socket is bound *before*
model loading begins so callers can connect immediately; requests that
arrive before the model is ready receive a structured "No model loaded"
error rather than a connection-refused.

Environment variables
---------------------
MODEL_SERVER_PORT   TCP port (default 8765)
MODELS_DIR          directory containing checkpoints (default ./models)
HF_HUB_CACHE        HuggingFace cache directory (default same as MODELS_DIR)
HF_TOKEN            HuggingFace token (required for gated models such as SAM 3)
MODELS_CONFIG       path to models.toml (default ./config/models.toml)
DEFAULT_MODEL       model id to load on startup (default: first available)
"""

from __future__ import annotations

import atexit
import contextlib
import os
import pickle
import socket
import struct
import threading
import time
import tomllib

import torch

from src.constants import CONFIG_FILE, MODELS_DIR, SERVER_HOST, SERVER_PORT
from src.server.context import ServerContext
from src.server.dispatch import dispatch
from src.server.loader import initial_load
from src.utils import get_logger

logger = get_logger(__name__)

_RECV_CHUNK = 65536


class _ServerState:
    socket: socket.socket | None = None


_server_state = _ServerState()


def _load_config() -> list[dict]:
    if not CONFIG_FILE.exists():
        logger.warning("Config not found, no models configured: %s", CONFIG_FILE)
        return []
    with CONFIG_FILE.open("rb") as f:
        return tomllib.load(f).get("models", [])


def _recv_all(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(_RECV_CHUNK, n - len(buf)))
        if not chunk:
            msg = "Client disconnected"
            raise ConnectionError(msg)
        buf += chunk
    return buf


def _handle_client(conn: socket.socket, context: ServerContext, lock: threading.Lock) -> None:
    with conn:
        try:
            msg_len = struct.unpack(">I", _recv_all(conn, 4))[0]
            msg = pickle.loads(_recv_all(conn, msg_len))
            with lock:
                resp = dispatch(msg, context)
        except Exception as exc:
            resp = {"error": str(exc)}

        try:
            payload = pickle.dumps(resp)
            conn.sendall(struct.pack(">I", len(payload)) + payload)
        except Exception:  # noqa: S110
            pass


def _cleanup_server() -> None:
    """Clean up server socket on shutdown."""
    sock = _server_state.socket
    if sock is not None:
        with contextlib.suppress(OSError):
            sock.shutdown(socket.SHUT_RDWR)
        with contextlib.suppress(OSError):
            sock.close()
        _server_state.socket = None
        logger.info("Server socket closed")


def run_server(default_model: str | None = None) -> None:
    """Bind the TCP socket immediately, then load the SAM model in the background.

    Binding first means the FastAPI API can connect and start serving
    non-inference requests (gallery, ping, etc.) while the model loads.
    Inference commands that arrive before loading completes are answered
    with a structured ``"No model loaded"`` error (see :func:`dispatch`).
    """
    start = time.monotonic()

    os.environ.setdefault("HF_HUB_CACHE", str(MODELS_DIR))
    device = "cuda" if torch.cuda.is_available() else "cpu"

    lock = threading.Lock()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    _server_state.socket = srv
    atexit.register(_cleanup_server)

    try:
        srv.bind((SERVER_HOST, SERVER_PORT))
        srv.listen(16)
        logger.info("TCP socket bound on %s:%s – loading model in background.", SERVER_HOST, SERVER_PORT)

        resolved_default = default_model or os.environ.get("DEFAULT_MODEL", "") or "sam2.1-large"
        context = ServerContext(models_config=_load_config(), device=device)

        def _load_model() -> None:
            initial_load(context, context.models_config, resolved_default)
            elapsed = time.monotonic() - start
            logger.info(
                "Model ready in %.2fs on %s:%s device=%s model=%s",
                elapsed,
                SERVER_HOST,
                SERVER_PORT,
                device,
                context.model_id or "FAILED",
            )

        threading.Thread(target=_load_model, daemon=True, name="model-loader").start()

        while True:
            conn, _ = srv.accept()
            threading.Thread(
                target=_handle_client,
                args=(conn, context, lock),
                daemon=True,
            ).start()
    finally:
        _cleanup_server()
