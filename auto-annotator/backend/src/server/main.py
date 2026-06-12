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
import socket
import struct
import threading
import time
import tomllib
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.config import AppConfig

import torch

from src.server import wire
from src.server.context import ServerContext
from src.server.dispatch import dispatch
from src.server.loader import initial_load
from src.utils import get_logger

logger = get_logger(__name__)


class _ServerState:
    socket: socket.socket | None = None


_server_state = _ServerState()


def _load_models_config(config_file: Path) -> list[dict]:
    if not config_file.exists():
        logger.warning("Config not found, no models configured: %s", config_file)
        return []
    with config_file.open("rb") as f:
        return tomllib.load(f).get("models", [])


def _handle_client(conn: socket.socket, context: ServerContext, lock: threading.Lock) -> None:
    with conn:
        try:
            msg = wire.recv(conn)
            with lock:
                resp = dispatch(msg, context)
        except Exception as exc:
            resp = {"error": str(exc)}

        try:
            wire.send(conn, resp)
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


def run_server(config: AppConfig | None = None) -> None:
    """Bind the TCP socket immediately, then load the SAM model in the background.

    Binding first means the FastAPI API can connect and start serving
    non-inference requests (gallery, ping, etc.) while the model loads.
    Inference commands that arrive before loading completes are answered
    with a structured ``"No model loaded"`` error (see :func:`dispatch`).
    """
    from src.config import AppConfig as _AppConfig

    cfg = config or _AppConfig.load()
    server_host = cfg.server.host
    server_port = cfg.server.port
    models_dir = cfg.paths.models_dir
    config_file = cfg.paths.config_file
    default_model = cfg.inference.default_model or None

    start = time.monotonic()

    os.environ.setdefault("HF_HUB_CACHE", str(models_dir))
    device = "cuda" if torch.cuda.is_available() else "cpu"

    lock = threading.Lock()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    _server_state.socket = srv
    atexit.register(_cleanup_server)

    try:
        srv.bind((server_host, server_port))
        srv.listen(16)
        logger.info("TCP socket bound on %s:%s – loading model in background.", server_host, server_port)

        resolved_default = default_model or "sam2.1-large"
        context = ServerContext(models_config=_load_models_config(config_file), device=device)

        def _load_model() -> None:
            initial_load(context, context.models_config, resolved_default)
            elapsed = time.monotonic() - start
            logger.info(
                "Model ready in %.2fs on %s:%s device=%s model=%s",
                elapsed,
                server_host,
                server_port,
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
