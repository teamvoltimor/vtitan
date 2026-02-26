"""src.server.main – SAM inference microserver runner.

Loads one SAM model at a time and keeps running so app.py can restart freely
without reloading the model.  Called from the root ``main.py`` when invoked in
``server`` mode.

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


def _handle_client(conn: socket.socket, ctx: ServerContext, lock: threading.Lock) -> None:
    with conn:
        try:
            msg_len = struct.unpack(">I", _recv_all(conn, 4))[0]
            msg = pickle.loads(_recv_all(conn, msg_len))
            with lock:
                resp = dispatch(msg, ctx)
        except Exception as exc:
            resp = {"error": str(exc)}

        try:
            payload = pickle.dumps(resp)
            conn.sendall(struct.pack(">I", len(payload)) + payload)
        except Exception:  # noqa: S110
            pass


def run_server() -> None:
    """Initialise the model and start the TCP server loop."""
    start = time.monotonic()

    os.environ.setdefault("HF_HUB_CACHE", str(MODELS_DIR))
    default_model = os.environ.get("DEFAULT_MODEL", "")
    device = "cuda" if torch.cuda.is_available() else "cpu"

    ctx = ServerContext(models_config=_load_config(), device=device)
    initial_load(ctx, default_model)

    lock = threading.Lock()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_HOST, SERVER_PORT))
    srv.listen(16)

    elapsed = time.monotonic() - start
    logger.info(
        "Ready in %.2fs on %s:%s device=%s model=%s",
        elapsed,
        SERVER_HOST,
        SERVER_PORT,
        device,
        ctx.model_id or "FAILED",
    )
    logger.info("Leave this running and restart main.py app freely.")

    while True:
        conn, _ = srv.accept()
        threading.Thread(
            target=_handle_client,
            args=(conn, ctx, lock),
            daemon=True,
        ).start()
