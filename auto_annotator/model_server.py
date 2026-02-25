"""
model_server.py – SAM inference microserver (thin entrypoint)
──────────────────────────────────────────────────────────────
Loads one SAM model at a time and stays running.  app.py connects over TCP
so the model never needs to be reloaded when you restart app.py.

Usage:
    uv run python model_server.py

Environment variables:
    MODEL_SERVER_PORT   TCP port (default 8765)
    MODELS_DIR          directory containing checkpoints (default ./models)
    HF_HUB_CACHE        HuggingFace cache dir (default same as MODELS_DIR)
    HF_TOKEN            HuggingFace token (required for gated models like SAM 3)
    MODELS_CONFIG       path to models.toml (default ./config/models.toml)
    DEFAULT_MODEL       model id to load on startup (default: first available)
"""

import os
import pickle
import socket
import struct
import threading
import time as _time
import tomllib
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

_SERVER_START = _time.monotonic()

import torch  # noqa: E402

from server.context import ServerContext  # noqa: E402
from server.dispatch import dispatch  # noqa: E402
from server.loader import initial_load  # noqa: E402
from src.constants import CONFIG_FILE, MODELS_DIR, SERVER_HOST, SERVER_PORT  # noqa: E402

# ── Config ────────────────────────────────────────────────────────────────────

os.environ.setdefault("HF_HUB_CACHE", str(MODELS_DIR))
DEFAULT_MODEL = os.environ.get("DEFAULT_MODEL", "")
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def _load_config() -> list[dict]:
    if not CONFIG_FILE.exists():
        print(f"[server] Warning: {CONFIG_FILE} not found — no models configured.")  # noqa: T201
        return []
    with CONFIG_FILE.open("rb") as f:
        return tomllib.load(f).get("models", [])


# ── Build server context ───────────────────────────────────────────────────────

ctx = ServerContext(models_config=_load_config(), device=DEVICE)
initial_load(ctx, DEFAULT_MODEL)


# ── TCP helpers ────────────────────────────────────────────────────────────────

def _recv_all(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(65536, n - len(buf)))
        if not chunk:
            msg = "Client disconnected"
            raise ConnectionError(msg)
        buf += chunk
    return buf


_lock = threading.Lock()


def _handle_client(conn: socket.socket) -> None:
    with conn:
        try:
            msg_len = struct.unpack(">I", _recv_all(conn, 4))[0]
            msg = pickle.loads(_recv_all(conn, msg_len))  # noqa: S301
            with _lock:
                resp = dispatch(msg, ctx)
        except Exception as exc:  # noqa: BLE001
            resp = {"error": str(exc)}

        try:
            payload = pickle.dumps(resp)
            conn.sendall(struct.pack(">I", len(payload)) + payload)
        except Exception:  # noqa: BLE001
            pass


# ── Main loop ──────────────────────────────────────────────────────────────────

def main() -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_HOST, SERVER_PORT))
    srv.listen(16)
    elapsed = _time.monotonic() - _SERVER_START
    print(  # noqa: T201
        f"[server] Ready in {elapsed:.2f}s  "
        f"on {SERVER_HOST}:{SERVER_PORT}  device={DEVICE}  "
        f"model={ctx.model_id or 'FAILED'}"
    )
    print("[server] Leave this running and restart app.py freely.")  # noqa: T201

    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_handle_client, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
