"""
model_server.py  –  SAM 2.1 inference microserver
──────────────────────────────────────────────────
Loads the model ONCE and stays running.  app.py connects to it over TCP
on localhost:8765 so the model never has to be reloaded when you restart
or edit app.py.

Usage
-----
    # terminal 1 – start once, leave running
    uv run python model_server.py

    # terminal 2 – restart as many times as you like
    uv run python app.py

Environment variables
---------------------
    MODEL_SERVER_PORT   TCP port (default 8765)
    MODELS_DIR          directory that contains sam2.1_l.pt (optional)
    HF_HUB_CACHE        HuggingFace cache dir (optional)

Protocol
--------
    All messages are length-prefixed pickle over TCP:
        4 bytes big-endian uint32  – payload length
        N bytes                    – pickle.dumps(dict)

    Commands:
        {"cmd": "ping"}
            → {"ok": True, "device": "cuda"|"cpu"}

        {"cmd": "set_image", "image": np.ndarray}   # H×W×3 uint8 RGB
            → {"ok": True}

        {"cmd": "predict", "coords": np.ndarray, "labels": np.ndarray}
            → {"masks": [bool H×W, ...], "scores": [float, ...]}
            # masks are the 3 SAM candidates, fine→coarse
"""

import contextlib
import os
import pickle
import socket
import struct
import threading
import time as _time
_SERVER_START = _time.monotonic()
from pathlib import Path

import numpy as np
import torch

# ── Config ────────────────────────────────────────────────────────────────────

HOST       = "127.0.0.1"
PORT       = int(os.environ.get("MODEL_SERVER_PORT", 8765))
MODELS_DIR = Path(os.environ.get("MODELS_DIR", Path(__file__).parent / "models"))
LOCAL_CKPT = MODELS_DIR / "sam2.1_l.pt"
DEVICE     = "cuda" if torch.cuda.is_available() else "cpu"

# Cache HuggingFace downloads inside models/ so app.py and model_server.py
# share the same weights and never download the model twice.
os.environ.setdefault("HF_HUB_CACHE", str(MODELS_DIR))


# ── Helpers ───────────────────────────────────────────────────────────────────

def _autocast_ctx():
    if DEVICE == "cuda":
        return torch.autocast("cuda", dtype=torch.bfloat16)
    return contextlib.nullcontext()


def _empty_cache():
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


def _recv_all(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(65536, n - len(buf)))
        if not chunk:
            raise ConnectionError("Client disconnected")
        buf += chunk
    return buf


# ── SAM 2.1 loading ───────────────────────────────────────────────────────────

predictor  = None
USE_NATIVE = False

try:
    from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore

    if LOCAL_CKPT.exists():
        from sam2.build_sam import build_sam2  # type: ignore
        import sam2 as _sam2_pkg

        _cfg_dir = Path(_sam2_pkg.__file__).parent / "configs" / "sam2.1"
        _cfg     = str(_cfg_dir / "sam2.1_hiera_l.yaml")
        _model   = build_sam2(_cfg, str(LOCAL_CKPT), device=DEVICE)
        predictor = SAM2ImagePredictor(_model)
        print(f"[server] Loaded local checkpoint: {LOCAL_CKPT}")
    else:
        predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2.1-hiera-large")
        print("[server] Loaded from HuggingFace: facebook/sam2.1-hiera-large")

    USE_NATIVE = True

except Exception as _err:
    print(f"[server] FATAL: could not load SAM2 — {_err}")


# ── Request dispatch ──────────────────────────────────────────────────────────

_lock = threading.Lock()   # SAM2 predictor is not thread-safe


def _dispatch(msg: dict) -> dict:
    cmd = msg.get("cmd")

    if cmd == "ping":
        return {"ok": True, "device": DEVICE, "model_loaded": predictor is not None}

    if predictor is None:
        return {"error": "Model not loaded"}

    if cmd == "set_image":
        img = msg["image"]   # numpy H×W×3 uint8 RGB
        with torch.inference_mode(), _autocast_ctx():
            predictor.set_image(img)
        _empty_cache()
        return {"ok": True}

    if cmd == "predict":
        coords     = msg["coords"]              # float32 (N, 2)
        labels     = msg["labels"]              # int32   (N,)
        mask_input = msg.get("mask_input")      # float32 (1, H', W') or None
        with torch.inference_mode(), _autocast_ctx():
            masks, scores, logits = predictor.predict(
                point_coords=coords,
                point_labels=labels,
                mask_input=mask_input,
                multimask_output=True,
            )
        _empty_cache()
        scores_flat = scores.flatten()
        return {
            "masks":  [masks[i].astype(bool) for i in range(len(masks))],
            "scores": scores_flat.tolist(),
            "logits": logits,   # (N, 1, H', W') – for iterative refinement
        }

    return {"error": f"Unknown command: {cmd!r}"}


# ── Connection handler ────────────────────────────────────────────────────────

def _handle_client(conn: socket.socket):
    with conn:
        try:
            msg_len = struct.unpack(">I", _recv_all(conn, 4))[0]
            msg     = pickle.loads(_recv_all(conn, msg_len))
            with _lock:
                resp = _dispatch(msg)
        except Exception as exc:
            resp = {"error": str(exc)}

        try:
            payload = pickle.dumps(resp)
            conn.sendall(struct.pack(">I", len(payload)) + payload)
        except Exception:
            pass


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(16)
    print(f"[server] Ready in {_time.monotonic() - _SERVER_START:.2f}s  "
          f"on {HOST}:{PORT}  device={DEVICE}  "
          f"model={'loaded' if predictor is not None else 'FAILED'}")
    print("[server] Leave this running and restart app.py freely.")

    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_handle_client, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
