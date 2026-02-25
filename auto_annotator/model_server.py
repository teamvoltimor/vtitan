"""
model_server.py  –  Multi-model SAM inference microserver
──────────────────────────────────────────────────────────
Loads one SAM model at a time and stays running.  app.py connects over TCP
so the model never needs to be reloaded when you restart app.py.

Supported model types: SAM 1, SAM 2 / 2.1, SAM 3
Model list is read from models.toml (or the file set via MODELS_CONFIG).

Usage
-----
    # terminal 1 – start once, leave running
    uv run python model_server.py

    # terminal 2 – restart as many times as you like
    uv run python app.py

Environment variables
---------------------
    MODEL_SERVER_PORT   TCP port (default 8765)
    MODELS_DIR          directory containing checkpoints (default ./models)
    HF_HUB_CACHE        HuggingFace cache dir (default same as MODELS_DIR)
    HF_TOKEN            HuggingFace token (required for gated models like SAM 3)
    MODELS_CONFIG       path to models.toml (default ./models.toml)
    DEFAULT_MODEL       model id to load on startup (default: first available)

Protocol
--------
    All messages are length-prefixed pickle over TCP:
        4 bytes big-endian uint32  – payload length
        N bytes                    – pickle.dumps(dict)

    Commands:
        {"cmd": "ping"}
            → {"ok": True, "device": str, "model_id": str|None}

        {"cmd": "list_models"}
            → {"models": [{id, label, type, available, active, supports_text}, ...]}

        {"cmd": "set_model", "model_id": str}
            → {"ok": True, "model_id": str}  |  {"error": str}

        {"cmd": "set_image", "image": np.ndarray}   # H×W×3 uint8 RGB
            → {"ok": True}

        {"cmd": "predict", "coords": np.ndarray, "labels": np.ndarray,
                           "mask_input": np.ndarray | None}
            → {"masks": [bool H×W, ...], "scores": [float, ...],
               "logits": np.ndarray | None}

        {"cmd": "predict_text", "image": np.ndarray, "class_names": [str, ...]}
            → {"results": [{"class_name": str,
                            "masks": [bool H×W, ...],
                            "scores": [float, ...],
                            "boxes": [[x1,y1,x2,y2], ...]}, ...]}
"""

import contextlib
import os
import pickle
import socket
import struct
import threading
import time as _time
import tomllib
_SERVER_START = _time.monotonic()
from pathlib import Path
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent / ".env")

import numpy as np
import torch

# ── Config ────────────────────────────────────────────────────────────────────

HOST          = "127.0.0.1"
PORT          = int(os.environ.get("MODEL_SERVER_PORT", 8765))
MODELS_DIR    = Path(os.environ.get("MODELS_DIR",    Path(__file__).parent / "models"))
CONFIG_FILE   = Path(os.environ.get("MODELS_CONFIG", Path(__file__).parent / "models.toml"))
DEFAULT_MODEL = os.environ.get("DEFAULT_MODEL", "")
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"

os.environ.setdefault("HF_HUB_CACHE", str(MODELS_DIR))


# ── Model config ───────────────────────────────────────────────────────────────

def _load_config() -> list[dict]:
    if not CONFIG_FILE.exists():
        print(f"[server] Warning: {CONFIG_FILE} not found — no models configured.")
        return []
    with open(CONFIG_FILE, "rb") as f:
        return tomllib.load(f).get("models", [])

_MODELS_CONFIG: list[dict] = _load_config()


def _resolve(p: str | None) -> Path | None:
    if p is None:
        return None
    path = Path(p)
    return path if path.is_absolute() else Path(__file__).parent / path


def _is_available(cfg: dict) -> bool:
    mtype = cfg.get("type", "")
    ckpt  = _resolve(cfg.get("checkpoint"))
    hf    = cfg.get("hf_repo", "")
    if mtype == "sam1":
        return ckpt is not None and ckpt.exists()
    if mtype == "sam2":
        return bool(hf) or (ckpt is not None and ckpt.exists())
    if mtype == "sam3":
        return bool(hf)
    return False


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


# ── SAM3 interactive wrapper ───────────────────────────────────────────────────
# Adapts Sam3TrackerModel (transformers) to the SAM2ImagePredictor interface.

class _SAM3Predictor:
    def __init__(self, model, processor):
        self.model     = model
        self.processor = processor
        self._pil      = None
        self._orig_hw  = None

    def set_image(self, image: np.ndarray) -> None:
        from PIL import Image as _PIL
        self._pil     = _PIL.fromarray(image)
        self._orig_hw = image.shape[:2]

    def predict(self, point_coords, point_labels,
                mask_input=None, multimask_output: bool = True):
        if self._pil is None:
            raise RuntimeError("set_image() must be called before predict()")

        # SAM3 input format: (image_dim, object_dim, point_dim, xy)
        input_points = [[[[float(c[0]), float(c[1])] for c in point_coords]]]
        input_labels = [[[int(l) for l in point_labels]]]

        proc_kwargs = dict(
            images=self._pil,
            input_points=input_points,
            input_labels=input_labels,
            return_tensors="pt",
        )
        if mask_input is not None:
            try:
                proc_kwargs["input_masks"] = [[mask_input]]
            except Exception:
                pass

        inputs = self.processor(**proc_kwargs).to(self.model.device)

        with torch.inference_mode():
            outputs = self.model(**inputs)

        try:
            masks_t = self.processor.post_process_masks(
                outputs.pred_masks,
                inputs["original_sizes"],
                inputs["reshaped_input_sizes"],
            )[0]
        except Exception:
            import torch.nn.functional as _F
            H, W = self._orig_hw
            raw     = outputs.pred_masks[0, :, 0:1].float()
            masks_t = (_F.interpolate(raw, (H, W), mode="bilinear")[:, 0] > 0)

        if masks_t.ndim == 4:
            masks_t = masks_t[:, 0]

        masks_np = masks_t.cpu().bool().numpy()

        if hasattr(outputs, "iou_scores"):
            scores_np = outputs.iou_scores[0].flatten().cpu().float().numpy()
        elif hasattr(outputs, "pred_scores"):
            scores_np = outputs.pred_scores[0].flatten().cpu().float().numpy()
        else:
            scores_np = np.ones(len(masks_np), dtype=np.float32)

        logits = outputs.pred_masks[0].cpu().numpy() if hasattr(outputs, "pred_masks") else None
        return masks_np, scores_np, logits


# ── SAM3 text segmenter ────────────────────────────────────────────────────────
# Uses Sam3Model (transformers) for text-prompted auto-annotation.
# One call per class name; returns all detected instances.

class _SAM3TextSegmenter:
    def __init__(self, hf_repo: str, device: str):
        try:
            from transformers import Sam3Processor, Sam3Model  # type: ignore
            self.processor = Sam3Processor.from_pretrained(hf_repo)
            self.model     = Sam3Model.from_pretrained(hf_repo).to(device)
        except (ImportError, AttributeError):
            from transformers import AutoProcessor, AutoModel  # type: ignore
            self.processor = AutoProcessor.from_pretrained(hf_repo)
            self.model     = AutoModel.from_pretrained(hf_repo).to(device)
        self.device = device

    def segment_by_text(self, image: np.ndarray,
                        class_names: list[str]) -> list[dict]:
        from PIL import Image as _PIL
        pil    = _PIL.fromarray(image)
        H, W   = image.shape[:2]
        results = []

        for class_name in class_names:
            try:
                inputs = self.processor(
                    images=pil,
                    text=[class_name],
                    return_tensors="pt",
                ).to(self.device)

                with torch.inference_mode():
                    outputs = self.model(**inputs)

                try:
                    masks_t = self.processor.post_process_masks(
                        outputs.pred_masks,
                        inputs["original_sizes"],
                        inputs["reshaped_input_sizes"],
                    )[0]
                except Exception:
                    import torch.nn.functional as _F
                    raw = outputs.pred_masks[0, :, 0:1].float()
                    masks_t = (_F.interpolate(raw, (H, W), mode="bilinear")[:, 0] > 0)

                if masks_t.ndim == 4:
                    masks_t = masks_t[:, 0]
                masks_np = masks_t.cpu().bool().numpy()

                if hasattr(outputs, "iou_scores"):
                    scores = outputs.iou_scores[0].flatten().cpu().float().tolist()
                else:
                    scores = [1.0] * len(masks_np)

                boxes: list = []
                if hasattr(outputs, "pred_boxes"):
                    for box in outputs.pred_boxes[0].cpu().float().numpy():
                        cx, cy, bw, bh = box * np.array([W, H, W, H])
                        boxes.append([float(cx - bw / 2), float(cy - bh / 2),
                                      float(cx + bw / 2), float(cy + bh / 2)])

                results.append({
                    "class_name": class_name,
                    "masks":  [masks_np[i] for i in range(len(masks_np))],
                    "scores": scores,
                    "boxes":  boxes,
                })
            except Exception as e:
                results.append({
                    "class_name": class_name,
                    "masks": [], "scores": [], "boxes": [],
                    "error": str(e),
                })

        return results


# ── Model loading ──────────────────────────────────────────────────────────────

active_predictor: object | None = None
active_text_seg:  object | None = None
active_model_id:  str    | None = None


def _unload_current() -> None:
    global active_predictor, active_text_seg
    active_predictor = None
    active_text_seg  = None
    _empty_cache()


def _load_model_by_cfg(cfg: dict) -> tuple:
    """Load a model; returns (predictor, text_segmenter | None)."""
    mtype = cfg.get("type", "")
    ckpt  = _resolve(cfg.get("checkpoint"))
    hf    = cfg.get("hf_repo", "")

    if mtype == "sam1":
        from segment_anything import sam_model_registry, SamPredictor  # type: ignore
        variant = cfg.get("variant", "vit_h")
        sam     = sam_model_registry[variant](checkpoint=str(ckpt))
        sam.to(device=DEVICE)
        print(f"[server] Loaded SAM1 ({variant}): {ckpt}")
        return SamPredictor(sam), None

    if mtype == "sam2":
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore
        if ckpt and ckpt.exists():
            from sam2.build_sam import build_sam2  # type: ignore
            hiera = cfg.get("hiera_config", "configs/sam2.1/sam2.1_hiera_l.yaml")
            model = build_sam2(hiera, str(ckpt), device=DEVICE)
            pred  = SAM2ImagePredictor(model)
            print(f"[server] Loaded SAM2 local checkpoint: {ckpt}")
        else:
            pred = SAM2ImagePredictor.from_pretrained(hf)
            print(f"[server] Loaded SAM2 from HuggingFace: {hf}")
        return pred, None

    if mtype == "sam3":
        from transformers import Sam3TrackerProcessor, Sam3TrackerModel  # type: ignore
        tracker      = Sam3TrackerModel.from_pretrained(hf).to(DEVICE)
        tracker_proc = Sam3TrackerProcessor.from_pretrained(hf)
        pred         = _SAM3Predictor(tracker, tracker_proc)
        print(f"[server] Loaded SAM3 interactive from HuggingFace: {hf}")

        text_seg = None
        if cfg.get("supports_text"):
            try:
                text_seg = _SAM3TextSegmenter(hf, DEVICE)
                print("[server] SAM3 text mode ready.")
            except Exception as e:
                print(f"[server] SAM3 text mode unavailable: {e}")

        return pred, text_seg

    raise ValueError(f"Unknown model type: {mtype!r}")


def _load_model(model_id: str) -> str | None:
    """
    Load a model by id.  Returns an error string on failure, None on success.
    Updates active_predictor / active_text_seg / active_model_id globals.
    """
    global active_predictor, active_text_seg, active_model_id

    cfg = next((c for c in _MODELS_CONFIG if c["id"] == model_id), None)
    if cfg is None:
        return f"Unknown model: {model_id!r}"
    if not _is_available(cfg):
        return f"Model {model_id!r} not available (checkpoint missing?)"

    try:
        pred, text_seg = _load_model_by_cfg(cfg)
        _unload_current()
        active_predictor = pred
        active_text_seg  = text_seg
        active_model_id  = model_id
        return None
    except Exception as e:
        return f"Failed to load {model_id}: {e}"


def _initial_load() -> None:
    candidates = []
    if DEFAULT_MODEL:
        candidates.append(DEFAULT_MODEL)
    candidates.extend(c["id"] for c in _MODELS_CONFIG)

    for model_id in dict.fromkeys(candidates):   # deduplicate, preserve order
        cfg = next((c for c in _MODELS_CONFIG if c["id"] == model_id), None)
        if cfg and _is_available(cfg):
            err = _load_model(model_id)
            if err is None:
                return
            print(f"[server] {err}")

    print("[server] FATAL: no models could be loaded.")

_initial_load()


# ── Request dispatch ───────────────────────────────────────────────────────────

_lock = threading.Lock()   # predictors are not thread-safe


def _dispatch(msg: dict) -> dict:
    cmd = msg.get("cmd")

    if cmd == "ping":
        return {
            "ok":           True,
            "device":       DEVICE,
            "model_loaded": active_predictor is not None,
            "model_id":     active_model_id,
        }

    if cmd == "list_models":
        return {
            "models": [
                {
                    "id":            cfg["id"],
                    "label":         cfg.get("label", cfg["id"]),
                    "type":          cfg.get("type", ""),
                    "available":     _is_available(cfg),
                    "active":        cfg["id"] == active_model_id,
                    "supports_text": cfg.get("supports_text", False),
                }
                for cfg in _MODELS_CONFIG
            ]
        }

    if cmd == "set_model":
        model_id = msg.get("model_id", "")
        err = _load_model(model_id)
        if err:
            return {"error": err}
        return {"ok": True, "model_id": model_id}

    if active_predictor is None:
        return {"error": "No model loaded"}

    if cmd == "set_image":
        img = msg["image"]
        with torch.inference_mode(), _autocast_ctx():
            active_predictor.set_image(img)
        _empty_cache()
        return {"ok": True}

    if cmd == "predict":
        coords     = msg["coords"]
        labels     = msg["labels"]
        mask_input = msg.get("mask_input")
        with torch.inference_mode(), _autocast_ctx():
            masks, scores, logits = active_predictor.predict(
                point_coords=coords,
                point_labels=labels,
                mask_input=mask_input,
                multimask_output=True,
            )
        _empty_cache()
        scores_flat = np.array(scores).flatten()
        return {
            "masks":  [masks[i].astype(bool) for i in range(len(masks))],
            "scores": scores_flat.tolist(),
            "logits": logits,
        }

    if cmd == "predict_text":
        if active_text_seg is None:
            return {"error": "Active model does not support text prompts"}
        results = active_text_seg.segment_by_text(msg["image"], msg["class_names"])
        return {"results": results}

    return {"error": f"Unknown command: {cmd!r}"}


# ── Connection handler ─────────────────────────────────────────────────────────

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


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(16)
    print(f"[server] Ready in {_time.monotonic() - _SERVER_START:.2f}s  "
          f"on {HOST}:{PORT}  device={DEVICE}  "
          f"model={active_model_id or 'FAILED'}")
    print("[server] Leave this running and restart app.py freely.")

    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_handle_client, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
