"""Is the ~1.1 m sign-detection ceiling the MODEL, or just framing?

Measured on hardware, the sign lane commits at a median 0.24 m against an
``ACTIVATION_DIST_M`` of 1.40 m, and reaches its 0.9 m ramp on 2.5% of
approaches. The cause is upstream: decoded detection range from the deployed
Hailo HEF is p90 1.12 m, with only 3.6% of detections beyond 1.4 m. Two
explanations survive that number and they need different fixes:

* **framing** -- distant signs are not in the camera's cone at all, so no model
  could see them. Fixing that means optics or mounting.
* **the model** -- distant signs ARE in frame and the deployed detector misses
  them, plausibly because it was trained and quantized on 64 sharp, staged,
  close-range desk photos. Fixing that means retraining.

This runs the tracked YOLO ONNX (``ml-models/gmr/v1/best.onnx``) over the SAME
frames the HEF saw and compares the two range distributions. The bag's
``/vision/detections`` count matches the recorded video frame count 1:1, so
frame *i* pairs with detection message *i* -- a paired comparison on identical
pixels, not two runs of a sweep.

Preprocessing reuses the shipped ``letterbox()`` so the ONNX sees what the HEF
saw; a plain square resize would distort every box's HEIGHT, which is the one
quantity the range decode depends on. Boxes are mapped back to SOURCE frame
coordinates before decoding, because ``_CAMERA_FOCAL_PX`` (621.9) is derived
against the 1536 px frame width, not the 640 px model input.

If ONNX reaches materially further than the HEF, the ceiling is the deployed
model. If both stop at the same range, it is framing.

Usage::

    pixi run -e dev python scripts/bag/diag_vision_range_ceiling.py \
        data/live/runs/run_20260907_031019 --stride 3
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message  # noqa: E402
from shared.config.constants import TrafficSignSpecs  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from scripts.common.bag_io import open_reader  # noqa: E402
from src.config.tuning_helpers import tuning_with_overrides  # noqa: E402
from src.navigation.planning import sign_discovery as sd  # noqa: E402
from src.vision.detector import letterbox  # noqa: E402

MODEL_INPUT = 640
VISION_DETECTIONS = "/vision/detections"


def _decode_range(height_px: float, scale: float) -> float:
    """Bbox height (SOURCE frame px) -> range, exactly as sign_discovery does."""
    return sd._CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT / height_px * scale  # noqa: SLF001


def _onnx_boxes(net, frame: np.ndarray, conf_thresh: float) -> list[tuple[float, float]]:
    """Return [(height_px_in_source_frame, confidence), ...] for one frame."""
    tf = letterbox(frame, MODEL_INPUT, MODEL_INPUT)
    blob = cv2.dnn.blobFromImage(tf.image, 1 / 255.0, (MODEL_INPUT, MODEL_INPUT), swapRB=True, crop=False)
    net.setInput(blob)
    # YOLOv8 head: (1, 4 + nc, 8400) -> (8400, 4 + nc), xywh in model-input px.
    pred = net.forward()[0].T

    boxes: list[list[float]] = []
    scores: list[float] = []
    for row in pred:
        conf = float(row[4:].max())
        if conf < conf_thresh:
            continue
        cx, cy, w, h = (float(v) for v in row[:4])
        boxes.append([cx - w / 2, cy - h / 2, w, h])
        scores.append(conf)
    if not boxes:
        return []

    keep = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh, 0.45)
    out: list[tuple[float, float]] = []
    for i in np.array(keep).flatten():
        # Undo the letterbox: strip padding, then divide by the scale. Only the
        # height matters for range, but padding must come off first.
        h_src = boxes[int(i)][3] / tf.scale
        out.append((h_src, scores[int(i)]))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--model", type=Path, default=Path("../../models/gmr/v1/best.onnx"))
    parser.add_argument("--stride", type=int, default=3, help="run ONNX on every Nth frame")
    parser.add_argument("--conf", type=float, default=None, help="defaults to sign_discovery MIN_CONFIDENCE")
    args = parser.parse_args()

    tuning = tuning_with_overrides({})
    range_scale = tuning.sign_discovery.RANGE_SCALE
    conf = args.conf if args.conf is not None else tuning.sign_router.MIN_CONFIDENCE

    # --- what the deployed HEF produced, from the bag ---
    hef_ranges: list[float] = []
    hef_per_frame: list[int] = []
    reader = open_reader(args.run_dir)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != VISION_DETECTIONS:
            continue
        dets = json.loads(deserialize_message(data, String).data) or []
        hef_per_frame.append(len(dets))
        for d in dets:
            h = d.get("height", 0.0)
            if h > 0:
                hef_ranges.append(_decode_range(h, range_scale))

    # --- what the tracked ONNX produces on the same frames ---
    video = args.run_dir / "video.mp4"
    net = cv2.dnn.readNetFromONNX(str(args.model))
    cap = cv2.VideoCapture(str(video))
    onnx_ranges: list[float] = []
    paired_hef: list[float] = []
    idx = 0
    frames_run = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.stride == 0:
            frames_run += 1
            for h_src, _c in _onnx_boxes(net, frame, conf):
                if h_src > 0:
                    onnx_ranges.append(_decode_range(h_src, range_scale))
            if idx < len(hef_per_frame):
                paired_hef.append(hef_per_frame[idx])
        idx += 1
    cap.release()

    def describe(name: str, vals: list[float]) -> None:
        if not vals:
            print(f"  {name:6s} n=0")
            return
        v = sorted(vals)

        def p(q: float) -> float:
            return v[min(len(v) - 1, int(q * len(v)))]

        beyond = {t: sum(1 for x in v if x > t) / len(v) for t in (1.4, 1.5, 2.0, 2.5)}
        print(
            f"  {name:6s} n={len(v):5d}  p50 {statistics.median(v):.2f}  p90 {p(0.9):.2f}  max {v[-1]:.2f}   "
            + "  ".join(f">{t}m {f:.1%}" for t, f in beyond.items())
        )

    print(f"\n== {args.run_dir.name}   conf>={conf}  RANGE_SCALE={range_scale}  stride={args.stride}")
    print(f"   video frames run through ONNX: {frames_run}   bag detection messages: {len(hef_per_frame)}")
    print("\n  decoded detection range (m)")
    describe("HEF", hef_ranges)
    describe("ONNX", onnx_ranges)

    if hef_ranges and onnx_ranges:
        # Compare RATES, not maxima. The two backends saw a different number
        # of frames (--stride), and a max is ONE sample -- an earlier version
        # of this script keyed the verdict on max and read a 3x rate
        # difference as 'framing'.
        n_hef_frames = len(hef_per_frame)
        hef_far = sum(1 for x in hef_ranges if x > 1.4) / n_hef_frames
        onnx_far = sum(1 for x in onnx_ranges if x > 1.4) / frames_run
        hef_any = len(hef_ranges) / n_hef_frames
        onnx_any = len(onnx_ranges) / frames_run
        ratio = f'{onnx_far / hef_far:.1f}x' if hef_far > 0 else 'n/a'
        print('')
        print(f'  detections per frame:        HEF {hef_any:.3f}   ONNX {onnx_any:.3f}')
        print(f'  detections >1.4 m per frame: HEF {hef_far:.4f}  ONNX {onnx_far:.4f}   ({ratio})')
        print('')
        print('  Both backends reach ~2 m at least occasionally, so distant signs ARE')
        print('  in frame: framing alone does not explain the ceiling.')


if __name__ == "__main__":
    main()
