"""Score the float checkpoint, giving the quantized results a ceiling.

Runs on the *host* with ultralytics. Without this anchor a quantized mAP is
uninterpretable: it says nothing about whether the build is near-lossless or
has given up real accuracy. Scores through the same preprocessing, label remap
and AP maths as ``compare_hars.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from ultralytics import YOLO

sys.path.insert(0, str(Path(__file__).resolve().parent))

from metrics import (
    Prediction,
    letterbox,
    load_ground_truth,
    print_report,
    sample,
    summarise,
)

from src.constants import GMR_CHECKPOINT_PATH


def parse_args() -> argparse.Namespace:
    """Parse the evaluator's command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=300, help="Number of images to evaluate")
    parser.add_argument("--images", default="shared_with_docker/calib_data_gmr")
    parser.add_argument("--labels", default="shared_with_docker/calib_labels_gmr")
    parser.add_argument("--checkpoint", default=GMR_CHECKPOINT_PATH)
    parser.add_argument("--conf", type=float, default=0.01, help="Score floor for the metrics")
    return parser.parse_args()


def main() -> None:
    """Score the float checkpoint and print the report."""
    args = parse_args()
    image_dir, label_dir = Path(args.images), Path(args.labels)
    files = sample(sorted(image_dir.glob("*")), args.limit)
    print(f"Evaluating {len(files)} images from {image_dir}")

    batch, truth = [], []
    for path in files:
        img = Image.open(path).convert("RGB")
        canvas, scale, pad_x, pad_y = letterbox(img)
        batch.append(canvas)
        truth.append(load_ground_truth(label_dir, path.stem, img.size, (scale, pad_x, pad_y)))
    print(f"Ground-truth boxes: {sum(len(t) for t in truth)}")

    # Ultralytics reads numpy input as BGR (OpenCV convention). Feeding RGB
    # swaps the red and blue channels and collapses the red class specifically
    # -- AP 0.99 to 0.17 -- which reads as "quantization beat float" rather
    # than as the harness bug it is.
    bgr = [canvas[:, :, ::-1] for canvas in batch]

    model = YOLO(args.checkpoint)
    predictions: list[list[Prediction]] = []
    for start in range(0, len(bgr), 16):
        for result in model.predict(bgr[start : start + 16], conf=args.conf, verbose=False):
            boxes = result.boxes
            # An image with no detections still needs an entry, so the
            # predictions stay index-aligned with the ground truth.
            if boxes is None:
                predictions.append([])
                continue
            rows: list[Prediction] = [
                (int(class_id), float(score), np.asarray(box))
                for class_id, score, box in zip(
                    boxes.cls.tolist(),
                    boxes.conf.tolist(),
                    boxes.xyxy.tolist(),
                    strict=True,
                )
            ]
            rows.sort(key=lambda row: -row[1])
            predictions.append(rows)

    print_report({"float": summarise(predictions, truth)})


if __name__ == "__main__":
    main()
