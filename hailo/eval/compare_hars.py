"""Score compiled HARs against each other in quantized emulation.

Runs *inside* the Hailo AI Software Suite container -- ``hailo_sdk_client`` only
exists there. ``task eval:compare`` copies this directory into the shared mount
and invokes it via ``docker exec``.

Emulation is used rather than the HEF because running a HEF needs the physical
Hailo-8; ``SDK_QUANTIZED`` executes the same quantized graph the HEF was built
from, on the host GPU.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from hailo_sdk_client import ClientRunner, InferenceContext
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from metrics import (
    IMAGE_SIZE,
    Prediction,
    letterbox,
    load_ground_truth,
    print_report,
    sample,
    summarise,
)

SHARED = Path("/local/shared_with_docker")


def decode(per_image: np.ndarray, min_score: float) -> list[Prediction]:
    """Flatten one image's NMS output into scored ``xyxy`` predictions.

    The ``nms_postprocess`` layer emits ``(n_classes, 5, max_boxes)`` where the
    five rows are ``[y_min, x_min, y_max, x_max, score]``, normalised to the
    input size.
    """
    rows: list[Prediction] = []
    for class_id, detections in enumerate(per_image):
        for index in range(detections.shape[1]):
            score = float(detections[4, index])
            if score < min_score:
                continue
            y1, x1, y2, x2 = detections[0:4, index]
            rows.append((class_id, score, np.array([x1, y1, x2, y2]) * IMAGE_SIZE))
    rows.sort(key=lambda row: -row[1])
    return rows


def parse_args() -> argparse.Namespace:
    """Parse the evaluator's command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=300, help="Number of images to evaluate")
    parser.add_argument("--images", default=str(SHARED / "calib_data_gmr"))
    parser.add_argument("--labels", default=str(SHARED / "calib_labels_gmr"))
    parser.add_argument("--conf", type=float, default=0.01, help="Score floor for the metrics")
    parser.add_argument(
        "--har",
        action="append",
        metavar="NAME=PATH",
        help="HAR to score; repeatable. Defaults to the two GMR builds.",
    )
    return parser.parse_args()


def main() -> None:
    """Score every requested HAR over the same images and print the report."""
    args = parse_args()
    specs = args.har or [
        f"cpu_opt0={SHARED / 'gmr_cpu_opt0.har'}",
        f"gpu_qat={SHARED / 'gmr_gpu_qat.har'}",
    ]

    image_dir, label_dir = Path(args.images), Path(args.labels)
    files = sample(sorted(image_dir.glob("*")), args.limit)
    print(f"Evaluating {len(files)} images from {image_dir}", flush=True)

    batch, truth = [], []
    for path in files:
        img = Image.open(path).convert("RGB")
        canvas, scale, pad_x, pad_y = letterbox(img)
        batch.append(canvas)
        truth.append(load_ground_truth(label_dir, path.stem, img.size, (scale, pad_x, pad_y)))
    stacked = np.stack(batch)
    print(f"Ground-truth boxes: {sum(len(t) for t in truth)}", flush=True)

    results = {}
    for spec in specs:
        name, _, har_path = spec.partition("=")
        print(f"--- {name} ---", flush=True)
        runner = ClientRunner(har=har_path)
        with runner.infer_context(InferenceContext.SDK_QUANTIZED) as context:
            outputs = np.asarray(runner.infer(context, stacked, batch_size=8))
        predictions = [decode(outputs[i], args.conf) for i in range(len(truth))]
        results[name] = summarise(predictions, truth)
        print(f"{name}: mAP@0.5 = {results[name]['mAP50']:.4f}", flush=True)

    print_report(results)


if __name__ == "__main__":
    main()
