"""Detection metrics shared by the quantized and float evaluators.

Deliberately free of both the Hailo SDK and ultralytics: this module is copied
into the container alongside ``compare_hars.py`` and imported directly by
``float_anchor.py`` on the host, so the two evaluators score identically and a
difference between them can only come from the model.
"""

from __future__ import annotations

import operator
from collections import defaultdict
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

if TYPE_CHECKING:
    from pathlib import Path

# Square input resolution the GMR detector was exported at.
IMAGE_SIZE = 640

# Ground-truth files are written in the dataset's class space while the model
# predicts in its own. The checkpoint's metadata is authoritative: running it
# per class folder predicts "green" on green_prism and "red" on red_prism.
# auto-annotator's data.yaml disagrees and is stale -- taking the order from
# there swaps red and green, which inverts the WRO pass-side rule.
LABEL_TO_MODEL = {0: 2, 1: 0, 2: 1}
MODEL_NAMES = {0: "green", 1: "magenta", 2: "red"}

# Letterbox canvas configuration
LETTERBOX_CANVAS_COLOR = (114, 114, 114)

# Average precision evaluation thresholds (IoU ranges)
AP_THRESHOLD_START = 0.5
AP_THRESHOLD_END = 1.0
AP_THRESHOLD_STEP = 0.05

# Confusion matrix thresholds (detection filtering)
CONFUSION_MIN_SCORE = 0.25
CONFUSION_MIN_OVERLAP = 0.5


@dataclass(frozen=True, slots=True)
class MetricsResult:
    """Typed result bundle returned by :func:`summarise`."""

    mAP50: float
    mAP75: float
    mAP50_95: float
    per_class: dict[int, float]
    per_class_5095: dict[int, float]
    confusion: dict[tuple[int, int], int]

    def to_dict(self) -> dict[str, object]:
        """Serialize to a plain dict for JSON/Docker-bound transport."""
        return {
            "mAP50": self.mAP50,
            "mAP75": self.mAP75,
            "mAP50_95": self.mAP50_95,
            "per_class": self.per_class,
            "per_class_5095": self.per_class_5095,
            "confusion": self.confusion,
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> MetricsResult:
        """Deserialize from a dict produced by :meth:`to_dict`."""
        return cls(
            mAP50=float(data["mAP50"]),
            mAP75=float(data["mAP75"]),
            mAP50_95=float(data["mAP50_95"]),
            per_class=dict(data["per_class"]),  # type: ignore[arg-type]
            per_class_5095=dict(data["per_class_5095"]),  # type: ignore[arg-type]
            confusion=dict(data["confusion"]),  # type: ignore[arg-type]
        )


# Pillow moved the resampling enum in 9.1; the suite image predates the move.
_BILINEAR = getattr(Image, "Resampling", Image).BILINEAR

Box = np.ndarray  # xyxy float[4]
Prediction = tuple[int, float, Box]  # class_id, score, xyxy
GroundTruthItem = tuple[int, Box]  # class_id, xyxy
GroundTruth = list[GroundTruthItem]


@dataclass
class LetterboxResult:
    """Result of letterboxing an image to a square canvas.

    Used by metrics.py for preprocessing before model inference.
    """

    canvas: np.ndarray
    scale: float
    pad_x: int
    pad_y: int


def letterbox(img: Image.Image) -> LetterboxResult:
    """Resize onto a square grey canvas without distorting the aspect ratio.

    Args:
        img: Source image in RGB.

    Returns:
        LetterboxResult with canvas array, scale applied, and x/y padding
        sufficient to map ground-truth boxes into the same space.
    """
    width, height = img.size
    scale = min(IMAGE_SIZE / width, IMAGE_SIZE / height)
    new_w, new_h = round(width * scale), round(height * scale)
    canvas = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE), LETTERBOX_CANVAS_COLOR)
    pad_x, pad_y = (IMAGE_SIZE - new_w) // 2, (IMAGE_SIZE - new_h) // 2
    canvas.paste(img.resize((new_w, new_h), _BILINEAR), (pad_x, pad_y))
    return LetterboxResult(
        canvas=np.asarray(canvas, dtype=np.uint8),
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
    )


def load_ground_truth(
    label_dir: Path,
    stem: str,
    size: tuple[int, int],
    placement: tuple[float, int, int],
) -> GroundTruth:
    """Read one YOLO label file into letterboxed pixel coordinates.

    Args:
        label_dir: Directory of flattened ``.txt`` label files.
        stem: Image filename stem, matched against the label filename.
        size: Original ``(width, height)`` of the image.
        placement: The ``(scale, pad_x, pad_y)`` returned by :func:`letterbox`.

    Returns:
        List of ``(model_class_id, xyxy)`` pairs; empty when unlabelled.
    """
    path = label_dir / f"{stem}.txt"
    if not path.exists():
        return []

    width, height = size
    scale, pad_x, pad_y = placement
    boxes: GroundTruth = []
    for line in path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        class_id = int(parts[0])
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        boxes.append(
            (
                LABEL_TO_MODEL[class_id],
                np.array(
                    [
                        (cx - box_w / 2) * width * scale + pad_x,
                        (cy - box_h / 2) * height * scale + pad_y,
                        (cx + box_w / 2) * width * scale + pad_x,
                        (cy + box_h / 2) * height * scale + pad_y,
                    ],
                ),
            ),
        )
    return boxes


def iou(a: Box, b: Box) -> float:
    """Intersection over union of two ``xyxy`` boxes."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if intersection <= 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return float(intersection / (area_a + area_b - intersection))


def average_precision(scored: list[tuple[float, bool]], n_ground_truth: int) -> float:
    """All-point-interpolated AP from ``(score, is_true_positive)`` pairs.

    Args:
        scored: One entry per prediction, in any order.
        n_ground_truth: Number of ground-truth boxes of this class.

    Returns:
        AP in ``[0, 1]``, or NaN when the class has no ground truth at all
        (so it can be excluded from the mean rather than counted as zero).
    """
    if n_ground_truth == 0:
        return float("nan")
    if not scored:
        return 0.0

    ordered = sorted(scored, key=operator.itemgetter(0), reverse=True)
    true_positives = np.cumsum([hit for _, hit in ordered])
    false_positives = np.cumsum([not hit for _, hit in ordered])
    recall = true_positives / n_ground_truth
    precision = true_positives / np.maximum(true_positives + false_positives, 1e-9)
    # Monotonic envelope, then integrate precision over the recall steps.
    precision = np.maximum.accumulate(precision[::-1])[::-1]
    steps = np.diff(np.concatenate(([0.0], recall)))
    taken = np.where(steps > 0)[0]
    return float(np.sum(steps[taken] * precision[taken]))


def _match(preds: list[Prediction], truth: GroundTruth, threshold: float) -> list[tuple[int, float, bool]]:
    """Greedily match predictions to ground truth of the same class."""
    matched: list[tuple[int, float, bool]] = []
    taken: set[int] = set()
    for class_id, score, box in preds:
        best, best_index = 0.0, -1
        for index, (true_class, true_box) in enumerate(truth):
            if true_class != class_id or index in taken:
                continue
            overlap = iou(box, true_box)
            if overlap > best:
                best, best_index = overlap, index
        hit = best >= threshold
        if hit:
            taken.add(best_index)
        matched.append((class_id, score, hit))
    return matched


def mean_average_precision(
    predictions: list[list[Prediction]],
    truth: list[GroundTruth],
    threshold: float,
) -> tuple[float, dict[int, float]]:
    """MAP at one IoU threshold, with its per-class breakdown.

    Args:
        predictions: Per-image predictions, each sorted by descending score.
        truth: Per-image ground truth, parallel to *predictions*.
        threshold: IoU above which a prediction counts as a hit.

    Returns:
        Tuple of the mean over classes that have ground truth, and the
        per-class AP mapping.
    """
    scored: dict[int, list[tuple[float, bool]]] = defaultdict(list)
    counts: dict[int, int] = defaultdict(int)
    for image_truth in truth:
        for class_id, _ in image_truth:
            counts[class_id] += 1

    for preds, image_truth in zip(predictions, truth, strict=True):
        for class_id, score, hit in _match(preds, image_truth, threshold):
            scored[class_id].append((score, hit))

    per_class = {class_id: average_precision(scored[class_id], counts[class_id]) for class_id in sorted(counts)}
    valid = [value for value in per_class.values() if not np.isnan(value)]
    return (float(np.mean(valid)) if valid else float("nan")), per_class


def confusion(
    predictions: list[list[Prediction]],
    truth: list[GroundTruth],
    min_score: float = CONFUSION_MIN_SCORE,
    min_overlap: float = CONFUSION_MIN_OVERLAP,
) -> dict[tuple[int, int], int]:
    """Count confident detections by ``(true_class, predicted_class)``.

    mAP conflates failure modes that matter differently here: a red sign read
    as green inverts the pass side, while a red sign merely missed does not.
    Class ``-1`` stands in for "no counterpart" -- ``(-1, c)`` is a false
    positive and ``(c, -1)`` a missed detection.
    """
    counts: dict[tuple[int, int], int] = defaultdict(int)
    for preds, image_truth in zip(predictions, truth, strict=True):
        taken: set[int] = set()
        for class_id, score, box in preds:
            if score < min_score:
                continue
            best, best_index = min_overlap, -1
            for index, (_, true_box) in enumerate(image_truth):
                if index in taken:
                    continue
                overlap = iou(box, true_box)
                if overlap >= best:
                    best, best_index = overlap, index
            if best_index >= 0:
                taken.add(best_index)
                counts[(image_truth[best_index][0], class_id)] += 1
            else:
                counts[(-1, class_id)] += 1
        for index, (true_class, _) in enumerate(image_truth):
            if index not in taken:
                counts[(true_class, -1)] += 1
    return dict(counts)


def sample(paths: list[Path], limit: int) -> list[Path]:
    """Take an evenly spread subset of *paths*, so classes stay represented."""
    if limit <= 0 or limit >= len(paths):
        return paths
    return paths[:: max(1, len(paths) // limit)][:limit]


def summarise(predictions: list[list[Prediction]], truth: list[GroundTruth]) -> MetricsResult:
    """Compute the full metric set for one model.

    Per-class AP at 0.5 saturates on this dataset, so the per-class view is
    also reported averaged across the 0.5:0.95 sweep where it discriminates.
    """
    sweep = [
        mean_average_precision(predictions, truth, t)
        for t in np.arange(AP_THRESHOLD_START, AP_THRESHOLD_END, AP_THRESHOLD_STEP)
    ]
    map50, per_class = mean_average_precision(predictions, truth, 0.5)
    map75, _ = mean_average_precision(predictions, truth, 0.75)
    per_class_swept = {
        class_id: float(np.nanmean([classes.get(class_id, np.nan) for _, classes in sweep])) for class_id in per_class
    }
    return MetricsResult(
        mAP50=map50,
        mAP75=map75,
        mAP50_95=float(np.nanmean([value for value, _ in sweep])),
        per_class=per_class,
        per_class_5095=per_class_swept,
        confusion=confusion(predictions, truth),
    )


def print_report(results: dict[str, MetricsResult]) -> None:
    """Print the comparison tables for one or more named models."""
    print("\n=== SUMMARY ===")
    print(f"{'model':14s} {'mAP@0.5':>8s} {'mAP@0.75':>9s} {'mAP@0.5:0.95':>13s}   per-class mAP@0.5")
    for name, result in results.items():
        per = " ".join(f"{MODEL_NAMES[c]}={v:.3f}" for c, v in result.per_class.items())
        print(f"{name:14s} {result.mAP50:8.4f} {result.mAP75:9.4f} {result.mAP50_95:13.4f}   [{per}]")

    print("\n=== PER-CLASS mAP@0.5:0.95 (discriminating; @0.5 saturates) ===")
    print(f"{'model':14s} " + " ".join(f"{MODEL_NAMES[c]:>9s}" for c in sorted(MODEL_NAMES)))
    for name, result in results.items():
        print(
            f"{name:14s} " + " ".join(f"{result.per_class_5095.get(c, float('nan')):9.4f}" for c in sorted(MODEL_NAMES)),
        )

    print("\n=== CLASS CONFUSION (conf >= 0.25, IoU >= 0.5) ===")
    for name, result in results.items():
        wrong = {k: v for k, v in result.confusion.items() if k[0] >= 0 and k[1] >= 0 and k[0] != k[1]}
        missed = sum(v for k, v in result.confusion.items() if k[1] == -1)
        spurious = sum(v for k, v in result.confusion.items() if k[0] == -1)
        detail = ", ".join(f"{MODEL_NAMES[t]}->{MODEL_NAMES[p]}={v}" for (t, p), v in sorted(wrong.items())) or "none"
        print(f"{name:14s} misclassified: {detail} | missed: {missed} | false positives: {spurious}")
