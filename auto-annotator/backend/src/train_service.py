"""src.train_service – YOLO training service with epoch-level progress callbacks."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable

from src.constants import BASE_DIR, DATA_YAML_PATH
from src.utils import get_logger

if TYPE_CHECKING:
    from src.db.repository import Repository
    from src.job_progress import ProgressReporter

logger = get_logger(__name__)

ProgressCallback = Callable[[dict], None]

RUNS_DIR: Path = BASE_DIR / "data" / "runs"


def run_training_job(
    model_name: str,
    epochs: int,
    batch: int,
    imgsz: int,
    on_progress: ProgressCallback,
    data_yaml: Path | None = None,
    repository: Repository | None = None,
    reporter: ProgressReporter | None = None,
) -> None:
    """Train YOLO model, calling on_progress after each epoch.

    Args:
        model_name: Model identifier, e.g. ``"yolo11s.pt"``.
        epochs:     Number of training epochs.
        batch:      Batch size.
        imgsz:      Input image size (pixels).
        on_progress: Callback receiving epoch progress dicts.
        data_yaml:  Path to data.yaml. Defaults to app DATA_YAML_PATH.
        repository: Repository for potential database updates. Unused in current implementation.
        reporter:   Optional ProgressReporter for fine-grained progress tracking.
    """
    from ultralytics import YOLO  # lazy import — heavy dependency

    yaml_path = data_yaml or DATA_YAML_PATH
    if not yaml_path.exists():
        on_progress({"error": f"data.yaml not found at {yaml_path}"})
        return

    RUNS_DIR.mkdir(parents=True, exist_ok=True)

    model = YOLO(model_name)

    def _on_epoch_end(trainer: object) -> None:
        epoch = getattr(trainer, "epoch", 0) + 1
        total_epochs = getattr(trainer, "epochs", epochs)
        loss_items = getattr(trainer, "loss_items", None)
        metrics: dict = getattr(trainer, "metrics", {}) or {}
        # loss_items is a tensor [box, cls, dfl] during training phase
        box_loss = float(loss_items[0]) if loss_items is not None and len(loss_items) > 0 else 0.0
        cls_loss = float(loss_items[1]) if loss_items is not None and len(loss_items) > 1 else 0.0
        on_progress({
            "epoch": epoch,
            "total": total_epochs,
            "box_loss": round(box_loss, 6),
            "cls_loss": round(cls_loss, 6),
            "map50": round(float(metrics.get("metrics/mAP50(B)", 0) or 0), 6),
        })

        if reporter:
            reporter.update("training", epoch / total_epochs, f"Epoch {epoch}/{total_epochs}")

    model.add_callback("on_train_epoch_end", _on_epoch_end)

    try:
        model.train(
            data=str(yaml_path),
            epochs=epochs,
            batch=batch,
            imgsz=imgsz,
            project=str(RUNS_DIR),
            name="train",
            exist_ok=True,
            verbose=False,
        )
        on_progress({"epoch": epochs, "total": epochs, "finished": True})
        logger.info("training_job_done", extra={"epochs": epochs, "model": model_name})
    except Exception as exc:
        on_progress({"error": str(exc)})
        logger.info("training_job_error", extra={"error": str(exc)})
