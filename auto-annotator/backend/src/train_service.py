"""src.train_service – YOLO training service with epoch-level progress callbacks."""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.constants import BASE_DIR, DATA_YAML_PATH
from src.utils import get_logger

if TYPE_CHECKING:
    from pathlib import Path

    from src.job_progress import ProgressReporter

logger = get_logger(__name__)

RUNS_DIR: Path = BASE_DIR / "data" / "runs"


def run_training_job(
    model_name: str,
    epochs: int,
    batch: int,
    imgsz: int,
    data_yaml: Path | None = None,
    reporter: ProgressReporter | None = None,
    **_kwargs: object,
) -> None:
    """Train YOLO model, emitting epoch progress through *reporter*.

    Args:
        model_name: Model identifier, e.g. ``"yolo11s.pt"``.
        epochs:     Number of training epochs.
        batch:      Batch size.
        imgsz:      Input image size (pixels).
        data_yaml:  Path to data.yaml. Defaults to app DATA_YAML_PATH.
        reporter:   ProgressReporter for epoch-level progress updates.
    """
    from ultralytics import YOLO  # lazy import — heavy dependency  # noqa: PLC0415

    yaml_path = data_yaml or DATA_YAML_PATH
    if not yaml_path.exists():
        if reporter:
            reporter.update("error", 0.0, f"data.yaml not found at {yaml_path}")
        return

    RUNS_DIR.mkdir(parents=True, exist_ok=True)

    model = YOLO(model_name)

    def _on_epoch_end(trainer: object) -> None:
        if not reporter:
            return
        epoch = getattr(trainer, "epoch", 0) + 1
        total_epochs = getattr(trainer, "epochs", epochs)
        loss_items = getattr(trainer, "loss_items", None)
        metrics: dict = getattr(trainer, "metrics", {}) or {}
        box_loss = float(loss_items[0]) if loss_items is not None and len(loss_items) > 0 else 0.0
        cls_loss = float(loss_items[1]) if loss_items is not None and len(loss_items) > 1 else 0.0
        reporter.update(
            "training",
            epoch / total_epochs,
            f"Epoch {epoch}/{total_epochs}",
            details={
                "epoch": epoch,
                "total": total_epochs,
                "box_loss": round(box_loss, 6),
                "cls_loss": round(cls_loss, 6),
                "map50": round(float(metrics.get("metrics/mAP50(B)", 0) or 0), 6),
            },
        )

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
        if reporter:
            reporter.update(
                "training",
                1.0,
                f"Epoch {epochs}/{epochs}",
                details={"epoch": epochs, "total": epochs, "finished": True},
            )
        logger.info("training_job_done", extra={"epochs": epochs, "model": model_name})
    except RuntimeError as exc:
        if reporter:
            reporter.update("error", 0.0, str(exc))
        logger.info("training_job_error", extra={"error": str(exc)})
