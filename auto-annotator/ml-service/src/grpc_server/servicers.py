"""gRPC servicers wrapping the SAM, augmentation, and training compute.

Heavy ML dependencies (torch, ultralytics, cv2/albumentations) are imported
lazily inside the RPC methods so the server process can start without loading
models; the cost is paid on first use.
"""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path
from typing import TYPE_CHECKING

import grpc

from src.grpc_server.pb import (
    compute_pb2 as pb,
    compute_pb2_grpc as pb_grpc,
)
from src.constants import (
    DEFAULT_TRAIN_BATCH,
    DEFAULT_TRAIN_EPOCHS,
    DEFAULT_TRAIN_IMGSZ,
    DEFAULT_TRAIN_MODEL,
)
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from src.models import AppContext, Shape

logger = get_logger(__name__)


def _to_pb_shape(shape: Shape) -> pb.Shape:
    return pb.Shape(
        id=shape.id,
        class_name=shape.class_name,
        points=[pb.Point(x=p.x, y=p.y) for p in shape.points],
    )


class SegmentationServicer(pb_grpc.SegmentationServiceServicer):
    """SAM point-prompted segmentation."""

    def __init__(self) -> None:
        self._ctx: AppContext | None = None
        self._lock = threading.Lock()

    def _app_context(self) -> AppContext:
        """Build (once) the AppContext holding the SAM client + inference state."""
        if self._ctx is None:
            with self._lock:
                if self._ctx is None:
                    from src.inference import initialize_inference
                    from src.model_server import connect_to_model_server
                    from src.models import AppContext

                    ctx = AppContext(client=connect_to_model_server())
                    initialize_inference(ctx.client, ctx.inference)
                    self._ctx = ctx
        return self._ctx

    def Segment(self, request: pb.SegmentRequest, context: grpc.ServicerContext) -> pb.SegmentResponse:
        """Run SAM inference for the request's click points (unary RPC)."""
        from src.models import ClassInfo
        from src.services.segmentation_service import SegmentationService
        from src.types import ClassId

        classes = [ClassInfo(id=ClassId(i), name=name, color="") for i, name in enumerate(request.class_names)]
        from src.models import ClickPoint

        points = [
            ClickPoint(x=p.x, y=p.y, point_type=p.point_type, class_name=p.class_name)
            for p in request.points
        ]

        # segment_path() resolves the image itself; no repository-backed
        # DB lookup is needed on this gRPC path (Go owns the DB).
        service = SegmentationService(repository=None)
        try:
            shape = service.segment_path(
                request.image_path, request.image_id, points, self._app_context(), classes,
            )
        except ValueError as exc:
            return pb.SegmentResponse(state="error", message=str(exc), shapes=[])
        except Exception as exc:  # surface any inference failure to the client
            logger.exception("segment_failed")
            return pb.SegmentResponse(state="error", message=str(exc), shapes=[])

        if shape is None:
            return pb.SegmentResponse(state="error", message="Mask too small to render", shapes=[])
        return pb.SegmentResponse(state="ready", message="Model mask ready", shapes=[_to_pb_shape(shape)])


class AugmentationServicer(pb_grpc.AugmentationServiceServicer):
    """albumentations-based augmentation; streams one event per created copy."""

    def RunAugmentation(
        self, request: pb.AugmentRequest, context: grpc.ServicerContext,
    ) -> Iterator[pb.JobProgress]:
        """Augment each source image, streaming one event per created copy."""
        from src.augment import augment_image_files
        from src.gallery_cache import AnnotationCache
        from src.label_store import LabelStore

        label_store = LabelStore(AnnotationCache())
        total = max(1, len(request.sources) * max(1, request.num_augmentations))
        done = 0

        for source in request.sources:
            for img in augment_image_files(
                source.path, source.format_used, source.image_id, request.num_augmentations, label_store,
            ):
                done += 1
                yield pb.JobProgress(
                    status="running",
                    stage="augmenting",
                    progress=done / total,
                    message=f"Augmented {Path(img.path).name}",
                    details=json.dumps({"done": done, "total": total}),
                    augmented=pb.AugmentedImage(
                        path=img.path, format_used=img.format_used, parent_id=img.parent_id,
                    ),
                )

        yield pb.JobProgress(
            status="running", stage="augmenting", progress=1.0, message="Done", finished=True,
            details=json.dumps({"done": done, "total": total}),
        )


class TrainingServicer(pb_grpc.TrainingServiceServicer):
    """Ultralytics YOLO training; streams epoch progress.

    ``model.train`` blocks and reports via callbacks, so it runs on a worker
    thread that pushes updates onto a queue this generator drains.
    """

    def RunTraining(
        self, request: pb.TrainRequest, context: grpc.ServicerContext,
    ) -> Iterator[pb.JobProgress]:
        """Train a YOLO model on a worker thread, streaming epoch progress."""
        from src.train_service import run_training_job

        events: queue.Queue = queue.Queue()

        class _QueueReporter:
            def update(self, stage: str, progress: float, message: str = "", details: dict | None = None) -> None:
                events.put(("update", stage, progress, message, details or {}))

        def worker() -> None:
            try:
                run_training_job(
                    model_name=request.model_name or DEFAULT_TRAIN_MODEL,
                    epochs=request.epochs or DEFAULT_TRAIN_EPOCHS,
                    batch=request.batch or DEFAULT_TRAIN_BATCH,
                    imgsz=request.imgsz or DEFAULT_TRAIN_IMGSZ,
                    data_yaml=Path(request.data_yaml_path) if request.data_yaml_path else None,
                    reporter=_QueueReporter(),
                )
                events.put(("done", None))
            except Exception as exc:  # relay any training failure to the client
                logger.exception("training_failed")
                events.put(("error", str(exc)))

        threading.Thread(target=worker, daemon=True, name="train-worker").start()

        while True:
            kind, *rest = events.get()
            if kind == "update":
                stage, progress, message, details = rest
                # run_training_job swallows failures as an "error" stage update.
                if stage == "error":
                    context.abort(grpc.StatusCode.INTERNAL, message)
                    return
                yield pb.JobProgress(
                    status="running", stage=stage, progress=progress,
                    message=message, details=json.dumps(details),
                )
            elif kind == "done":
                yield pb.JobProgress(status="completed", finished=True, message="Training completed")
                return
            elif kind == "error":
                context.abort(grpc.StatusCode.INTERNAL, rest[0])
                return
