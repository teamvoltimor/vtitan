"""src.constants – True application-wide invariants.

Only constants that will never be derived from environment variables belong
here: fixed strings, numeric thresholds, and rendering defaults
that are the same in every deployment.

Environment-derived configuration (paths, ports, URLs, model settings) lives
in src.config.AppConfig, loaded at startup and injected via DI.
"""

from __future__ import annotations

API_V1_PREFIX: str = "/api/v1"
"""Version prefix every domain router is mounted under (single source of truth)."""

VALID_EXTS: frozenset[str] = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})
"""Lowercase image file extensions that the annotator will import."""

MASK_LABELS: list[str] = ["Precise (0)", "Object (1)", "Broad (2)"]
"""Human-readable labels for SAM's three mask granularity outputs (index 0–2)."""

INFERENCE_LOG_MAX_ENTRIES: int = 50
"""Maximum number of timestamped log entries kept in AppState.log_entries."""

DEVICE_CUDA: str = "cuda"
"""CUDA GPU device identifier used with torch and SAM backends."""

DEVICE_CPU: str = "cpu"
"""CPU device identifier used as fallback when no CUDA GPU is available."""

# Geometry constants used by src/geometry.py.

GEOMETRY_DEFAULT_EPSILON_FACTOR: float = 0.002
"""Douglas-Peucker epsilon as a fraction of contour arc-length; controls polygon simplification."""

GEOMETRY_MINIMUM_CONTOUR_AREA: float = 10.0
"""Contours with area (px²) below this threshold are discarded as noise."""

GEOMETRY_MINIMUM_POLYGON_POINTS: int = 3
"""Minimum number of vertices required to form a valid YOLO polygon."""

# Training defaults used by grpc_server/servicers.py and frame_extractor.

DEFAULT_TRAIN_MODEL: str = "yolo11s.pt"
"""Default YOLO checkpoint for training."""

DEFAULT_TRAIN_EPOCHS: int = 50
"""Default number of training epochs."""

DEFAULT_TRAIN_BATCH: int = 16
"""Default training batch size."""

DEFAULT_TRAIN_IMGSZ: int = 640
"""Default training image size (px)."""

# YOLO label coordinate-count constants used by src/augment.py.

YOLO_BBOX_COORD_COUNT: int = 4
"""Flat coordinate count for a YOLO detection label (``xc, yc, w, h``)."""
