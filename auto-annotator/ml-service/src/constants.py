"""src.constants – True application-wide invariants.

Only constants that will never be derived from environment variables belong
here: fixed strings, numeric thresholds, numpy arrays, and rendering defaults
that are the same in every deployment.

Environment-derived configuration (paths, ports, URLs, model settings) lives
in src.config.AppConfig, loaded at startup and injected via DI.

Database-specific constants (column names, status labels, status icons) live
in src.db.constants, not here, to keep concerns separated.
"""

from __future__ import annotations

import numpy as np

API_V1_PREFIX: str = "/api/v1"
"""Version prefix every domain router is mounted under (single source of truth)."""

# Accepted image file extensions (lowercase, dot-prefixed).

VALID_EXTS: frozenset[str] = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})
"""Lowercase image file extensions that the annotator will import."""

MASK_LABELS: list[str] = ["Precise (0)", "Object (1)", "Broad (2)"]
"""Human-readable labels for SAM's three mask granularity outputs (index 0–2)."""

INFERENCE_LOG_MAX_ENTRIES: int = 50
"""Maximum number of timestamped log entries kept in AppState.log_entries."""

# Compute device name literals used when selecting CPU vs GPU backends.

DEVICE_CUDA: str = "cuda"
"""CUDA GPU device identifier used with torch and SAM backends."""

DEVICE_CPU: str = "cpu"
"""CPU device identifier used as fallback when no CUDA GPU is available."""

# RGB colour tuples (R, G, B) used by render.py and utils.py.

COLOR_BLACK_RGB: tuple[int, int, int] = (0, 0, 0)
"""Pure black in RGB order; used for contour shadow outlines."""

COLOR_WHITE_RGB: tuple[int, int, int] = (255, 255, 255)
"""Pure white in RGB order; used for pending-mask outline and point borders."""

COLOR_RED_RGB: tuple[int, int, int] = (255, 0, 0)
"""Pure red in RGB order; fallback returned when a hex colour string is invalid."""

COLOR_GREEN_RGB: tuple[int, int, int] = (0, 255, 0)
"""Pure green in RGB order; default positive-point colour when no class is active."""

COLOR_BLUE_RGB: tuple[int, int, int] = (0, 0, 255)
"""Pure blue in RGB order; used to fill negative (exclusion) annotation points."""

# Hex colour string constants for cases where a CSS hex string is needed.

COLOR_WHITE_HEX: str = "#ffffff"
"""Pure white as a CSS hex string; used as a fallback pending-mask colour."""

COLOR_GREEN_HEX: str = "#00ff00"
"""Pure green as a CSS hex string; fallback colour for positive annotation points."""

# Luminance conversion constants for the high-contrast outline mode.

COLOR_LUMINANCE_WEIGHTS: tuple[float, float, float] = (0.299, 0.587, 0.114)
"""ITU-R BT.601 per-channel weights for converting RGB to perceived luminance."""

CANVAS_LUMINANCE_THRESHOLD: int = 128
"""Luminance value (0–255) above which the outline switches from white to black."""

# Canvas rendering constants – sizes, thicknesses, alpha values, and layout offsets.

CANVAS_ANNOTATION_FILL_ALPHA: float = 0.45
"""Opacity of accepted-annotation colour fills blended over the base image."""

CANVAS_ANNOTATION_BASE_ALPHA: float = 0.55
"""Complement of CANVAS_ANNOTATION_FILL_ALPHA (base-image weight in blend)."""

CANVAS_POINT_RADIUS: int = 6
"""Pixel radius of click-point circles drawn on the canvas."""

CANVAS_POINT_BORDER_THICKNESS: int = 1
"""White border thickness (px) around each click-point circle."""

CANVAS_CONTOUR_SHADOW_THICKNESS: int = 4
"""Thickness (px) of the black shadow drawn behind annotation contours."""

CANVAS_CONTOUR_OUTLINE_THICKNESS: int = 2
"""Thickness (px) of the coloured contour outline drawn over the shadow."""

CANVAS_NEGATIVE_CROSS_SIZE: int = 4
"""Half-length (px) of the × mark drawn inside negative-point circles."""

CANVAS_BORDER_DILATION_KERNEL_SIZE: int = 5
"""Kernel size (px) for morphological dilation when computing high-contrast borders."""

CANVAS_LEGEND_START_Y: int = 10
"""Y-pixel offset (from image top) where the top-left class legend begins."""

CANVAS_LEGEND_SWATCH_X1: int = 8
"""Left edge (px) of the colour swatch rectangle in the class legend."""

CANVAS_LEGEND_SWATCH_X2: int = 26
"""Right edge (px) of the colour swatch rectangle in the class legend."""

CANVAS_LEGEND_SWATCH_HEIGHT: int = 16
"""Height (px) of the colour swatch rectangle in the class legend."""

CANVAS_LEGEND_TEXT_X: int = 30
"""Left edge (px) where class name text starts in the class legend."""

CANVAS_LEGEND_TEXT_OFFSET_Y: int = 13
"""Vertical offset (px) within each legend row for text baseline alignment."""

CANVAS_LEGEND_ROW_STEP_Y: int = 22
"""Vertical spacing (px) between rows in the class legend."""

CANVAS_LEGEND_FONT_SCALE: float = 0.5
"""OpenCV font scale used for class-name labels in the legend."""

CANVAS_PLACEHOLDER_HEIGHT: int = 480
"""Height (px) of the black placeholder returned when no image is loaded."""

CANVAS_PLACEHOLDER_WIDTH: int = 640
"""Width (px) of the black placeholder returned when no image is loaded."""

# Pre-built dilation kernel derived from CANVAS_BORDER_DILATION_KERNEL_SIZE.
# Used by render.py when computing the high-contrast border luminance.
DILATION_KERNEL: np.ndarray = np.ones(
    (CANVAS_BORDER_DILATION_KERNEL_SIZE, CANVAS_BORDER_DILATION_KERNEL_SIZE),
    np.uint8,
)
"""Square all-ones kernel for morphological border dilation in high-contrast outline mode."""

# Geometry constants used by src/geometry.py.

GEOMETRY_DEFAULT_EPSILON_FACTOR: float = 0.002
"""Douglas-Peucker epsilon as a fraction of contour arc-length; controls polygon simplification."""

GEOMETRY_MINIMUM_CONTOUR_AREA: float = 10.0
"""Contours with area (px²) below this threshold are discarded as noise."""

GEOMETRY_MINIMUM_POLYGON_POINTS: int = 3
"""Minimum number of vertices required to form a valid YOLO polygon."""
