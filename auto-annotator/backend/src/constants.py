"""src.constants – Application-wide constants for paths, server, UI, and rendering.

All magic strings, numeric thresholds, numpy arrays, and filesystem defaults are
centralised here so every module can import named constants instead of scattering
literals.

Database-specific constants (column names, status labels, status icons) live in
src.db.constants, not here, to keep concerns separated.
"""

from __future__ import annotations

import os
import tomllib
from pathlib import Path
from typing import Any

import numpy as np

# Absolute path to the project root directory (parent of src/).
BASE_DIR: Path = Path(__file__).parent.parent

# Environment variable name constants.
# Defined once here so callers never use bare string literals with os.environ.

ENV_MODELS_DIR: str = "MODELS_DIR"
"""Environment variable: directory containing model checkpoint files."""

ENV_DB_PATH: str = "DB_PATH"
"""Environment variable: path to the SQLite manifest database."""

ENV_MODELS_CONFIG: str = "MODELS_CONFIG"
"""Environment variable: path to the models.toml configuration file."""

ENV_SERVER_CONFIG: str = "SERVER_CONFIG"
"""Environment variable: path to the server config TOML file."""

ENV_MODEL_SERVER_PORT: str = "MODEL_SERVER_PORT"
"""Legacy environment variable used to override the model server port, kept for compatibility."""

ENV_SERVER_PORT: str = "SERVER_PORT"
"""Environment variable used to override the port specified in the server config file."""

ENV_HF_HUB_CACHE: str = "HF_HUB_CACHE"
"""Environment variable: HuggingFace hub cache directory."""

ENV_HF_TOKEN: str = "HF_TOKEN"
"""Environment variable: HuggingFace API token (required for gated models)."""

ENV_API_PORT: str = "API_PORT"
"""HTTP API server port (run via ``python main.py api`` or ``uvicorn src.api.app:app``)."""

ENV_API_PUBLIC_URL: str = "API_PUBLIC_URL"
"""Base URL exposed to clients when building image URLs."""

ENV_DEFAULT_MODEL: str = "DEFAULT_MODEL"
"""Environment variable: model ID to load on server startup."""

# Filesystem paths derived from environment variables or fixed defaults.

MODELS_DIR: Path = Path(os.environ.get(ENV_MODELS_DIR, str(BASE_DIR / "models")))
"""Directory where model checkpoints are stored (overridable via ENV_MODELS_DIR)."""

PENDING_DIR: Path = BASE_DIR / "data" / "pending"
"""Directory where unprocessed images are placed for annotation."""

LABELS_DIR: Path = BASE_DIR / "data" / "labels"
"""Directory where YOLO .txt label files are written on save."""

IMAGES_DIR: Path = BASE_DIR / "data" / "images"
"""Directory where annotated images are copied, organised by class subfolder."""

DATA_YAML_PATH: Path = BASE_DIR / "data" / "data.yaml"
"""Path to the YOLO data.yaml generated after each save."""

DB_PATH: Path = Path(os.environ.get(ENV_DB_PATH, str(BASE_DIR / "data" / "manifest.db")))
"""Path to the SQLite manifest database (overridable via ENV_DB_PATH)."""

CONFIG_FILE: Path = Path(
    os.environ.get(ENV_MODELS_CONFIG, str(BASE_DIR / "config" / "models.toml")),
)
"""Path to the TOML models configuration file (overridable via ENV_MODELS_CONFIG)."""

SERVER_CONFIG_FILE: Path = Path(
    os.environ.get(ENV_SERVER_CONFIG, str(BASE_DIR / "config" / "server.toml")),
)
"""Path to the TOML server configuration file (overridable via ENV_SERVER_CONFIG)."""


def _load_server_config() -> dict[str, Any]:
    if not SERVER_CONFIG_FILE.exists():
        return {}
    with SERVER_CONFIG_FILE.open("rb") as fp:
        try:
            return tomllib.load(fp).get("server", {})
        except (tomllib.TOMLDecodeError, OSError):  # pragma: no cover - best-effort config parsing
            return {}


SERVER_CONFIG: dict[str, Any] = _load_server_config()
"""Contents of the server configuration file (empty when the file is absent or invalid)."""

DEFAULT_SERVER_PORT: int = 8765
"""Fallback TCP port for the server when no config or environment overrides are provided."""

# Model server network settings.

SERVER_HOST: str = "127.0.0.1"
"""Localhost address used by both the model server and the TCP client."""

_env_port = os.environ.get(ENV_SERVER_PORT)
_legacy_env_port = os.environ.get(ENV_MODEL_SERVER_PORT)
_config_port = SERVER_CONFIG.get("port")
if _env_port is not None:
    _resolved_port = _env_port
    _port_origin = f"env:{ENV_SERVER_PORT}"
elif _legacy_env_port is not None:
    _resolved_port = _legacy_env_port
    _port_origin = f"env:{ENV_MODEL_SERVER_PORT}"
elif _config_port is not None:
    _resolved_port = _config_port
    _port_origin = f"config:{SERVER_CONFIG_FILE.name}"
else:
    _resolved_port = DEFAULT_SERVER_PORT
    _port_origin = "default"
SERVER_PORT: int = int(_resolved_port)
"""TCP port the model server binds to; resolved from envs, config file, or default."""

SERVER_PORT_SOURCE: str = _port_origin
"""Description of where the current server port value originated."""

DEFAULT_API_PORT: int = 8000
"""Port used by the HTTP API when no override is provided."""

API_PORT: int = int(os.environ.get(ENV_API_PORT, DEFAULT_API_PORT))
"""Port that ``uvicorn src.api.app:app`` listens on by default."""

DEFAULT_API_PUBLIC_URL: str = f"http://localhost:{API_PORT}"
"""Default base URL returned to React for image downloads (can be overridden for reverse proxies)."""

API_PUBLIC_URL: str = os.environ.get(ENV_API_PUBLIC_URL, DEFAULT_API_PUBLIC_URL)
"""Base URL used when building ``GalleryItem.src`` responses."""

RECV_CHUNK_SIZE: int = 65536
"""Maximum number of bytes read per socket recv() call in the TCP client and server."""

# Accepted image file extensions (lowercase, dot-prefixed).

VALID_EXTS: frozenset[str] = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})
"""Lowercase image file extensions that the annotator will import."""

MASK_LABELS: list[str] = ["Precise (0)", "Object (1)", "Broad (2)"]
"""Human-readable labels for SAM's three mask granularity outputs (index 0–2)."""

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

# SAM2 local inference constants used by src/inference.py.

SAM2_LOCAL_CHECKPOINT_FILENAME: str = "sam2.1_l.pt"
"""Filename of the SAM 2.1 Large checkpoint searched for in MODELS_DIR."""

SAM2_DEFAULT_HF_REPO: str = "facebook/sam2.1-hiera-large"
"""HuggingFace repository used when no local checkpoint is found."""

SAM2_DEFAULT_HIERA_CONFIG: str = "configs/sam2.1/sam2.1_hiera_l.yaml"
"""Relative path to the Hiera YAML config required by build_sam2()."""

INFERENCE_DEFAULT_MASK_SCORE: float = 1.0
"""Fallback confidence score assigned when the ultralytics backend returns no scores."""

INFERENCE_LOG_MAX_ENTRIES: int = 50
"""Maximum number of timestamped log entries kept in AppState.log_entries."""

# Compute device name literals used when selecting CPU vs GPU backends.

DEVICE_CUDA: str = "cuda"
"""CUDA GPU device identifier used with torch and SAM backends."""

DEVICE_CPU: str = "cpu"
"""CPU device identifier used as fallback when no CUDA GPU is available."""
