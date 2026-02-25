"""src.constants – Application-wide constants."""

import os
from pathlib import Path

# ── Directories ────────────────────────────────────────────────────────────────

BASE_DIR = Path(__file__).parent.parent
MODELS_DIR = Path(os.environ.get("MODELS_DIR", BASE_DIR / "models"))
PENDING_DIR = BASE_DIR / "data" / "pending"
LABELS_DIR = BASE_DIR / "data" / "labels"
DB_PATH = Path(os.environ.get("DB_PATH", BASE_DIR / "data" / "manifest.db"))
CONFIG_FILE = Path(os.environ.get("MODELS_CONFIG", BASE_DIR / "config" / "models.toml"))

# ── Network ────────────────────────────────────────────────────────────────────

SERVER_HOST = "127.0.0.1"
SERVER_PORT = int(os.environ.get("MODEL_SERVER_PORT", 8765))

# ── Image extensions ───────────────────────────────────────────────────────────

VALID_EXTS: frozenset[str] = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})

# ── Rendering ──────────────────────────────────────────────────────────────────

OUTLINE_MODES: list[str] = ["Class color", "High contrast", "Black", "White"]
MASK_LABELS: list[str] = ["Precise (0)", "Object (1)", "Broad (2)"]

# ── DB column names ────────────────────────────────────────────────────────────

COL_ID = "id"
COL_PATH = "path"
COL_STATUS = "status"
COL_FORMAT_USED = "format_used"
COL_NAME = "name"
COL_COLOR = "color"
COL_CREATED_AT = "created_at"
COL_UPDATED_AT = "updated_at"
