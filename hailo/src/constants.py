"""Global constants for image processing, drawing, and CLI defaults."""

from __future__ import annotations

# Image Processing
DEFAULT_IMG_SIZE = 640
LETTERBOX_PAD_COLOR = (114, 114, 114)
NORMALIZE_FACTOR = 255.0
TRANSPOSE_HWC_TO_CHW = (2, 0, 1)

# Drawing & Annotation
BOX_LINE_THICKNESS = 2
TEXT_FONT_SCALE = 0.5
TEXT_THICKNESS = 2
TEXT_Y_OFFSET = 10

# Detection Output
DETECTION_OUTPUT_COLS = 6  # Post-NMS: (x1, y1, x2, y2, score, class)
DETECTION_BOX_COLS = 4  # xyxy coordinates
DETECTION_SCORE_COL = 4
DETECTION_CLASS_COL = 5

# File Handling
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}

# CLI Defaults
DEFAULT_COCO_SAMPLES = 2048
DEFAULT_CONFIDENCE = 0.3
DEFAULT_CALIB_INPUT = "./calib_data"
DEFAULT_CALIB_OUTPUT = "./calib_data_npy"
DEFAULT_CALIB_NAME = "calib_data"

# Export format identifiers
EXPORT_FORMAT_ONNX = "onnx"

# COCO dataset identifiers
COCO_DATASET = "coco-2017"
COCO_SPLIT = "validation"

# File-handling extensions
LABEL_EXTENSIONS: frozenset[str] = frozenset({".txt"})

# Path flattening separator
FLATTEN_SEPARATOR = "_"

# Visualisation / overlay constants
MASK_THRESHOLD = 0.5
OVERLAY_ALPHA = 0.7
OVERLAY_BETA = 0.3

# ONNX opset versions
OPSET_YOLO11 = 13
OPSET_YOLO12 = 11

# Cross-repo checkpoints
GMR_CHECKPOINT_PATH = "../auto-annotator/ml-service/models/gmr/best.pt"
# X11 display default now lives in src.settings.HailoSettings (host-specific,
# HAILO_X11_DISPLAY-overridable), not here.
