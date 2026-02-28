"""src.ui.constants – String constants for all Gradio UI components.

All button labels, tab names, placeholder text, default values, accordion
headings, and dataframe configuration strings are centralised here so the UI
builder modules never embed bare string literals.
"""

from typing import Literal

# Tab names

TAB_ANNOTATE: str = "Annotate"
TAB_BROWSE: str = "Browse"
TAB_SETTINGS: str = "Settings"

# Accordion labels

ACCORDION_MODEL: str = "Model"
ACCORDION_CLASSES: str = "Classes"
ACCORDION_DISPLAY: str = "Display"

# Button labels

BTN_ACCEPT_MASK: str = "Accept mask \u2713"
BTN_UNDO: str = "Undo \u21a9"
BTN_RUN_SAM: str = "Re-run SAM"
BTN_CLEAR_POINTS: str = "Clear points \u2715"
BTN_SAVE_NEXT: str = "Save & Next \u2192"
BTN_SKIP: str = "Skip \u23ed"
BTN_PREV: str = "\u2190 Prev"
BTN_RESET_ZOOM: str = "Reset zoom"
BTN_REFRESH: str = "Refresh"
BTN_IMPORT: str = "Import Images"
BTN_ADD_CLASS: str = "Add / update class"
BTN_UPDATE_COLOR: str = "Update colour"
BTN_LOAD_MODEL: str = "Load model"
BTN_AUTO_ANNOTATE: str = "Auto-annotate (text)"
BTN_MODAL_PREV: str = "\u2190 Prev"
BTN_MODAL_NEXT: str = "Next \u2192"
BTN_MODAL_CLOSE: str = "Close \u2715"

# Radio and dropdown option lists

RADIO_POINT_TYPES: list[str] = ["Positive", "Negative"]
RADIO_EXPORT_FMTS: list[str] = ["Segmentation", "Detection"]
RADIO_VIEW_OPTIONS: list[str] = ["List", "Grid"]

# Default values for radio / dropdown selections

DEFAULT_POINT_TYPE: str = "Positive"
DEFAULT_EXPORT_FMT: str = "Segmentation"
DEFAULT_MASK_LEVEL: str = "Object (1)"
DEFAULT_VIEW: str = "List"
DEFAULT_OUTLINE_MODE: str = "Class color"
ZOOM_MIN: float = 0.5
ZOOM_MAX: float = 3.0
ZOOM_STEP: float = 0.1
DEFAULT_ZOOM: float = 1.0

# Component labels

LABEL_IMAGE: str = "Image"
LABEL_MASK_GRANULARITY: str = "Mask granularity"
LABEL_CURRENT_IMAGE: str = "Current image"
LABEL_POINT_TYPE: str = "Point type"
LABEL_ACTIVE_CLASS: str = "Active class"
LABEL_LOG: str = "Log"
LABEL_ANNOTATIONS: str = "Annotations"
LABEL_EXPORT_FORMAT: str = "Export format"
LABEL_SAM_MODEL: str = "SAM model"
LABEL_MODEL_STATUS: str = "Model status"
LABEL_NEW_CLASS: str = "New class name"
LABEL_COLOR: str = "Colour"
LABEL_OUTLINE_MODE: str = "Outline colour mode"
LABEL_CLASS_TO_EDIT: str = "Class to edit"
LABEL_NEW_COLOR: str = "New colour"
LABEL_EDIT_STATUS: str = "Edit status"
LABEL_IMAGES: str = "Images"
LABEL_IMPORT_STATUS: str = "Import status"
LABEL_VIEW: str = "View"
LABEL_PREVIEW: str = "Preview"
LABEL_ZOOM: str = "Canvas zoom"

# Placeholder text

PLACEHOLDER_LOG: str = "Activity log\u2026"
PLACEHOLDER_CLASS_NAME: str = "e.g. red_prism"
PLACEHOLDER_MODEL_STATUS: str = "No model loaded."

# Section headings

HEADING_EDIT_CLASS_COLOR: str = "### Edit class colour"

# Annotation list placeholder

ANN_NONE: str = "(none)"

# Status icons used in the browse modal info line

STATUS_ICONS: dict[str, str] = {
    "pending": "\u23f3",
    "done": "\u2713",
    "skipped": "\u23ed",
}

# Browse dataframe configuration

DataType = Literal["number", "str", "bool", "date", "markdown", "html"]


BROWSE_DF_HEADERS: tuple[str, str, str, str, str] = ("ID", "Filename", "Status", "Format", "Updated")
BROWSE_DF_DATATYPES: tuple[DataType, DataType, DataType, DataType, DataType] = (
    "number",
    "str",
    "str",
    "str",
    "str",
)

# Gallery display settings

GALLERY_COLUMNS: int = 4
GALLERY_OBJECT_FIT: Literal["contain", "cover", "fill", "none", "scale-down"] = "contain"
GALLERY_HEIGHT: str = "auto"

# Canvas elem_id used for CSS targeting

ELEM_ID_ANNOTATE_CANVAS: str = "annotate-canvas"

# Log textbox line count

LOG_LINES: int = 4
ANN_BOX_LINES: int = 6
