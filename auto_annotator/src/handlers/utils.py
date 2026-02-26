"""src.handlers.utils – Shared helpers used across multiple event handler modules.

Consolidates utilities that would otherwise be duplicated three or more times
across annotation.py, navigation.py, and model_ctrl.py.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.models import Annotation


def format_annotations_summary(annotations: list[Annotation]) -> str:
    """Render a human-readable annotation list for the status text-box.

    Args:
        annotations: Accepted annotations for the current image.

    Returns:
        Multi-line string with one numbered entry per annotation, or
        ``"(none)"`` when the list is empty.
    """
    if not annotations:
        return "(none)"
    return "\n".join(
        f"{index}. [{ann.yolo_class_id}] {ann.class_name}  ({len(ann.polygon) // 2} pts)"
        for index, ann in enumerate(annotations, 1)
    )
