"""src.handlers.responses – Typed response dataclasses for all Gradio event handlers.

Each response dataclass maps to one or more handler functions and exposes a
``to_gradio()`` method that unpacks into the exact positional tuple expected by
the corresponding Gradio output list.

Handlers return these instead of raw tuples so callers have named fields for
every output slot and the Gradio wiring remains the single source of truth for
output ordering.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np

    from src.models import AppState


@dataclass
class ClickRunResponse:
    """Response from ``handle_click`` and ``run_sam``.

    Outputs: ``[display_img, state, status_box, accept_btn, mask_level_dd]``
    """

    display_img: np.ndarray
    state: AppState
    log_str: str
    accept_btn_update: Any
    mask_level_update: Any

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (
            self.display_img,
            self.state,
            self.log_str,
            self.accept_btn_update,
            self.mask_level_update,
        )


@dataclass
class AcceptUndoResponse:
    """Response from ``accept_mask`` and ``undo_last``.

    Outputs: ``[display_img, state, status_box, ann_box, mask_level_dd]``

    ``ann_summary`` may be a plain string or a ``gr.update()`` no-op.
    """

    display_img: np.ndarray
    state: AppState
    log_str: str
    ann_summary: Any
    mask_level_update: Any

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (
            self.display_img,
            self.state,
            self.log_str,
            self.ann_summary,
            self.mask_level_update,
        )


@dataclass
class ClearPointsResponse:
    """Response from ``clear_points``.

    Outputs: ``[display_img, state, status_box, mask_level_dd]``
    """

    display_img: np.ndarray
    state: AppState
    log_str: str
    mask_level_update: Any

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.display_img, self.state, self.log_str, self.mask_level_update)


@dataclass
class SelectMaskResponse:
    """Response from ``select_mask_level``.

    Outputs: ``[display_img, state]``
    """

    display_img: np.ndarray
    state: AppState

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.display_img, self.state)


@dataclass
class NavigationResponse:
    """Response from ``save_and_next``, ``skip_image``, ``go_prev``, ``go_next_pending``.

    Outputs: ``[display_img, state, status_box, stats_box, image_label, ann_box]``
    """

    display_img: np.ndarray
    state: AppState
    status_msg: str
    stats_html: str
    image_label: str
    ann_summary: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (
            self.display_img,
            self.state,
            self.status_msg,
            self.stats_html,
            self.image_label,
            self.ann_summary,
        )


@dataclass
class BrowseResponse:
    """Response from ``refresh_browse``.

    Outputs: ``[browse_df, browse_stats]``
    """

    df_data: list[list]
    stats_html: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.df_data, self.stats_html)


@dataclass
class ImportResponse:
    """Response from ``import_images``.

    Outputs: ``[browse_df, browse_stats, import_status]``
    """

    df_data: list[list]
    stats_html: str
    status_msg: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.df_data, self.stats_html, self.status_msg)


@dataclass
class SwitchModelResponse:
    """Response from ``switch_model``.

    Outputs: ``[state, model_status_box, auto_annotate_btn]``
    """

    state: AppState
    status_msg: str
    auto_btn_update: Any

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.state, self.status_msg, self.auto_btn_update)


@dataclass
class AutoAnnotateResponse:
    """Response from ``auto_annotate``.

    Outputs: ``[display_img, state, model_status_box, ann_box]``
    """

    display_img: np.ndarray
    state: AppState
    status_msg: str
    ann_box_update: Any

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.display_img, self.state, self.status_msg, self.ann_box_update)


@dataclass
class StartupResponse:
    """Response from ``load_first_image``.

    Outputs: ``[display_img, state, status_box, stats_box,``
              ``image_label, ann_box, browse_df, browse_stats]``
    """

    display_img: np.ndarray
    state: AppState
    status_msg: str
    stats_html: str
    image_label: str
    ann_summary: str
    browse_df: list[list]
    browse_stats_html: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (
            self.display_img,
            self.state,
            self.status_msg,
            self.stats_html,
            self.image_label,
            self.ann_summary,
            self.browse_df,
            self.browse_stats_html,
        )


@dataclass
class AddClassResponse:
    """Response from ``add_class``.

    Outputs: ``[class_dropdown, state, color_picker, edit_class_dd, class_swatch]``
    """

    class_dd_update: Any
    state: AppState
    color_picker_value: str
    edit_dd_update: Any
    swatch_html: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (
            self.class_dd_update,
            self.state,
            self.color_picker_value,
            self.edit_dd_update,
            self.swatch_html,
        )


@dataclass
class UpdateColorResponse:
    """Response from ``update_class_color``.

    Outputs: ``[state, display_img, edit_status]``
    """

    state: AppState
    display_img: np.ndarray
    status_msg: str

    def to_gradio(self) -> tuple:
        """Unpack to the positional output tuple for Gradio wiring."""
        return (self.state, self.display_img, self.status_msg)
