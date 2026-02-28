"""src.ui.browse_tab – Browse tab UI builder and event wiring."""

from __future__ import annotations

import cv2
import gradio as gr

from src import db as _db
from src.handlers.browse import import_images, refresh_browse
from src.ui.components import BrowseTabComponents
from src.ui.constants import (
    BROWSE_DF_DATATYPES,
    BROWSE_DF_HEADERS,
    BTN_IMPORT,
    BTN_MODAL_CLOSE,
    BTN_MODAL_NEXT,
    BTN_MODAL_PREV,
    BTN_REFRESH,
    DEFAULT_VIEW,
    GALLERY_COLUMNS,
    GALLERY_HEIGHT,
    GALLERY_OBJECT_FIT,
    LABEL_IMAGES,
    LABEL_IMPORT_STATUS,
    LABEL_PREVIEW,
    LABEL_VIEW,
    RADIO_VIEW_OPTIONS,
    STATUS_ICONS,
    TAB_BROWSE,
)


def build() -> BrowseTabComponents:
    """Build the Browse tab and return a typed component dataclass."""
    with gr.Tab(TAB_BROWSE):
        with gr.Column(elem_classes="aa-browse-toolbar"):
            refresh_btn = gr.Button(BTN_REFRESH, variant="secondary", elem_classes="aa-toolbar-button")
            import_btn = gr.UploadButton(
                BTN_IMPORT,
                file_count="multiple",
                file_types=["image"],
                variant="primary",
                elem_classes="aa-toolbar-button",
            )
            view_toggle = gr.Radio(
                RADIO_VIEW_OPTIONS,
                value=DEFAULT_VIEW,
                label=LABEL_VIEW,
                elem_classes="aa-view-toggle",
            )
            import_status = gr.Textbox(
                label=LABEL_IMPORT_STATUS, interactive=False, visible=False, elem_classes="aa-browse-toolbar",
            )
            browse_stats = gr.HTML(elem_classes="aa-browse-toolbar")
            browse_df = gr.Dataframe(
                headers=list(BROWSE_DF_HEADERS),
                datatype=list(BROWSE_DF_DATATYPES),
                interactive=False,
                label=LABEL_IMAGES,
                visible=True,
                elem_classes="aa-browse-toolbar",
            )

        browse_gallery = gr.Gallery(
            label=LABEL_IMAGES,
            columns=GALLERY_COLUMNS,
            visible=False,
            object_fit=GALLERY_OBJECT_FIT,
            height=GALLERY_HEIGHT,
        )

        with gr.Row(visible=False) as modal_row, gr.Column():
            modal_img = gr.Image(label=LABEL_PREVIEW, interactive=False, type="numpy")
            modal_info = gr.Markdown("")
            with gr.Row():
                modal_prev_btn = gr.Button(BTN_MODAL_PREV)
                modal_next_btn = gr.Button(BTN_MODAL_NEXT)
                modal_close_btn = gr.Button(BTN_MODAL_CLOSE)

        modal_idx_state = gr.State(value=None)

    return BrowseTabComponents(
        refresh_btn=refresh_btn,
        import_btn=import_btn,
        view_toggle=view_toggle,
        import_status=import_status,
        browse_stats=browse_stats,
        browse_df=browse_df,
        browse_gallery=browse_gallery,
        modal_row=modal_row,
        modal_img=modal_img,
        modal_info=modal_info,
        modal_prev_btn=modal_prev_btn,
        modal_next_btn=modal_next_btn,
        modal_close_btn=modal_close_btn,
        modal_idx_state=modal_idx_state,
    )


def _open_modal(idx: int) -> tuple:
    """Load image at *idx* in the all-images list and return modal outputs."""
    rows = _db.get_all_images()
    if not rows or idx is None or idx < 0 or idx >= len(rows):
        return gr.update(visible=False), None, "", idx

    r = rows[idx]
    record = _db.get_by_id(r.id)
    path = record.path if record else None

    img_np = None
    if path:
        bgr = cv2.imread(path)
        if bgr is not None:
            img_np = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    icon = STATUS_ICONS.get(r.status, "?")
    info = f"**{r.filename}**  {icon} {r.status}  Format: {r.format or '\u2013'}"
    return gr.update(visible=True), img_np, info, idx


def wire_events(c: BrowseTabComponents) -> None:
    """Wire all Gradio events for the Browse tab."""

    def _toggle_view(view: str) -> tuple:
        return gr.update(visible=view == "List"), gr.update(visible=view == "Grid")

    c.view_toggle.change(
        fn=_toggle_view,
        inputs=[c.view_toggle],
        outputs=[c.browse_df, c.browse_gallery],
    )

    c.refresh_btn.click(
        fn=lambda: refresh_browse().to_gradio(),
        outputs=[c.browse_df, c.browse_stats],
    )

    def _import(files: list) -> tuple:
        result = import_images(files)
        return result.df_data, result.stats_html, gr.update(value=result.status_msg, visible=True)

    c.import_btn.upload(
        fn=_import,
        inputs=[c.import_btn],
        outputs=[c.browse_df, c.browse_stats, c.import_status],
    )

    def _handle_df_select(evt: gr.SelectData | None) -> tuple:
        if evt is None or evt.index is None:
            return gr.update(visible=False), None, "", None
        return _open_modal(evt.index[0])

    def _handle_gallery_select(evt: gr.SelectData | None) -> tuple:
        if evt is None or evt.index is None:
            return gr.update(visible=False), None, "", None
        return _open_modal(evt.index)

    c.browse_df.select(
        fn=_handle_df_select,
        outputs=[c.modal_row, c.modal_img, c.modal_info, c.modal_idx_state],
    )

    c.browse_gallery.select(
        fn=_handle_gallery_select,
        outputs=[c.modal_row, c.modal_img, c.modal_info, c.modal_idx_state],
    )

    def _modal_prev(idx: int | None) -> tuple:
        return _open_modal(max(0, (idx or 0) - 1))

    c.modal_prev_btn.click(
        fn=_modal_prev,
        inputs=[c.modal_idx_state],
        outputs=[c.modal_row, c.modal_img, c.modal_info, c.modal_idx_state],
    )

    def _modal_next(idx: int | None) -> tuple:
        total = len(_db.get_all_images())
        return _open_modal(min(total - 1, (idx or 0) + 1))

    c.modal_next_btn.click(
        fn=_modal_next,
        inputs=[c.modal_idx_state],
        outputs=[c.modal_row, c.modal_img, c.modal_info, c.modal_idx_state],
    )

    c.modal_close_btn.click(
        fn=lambda: gr.update(visible=False),
        outputs=[c.modal_row],
    )
