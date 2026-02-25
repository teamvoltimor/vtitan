"""src.ui.browse_tab – Browse tab UI builder and event wiring."""

from __future__ import annotations

import gradio as gr

from src.handlers.browse import import_images, refresh_browse


def build() -> dict:
    """Build the Browse tab.  Returns a dict of component references."""
    with gr.Tab("Browse"):
        with gr.Row():
            refresh_btn = gr.Button("Refresh", variant="secondary")
            import_btn = gr.UploadButton(
                "Import Images",
                file_count="multiple",
                file_types=["image"],
                variant="primary",
            )
            view_toggle = gr.Radio(["List", "Grid"], value="List", label="View")

        import_status = gr.Textbox(label="Import status", interactive=False, visible=False)
        browse_stats = gr.HTML()

        # List view
        browse_df = gr.Dataframe(
            headers=["ID", "Filename", "Status", "Format", "Updated"],
            datatype=["number", "str", "str", "str", "str"],
            interactive=False,
            label="Images",
            visible=True,
        )

        # Grid view
        browse_gallery = gr.Gallery(
            label="Images",
            columns=4,
            visible=False,
            object_fit="contain",
            height="auto",
        )

        # Modal for image preview
        with gr.Row(visible=False) as modal_row:
            with gr.Column():
                modal_img = gr.Image(label="Preview", interactive=False, type="numpy")
                modal_info = gr.Markdown("")
                with gr.Row():
                    modal_prev_btn = gr.Button("← Prev")
                    modal_next_btn = gr.Button("Next →")
                    modal_close_btn = gr.Button("Close ✕")

        modal_idx_state = gr.State(value=None)

    return dict(
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


def _open_modal(idx: int):
    """Load image at *idx* in the all-images list and return modal outputs."""
    import cv2  # noqa: PLC0415

    from src import db  # noqa: PLC0415

    rows = db.get_all_images()
    if not rows or idx is None or idx < 0 or idx >= len(rows):
        return gr.update(visible=False), None, "", idx

    r = rows[idx]
    record = db.get_by_id(r["id"])
    path = record["path"] if record else None

    img_np = None
    if path:
        bgr = cv2.imread(path)
        if bgr is not None:
            img_np = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    status_icon = {"pending": "⏳", "done": "✓", "skipped": "⏭"}.get(r["status"], "?")
    info = f"**{r['filename']}**  {status_icon} {r['status']}  Format: {r['format'] or '–'}"
    return gr.update(visible=True), img_np, info, idx


def wire_events(c: dict) -> None:
    """Wire all Gradio events for the Browse tab."""

    # View toggle: show/hide list vs grid
    def _toggle_view(view):
        return gr.update(visible=view == "List"), gr.update(visible=view == "Grid")

    c["view_toggle"].change(
        fn=_toggle_view,
        inputs=[c["view_toggle"]],
        outputs=[c["browse_df"], c["browse_gallery"]],
    )

    # Refresh
    c["refresh_btn"].click(
        fn=refresh_browse,
        outputs=[c["browse_df"], c["browse_stats"]],
    )

    # Import images
    def _import(files):
        data, stats, msg = import_images(files)
        return data, stats, gr.update(value=msg, visible=True)

    c["import_btn"].upload(
        fn=_import,
        inputs=[c["import_btn"]],
        outputs=[c["browse_df"], c["browse_stats"], c["import_status"]],
    )

    # List row click → open modal
    def _list_select(evt: gr.SelectData):
        return _open_modal(evt.index[0])

    c["browse_df"].select(
        fn=_list_select,
        outputs=[c["modal_row"], c["modal_img"], c["modal_info"], c["modal_idx_state"]],
    )

    # Gallery click → open modal
    def _gallery_select(evt: gr.SelectData):
        return _open_modal(evt.index)

    c["browse_gallery"].select(
        fn=_gallery_select,
        outputs=[c["modal_row"], c["modal_img"], c["modal_info"], c["modal_idx_state"]],
    )

    # Modal prev
    def _modal_prev(idx):
        new_idx = max(0, (idx or 0) - 1)
        return _open_modal(new_idx)

    c["modal_prev_btn"].click(
        fn=_modal_prev,
        inputs=[c["modal_idx_state"]],
        outputs=[c["modal_row"], c["modal_img"], c["modal_info"], c["modal_idx_state"]],
    )

    # Modal next
    def _modal_next(idx):
        from src import db as _db  # noqa: PLC0415

        total = len(_db.get_all_images())
        new_idx = min(total - 1, (idx or 0) + 1)
        return _open_modal(new_idx)

    c["modal_next_btn"].click(
        fn=_modal_next,
        inputs=[c["modal_idx_state"]],
        outputs=[c["modal_row"], c["modal_img"], c["modal_info"], c["modal_idx_state"]],
    )

    # Modal close
    c["modal_close_btn"].click(
        fn=lambda: gr.update(visible=False),
        outputs=[c["modal_row"]],
    )
