# Auto-Annotator

SAM2-based interactive image annotator for generating YOLO training data.

## Features

- **Interactive annotation** – click positive/negative points, SAM generates masks
- **Multi-class support** – per-class colour pickers, default WRO classes seeded on first run
- **Mask granularity** – choose Precise / Object / Broad from a dropdown
- **Iterative refinement** – each added point feeds previous logits back to SAM for sharper masks
- **YOLO export** – segmentation polygons and/or bounding boxes
- **Browse tab** – list/grid toggle, image preview modal, import via UploadButton
- **Settings tab** – model loading, class management, display options
- **Model server** – SAM loads once, survives `app.py` restarts

## Quick Start

```bash
# 1. Install dependencies
uv sync

# 2. (Optional) Start the model server in one terminal
make model-server

# 3. Start the Gradio app in another terminal
make run
```

Then open http://localhost:7860.

## Project Structure

```
auto_annotator/
├── src/                 # Application package
│   ├── constants.py     # Paths, ports, render constants
│   ├── enums.py         # Status, ExportFormat
│   ├── schema.py        # SQLite DDL + default classes
│   ├── models.py        # AppState, Annotation, ClassInfo dataclasses
│   ├── utils.py         # JSON logger, colour helpers
│   ├── html.py          # stats_html(), _swatch_html()
│   ├── db.py            # SQLite persistence layer
│   ├── geometry.py      # mask_to_yolo_polygon, mask_to_yolo_bbox
│   ├── render.py        # render_state_image
│   ├── sam_client.py    # TCP client for model server
│   ├── inference.py     # SAM inference (server or local fallback)
│   ├── handlers/        # Gradio event handlers
│   └── ui/              # Tab builders (annotate, browse, settings)
├── server/              # Model server package
│   ├── context.py       # ServerContext dataclass
│   ├── sam1/2/3.py      # Per-model loaders
│   ├── loader.py        # is_available(), load_model(), initial_load()
│   └── dispatch.py      # Request routing
├── config/
│   └── models.toml      # SAM model definitions
├── data/
│   ├── pending/         # Images to annotate
│   ├── labels/          # YOLO .txt output
│   └── manifest.db      # SQLite DB
├── app.py               # Gradio entrypoint
├── model_server.py      # TCP server entrypoint
├── Makefile             # Convenience targets
└── Dockerfile
```

## Data

Place images in `data/pending/`. They are auto-registered on startup.

Labels are written to `data/labels/<stem>.txt` on Save.

## Configuration

- `config/models.toml`: describes each SAM / YOLO model (ids, checkpoints, metadata).
- `config/server.toml`: controls model-server runtime settings.  Only `server.port` is read today, with `SERVER_PORT` and the legacy `MODEL_SERVER_PORT` environment variables taking priority.

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `SERVER_CONFIG` | `./config/server.toml` | TOML file used by the model server (overrides `server.port`). |
| `SERVER_PORT` | `8765` | Port the model server listens on (overrides both `server.toml` and the legacy `MODEL_SERVER_PORT`). |
| `MODEL_SERVER_PORT` | `8765` | Legacy port override (kept for backwards compatibility). |
| `MODELS_DIR` | `./models` | Directory for SAM checkpoints |
| `MODELS_CONFIG` | `./config/models.toml` | Model configuration file |
| `DB_PATH` | `./data/manifest.db` | SQLite database path |
| `DEFAULT_MODEL` | first available | Model ID to load on startup |
| `HF_TOKEN` | – | HuggingFace token (required for SAM 3) |

## Keyboard shortcuts (Gradio defaults)

- Click canvas with **Positive** selected → add foreground point
- Click canvas with **Negative** selected → add background exclusion point
- **Accept mask** → commit current mask as annotation
- **Undo** → pop last point / clear pending mask / remove last annotation
- **Save & Next** → write YOLO label, advance to next image
