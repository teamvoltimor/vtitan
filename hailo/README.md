# Hailo Pipeline

Host-side CLI toolchain for compiling YOLO models to Hailo 8 HEF format.

## Quick Start

```bash
# Install dependencies
uv sync

# Export a YOLO model to ONNX
uv run hailo export --model yolo11s

# Prepare calibration data
uv run hailo calib download
uv run hailo calib convert

# Stage files for Docker and run the compiler
uv run hailo stage --model yolo11s
uv run hailo compile --model yolo11s --docker hailo8_ai_sw_suite_2025-10_container
```

Or use the Taskfile for the full pipeline:

```bash
task env:setup         # uv sync
task model:export      # yolo11s → ONNX
task calib:prepare     # download + convert
task workflow:full     # export → calib → test → stage → start Docker
```

## Project Structure

```
hailo/
├── main.py               # CLI entry point (argparse dispatch)
├── src/
│   ├── config.py         # Frozen dataclasses per command (no logic)
│   ├── docker.py         # Container lifecycle + docker exec
│   ├── export.py         # YOLO .pt → ONNX via Ultralytics
│   ├── hailomz.py        # stage / compile / eval / profile
│   ├── calib.py          # COCO download (fiftyone) + image → .npy
│   ├── graph.py          # ONNX graph inspection
│   ├── test.py           # 3-backend inference runner (pt/onnx/ultraonnx)
│   ├── image.py          # Letterbox, preprocessing, annotation drawing
│   ├── coco.py           # COCO 2017 class labels + per-class colours
│   ├── registry.py       # Model registry (known YOLO variants)
│   ├── enums.py          # Backend, Task, HWArch, EvalTarget
│   ├── errors.py         # HailoError hierarchy + require_dep guard
│   └── log.py            # Structured JSON logging
├── pyproject.toml
├── Taskfile.yml          # Task runner workflows
├── shared_with_docker/   # Mounted into the Hailo suite container
├── data/                 # Model checkpoints and ONNX exports
├── calib_data/           # Raw calibration images
├── calib_data_npy/       # float32 .npy arrays for hailomz
└── test_output/          # Annotated inference results
```

## Environment

The host-side commands (`export`, `calib`, `inspect`, `test`, `stage`) run on
any platform, including Windows.

The Hailo AI Software Suite is Linux-only, so `docker run`, `compile`, `eval`,
and `profile` require a Linux environment:

- **Compile and emulator-based eval** (`--target emulator`) need no Hailo
  hardware and can run inside **WSL2**.
- **Hardware eval/profile** (`--target hailo8`) need a native Linux host with
  the Hailo-8 device attached (PCIe passthrough is not available under WSL2).

The suite image and container are overridable per build: pass `--image`/
`--container` to `hailo docker run`, or override `DOCKER_IMAGE`/`DOCKER_CONTAINER`
when calling a `task` command.

## Commands

| Command | Description |
|---|---|
| `export` | Export a YOLO `.pt` checkpoint to ONNX |
| `calib download` | Download COCO 2017 validation images for calibration |
| `calib convert` | Convert calibration images to float32 `.npy` arrays |
| `inspect` | Print ONNX graph structure, inputs, outputs |
| `test` | Run inference locally (`--backend pt`/`onnx`/`ultraonnx`) |
| `stage` | Copy ONNX and calibration data into `shared_with_docker/` |
| `compile` | Run `hailomz compile` inside the suite container |
| `eval` | Run `hailomz eval` (emulator or Hailo-8 hardware) |
| `profile` | Run `hailomz profile` (HEF performance measurement) |
| `docker run` | Start the Hailo AI Software Suite container |

## Taskfile Reference

The pipeline includes a [`Taskfile.yml`](Taskfile.yml) with workflows for every stage. Install
[Task](https://taskfile.dev/) and run:

```bash
task --list
```

All tasks follow the `namespace:verb` convention. Variables are overridable on
the command line:

```bash
task model:export MODEL=yolo12n
task eval:run TARGET=hailo8 DATA_COUNT=100
```

### Variables

| Variable | Default | Description |
|---|---|---|
| `MODEL` | `yolo11s` | Model registry key |
| `IMGSZ` | `640` | Input resolution |
| `SAMPLES` | `2048` | Calibration dataset size |
| `CALIB_INPUT` | `./calib_data` | Raw calibration images directory |
| `CALIB_OUTPUT` | `./calib_data_npy` | float32 .npy output directory |
| `TEST_INPUT` | `./calib_data` | Test images directory |
| `TEST_OUTPUT` | `./test_output` | Annotated output directory |
| `TEST_CONF` | `0.3` | Confidence threshold |
| `SHARED_DIR` | `./shared_with_docker` | Docker shared volume mount |
| `DOCKER_CONTAINER` | `hailo8_ai_sw_suite_2025-10_container` | Container name |
| `DOCKER_IMAGE` | `hailo8_ai_sw_suite_2025-10:1` | Suite image tag |
| `HW_ARCH` | `hailo8` | Target hardware |
| `TARGET` | `emulator` | Evaluation target |
| `DATA_COUNT` | `512` | Evaluation samples |

### Environment

| Task | Description |
|---|---|
| `task env:setup` | Install dependencies (`uv sync`) |
| `task env:list` | Show installed packages |

### Model Export

| Task | Description |
|---|---|
| `task model:export` | Export `MODEL` to ONNX |
| `task model:export-all` | Export all registered models (n/s/m) |
| `task model:inspect` | Print ONNX graph of `MODEL` |
| `task model:export-inspect` | Export then inspect |

### Calibration Data

| Task | Description |
|---|---|
| `task calib:download` | Download COCO 2017 validation images |
| `task calib:convert` | Convert images to float32 `.npy` |
| `task calib:prepare` | Download + convert (full pipeline) |

### Testing / Inference

| Task | Description |
|---|---|
| `task test:pt` | Test with PyTorch backend |
| `task test:onnx` | Test with raw ONNX backend |
| `task test:ultraonnx` | Test with Ultralytics ONNX backend |
| `task test:run BACKEND=<backend>` | Run with explicit backend |

### Staging for Docker

| Task | Description |
|---|---|
| `task stage:run` | Stage ONNX + calibration data into `shared_with_docker/` |
| `task stage:no-calib` | Stage ONNX only (no calibration data) |

### Docker Container Management

| Task | Description |
|---|---|
| `task docker:run` | Start the Hailo AI Software Suite container |
| `task docker:dry` | Print the docker run command without executing |
| `task docker:status` | Check if the container is running |
| `task docker:stop` | Stop the container |
| `task docker:logs` | Follow container logs |

### Compile, Evaluate, Profile

| Task | Description |
|---|---|
| `task compile:run` | Compile ONNX → HEF (requires running container) |
| `task compile:dry` | Print compile command only |
| `task eval:run` | Evaluate HEF on target |
| `task eval:dry` | Print eval command only |
| `task eval:visual` | Evaluate with visualization |
| `task profile:run` | Profile HEF performance |
| `task profile:dry` | Print profile command only |

### End-to-End Workflows

| Task | Description |
|---|---|
| `task workflow:full` | export → calib → test → stage → start Docker |
| `task workflow:compile` | stage → compile in Docker |
| `task workflow:eval` | compile → eval → profile |

### Maintenance

| Task | Description |
|---|---|
| `task clean:output` | Remove test outputs and exported models |
| `task clean:calib` | Remove calibration data |
| `task clean:all` | Remove all generated files |
| `task log:debug ARGS=<cmd>` | Run a command with DEBUG logging |
| `task log:info ARGS=<cmd>` | Run a command with INFO logging |
| `task log:warning ARGS=<cmd>` | Run a command with WARNING logging |

## Model Registry

The [`src/registry.py`](src/registry.py) module registers known YOLO variants with
their export metadata and Hailo Model Zoo identifiers:

| Key | Task | Opset | Zoo Name | Notes |
|---|---|---|---|---|
| `yolo11n` | detect | 13 | `yolov11n` | Ultralytics defaults |
| `yolo11s` | detect | 13 | `yolov11s` | Ultralytics defaults |
| `yolo12n` | detect | 11 | `yolov12n` | `nms=False`, `simplify=True` |
| `yolo26n` | detect | 11 | — | No zoo entry |
| `yolo26l` | detect | 11 | — | No zoo entry |
| `yolo26l-seg` | segment | 11 | — | Segmentation variant |

Models without a zoo name require `--zoo-name` when calling `compile`/`eval`/`profile`.

## Docker Workflow

The Hailo AI Software Suite runs inside a Linux Docker container. The pipeline
manages the full lifecycle:

1. **Start the container** — `task docker:run` mounts `shared_with_docker/` at
   `/local/shared_with_docker/` inside the container, forwards X11 for GUI
   tools, and exposes GPU devices.
2. **Stage files** — `task stage:run` copies the ONNX model and calibration
   data into `shared_with_docker/`.
3. **Compile** — `task compile:run` executes `hailomz compile` via `docker exec`,
   producing a `.har` (Hailo Archive) and `.hef` (Hailo Executable Format) file.
4. **Evaluate** — `task eval:run` runs `hailomz eval` on the emulator or
   connected Hailo-8 hardware.
5. **Profile** — `task profile:run` measures inference performance.

All docker commands accept `--docker CONTAINER` to target a specific container,
or omit it to print the equivalent shell command for manual execution.

## Development

```bash
# Setup
uv sync

# List all taskfile workflows
task list

# Lint
uv run --group dev ruff check src/

# Run the full pipeline
task workflow:full
```
