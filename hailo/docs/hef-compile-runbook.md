# HEF compile runbook

How to get from a retrained YOLO `.pt` checkpoint to a Hailo `.hef`, and the
non-obvious failures that sit along the way. Written against the
`hailo8_ai_sw_suite_2025-10` suite (Hailo Model Zoo v2.17.0) driving the 3-class
GMR detector, on Windows 11 + Docker Desktop.

`hailomz` only exists inside the Hailo AI Software Suite container. Everything in
`hailo/` is a host-side wrapper that stages files into a bind mount and then
`docker exec`s into that container.

## One-time: load the suite image

The image is a manual download from the Hailo Developer Zone — it is not on any
registry, so `docker pull` will never find it. You get a directory containing a
~9.3 GB tarball and a vendor run script:

```
hailo8_ai_sw_suite_2025-10_docker/
  hailo8_ai_sw_suite_2025-10.tar.gz
  hailo_ai_sw_suite_docker_run.sh
```

Load it — do not copy it into the repo:

```sh
task docker:load          # docker load -i $SUITE_TARBALL
```

Expect 20–40 minutes for the 9.3 GB gzip stream, and no output until it finishes.
The tag only registers after every layer commits, so `docker images` showing
nothing is normal until the very end. Budget ~18 GB of disk for the loaded image.

## Start the container

**On Linux**, use the full run — it carries the device, X11 and dbus mounts the
suite expects:

```sh
task docker:run
```

**On Windows / Docker Desktop**, that command cannot work. The vendor script's
`prepare_docker_args()` mounts `/dev`, `/lib/modules`, `/lib/firmware`,
`/usr/src`, `/var/run/dbus/system_bus_socket` and the X11 sockets, none of which
exist on a Docker Desktop host. Use the minimal form instead:

```sh
task docker:run-compile-only
```

This starts a detached container with only the shared mount, plus the GPU flags
described below. Dropping the device passthrough costs nothing for a compile —
the Dataflow Compiler never touches the NPU. Only `hailomz eval --target hailo8`
does, and that needs a real Linux host with the Hailo PCIe/USB device anyway.

The container runs `sleep infinity` rather than the image's own command, which is
an interactive shell that exits immediately when detached and leaves nothing for
`docker exec` to attach to.

## Compile

```sh
task gmr:export     # .pt -> .onnx  (host, needs ultralytics)
task gmr:stage      # copy .onnx + calibration images into shared_with_docker/
task gmr:compile    # hailomz compile, inside the container
```

or `task gmr:workflow` for all three. The generated command is:

```
hailomz compile yolov11n \
  --ckpt /local/shared_with_docker/gmr.onnx \
  --calib-path /local/shared_with_docker/calib_data_gmr \
  --hw-arch hailo8 --classes 3
```

`--classes 3` matters. A retrained checkpoint keeps the zoo model's graph but not
its class count, so without it the NMS config is regenerated for COCO's 80
classes and the HEF decodes garbage.

## Failure modes

### `uv run hailo` — "program not found"

`pyproject.toml` declares `[project.scripts] hailo = "main:main"`, but uv skips
entry points for a project with no build backend:

```
warning: Skipping installation of entry points (`project.scripts`) for package
`hailo` because this project is not packaged
```

Every task in `Taskfile.yml` calls `uv run hailo`, so this broke all of them.
Fixed by declaring a `[build-system]` in `pyproject.toml`. If it recurs, check
that `.venv/Scripts/hailo.exe` (or `.venv/bin/hailo`) exists after `uv sync`;
`uv run python main.py ...` is the fallback that always works.

### Git Bash rewrites container paths

MSYS path conversion mangles any argument that looks like a Unix absolute path:

```
--calib-path /local/shared_with_docker/calib_data_gmr
          -> C:/Program Files/Git/local/shared_with_docker/calib_data_gmr
```

which surfaces inside the container as
`FileNotFoundError: Couldn't find dataset in C:/Program Files/Git/local/...`.
Paths built inside Python (`--ckpt`, from `DOCKER_SHARED_MOUNT`) are unaffected;
only shell arguments are rewritten. Export `MSYS_NO_PATHCONV=1` before invoking,
or run from PowerShell, which does not do this.

### Optimization silently drops to level 0

> **Measured caveat, GMR 2026-07-26.** Everything below is the vendor's
> reasoning, and it is a sound default — but it did not hold for this model.
> Level 0 came out near-lossless and the fully optimized build was *worse*. See
> "Measured: level 0 beat level 2" at the end of this document before spending
> hours on a GPU compile.

The most damaging failure, because it produces a valid HEF rather than an error:

```
[warning] Reducing optimization level to 0 (the accuracy won't be optimized and
          compression won't be used) because there's no available GPU
```

Finetune encoding, Bias Correction, Adaround, Quantization-Aware Fine-Tuning and
Layer Noise Analysis are all skipped. For a colour-discriminating detector that
is exactly the accuracy you cannot afford to lose.

The cause is not a broken CUDA stack. TensorFlow inside the container reports the
GPU correctly. `hailo_model_optimization/__init__.py` picks a device *before*
importing TensorFlow:

```python
if "CUDA_VISIBLE_DEVICES" not in os.environ:
    gpu = select_least_used_gpu()
    if gpu is not None:
        os.environ["CUDA_VISIBLE_DEVICES"] = gpu
    else:
        os.environ["CUDA_VISIBLE_DEVICES"] = "99"      # force CPU
```

and `select_least_used_gpu(max_memory_utilization=0.05)` only accepts a GPU less
than 5% full. A laptop GPU also driving a Windows desktop never qualifies:

```
$ nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv,noheader,nounits
0, 657, 6141          # 10.7% -> rejected -> CUDA_VISIBLE_DEVICES=99 -> CPU
```

Nothing about this is Windows-specific — any GPU also driving a display sits
above the threshold — so both `task docker:run` and
`task docker:run-compile-only` pass `--cuda-device 0`.

The guard is the fix: setting `CUDA_VISIBLE_DEVICES` explicitly skips the
selector. Measured side by side in the same container:

```sh
$ docker exec $C python -c "import hailo_model_optimization, os, tensorflow as tf; \
    print(os.environ['CUDA_VISIBLE_DEVICES'], tf.config.list_physical_devices('GPU'))"
[info] No GPU chosen and no suitable GPU found, falling back to CPU.
99 []

$ docker exec -e CUDA_VISIBLE_DEVICES=0 $C python -c "...same..."
0 [PhysicalDevice(name='/physical_device:GPU:0', device_type='GPU')]
```

After a compile, confirm the level-0 warning is absent from the log — that, not
the env var alone, is what proves the optimization actually ran.

### Container flags appear to have no effect

`docker run` reuses an existing container of the same name — it `docker start`s
it rather than recreating it, so anything baked in at creation time (the GPU
flags, `CUDA_VISIBLE_DEVICES`, the shared mount) keeps its old value however you
change the command line. Symptom: `--cuda-device` is passed, yet the compile
still reports level 0. Check what the container actually has, and recreate it if
it disagrees:

```sh
docker exec $C sh -c 'echo $CUDA_VISIBLE_DEVICES'
docker rm -f $C && task docker:run-compile-only
```

### The image's driver pin rejects modern drivers

The image sets `NVIDIA_REQUIRE_CUDA=... driver>=470,driver<471`. Any current
driver fails that check and the NVIDIA runtime refuses to start the container.
`--gpus all` is therefore paired with `-e NVIDIA_DISABLE_REQUIRE=1`; both are
emitted by `_gpu_args()`. Use `--no-gpu` on hosts with no NVIDIA GPU, where
`--gpus all` makes `docker run` fail outright.

### Only 64 calibration images are used

`task gmr:stage` copies 1339 prism images, but the log reads:

```
[info] Using dataset with 64 entries for calibration
```

64 is the SDK's default `calibset_size`, not a bug, and `hailomz compile` has no
flag for it — it lives in the `.alls` model script. To raise it, pass
`--model-script`. Note that a custom script *replaces* the zoo's
`cfg/alls/generic/<model>.alls` rather than extending it, so it must restate that
file's contents (for `yolov11n`: `normalization1`, three
`change_output_activation(..., sigmoid)` calls, `nms_postprocess(...)` and
`allocator_param(...)`) before adding `model_optimization_config(calibration,
calibset_size=N)`. Omitting them yields a HEF that compiles and decodes wrongly.

`--performance` compiles at the highest optimization level and is the simpler
lever when a GPU is available; it is mutually exclusive with `--model-script`
(`task gmr:compile-performance`).

## What a healthy run looks like

```
[info] Translation completed on ONNX model yolov11n
[info] Saved HAR to: /local/shared_with_docker/yolov11n.har
[info] Found model with 3 input channels, using real RGB images for calibration
[info] Starting Model Optimization
Calibration: 100%|##########| 64/64
[info] Trying to compile the network in a single context
[info] Single context flow failed: Recoverable single context error
[info] Using Multi-context flow
[info] Found valid partition to 3 contexts
```

The end nodes should be the six detect-head convolutions
(`/model.23/cv2.{0,1,2}/...`, `/model.23/cv3.{0,1,2}/...`). Anything else means
the checkpoint did not map onto the zoo graph.

Multi-context is expected here and is not an error: the model did not fit in one
context, so it is split across three and swapped at inference time. It costs
throughput relative to a single-context HEF.

Artifacts land in `shared_with_docker/`, named after the *zoo* model rather than
the registry key — so a GMR compile writes `yolov11n.har` / `yolov11n.hef`.
Rename on the way to `platform/robot` to avoid confusing it with the stock
COCO-trained `yolov11n.hef`.

## Measured: level 0 beat level 2 (GMR, 2026-07-26)

Both builds of the same checkpoint, evaluated through identical preprocessing in
`SDK_QUANTIZED` emulation, against the float checkpoint as the ceiling:

| Model | mAP@0.5 | mAP@0.75 | mAP@0.5:0.95 | red @0.5 |
|---|---|---|---|---|
| float (ceiling) | 0.9955 | 0.9767 | 0.8885 | 0.987 |
| level 0, CPU | 0.9954 | 0.9741 | 0.8808 | 0.986 |
| level 2 + QAT, GPU | 0.9689 | 0.9468 | 0.8096 | 0.930 |

Level 0 is within 0.01% of float at mAP@0.5 and 0.8% at mAP@0.5:0.95 — there was
almost no quantization error left to recover. Quantization-Aware Fine-Tuning
optimizes a distillation loss over *unlabelled* calibration images, and with that
little headroom it moved the weights away from the optimum instead of toward it.
Confirmed on two independent subsets (300 and 600 images): −2.0% to −2.7%
mAP@0.5, −7.5% to −8.1% mAP@0.5:0.95.

Per-class mAP@0.5 saturates and hides the shape of the loss; at mAP@0.5:0.95 the
regression is uniform rather than concentrated on any one colour:

| Model | green | magenta | red |
|---|---|---|---|
| level 0, CPU | 0.8661 | 0.9135 | 0.8688 |
| level 2 + QAT | 0.7950 | 0.8592 | 0.7955 |

The confusion counts matter more than mAP for this robot, because misclassifying
red as green inverts the pass side while merely *missing* a sign does not
(600 images, conf ≥ 0.25, IoU ≥ 0.5):

| Model | misclassified | missed | false positives |
|---|---|---|---|
| level 0, CPU | none | 2 | 119 |
| level 2 + QAT | magenta→red 1, red→magenta 1 | 23 | 31 |

Neither model ever confuses red with green. QAT's lower false-positive count is
not better precision — it detects less across the board, which is also where its
23 missed detections come from. The level-0 build's 119 false positives are
unaudited: with multiple prisms per image and one folder per dominant class,
some may be unlabelled ground truth rather than true errors. A deployment
confidence threshold above 0.25 suppresses most of them.

The lesson is not "skip optimization" — it is that the optimization level is an
empirical question per model, and cheap to settle. Compile both and measure
before shipping either.

Both eval harnesses live in `shared_with_docker/` (gitignored):
`eval_compare.py` runs inside the container over the HARs, `eval_float.py` runs
on the host for the float anchor. Two traps they encode:

- Ground-truth ids are in the *dataset's* class space while the model predicts
  in its own; `LABEL_TO_MODEL` bridges them. See "Class ordering" below.
- Ultralytics reads numpy input as **BGR**. Feeding RGB silently collapses the
  red class (AP 0.99 → 0.17) and makes the float model look worse than its own
  quantization — a wrong conclusion that looks entirely plausible.

## Class ordering

Three orderings are in circulation and only one is authoritative:

| Source | 0 | 1 | 2 |
|---|---|---|---|
| **Checkpoint / ONNX metadata (authoritative)** | green | magenta | red |
| `auto-annotator/ml-service/data/data.yaml` + label files | red | green | magenta |
| `platform/robot` `_DEFAULT_CLASS_TO_COLOR` (before 2026-07-26) | red | green | magenta |

The checkpoint wins: running it on the per-class image folders predicts "green"
on `green_prism`, "red" on `red_prism`. The `data.yaml` in the dataset directory
is stale — its `path` points at an archived OneDrive location. Consuming the HEF
with the dataset ordering swaps red and green, which inverts the WRO pass-side
rule on every obstacle, and nothing about it fails loudly.
