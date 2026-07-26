# Wiring the GMR detector HEF into the robot

Status as of 2026-07-26. The 3-class GMR traffic-sign detector is compiled and
measured, and the robot-side code path now consumes it: the blocking items
below are **done**. What remains needs the Pi, plus deployment of the artifact
itself.

**Deployed and verified on the Pi 5, 2026-07-26.** The detector runs on the
Hailo-8 and publishes correct colours on `/vision/detections`. Deploy with
`bash scripts/deploy-to-pi5.sh`; verify with `scripts/diag_hailo_detector.py`
(direct, needs the NPU free) or `scripts/diag_vision_topic.py` (through the
running stack).

Measured on device:

| Check | Result |
|---|---|
| HEF throughput | 101.5 FPS (`hailortcli run`, 3 contexts) |
| Runtime NMS layout | `list` of per-class `(n_boxes, 5)` arrays |
| Channel order | **RGB**. BGR reads red as green or drops it entirely |
| End-to-end colours | red 0.74 · green 0.94 · magenta 0.95 on known photographs |

**The one thing still missing is the camera feed.** Nothing publishes
`/camera/image_raw`, so the detector has no live input — see "No camera node"
below. Everything above was verified by injecting frames onto that topic.

Compile-side background lives in
[`hailo/docs/hef-compile-runbook.md`](../../../hailo/docs/hef-compile-runbook.md).

## What already exists

| Artifact | Where | Notes |
|---|---|---|
| `gmr.hef` | `auto-annotator/ml-service/models/gmr/gmr.hef` | The build to ship. Gitignored, so it is local-only — recompile with `task gmr:workflow` if lost |
| `gmr_cpu_opt0.hef` / `.har` | `hailo/shared_with_docker/` | Same build, original name |
| `gmr_gpu_qat.hef` / `.har` | `hailo/shared_with_docker/` | Rejected, see below |
| Class-colour map | `platform/robot/src/vision/detector.py` | `DEFAULT_CLASS_TO_COLOR`, already corrected |

The HEF's interface, from `hailortcli parse-hef`:

```
Architecture: HAILO8      Multi Context - Number of contexts: 3
Input   yolov11n/input_layer1   UINT8, NHWC(640x640x3)
Output  yolov11n/yolov8_nms_postprocess  FLOAT32,
        HAILO NMS BY CLASS (classes: 3, max bboxes per class: 100,
                            maximum frame size: 6012)
Op YOLOV8 — Classes: 3, Score threshold: 0.200, IoU threshold: 0.70
```

Three facts from that block drive most of the work below: the input is **UINT8**
(normalization is baked into the graph), the output is **NMS BY CLASS** (no
class-id column — class identity is positional), and it is a **3-context** model.

Which build won, measured on 300 and 600 images against the float checkpoint:

| Model | mAP@0.5 | mAP@0.5:0.95 | misclassified | missed |
|---|---|---|---|---|
| float (ceiling) | 0.9955 | 0.8885 | none | 0 |
| **`gmr_cpu_opt0`** | 0.9954 | 0.8808 | none | 2 |
| `gmr_gpu_qat` | 0.9689 | 0.8096 | 2 | 23 |

Reproduce with `task gmr:stage-eval && task accuracy:all` from `hailo/`.

## Fixed 2026-07-26 (kept for the reasoning)

### 1. `HailoDetector.detect` parsed a format this HEF does not emit

`platform/robot/src/vision/detector.py:184`

```python
for box in output:
    conf = float(box[4])
    color = self._config.get_color(int(box[5]))   # <- class id in column 5
```

This expects flat 6-column rows `[y1, x1, y2, x2, conf, class_id]`, the layout of
NMS-by-score. The HEF emits NMS **by class**: 5 values per box, class identity
positional. `6012 = 3 x (1 + 100 x 5) x 4` bytes confirms it — a per-class count
followed by up to 100 five-value boxes. There is no `box[5]`.

**Fixed** by `iter_nms_by_class` in `src/hardware/hailo/inferences.py`, which
both `HailoDetector.detect` and `parse_yolo_nms_output` now go through, so there
is one decoder rather than two that can disagree. It normalises every layout
HailoRT or the SDK emulator is known to produce and raises `NmsFormatError` on
anything else — a 6-column NMS-by-score tensor now fails loudly instead of
reading a coordinate as a confidence. Covered by
`tests/unit/test_nms_by_class.py`.

Two further breaks surfaced while fixing this, both from the in-flight
dataclass-to-pydantic refactor and both fatal to the vision path: `detector.py`
used `@dataclass` without importing it (`NameError` at import), and constructed
`SignDetection` — now a pydantic `BaseModel` — positionally (`TypeError` at
runtime). Both fixed.

### 1b. The published payload was not the one the navigator reads

The vision node published `SignDetection.to_dict()` — `color`, `bbox`,
`confidence`. The navigator's `_vision_callback` rebuilds a `Detection` from
`class_name`, `x`, `y`, `width`, `height`, `area`, and `sign_router` keys colour
confirmation off `class_name in (RED, GREEN)`. None of those keys were present,
so every detection arrived with an empty `class_name` and a zero centroid, and
colour confirmation could never fire — with no error anywhere. The node now
publishes `asdict(d.to_detection())`, which is that schema.

### 2. The streaming path double-normalized

`platform/robot/src/hardware/hailo/streaming.py:33`

```python
resized.astype(np.float32) / 255.0
```

The input tensor is UINT8 and the graph already contains
`normalization1 = normalization([0,0,0], [255,255,255])`, so this both mismatched
the dtype and normalized twice. **Fixed** — `preprocess` now returns the resized
frame as uint8.

### 3. The driver carried a fourth class ordering, with a wrong fallback

`platform/robot/src/hardware/hailo/config.py`

```python
data_yaml_path: str = "/usr/local/hailo/models/data.yaml"
...
except (FileNotFoundError, ValueError, KeyError):
    self.class_map = {0: "red_pillar", 1: "green_pillar", 2: "wall"}
```

Wrong order, and `wall` is not a class this model has. The authoritative order is
the checkpoint's own metadata:

| id | 0 | 1 | 2 |
|---|---|---|---|
| **model (authoritative)** | green | magenta | red |
| `auto-annotator/.../data.yaml` (stale) | red | green | magenta |
| `_DEFAULT_CLASS_TO_COLOR` (before 2026-07-26) | red | green | magenta |
| driver `class_map` fallback | red_pillar | green_pillar | wall |

Confirmed by running the checkpoint per class folder: `green_prism` images
predict green, `red_prism` predict red. The dataset `data.yaml` is stale — its
`path` points at an archived OneDrive directory. Getting this wrong swaps red and
green, which **inverts the WRO pass-side rule on every obstacle** and fails
silently.

**Fixed** — `GMR_CLASS_NAMES` in `platform/shared/src/shared/domain/enums.py` is
now the single declaration. `DEFAULT_CLASS_TO_COLOR` is derived from it and the
driver's fallback `class_map` is a copy of it, so the two cannot drift apart
again. If `data_yaml_path` resolves to a real file it still wins, so a stale
`data.yaml` deployed beside the HEF can still override it — ship one in *model*
order or delete it.

## No camera node — the remaining blocker

`/camera/image_raw` has a subscriber (the vision node) and **no publisher**. The
camera driver in `src/hardware/camera/` is a plain Python class with no ROS
wrapper, and `rpi5_nodes.launch.py` starts no camera node, so on the real robot
the detector never receives a frame. Until that bridge exists, obstacle
navigation cannot use vision no matter how correct the detector is.

The hardware itself is fine — `rpicam-hello --list-cameras` reports
`imx708_wide` and `rpicam-still` captures at 1536×864 (specs in
[robot-physical-constants.md](robot-physical-constants.md)). A test capture on
2026-07-26 came back framed on the robot's own ribbon cable and badly out of
focus at close range, so **check aim and obstruction** before reading anything
into an empty detection list.

Writing the node is small: capture with Picamera2, publish `sensor_msgs/Image`
with encoding `rgb8` on `/camera/image_raw`. The vision node converts `bgr8` to
RGB itself, so either encoding is safe as long as it is labelled honestly —
mislabelling is the silent red/green swap again.

## Needs the Pi in front of you

### 4. RGB vs BGR is unverified, and the failure is silent

There is no `cvtColor` anywhere under `src/hardware/camera/`,
`src/hardware/hailo/`, or `src/vision/`. The model expects RGB; OpenCV is BGR and
Picamera2's `RGB888` is byte-reversed in numpy. Feeding the wrong order does not
error — during evaluation it dropped red-class AP from 0.99 to **0.17** while
green stayed at 0.99, which reads as a model problem rather than a plumbing one.

Check by pointing the camera at a red prism and confirming it reports red, not
magenta or nothing.

### 5. Runtime tensor layout

SDK emulation returned `(classes, 5, boxes)`; `parse_yolo_nms_output` expects
`(classes, boxes, 5)`. HailoRT on-device may differ from emulation. Print the raw
tensor shape once on hardware before trusting either.

## Smaller

6. **`cv2.resize` stretches** (`detector.py:180`, `streaming.py:33`); the model was
   trained — and evaluated — letterboxed. Costs accuracy on non-square frames.
7. ~~Three `model_path` defaults disagree.~~ **Fixed** — all three now resolve to
   `/usr/local/hailo/models/gmr.hef`, and `create_detector` reads it from
   `HailoConfig` (so `HAILO_MODEL_PATH` is the one override point) rather than
   hardcoding a second default.
8. **Deploy the artifact** — copy `gmr.hef` to the Pi and set `HAILO_MODEL_PATH`.
   If `data_yaml_path` is kept, ship a `data.yaml` in *model* order beside it.
9. ~~Uncommitted vision changes.~~ Committed.
10. **Test-suite caveat.** The suite runs on Windows via `pixi run -e dev test`
    (not uv). It has ~45 pre-existing failures from in-flight navigation work,
    plus a handful of tests that flake between runs: ROS2 topic-discovery races
    (`test_button_node`, `test_state_machine_node`), a timing budget
    (`test_lidar_localizer`), and `test_imu_bno08x_uart_rvc_node`, which needs
    `adafruit-platformdetect` to recognise the board and so cannot pass off-Pi.
    Compare failure *sets* against a baseline rather than counts.
11. **Benchmark on-device**: 3 contexts means per-frame context-switch overhead a
    single-context model would not have. If the frame rate is short, revisit
    `--performance` or a smaller input size.

## Suggested order

1–3 and 7 are done. What is left all needs the device: deploy the HEF (8), then
settle the colour order (4) and the runtime tensor layout (5) on the first run,
then characterise the frame rate (11).

The quickest way to settle 4 and 5 together is one run on the Pi with the camera
pointed at a red prism: print the raw tensor shape once, and check the reported
colour. A red prism reading as red confirms both the channel order and the class
mapping; reading as magenta or nothing points at RGB/BGR.

One caution carried over from the compile work: the level-0 build beating the
fully optimized one was **measured, not predicted** — the reverse of what the
vendor warning implies. Re-measure with `task accuracy:all` rather than
reasoning from first principles if the model is ever recompiled.
