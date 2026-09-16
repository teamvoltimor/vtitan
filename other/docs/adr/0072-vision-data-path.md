# 0072. The camera feeds the NPU in-process and vision owns identity, not distance

- Status: accepted
- Date: 2026-09-15

## Context

The vision path had several silent-failure modes. A class-order or channel-order
mistake swaps red and green with nothing failing loudly, which inverts the WRO
pass-side rule on every obstacle. A ROS image hop would add latency to a pipeline
already budgeted tightly. And a vision miss must not remove collision safety.

## Options considered

- (a) Publish `sensor_msgs/Image` and give the NPU its own subscriber; trust the
      dataset's `data.yaml` class order and a single NMS layout.
- (b) Open the camera in-process and feed the NPU directly; pin the class order to
      the checkpoint; keep one NMS decoder; give identity and bearing to vision
      and distance and safety to the LIDAR.

## Decision

(b). The vision node opens the camera in-process (`camera_source = "direct"`) and
feeds frames straight to the Hailo NPU; a race publishes only `/vision/detections`
and no imagery (debug image topics are opt-in and cost about 4 MB per frame). The
live chain runs at 15 Hz, set by `capture_fps` (the HEF itself benchmarks about
101 FPS, so the cap is the timer, not the model). The camera backend prefers
Picamera2 and falls back to `rpicam-vid` MJPEG, because picamera2's libcamera
bindings target the system interpreter while ROS nodes run a different pixi
Python, and OpenCV cannot open libcamera media nodes.

The rpicam colour and noise keys ship at `rpicam-vid`'s own defaults, so the
image reaching the detector is unchanged; the keys exist for reachability, not
as a vision-baseline change. Red/green classification is threshold-based, and
auto AWB re-tints the frame as framing changes, so a wrong fixed preset is worse
than `auto`; the intended race preset is `fluorescent`, flipped only after
re-checking the thresholds against venue footage. Both metering modes are
ignored once `camera_exposure_time_us` is set (it turns AE off entirely), which
is the real answer if the white mat fools the meter; `+ev` compensation instead
lengthens the shutter, which smears signs in corners, and `spot` meters the
frame centre where signs are not reliably placed, so it often meters bare mat.

The debug video keeps the camera's native width rather than downscaling: the
about 6x larger per-run file (about 20 to 120 MB for 3 min) is free against the
SD card's 460 GB, and `annotate()` draws boxes on the full-resolution frame
before the resize, so they scale with the image and need no separate coordinate
transform.

The class order is authoritative from the checkpoint metadata (0 green, 1 magenta,
2 red); `GMR_CLASS_NAMES` is the one declaration and the driver's colour mapping
derives from it. `iter_nms_by_class` is the single decoder, normalising the
list-of-per-class Hailo layout and raising on any other shape. The HEF is compiled
with `--classes 3`, because without it the NMS config is regenerated for COCO's 80
classes and the HEF decodes garbage.

The confidence floors are 0.45 at the detector (below that a detection never
reaches the navigator) and 0.25 at the sign router, which is only for late
confirmation of an already-discovered sign, never to create a new track. A
backend change goes through `HAILO_MIN_CONFIDENCE` / `DETECTOR_MIN_CONFIDENCE`,
never a literal, so it is deliberate rather than a forgotten value.

The division of labour: the LIDAR proposes WHERE and the camera decides WHAT. A
LIDAR proposal is position-only and never publishes a sign by itself. The camera
is explicitly NOT the safety net: a false detection causes an unnecessary dodge,
a missed one leaves the LIDAR collision controller active. This is carried into
0058.

## Consequences

- No image hop in the normal path, so the latency budget holds.
- A wrong channel or class order is caught at one declaration, not four.
- Camera or NPU failure degrades strategy, not collision avoidance.
- The ungated LIDAR range fusion is the version measured harmful; the cluster gate
  is mandatory (see 0058).

## History

- 85ebafed 2026-07-26: compile the retrained 3-class detector with `--classes 3`.
- 1e3df713 2026-07-26: consume the HEF and collapse class-order drift; one NMS
  decoder and `GMR_CLASS_NAMES`.
- daf70c06 2026-07-26: level 0 beat level 2 plus QAT: level 0 is within 0.01
  percent mAP@0.5 and 0.8 percent mAP@0.5:0.95 of float, while level 2 + QAT lost
  2.0 to 2.7 percent mAP@0.5 and 7.5 to 8.1 percent mAP@0.5:0.95 and produced the
  only red/magenta confusions.
- 6b97d7ca 2026-07-26: capture in-process and add an opt-in debug video; rpicam
  MJPEG fallback.
- f9b3d6b9 and 36689f7c 2026-07-26 / 2026-08-02: make the confidence floor
  actually gate detections; align the detector floor to 0.45.
- d9c62a17 2026-08-12: letterbox replaces `cv2.resize` for the NPU (pad 114).
- 6d3e3328 2026-09-06: the camera's bearing was mirrored, placing every sign on
  the far wall (174 px upright against 528 px mirrored); turns ungated range fusion
  off.
- 91b2ffe3 2026-09-10: one shipped detection-confidence floor; Hailo fields resolve
  back to `detector.toml` (three copies collapsed).
- 0e088ada 2026-09-14: the emulated camera gets the hardware's detection rate and
  latency (54.5 percent to 11.0 percent, against hardware's 11.6 percent).
- c9358428 2026-09-15: range is not the limit; the residual is zero-mean bearing
  scatter (see 0058).
- Real detector median detection range is 0.70 m against the emulator's 10 m far
  clip; the range model alone left the emulated camera carrying a detection on about
  54.5 percent of ticks against hardware's 11.6 percent over 125 bags, and 15 Hz
  against the 20 Hz loop explains about a quarter of that gap.
- Colour flip: 111 of 2,162 detections carry the opposite colour (5.1 percent), and
  the real errors are concentrated (most pillars near 0 percent, one at 47 percent)
  where the model is i.i.d; the shipped `vision_color_flip_rate = 0.0` is unmeasured
  and contradicts this.
- Bearing scatter is zero-mean with sigma 0.232 rad (13.3 deg) and IQR -9.36 to
  +8.59 deg over 2,588 detections; at 1.5 m that is about 0.35 m of lateral miss,
  wider than `association_dist_m` 0.25 and `detection_match_dist_m` 0.30, while the
  pose estimate alone carried about 1.7 deg.
- Confidence is calibrated at the median only: across 3,315 real detections p10 is
  0.515 and p90 0.917, against p10 0.477 and p90 0.942 from evenly-spaced sampler
  levels.
- 4f8dbdf 2026-08-02: `_vision_callback`'s deferred import named `ros2.vision...`
  (missing the `src.` prefix), so every real `/vision/detections` message raised
  `ModuleNotFoundError` out of the callback and `_latest_detections` stayed empty;
  `node.py`'s top-level import had the same class of bug and crash-looped
  `vision_node`.

## Cross-references

- 0058 owns the sign discovery and range fusion this path feeds.
- 0069 owns the config; 0073 owns the challenge resolved at runtime.
- 0044 (camera autofocus) is carried in the hardware batch.
