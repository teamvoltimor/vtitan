# 0091. Dataset frames are captured periodically beside the run artifacts

- Status: accepted
- Date: 2026-09-16

## Context

Hardware rounds are the only source of real-camera training data, and a round
cannot be repeated. The other per-run artifacts are consumed for analysis, not
for fine-tuning: the MCAP bag excludes `/camera/image_raw` (0071) and the debug
video carries detection boxes and the HUD burned in. Neither yields clean, raw
frames showing the track's real lighting, lens and motion.

## Options considered

- (a) Pull frames from the MCAP bag offline.
- (b) Capture raw frames to disk during the race, on a bounded cadence.

## Decision

(b). `capture_dataset_frames = true` writes the same RGB frame the detector sees,
before `overlay.annotate()` touches it, to `<run_dir>/<capture_subdir>/` (shipped
`captures`, every `capture_interval_s = 10.0`). Open Challenge saves every
interval unconditionally. Obstacles Challenge stays on the same cadence but,
once an interval has elapsed, waits frame over frame until a detection is
present, so every Obstacles frame contains an obstacle instead of empty track by
coincidence.

The run directory is owned by `bag_recorder_node`; the capture only polls for it,
the same rule the video recorder follows. JPEG encoding runs on a dedicated
writer thread behind a bounded queue (`_QUEUE_MAXSIZE = 8`), so `cv2.imwrite`
never lands on the capture/inference tick; a stalled disk drops older frames
rather than buffering without limit. `flush()` drains the queue and `close()`
stops the thread at shutdown.

## Consequences

- Hardware rounds accumulate real-world training data for free, beside the bag
  and the debug video in the same run directory.
- A stalled disk drops frames instead of growing RAM, which is acceptable for a
  periodic training set.
- The capture is raw and un-annotated by design, so it is directly usable for
  fine-tuning and cannot inherit a detector or HUD error.

## Cross-references

- 0071 owns the per-run MCAP recording and the run-directory layout; this adds
  another artifact beside the bag, the provenance file and the debug video.
- 0072 owns the vision data path the captured frame is taken from.
