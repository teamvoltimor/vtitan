# Robot scripts

Grouped by **what you need in order to run it**, which is the axis that actually
decides where a script can execute and what it can tell you.

| folder | needs | what lives here |
|---|---|---|
| `bag/` | a recorded `.mcap` run | replay diagnostics - drive the real navigation classes over a bag and report what they did |
| `sim/` | nothing | closed-loop simulation sweeps and scenario harnesses |
| `hardware/` | the real robot | live probes and motor/vision utilities that talk to ROS2 topics or peripherals |
| `vision/` | the `vision` pixi env | HUD preview and camera-logo tooling |
| `provisioning/` | SSH to a Pi | bootstrap, deploy, shutdown and recovery shell ops |
| `sync/` | SSH to a Pi | pull/push recorded runs and videos to/from a Pi |
| `common/` | - | shared helpers imported by the above, not run directly |
| `*.sh` | SSH to a Pi | bench-mode HUD recording, vision recording, joystick pairing |

The shell ops scripts in `provisioning/`, `sync/` and the top level stay where
they are because `pixi.toml` tasks, the platform `Taskfile.yml` and the docs
all pin their paths.

## Map of the diagnostics

There are ~150 `diag_*.py` scripts, and `bag/` alone holds ~100 of them. They
are **not** split into subfolders: the filename is the namespace, and every
script is one self-contained question about one domain. The glob below is the
index -- list a row's glob to see the scripts in that domain. Read the module
docstring at the top of each: it is the question, the method, and the result,
not a summary of the code.

### `bag/` - replays of a recorded run

| domain | scripts | what the domain answers |
|---|---|---|
| pass side / routing | `diag_bag_pass_*.py`, `diag_bag_pair_crossing*.py`, `diag_bag_cross_attempt.py`, `diag_bag_exec_failures.py`, `diag_bag_side_correction_outcome.py`, `diag_bag_reverse_budget.py`, `diag_bag_lane_reconstruct.py` | did the router command the legal side, and did the chassis get there; separate routing from execution |
| signs | `diag_bag_sign_*.py`, `diag_bag_green_loss.py`, `diag_bag_colour_split.py` | discovery, colour voting, track birth, believed-layout reconstruction |
| commitment | `diag_bag_commit_*.py` | when and why the router committed to a sign |
| escape / stuck | `diag_bag_escape*.py`, `diag_bag_dwell_loop.py`, `diag_bag_creep_stall.py`, `diag_bag_blend_reachability.py` | the reactive layer: when it fires, which side, and what it buys |
| bay / parking | `diag_bag_bay_*.py`, `diag_bag_barrier_gate.py`, `diag_bay_slip.py`, `diag_bag_parking_attempts.py` | the pocket and parking manoeuvres |
| contact / clearance | `diag_bag_contact_*.py`, `diag_bag_proximity.py`, `diag_bag_side_ray_robustness.py`, `diag_bag_subfloor_ranges.py`, `diag_bag_wedge_trace.py`, `diag_bag_scan_occupancy.py` | how close contact came, and LIDAR dropouts under it |
| localizer / LIDAR | `diag_localizer_*.py`, `diag_bag_localizer_divergence.py`, `diag_bag_lidar_proposer.py`, `diag_bag_mask_*.py`, `diag_bag_rear_sector_measured.py`, `diag_bag_lidar_frame_census.py`, `diag_bag_yaw_frame_offset.py`, `diag_yaw_flip_replay.py` | pose and scan matching against the believed track; the census prints BOTH frames, because the mount offset was read wrong once |
| lap / race | `diag_bag_lap_*.py` | lap counting, the start/finish line, and what each lap cost (`diag_bag_lap_timeline.py`) |
| direction | `diag_bag_direction_*.py` | direction inference gates and votes |
| steering / motion | `diag_bag_steer*.py`, `diag_bag_steering_response.py`, `diag_bag_drive_response.py`, `diag_bag_servo_echo.py`, `diag_bag_turn_*.py`, `diag_bag_corner_*.py`, `diag_bag_angle_error.py` | actuator response and cornering |
| vision | `diag_bag_vision.py`, `diag_bag_camera_smear.py`, `diag_bag_detection_reach.py`, `diag_vision_range_ceiling.py` | detection quality on recorded frames |
| corpus / sessions | `diag_bag_session_*.py`, `diag_bag_fleet_compare.py`, `diag_bag_summary.py`, `diag_bag_review.py`, `diag_bag_sim_fidelity.py`, `diag_bag_topic_gaps.py` | inventory, cross-run reports, and whether a topic went silent |
| sim fidelity | `diag_bag_fidelity_axes.py` | the HARDWARE half of the divergence audit -- pair it with `sim/diag_fidelity_axes.py` |

Anything not matched above is a one-off trace (`diag_bag_state_timeline.py`,
`diag_bag_imu_trace.py`, `diag_bag_path_replay.py`, ...); it still follows the
same one-script-one-question rule.

### `sim/` - closed-loop sweeps, need no hardware or bag

| domain | scripts | what the domain answers |
|---|---|---|
| Open Challenge population | `diag_open_*.py` | direction, laps, narrow corridors, corner geometry over the 640-case space |
| sign routing | `diag_sign_*.py` | lane building, pass side, hits, pairs, and router-flag A/Bs |
| A/B tuning harnesses | `diag_today_stack_ab.py`, `diag_vision_range_ab.py` | the same sample under two tunings, with the control printed |
| bay / parking | `diag_bay_*.py`, `diag_park_*.py`, `diag_pair_cells.py` | pocket entry/exit and parking feasibility |
| escape / blend | `diag_escape_*.py`, `diag_flush_*.py` | escape duration and side-correction blends |
| localizer / LIDAR | `diag_localization.py`, `diag_sim_lidar_proposer.py` | pose recovery and the LIDAR cluster proposer |
| path / width / clearance | `diag_path_track*.py`, `diag_width_probe.py`, `diag_wide_wall_hug.py`, `diag_collision_margin.py` | path geometry and margins |
| blind / failures | `diag_blind_layout.py`, `diag_failure_split.py`, `diag_obstacles_matrix.py` | blind-mode layout and failure attribution |
| sim fidelity | `diag_fidelity_axes.py` | the SIM half of the divergence audit -- pair it with `bag/diag_bag_fidelity_axes.py` |
| single runner | `run_scenario.py` | run one fixture, print its result as JSON, exit non-zero on a bad run |

### `hardware/` - talks to the real robot

`test_motors.py`, `calibrate_encoder.py`, `sweep_open_loop.py`,
`reset_motors.py`, `diag_servo_slew.py` (motors); `diag_hailo_detector.py`,
`diag_vision_topic.py`, `record_vision_video.py`, `watch_vision_detections.py`,
`diag_focus_sweep.py` (vision); `diag_scan_probe.py`, `diag_corridor_measure.py`
(sensors); `diag_track_run.py` (a whole flagged run). Only
`diag_hailo_detector.py` needs the `vision` pixi env.

### `common/` - imported, never run directly

| group | modules |
|---|---|
| bag replay | `bag_io.py`, `scenarios.py`, `pass_side.py`, `cross_attempt.py`, `episodes.py`, `sign_router_capture.py` |
| sim sweeps | `diag_base.py`, `sim_defaults.py`, `open_cases.py` |
| hardware/sim comparison | `fidelity_axes.py` -- the axis definitions the two `*fidelity_axes.py` probes must share |
| reporting | `tables.py`, `stats.py`, `formats.py`, `binning.py` |
| environment | `provenance.py`, `sensor_errors.py`, `hardware_defaults.py` |
| other | `lidar_clusters.py`, `motor_hold.py`, `analyze_rtps_pcap.py`, `check_imports.py` |

Import these as `from scripts.common.bag_io import ...`. A private helper that
two scripts share belongs here, not in one script imported by the other: a
private cross-script import is a hidden library, and the whole tree used to be
full of them.

## Running one

Every script is run from `src/python/` with the repo root on `PYTHONPATH`,
which the `dev` pixi env provides:

```
pixi run -e dev python scripts/bag/diag_bag_lap_replay.py ../../other/data/live/runs/run_20260806_180154
pixi run -e dev python scripts/sim/diag_open_laps.py
pixi run -e vision python scripts/hardware/diag_hailo_detector.py IMAGE...
```

`scripts/hardware/diag_hailo_detector.py` needs the `vision` env - `hailo_platform`
is only installed there and on the Pi 5.

## Adding one

Put it in the folder matching what it needs to run. Import shared helpers as
`from scripts.common.bag_io import ...`, and put the repo's robot root on the path
with `parents[2]`, not `parents[1]`:

```python
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import load_nav_debug_rows, print_table
```

## Checking the tree still imports

A folder move or a renamed helper breaks imports silently - nothing runs these
in CI. `scripts/common/check_imports.py` loads every script's top level (without
running `main()`) and reports what failed:

```
pixi run -e dev python scripts/common/check_imports.py
```

One expected failure off the Pi 5: `hardware/diag_hailo_detector.py`, for the
missing `hailo_platform` runtime.
