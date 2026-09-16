# Python↔Go comparison, provisioning, tests & resource profiling

Date: 2026-08-30
Last updated: 2026-08-30
Status: draft (working plan; update `Last updated` and the sections below as the
codebase moves - see "Change ledger" at the end for the rolling diff).

## Scope

This plan tracks four comparisons between the Python runtime
(`platform/robot`) and the Go port (`platform/robot-go`):

1. **Code changes since 2026-08-28** - what moved in each tree, and whether a
   Python change has a Go counterpart yet (or vice versa).
2. **Provisioning & configuration** - how each program is built, deployed, and
   configured; whether the config schemas line up.
3. **Tests vs current codebase** - coverage of the ported packages, and the
   parity gate that diffs Go output against Python/recorded bags.
4. **Memory & CPU** - measured footprint of each program against its original
   counterpart (Go node vs Python node; Go sim vs `run_scenario.py`).

## 1. Changes since 2026-08-28

### Method
```pwsh
# Python tree
git -C platform/robot log --since=2026-08-28 --oneline -- platform/robot
# Go tree
git -C platform/robot-go log --since=2026-08-28 --oneline
```
Re-run these and update the ledger below. Commits touching BOTH trees with the
same hash (e.g. `d0352e5e`, `5bd18568`) are shared fixes - the Go side absorbed
them via the parity gate.

### Baseline note (captured 2026-08-30)
As of this date the Go tree has **already ported the entire blind stack** that
the earlier `2026-08-30-sim-go-native.md` plan assumed missing:
- `ed088467` port corridor_follower (blind creep)
- `50c83322` port corridor_estimator (corridor widths from LIDAR)
- `b1977a32` port wall_heading (absolute heading from Manhattan walls)
- `de440b25` port localization (LIDAR pose search against known walls)
- `e285a305` port race_tracker (geometric lap detection + race metrics)
- `7e5c761d` port internal/nav/navigator (composition root)

So the remaining Go-native-sim gap (per `2026-08-30-sim-go-native.md`) is now
**narrower**: the closed-loop `SimHardwareGateway` harness + `native_runner.go`
are still unported (Gap 1), but blind *navigation* itself is largely in Go.
`SignRouter.discover` (incremental sign publication from camera) and the
`SimulatedHardwareGateway` glue remain the open items.

### Change ledger (rolling)
| Date | Tree | Commit | Summary | Go counterpart? |
|------|------|--------|---------|-----------------|
| 2026-08-30 | go | ed088467 | port corridor_follower (blind creep) | native |
| 2026-08-30 | go | 50c83322 | port corridor_estimator | native |
| 2026-08-30 | go | b1977a32 | port wall_heading | native |
| 2026-08-30 | go | de440b25 | port localization | native |
| 2026-08-30 | go | e285a305 | port race_tracker | native |
| 2026-08-30 | go | 7e5c761d | port navigator composition root | native |
| 2026-08-28+ | py | d0352e5e | blind creep corner steering angle | ✅ shared hash |
| 2026-08-28+ | py | 5bd18568 | in-bay start deadlock fix | ✅ shared hash |
| 2026-08-28+ | py | bfd644d4 | calibrate AckermannKinematics vs bag | ✅ shared hash |
| 2026-08-28+ | py | d47b1cbe | encoder calibration per-motor overlay | go: hardware driver only |
| 2026-08-28+ | py | 76abd3fe | side-ray centre-offset by corner dist | go: diag only |
| 2026-08-30 | go | 20500148 | sim scenario-orchestrator, CLI cobra migrate | shared fix (parity gate) |
| 2026-08-28 | py | 1105b542 | forward extra pixi args; require explicit motor backend | none (tooling/Taskfile only) |
| 2026-08-28 | py | 4f254ef4 | recalibrate encoder for new drive motor, PID-step log | none (hardware-only) |
| 2026-08-28 | py | 7656b2ff | calibrate_encoder.py --speed default predates data | none (diag-only) |
| 2026-08-28 | py | d8791b89 | assert BTS7960 R_EN/L_EN after PWM 0 duty | go: BTS7960 driver (01e7cb78) |
| 2026-08-28 | py | 47915e34 | widen SpeedEstimator sample window (RPM quant) | none (hardware-only) |
| 2026-08-28 | py | e4de2f27 | enable camera_inverted for upside-down mount | none (hardware-only) |
| 2026-08-28 | py | 2be3eae2 | SIDE_CORRECTION escape checks forward contact | go: collision ported (4da75303) |
| 2026-08-28 | py | c6081597 | apply LIDAR yaw offset to HUD radar scan angles | go: yaw offset in profile (d35e5570) |
| 2026-08-28 | py | 310ff30d | forward-lane no-return not clear road | none (nav-only) |
| 2026-08-28 | py | 822a7b06 | slow corners first lap (Open) | none (nav-only) |
| 2026-08-29 | py | 1913975c | counts_per_rev 86->60, unfreeze pose >0.25 m/s | none (hardware-only) |
| 2026-08-29 | py | 264d58ab | affine feedforward live-verified docs | none (docs only) |
| 2026-08-29 | py | 27146f7a | speed feedforward duty deadband, max_rpm 175->348 | none (hardware-only) |
| 2026-08-29 | py | 958011fa | counts_per_rev=60 verified docs | none (docs only) |
| 2026-08-29 | py | 38d38724 | split centreline bias by corridor width | go: corridor ported (ed088467) |
| 2026-08-29 | py | 90bdfcfa | narrow/wide corridor own bias side | go: corridor ported (ed088467) |
| 2026-08-29 | py | bbfd700c | ramp lookahead instead of switching | none (nav-only) |
| 2026-08-29 | py | 9fdfa264 | hold corner preview open until turn driven | go: corner_latch ported (6ca98c41) |
| 2026-08-29 | py | 6fab9d17 | measure steering weave, sweep corner latch release | go: sim kinematics+collision (4da75303) |
| 2026-08-30 | py | 93ce99df | pull-runs match documented run prefixes | none (tooling only) |
| 2026-08-29 | py | 4ba4e9c8 | repair diag_bag_corner_window import | go: diag Aggregator (acbba131) |
| 2026-08-29 | py | bbf9660d | raise max_rpm to 175 (feedforward saturating) | none (hardware-only) |
| 2026-08-30 | go | e6a88982 | port parking maneuver (last nav module) | native |
| 2026-08-30 | go | 468cc5cb | NATS HardwareGateway adapter + track-navigator | native |
| 2026-08-30 | go | 4bb2defc | navigator golangci-lint clear + split | native |
| 2026-08-30 | go | 6ca98c41 | consolidate angle-to-steering-norm into navutil | native |
| 2026-08-30 | go | 84994d8f | port start_conditions + start_measurement | native |
| 2026-08-30 | go | 8986b41a | adapt state/ui schemas, race-metrics units | native |
| 2026-08-30 | go | 3da0c66d | wire nav+state schemas onto domain types | native |
| 2026-08-30 | go | 9ffb4bb7 | navigator Step branches + path mgmt tests | native |
| 2026-08-30 | go | 4a0393fe | protobuf schemas for remaining topics | native |
| 2026-08-30 | go | 6ad220ab | NavigatorDebug protobuf schema | native |
| 2026-08-30 | go | a0f25133 | MCAP bag-replay parity harness | native |
| 2026-08-30 | go | e0d0e1e6 | tag node/motor linux-only for off-device build | native |
| 2026-08-30 | go | 4da75303 | port signrouter, controllers, sim kin/collision | native |
| 2026-08-29 | go | 927d8709 | viper profile -> nav-tuning TOML tree | native |
| 2026-08-29 | go | ddb4ea84 | port waypoint geometry/class/assembly | native |
| 2026-08-29 | go | 5844ef6d | port direction estimator + track geometry | native |
| 2026-08-29 | go | 3efd2679 | golangci-lint install + clear findings | native |
| 2026-08-29 | go | 34336dc2 | cmd/state-machine gRPC client to backend | native |
| 2026-08-29 | go | 49bc4c8b | viper config -> full specs/track/drivers | native |
| 2026-08-29 | go | d35e5570 | internal/config/profile + motor/LIDAR wiring | native |
| 2026-08-29 | go | f1c60f84 | centralize NATS subjects/URL, shared motor loop | native |
| 2026-08-29 | go | e168e4bc | assemble cmd/pi-zero (motor+button+OLED) | native |
| 2026-08-29 | go | 8dde0174 | sd_notify abstract-socket test vs real listener | native |
| 2026-08-29 | go | 00bf79f6 | merge internal/supervise implementation | native |
| 2026-08-29 | go | 0783fe14 | supervise pkg (restart + sd_notify) | native |
| 2026-08-29 | go | acbba131 | cmd/telemetry-node + diag.Aggregator on NATS | native |
| 2026-08-29 | go | 40aae82d | button_event + telemetry_summary proto | native |
| 2026-08-29 | go | 8e34a8a6 | wire cmd/imu+lidar-node on NATS | native |
| 2026-08-29 | go | 31e35a04 | wire cmd/motor-node on NATS | native |
| 2026-08-29 | go | 19bdff64 | NATS transport layer Pub/Sub[T] | native |
| 2026-08-29 | go | 5df9fe88 | core state machine + cmd/telemetry plumbing | native |
| 2026-08-29 | go | b61b6b0a | golines formatting in sim-runner | native |
| 2026-08-29 | go | 1cc9fc4b | telemetry-summary aggregation logic | native |
| 2026-08-28 | go | f7d82b18 | SSD1306 OLED I2C driver | native |
| 2026-08-28 | go | 2bfdf7ab | button driver (debounce + hold) | native |
| 2026-08-29 | go | 01e7cb78 | BTS7960 motor driver, keep boot-kick fix | native |
| 2026-08-28 | go | 43662ee0 | scaffold Go/NATS workspace (ROS2 replace) | native |
| 2026-08-28 | go | 86669f97 | LIDAR RPLIDAR C1 driver, SCAN mode | native |
| 2026-08-28 | go | f6b46da9 | IMU UART-RVC driver, ground-truth verified | native |
| (add new rows here as code moves) | | | | |

## 2. Provisioning & configuration

### Python (`platform/robot`)
- **Provisioner**: `pixi.toml` (RoboStack/conda, channels `robostack-kilted` +
  `conda-forge`). Pulls full ROS2 Kilted (`ros-base`, `geometry-msgs`,
  `nav-msgs`, `sensor-msgs`, `ackermann-msgs`, `cv-bridge`), Python 3.12,
  numpy<2, opencv, pydantic, requests. Platforms: `linux-64`/`linux-aarch64`
  (glibc 2.34) + `win-64` (no glibc).
- **Deployment**: ROS2 nodes launched via `launch` files; systemd units under
  `platform/robot/systemd`; configs are TOML under `shared/config/**` and
  `src/config/**`, loaded by `pydantic-settings`.
- **Runtime deps**: ROS2 DDS, gRPC backend channel, camera/IMU/LIDAR drivers.

### Go (`platform/robot-go`)
- **Provisioner**: none. `go.mod` only; builds with `go build ./...`. No pixi,
  no Dockerfile, no ROS2. Proto codegen via `buf.gen.yaml` (`buf generate`).
- **Deployment**: native binaries per `cmd/*` (e.g. `motor-node`, `imu-node`,
  `navigator` node, `foxglove-bridge` stub). Comms over **NATS**
  (`internal/transport/nats`) + protobuf, not ROS2/DDS.
- **Config**: Go-native `internal/config/profile` - TOML profiles loaded by
  `viper`, mirroring the Python `shared/config` TOMLs. `configs/profiles/`
  holds deployment overlays.

### Comparison / gaps
- **Config schema parity**: `internal/config/profile/*` (e.g. `corridor_follower.go`,
  `direction_estimator.go`, `wall_heading.go`, `start_measurement.go`,
  `simulation.go`) are 1:1 mirrors of the Python TOMLs. Diff them field-by-field
  per package.
- **Provisioning gap**: the Go side has **no deployment story** (no pixi/Docker/
  systemd). If Go nodes run on-device they need a provisioner (a `pixi.toml`
  with a `go` dependency, or a Dockerfile/static binary + systemd unit). This is
  a deployment task, not a code task.
- **Config source of truth**: Python `shared/config` is still authoritative for
  tuning values; the Go `profile` package must be re-validated against it after
  each Python tuning change (see ledger).

### Config field-by-field diff (simulation.toml)
Source: `platform/shared/config/navigation/simulation/simulation.toml` vs
`internal/config/profile/simulation.go`. `collision_margin_m` /
`axis_align_tolerance` ARE ported (`SimulationConfig`); the rest are sim
scoring/sensor-emulation the Go side does not yet consume (per `simulation.go`
doc comment: "scoring/sensor-emulation logic not ported to Go yet").

| Field | Python type+default | Go (`SimulationConfig`) | Drift? |
|-------|---------------------|--------------------------|--------|
| `collision_margin_m` | float 0.0 | `CollisionMarginM float64` | no |
| `axis_align_tolerance` | float 1e-6 | `AxisAlignTolerance float64` | no |
| `start_collision_window_s` | float 2.0 | - absent | **unported** (start-collision grace window) |
| `start_collision_grace_s` | float 15.0 | - absent | **unported** (start-collision grace window) |
| `lidar_invalid_ray_rate` | float 0.01 | - absent | **unported** (LIDAR dropout rate; raycast noise in harness §5a) |
| `detection_confidence` | float 0.9 | - absent | **unported** (vision-emulator confidence threshold) |

**Unported sim fields (action item for §5a harness):** `start_collision_window_s`,
`start_collision_grace_s`, `lidar_invalid_ray_rate`, `detection_confidence` - plus
the "no-progress detection" scoring concept (not a TOML key; lives in
`run_scenario.py` terminal-state logic). The native runner (§5a) must source
these from `simulation.toml` via an extended `SimulationConfig`, or hardcode
parity defaults, before the Python wrapper can be dropped.

## 3. Tests vs current codebase

### Go tests
- 96 `*_test.go` files across `internal/**`. Coverage includes `navigator`
  (`Step` branches, path management - `9ffb4bb7`), `config/profile` (load +
  field tests), `sim/collision`, `sim/kinematics`, `sim/scenario` (orchestrator,
  subprocess runner).
- **Parity gate**: `a0f25133` added an MCAP bag-replay harness
  (`test/bagreplay`) that replays recorded Python/ROS2 bags through the Go
  schemas - the mechanism for diffing Go output against the Python original.
- **No benchmarks / pprof** exist yet (`*_bench_test.go` count = 0).

### Python tests
- `platform/robot` has its own pytest suite (scenario simulator, nav, hardware).
- The `subprocess_runner_integration_test.go` (Go) replays `run_scenario.py`
  behind the `integration` tag - a frozen parity oracle for the sim until the
  native runner lands.

### Comparison actions
1. **Per-package coverage map**: for each ported Go package, confirm a `*_test.go`
   exists and what it asserts (parity vs Python behaviour, not just smoke).

   **Per-package coverage map** (29 packages under internal/nav + internal/sim):

   | Package | Has test? | Asserts what? |
   |---------|-----------|---------------|
   | nav/controllers | yes | parity vs py (threat dir, blind-wedge masking, stuck history, rate-limit) |
   | nav/corridorestimator | yes | parity vs py corridor-width estimation |
   | nav/corridorfollower | yes | parity vs py blind-creep steering/clearance |
   | nav/directionestimator | yes | settle-after-min-votes, ambiguous-scan ignore, infer direction |
   | nav/localization | yes | parity vs py LIDAR pose-search fixture poses |
   | nav/navigator | yes | New gateway guard + Step-branch/read-publish + fake gateway |
   | nav/navutil | yes | angle wrap/offset + raycast nearest-ray/forward-clearance math |
   | nav/parking | yes | parity vs py block positions + black-box parking |
   | nav/racetracker | yes | parity vs py lap-detector geometry + clock-free tracker |
   | nav/signrouter | yes | parity vs py routing table, deformation, wrong-side, lane |
   | nav/startconditions | yes | spawn pose centerline + yaw alignment |
   | nav/startmeasurement | yes | ports py start-measurement cases (scan oracle) |
   | nav/trackmodel | yes | corridor geometry, path projection, wall raycast |
   | nav/wallheading | yes | parity vs py wall-heading properties |
   | nav/waypoints | yes | classification, generation feasibility, geometry, segments |
   | sim/collision | yes | geometry + track-model raycast (rect formulas) |
   | sim/corpus | yes | load dir/single-file, ignore non-matching sibling |
   | sim/kinematics | yes | 20Hz Ackermann kinematics vs py _DT |
   | sim/scenario | yes | orchestrator, result parse, subprocess-runner + integration replay |

   **Blind-package parity status (§3b gate target):** all 7 newly-ported blind
   packages - localization, wall_heading, 
ace_tracker, corridor_follower,
   directionestimator, startmeasurement, signrouter - **HAVE tests** that
   port the Python oracle cases directly. None lack coverage; the open item is
   extending 	est/bagreplay (action 2) to exercise them end-to-end.

2. **Parity gate extension**: extend `test/bagreplay` to cover the newly ported
   blind packages (localization, wall_heading, race_tracker, corridor_follower)
   - replay a blind Open bag and assert Go `NavigatorDebug` matches.
3. **Shared-fix regression**: commits with identical hashes in both trees
   (ledger) must keep both sides green; add a CI job running `go test ./...` and
   `pytest` on the same trigger.

## 4. Memory & CPU profiling

No measurements exist yet for either program. Plan to capture them:

### Targets
| Program | Original (Python) | Port (Go) |
|---------|-------------------|-----------|
| nav stack | `core_navigator` + ROS2 nodes | `cmd/navigator` (Go) |
| sim | `scripts/sim/run_scenario.py` | `cmd/sim-runner` (native runner, once ported) |
| motor/imu | `motor_node.py` / `uart_rvc_node.py` | `cmd/motor-node` / `cmd/imu-node` |

### Expected vs measured
The "expected" column is the architectural reasoning (NOT measured). Fill the
"measured" column from §4's benchmarks; do not quote numbers until then.

Status 2026-08-30 - see `profile-2026-08-30.md`:
- **Go measured**, but on a **Windows workstation, not the Pi**, and against a
  **synthetic fixed track with no obstacles/noise**, not a corpus scenario.
- **Python NOT run** - ROS2 unavailable in that environment. Left `_TBD_`
  rather than fabricated.
- The two columns are therefore **not yet comparable** (different workload,
  different hardware, no shared seed). Apples-to-apples needs the Go harness on
  the same `*_metadata.json` corpus + same seed + both on the Pi.

| Metric | Expected (why) | Measured (Python) | Measured (Go) |
|--------|----------------|-------------------|---------------|
| **Peak RSS** | Go far lower: one static binary, goroutines share one process; Python pays a CPython interpreter (~30–50 MB) **per ROS2 node** + FastDDS `/dev/shm` + conda libs (OpenCV/numpy/pydantic) across `state_machine`/`vision`/`imu`/`lidar`/`bridge`. Go still needs `nats-server` (~10 MB) + same HW drivers. Expect 1–2 orders of magnitude lower baseline. | _TBD (see profile-2026-08-30.md; Python not run here)_ | Go heap `inuse_space` **<2 MB** steady-state (3.2 MB total incl. pprof's own buffer); process peak working set 50.83 MB on Windows - **not** Linux-RSS comparable, needs `/usr/bin/time -v` on the Pi. `alloc_space` 602 MB/200k steps, 99.4% from `TrackWalls.Raycast` (3072 B/step scan slice - churn, not a leak). |
| **Mean CPU %** | Go likely lower: no interpreter dispatch, no GIL, compiled hot loop (pursuit/sectors/raycast/sign-deform at `CONTROL_HZ`); no DDS serialization/discovery tax. Narrows if Python hot loop is already numpy/C-bound. | _TBD (see profile-2026-08-30.md; Python not run here)_ | Not measured as a %; CPU profile attribution instead (200k steps, 5550 ms samples): `TrackWalls.Raycast` 41.98% flat / **85.23% cum**, `math.cos` 26.31%, `math.sin` 8.47%. Raycast trig is the hotspot. |
| **Per-step time (p50/p99)** | Go likely faster: compiled, inlined, no boxing. Measure ns/op on `Navigator.Step` + raycast via `*_bench_test.go`. | _TBD (see profile-2026-08-30.md; Python not run here)_ | `BenchmarkScenarioStep` 21344 ns/op @50x → **25296–31891 ns/op** @2000x (3072 B/op, 1 alloc). `BenchmarkNavigatorStep` 59562 ns/op @50x is **warmup-dominated** → **3675–7947 ns/op** @2000x (1129 B/op, 7 allocs). Harness: 500 steps 15.75 ms (31504.8 ns/op); 200k steps 5.176 s (25879.3 ns/op, 38641 steps/s). **p50/p99 not derivable** - no per-step timestamps emitted. |
| **Cmd latency (nav→motor)** | Go likely lower - **driven mostly by transport swap, not language**: ROS2/FastDDS has documented discovery stalls + SHM lock-file wedging (`fastdds_udp_only.xml` was added to kill it; `vtitan-pi5.service` notes 90–140 s discovery stalls). NATS has static subjects, immediate connect, cheaper protobuf vs CDR. Go GC pauses are sub-ms at 20 Hz - negligible vs DDS jitter. | _TBD (see profile-2026-08-30.md; Python not run here)_ | _Not measured_ - needs both stacks live on the Pi with NATS/DDS running; out of scope for an offline workstation bench. |
| **Cross-board latency (Pi5↔Zero)** | Language-independent: USB gadget link is HW/serial, unchanged by stack swap. Not expected to move. | _TBD (see profile-2026-08-30.md; Python not run here)_ | _Not measured_ - requires both boards + USB gadget link; unchanged by stack swap. |

Key caveat: if the Python hot loop is already numpy/C-extension bound, the
*compute* delta may be small and only the *transport* delta large. Measurement
decides - see Method.

### Method
- **Go**: build with `-gcflags="-m"` for escape analysis; run under `go test
  -bench` or `runtime/pprof` (`pprof.StartCPUProfile` / `StartMemProfile`) during
  a fixed scenario (e.g. one Obstacles corpus run). Use `go tool pprof -top`.
  Add a `*_bench_test.go` in `internal/sim/scenario` and `internal/nav/navigator`
  that runs N steps against a fixed track, so numbers are reproducible.
- **Python**: `python -m cProfile -o sim.prof` on `run_scenario.py` for CPU;
  `tracemalloc` for peak RSS; `RSS` via `/usr/bin/time -v` or `psutil` for the
  ROS2 nodes.
- **Compare**: same scenario corpus, same seed, same config profile. Report
  peak RSS (MB), mean CPU %, p50/p99 step time (ms), and total wall time per
  scenario. The Go sim is expected to be lower-RSS and faster (no ROS2/DDS, no
  interpreter); the gate is to *prove* it, not assume it.
- **Baseline artifact**: `profile-2026-08-30.md` in this directory - holds the Go measurements
  above, the ready-to-run Python capture snippet, and the corpus/seed caveats.
  Later runs accumulate as further `profile-<date>.md` files.

### Profiling gaps to add
- No `*_bench_test.go` in Go → add them (navigator Step, sim step, raycast).
- No `pprof` wiring in `cmd/*` → add a `--cpuprofile` / `--memprofile` flag to
  `sim-runner` and `navigator` node.
- Python side has no saved profiles → capture once as the comparison baseline.

## 5. Provisioning & dual-stack swap on the Pi

### Current Pi deployment (Python)
Two boards, two systemd units, both pixi-managed ROS2:
- **Pi 5** - `vtitan-pi5.service` runs `pixi run -e vision launch-rpi5`
  (state_machine, vision, IMU, LiDAR, backend bridge). `Wants=hailort`,
  `vtitan-lidar`; `ExecStartPre` waits for the USB gadget link + restarts LiDAR.
- **Pi Zero** - `vtitan-pi-zero.service` runs `pixi run --as-is -e dev
  launch-rpi-zero` (motors, button, OLED). `ExecStopPost` re-drives BTS7960
  enable pins low (motor-safety on stop).
- Provisioning scripts: `scripts/provisioning/` (`deploy-to-pi5.sh`,
  `deploy-dev-env-to-zero.sh`, `bootstrap-fresh-zero.sh`, `wait-for-gadget-link.sh`,
  audits). Configs are TOML under `shared/config/**`, loaded via `pydantic-settings`.

### Go deployment shape (chosen: static binary + systemd)
**Decision: static native binary + systemd, no pixi/conda/ROS2 on the Go side.**
The Go tree has **no provisioner today** - `go.mod` only, native binaries per
`cmd/*`, comms over **NATS + protobuf** (not ROS2/DDS). To run on-device:
1. **Cross-compile + install** - `GOOS=linux GOARCH=arm64 go build ./cmd/...`
   into a staging dir (e.g. `/opt/vtitan-go/bin`), copied to the Pi by a deploy
   script mirroring `deploy-to-pi5.sh` / `deploy-dev-env-to-zero.sh` but with no
   conda env swap. A static binary (`CGO_ENABLED=0` where possible - note
   `periph.io`/`go-gpiocdev` are pure-Go, so this is achievable) drops glibc
   coupling entirely. No pixi env, no DDS, no RoboStack.
2. **systemd units per Go node** - mirror the Python units but launch the Go
   binary instead of `pixi run launch-...`. The Go `foxglove-bridge`
   (`cmd/foxglove-bridge`, currently stubbed) is the NATS→Foxglove replacement
   for the Python backend bridge; the per-role nodes (`motor-node`, `imu-node`,
   `navigator`) map 1:1 onto the Python `launch-rpi5`/`launch-rpi-zero` roles.
3. **Config parity** - Go reads `internal/config/profile` TOMLs; ship the same
   profile values as the Python `shared/config` (kept in sync via the ledger).
   Profiles live on disk; the deploy script copies them alongside the binary.

### Easy swap: run either stack
Both trees expose the **same logical nodes**, just on different transports
(ROS2/DDS vs NATS). The clean swap is a **systemd template unit keyed by a
`STACK` variable** so one `systemctl` toggle flips the whole robot between
Python and Go without touching unit files:

```
# /etc/systemd/system/vtitan-robot@.service  (template; %i = stack)
[Unit]
Description=vTitan robot stack (%i)
Wants=vtitan-lidar.service
After=vtitan-lidar.service

[Service]
Type=exec
User=__TARGET_USER__
Environment=STACK=%i
EnvironmentFile=-__TARGET_HOME__/vtitan/platform/robot/.env
# Python branch
ExecStart=/bin/bash -lc '\
  if [ "%i" = python ]; then \
    __TARGET_HOME__/.pixi/bin/pixi run -e vision launch-rpi5; \
  else \
    __TARGET_HOME__/vtitan-go/bin/robotd --stack %i; \
  fi'
# Go branch assumes a static binary at ~/vtitan-go/bin (no pixi env).
Restart=on-failure
```

Concrete toggle:
```pwsh
# run the Python stack
sudo systemctl start vtitan-robot@python
# run the Go stack
sudo systemctl start vtitan-robot@go
```
Per-node granularity is also possible (run motor-node in Go, navigator in
Python) because each node is independently launched and only needs its transport
peers present - but cross-transport interop requires a **NATS↔ROS2 bridge** (the
`foxglove-bridge` is NATS→Foxglove, not ROS2; a ROS2↔NATS shim would be needed
for mixed-mode). Recommendation: swap **whole-stack** first (simplest, no bridge),
mixed-mode later once a ROS2↔NATS adapter exists.

### Swap prerequisites / risks
- **Motor-safety `ExecStopPost`** (BTS7960 pins low) must be replicated in the
  Go motor unit - a Go crash must not leave the H-bridge floating (the
  `pinctrl` call in `internal/driver/motor/driver.go:211` covers init only, not
  stop; add the stop-hook to the unit, or have the Go motor-node drive pins low
  on SIGINT/SIGTERM).
- **USB gadget link + LiDAR pre-start** are transport-agnostic (hardware), so
  they stay in the template regardless of `STACK`.
- **Config drift** between Python `shared/config` and Go `profile` is the top
  parity risk during a swap - validate both from the ledger before flipping.

## Open questions
- Is `shared/config` (Python) still the tuning source of truth, or should Go
  `profile` become authoritative once the native sim lands? A drift here breaks
  parity silently.
- Go on-device: **static binary + systemd (no pixi) - chosen.** Cross-compile
  `linux/arm64`, `CGO_ENABLED=0` where possible; deploy script mirrors
  `deploy-to-pi5.sh` without the conda env swap.
- Should the parity gate run in CI on every shared-hash commit?
- Mixed-mode (per-node Python/Go) needs a ROS2↔NATS shim - is whole-stack swap
  enough for now?

## Change ledger (rolling - append, don't rewrite)
- 2026-08-30: created plan. Go blind stack already ported (ed088467 … 7e5c761d).
  Sim harness (`SimHardwareGateway`) + `SignRouter.discover` still unported.
  No Go provisioner, no Go benchmarks/pprof. Python uses pixi/RoboStack/ROS2.
  Added §5: provisioning + systemd template `STACK` swap (python|go), whole-stack
  toggle; mixed-mode deferred pending a ROS2↔NATS shim. Motor-safety ExecStopPost
  must be replicated in the Go motor unit. Go deployment = static binary + systemd
  (CGO_ENABLED=0, no pixi/ROS2) - chosen over a go-only pixi env.

## Implementation: parallel / sequential agent plan

Each prompt is an agent task. Files written dictate what can run concurrently.

### Conflict rules (why the waves are shaped this way)
- `cmd/sim-runner/main.go` would be edited by both §4 (pprof flags) and §5a
  (`--runner` flag) → **§4 must write a standalone `cmd/bench-harness` instead
  of touching `sim-runner`**, so §4 and §5a never collide.
- `test/bagreplay` parity extension (§3b) compares the native runner → waits
  for §5a.
- §5c flip/validation diffs configs → waits for §2.
- Everything else writes disjoint packages → parallel.

### Wave 0 - parallel (research, no shared files)
| Agent | Prompt | Touches |
|-------|--------|---------|
| A | §1 ledger: git log since 08-28 both trees, append rows to ledger, mark Go-counterpart status, update `Last updated`. | plan doc only |
| B | §2 config map: field-by-field diff `internal/config/profile/*` vs `shared/config/**` + `src/config/**`; list unported sim fields. | plan doc only |
| C | §3a coverage map: which ported packages have `*_test.go` + one-line assertion summary. | plan doc only |
| D | §5c files: write `scripts/provisioning/build-go.sh` (CGO_ENABLED=0 GOOS=linux GOARCH=arm64) + `systemd/vtitan-go-pi5.service`, `vtitan-go-pi-zero.service`, `vtitan-robot@.service` template; replicate BTS7960 ExecStopPost. | new shell + systemd files |

### Wave 1 - parallel (disjoint Go code; §4 uses standalone cmd)
| Agent | Prompt | Touches |
|-------|--------|---------|
| E | §4 bench code: `internal/sim/scenario/bench_test.go`, `internal/nav/navigator/bench_test.go` (ns/op Step + raycast); **new `cmd/bench-harness`** with `--cpuprofile`/`--memprofile` running a fixed Obstacles scenario. No edit to `sim-runner`. | `internal/sim`, `internal/nav/navigator`, new `cmd/bench-harness` |
| F | §5a harness: `internal/sim/harness` (SimHardwareGateway impl of controllers.HardwareGateway: pose/scan/drive + kinematics advance + raycast noise), `internal/sim/scenario/native_runner.go` (Runner impl → scenario.Result), `--runner native|python` flag in `cmd/sim-runner/main.go`. | `internal/sim/harness` (new), `internal/sim/scenario/native_runner.go` (new), `cmd/sim-runner/main.go` |
| G | §5b blind: `internal/nav/signrouter/discovery.go` (ObservedSignMap + pinhole `_detection_to_world` + `detection_to_observation`), navigator `BLIND_CREEP` phase, `directionestimator` infer, `startmeasurement` believed-offset, `SignRouter.discover`. | `internal/nav/signrouter`, `navigator`, `directionestimator`, `startmeasurement` |

### Wave 2 - sequential (depend on Wave 0/1)
| Agent | Prompt | Blocked by |
|-------|--------|-----------|
| H | §3b parity extension: extend `test/bagreplay` to replay a blind Open bag, assert Go `NavigatorDebug` matches; also diff native-runner `Result` vs python. | F (§5a) |
| I | §4 fill: run E's benchmarks + python cProfile/tracemalloc on same corpus/seed; fill §4 "Measured" columns in `profile-<date>.md`. | E (§4 bench code) |
| J | §5c flip: diff §2 config map (B), deploy binaries to Pi, validate before `systemctl start vtitan-robot@go`. | B (§2) + E/F/G compile |

### Agent briefs (acceptance criteria)

**A - §1 ledger**
- Inputs: `git -C platform/robot log --since=2026-08-28 --oneline -- platform/robot`;
  `git -C platform/robot-go log --since=2026-08-28 --oneline`.
- Output: append rows to "Change ledger" (Date/Tree/Commit/Summary/Go-counterpart?);
  identical hashes → "shared fix (parity gate)"; update `Last updated`.
- Done when: ledger reflects both trees; no source changed.

**B - §2 config map**
- Inputs: `internal/config/profile/*.go`; `shared/config/**`, `src/config/**` (Python).
- Output: §2 field-by-field table (field / py type+default / go type+default / drift);
  enumerate unported sim fields from `simulation.go` doc comment.
- Done when: every profile file mapped; drift list explicit.

**C - §3a coverage map**
- Inputs: `internal/nav/**`, `internal/sim/**` `*_test.go`.
- Output: §3 per-package "has test? / asserts what?" summary.
- Done when: all ported packages listed; no code changed.

**D - §5c files**
- Inputs: `systemd/vtitan-pi5.service`, `vtitan-pi-zero.service`; `scripts/provisioning/deploy-to-pi5.sh`.
- Output: `scripts/provisioning/build-go.sh` (CGO_ENABLED=0, GOOS=linux,
  GOARCH=arm64, copy bins + `configs/profiles/`); `systemd/vtitan-go-pi5.service`,
  `vtitan-go-pi-zero.service`; `vtitan-robot@.service` template (STACK=python|go);
  BTS7960 ExecStopPost replicated (or motor-node drives pins low on SIGINT/SIGTERM).
- Done when: units + script present; `go build ./...` unaffected; no Go source edited.

**E - §4 bench code**
- Inputs: `internal/sim/collision` (RaycastScan), `internal/nav/navigator` (Step),
  `internal/sim/kinematics`.
- Output: `internal/sim/scenario/bench_test.go` (N-step scenario, ns/op);
  `internal/nav/navigator/bench_test.go` (Step + raycast ns/op); **new**
  `cmd/bench-harness` with `--cpuprofile`/`--memprofile` running fixed Obstacles
  scenario → writes pprof files. NO edit to `cmd/sim-runner`.
- Done when: `go test -bench ./...` passes; `bench-harness` builds.

**F - §5a harness**
- Inputs: `internal/nav/controllers/ports.go` (HardwareGateway iface),
  `internal/sim/collision/track_model.go`, `internal/sim/kinematics`,
  `internal/sim/scenario/runner.go` (Runner iface), `result.go`;
  `platform/robot/src/simulation/simulated_hardware_gateway.py` (behavior oracle).
- Output: `internal/sim/harness/gateway.go` (SimHardwareGateway: GetCurrentPose
  ground-truth, GetLidarScan raycast+noise+dropout, PublishDrive kinematics advance,
  optional wall_heading localization); `internal/sim/scenario/native_runner.go`
  (Runner → builds harness+navigator, loops Step, scores Result); `--runner
  native|python` flag in `cmd/sim-runner/main.go` (default python until parity).
- Done when: `go test ./...` green; native runner produces `Result` matching
  python on a smoke corpus (parity gate).

**G - §5b blind**
- Inputs: `platform/robot/src/navigation/planning/sign_discovery.py`,
  `corridor_follower.py`, `direction_estimator.py`, `start_measurement.py`;
  `navigator/doc.go`, `signrouter/doc.go` (scope notes).
- Output: `internal/nav/signrouter/discovery.go` (ObservedSignMap, pinhole
  `_detection_to_world`, `detection_to_observation`); navigator `BLIND_CREEP`
  phase; `directionestimator` infer/`_resolve_direction`; `startmeasurement`
  believed-offset (belief→map yaw); `SignRouter.discover` + `is_discovering`.
- Done when: `go test ./...` green; blind navigator compiles; sighted tests still pass.

**H - §3b parity extension**
- Inputs: F's `native_runner.go`; `test/bagreplay`; a blind Open bag.
- Output: bagreplay asserts Go `NavigatorDebug` matches recorded bag for
  localization/wall_heading/race_tracker/corridor_follower; diff native `Result`
  vs python `Result` over corpus.
- Done when: parity gate green for blind packages + native runner.

**I - §4 fill**
- Inputs: E's bench code; python `cProfile`/`tracemalloc` on `run_scenario.py`;
  `/usr/bin/time -v` on ROS2 nodes.
- Output: fill §4 "Measured (Python)" + "Measured (Go)" columns (peak RSS, mean
  CPU %, p50/p99 step ms, wall time) in `profile-<date>.md` under `src/go/docs/`.
- Done when: table filled from real runs, same corpus+seed+profile.

**J - §5c flip**
- Inputs: B's config map; D's units/script; E/F/G compiled binaries.
- Output: diff `shared/config` vs Go `profile` (rule out drift); deploy to Pi;
  validate; document `systemctl start vtitan-robot@go` toggle.
- Done when: Go stack boots on Pi; motor-safety stop verified; swap toggles clean.

### Execution order
```
Wave 0:  A ║ B ║ C ║ D          (all parallel)
Wave 1:  E ║ F ║ G              (all parallel; E uses standalone cmd)
Wave 2:  H(after F)  I(after E)  J(after B + E/F/G compile)
```
Hard rules: §4 → standalone `cmd/bench-harness` (never edit `sim-runner`);
§3b waits for §5a; §5c flip waits for §2 + compile.
