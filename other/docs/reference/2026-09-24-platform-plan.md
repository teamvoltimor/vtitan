# vTitan platform plan

Date: 2026-09-24. Status: accepted 2026-09-24; phase 0 done except 0.5 (deferred); open decisions in section 10. Updated 2026-09-25 with the reference study (section 15) and the context-aware board proposal (section 16).

Context: the team is not competing again in WRO 2026 (did not win the national).
The goal is to turn vTitan into a platform ready for whatever the next challenge
is: reusable, measurable, and testable without the car. This plan consolidates:

- the reuse analysis (`2026-09-24-reutilizacion-si-cambia-el-reto.md`, same
  folder): what is reusable, what to generalize, scenarios A-H, 3D maps;
- the Pico / board work already shipped (`948f49dc`, `55d854f8`, `233f94ae`,
  `f6e67f89`);
- two research passes on how other projects do SITL/HIL and latency / fault
  testing (sources in section 12);
- a reference study of MuJoCo, robosuite, robomimic and the JCIIOT 2026
  platform that uses all three (2026-09-25, section 15; full analysis in
  `2026-09-25-mujoco-robosuite-robomimic-comparison.md`, same folder).

## 1. Principles adopted from the research

1. **The simulator owns time; the stack cannot tell it is in simulation.** Only
   the thin sensor/actuator bridge changes (PX4, ArduPilot, ROS 2 sim time).
2. **Measure before you simulate.** Every emulated latency, loss or noise value
   comes from a recording or a bench measurement, and says so when it is an
   estimate. The history here proves it: making the sim realistic moved the
   corpus 16 -> 12, in the unexpected direction.
3. **Worst case, not average.** Report p95/p99 and data age, not means; tail
   latency is what destabilizes control (F1TENTH sim-to-real).
4. **Deterministic by default, timing faults injected explicitly.** CI runs in
   lockstep with seeded faults; real-time behaviour is tested on the bench.
5. **Realistic fault shapes.** USB retries corrupted packets in hardware, so real
   link faults are bursts, stalls, truncation and disconnects, not independent
   bit flips.
6. **Gate on separate predicates, not one score.** A composite score gets gamed
   (CARLA changed its penalty after teams learned to stop early). Keep laps,
   wrong-side pass, contact and time as separate pass/fail conditions, over
   several seeds (single-scenario verdicts here are knife-edge).
7. **Separate the challenge from the platform, and the board role from the
   chip.** The board contract is `boardloop` + `boardlink`, not "the Pico".
8. **The world owns the score, and physics violations are invariants.**
   Scoring reads simulator ground truth, never an artefact written by the
   stack under test. A body that jumps further than its limits allow, or
   sinks into a surface, fails the run automatically. Exemptions are
   declared with a reason, never silent ignore lists. (JCIIOT 2026 broke
   all three rules; 15 teams tied at 100/100 and humans had to catch
   teleports and wall pass-through. Section 15.)

## 2. Where we are

| Area | State |
|---|---|
| Virtual actuation board | `pkg/boardsim`: real `boardloop` on in-memory hardware, link latency/jitter/bit flips from `board_sim.toml`, end-to-end tests against `picolink`. On `origin/master` (`233f94ae`, `f6e67f89`) |
| In-process world sim | `internal/sim/harness` + `kinematics` + `collision` + `sensorerrors`: drivetrain lag and servo slew; transport delays since 2.3/2.7. **No contact response**: `collision.AllowedStep` is ported but never called, so the chassis passes through walls and pillars and contact is only scored (`harness/gateway.go:233`); the Python oracle slides. **No snapshot/restore** of state mid-run (neither sim) |
| Go vs Python sim realism | Go lacks what Python has: LIDAR chassis occlusion and self-returns, vision colour flips / bearing scatter / range falloff, the IMU error budget on by default, tick jitter, the reverse-run rule 9.21, solid walls. Go-vs-Python parity (ADR 0068) is partly parity with an easier world |
| Recording | MCAP of real runs has `/ackermann_cmd`, `/joint_states`, `/imu/data`, `/vision/detections`, `/nav_debug`, ... but **no `motor_status`** (so no `command_age_ms`) and no CPU/thermal health |
| Known fidelity gaps | vision latency 0 in sim vs 0.85 s measured; LIDAR return loss 1% vs 25%; sim under-rotates 40-50% in escapes; sim never saturates the steering |
| Timing observability | none end to end: no capture time / causal chain across messages |
| Config hygiene | Fixed in `7f6bb744`: the 5 stale navigation DTOs regenerated, go-jsonschema pinned, CI fails on stale DTOs |
| Challenge coupling | WRO spread across `nav`, `sim`, `statemachine`; `[4]Section` hardcoded; 4-fold symmetric wall model |

## 3. Phase 0: housekeeping and decisions (days)

| # | Item | Done when |
|---|---|---|
| 0.1 | Push `233f94ae` and `f6e67f89` | DONE 2026-09-24 |
| 0.2 | Regenerate the 5 stale DTOs in their own commit; add a CI step that runs `configgen generate-go` and fails on `git diff` | DONE `7f6bb744` |
| 0.3 | Decide ADR 0068's parity target (recommendation: functional parity, Go corpus >= Python, then cut over and freeze Python) | DONE `7f21e65f` |
| 0.4 | Move the reuse doc and this plan to a tracked location (`other/docs/reference/`) | DONE |
| 0.5 | Record `motor_status` and a new system-health topic (CPU temperature, frequency, `vcgencmd get_throttled` live and sticky bits, load) in the MCAP | DEFERRED: build first, bags later. Needs a Go recorder node (subscribes to configured NATS subjects, state-gated) and a `SystemHealth` publisher; the Python recorder is left alone |

Build order (decided 2026-09-24: build first, calibrate from bags later):
2.1 + 2.2, then 2.3, then the 4.11 spike, then phase 3 and the rest of phase 4.
Phase 1 and item 0.5 follow once there is something to calibrate.

## 4. Phase 1: measure time (observability)

Why it matters: every later emulation needs real numbers, and today nobody can say how
old the data behind a steering command is.

| # | Item | Done when |
|---|---|---|
| 1.1 | Message timing envelope on every NATS message: **capture time** (sensor/driver; board time mapped to host), **publish time**, **sequence number** per publisher, and for derived messages the capture times of their inputs | all node outputs carry it; recorded in MCAP |
| 1.2 | Board-side stamps: board time when a Command is **applied** (in Status), RX stamped in the receive path, TX stamped just before send | visible on host |
| 1.3 | Offline Go tool over MCAP: per-stage p50/p95/p99, **data age** at the actuator command (`command.publish - min(input capture)`), drops/duplicates/reorders from sequence gaps. Never use `log_time` as event time | report on the September bags (where fields exist) and on new bags |
| 1.4 | Declared **latency budget** per chain (camera -> Hailo -> decoder -> planner -> link -> PWM; LIDAR -> localizer -> planner -> link) with alert thresholds | ADR with the budget; tool flags violations |
| 1.5 | Clock sync hardening in `picolink`: min-RTT window filter, drift (skew) by linear regression, stamps as close to the wire as possible; document that offset error is bounded by half the path asymmetry | tests in `boardsim` with **asymmetric** delay assert the bound |
| 1.6 | **Run tags** (robomimic dataset filter keys): an index over `other/data/live/runs` with challenge, direction, deploy commit, hardware profile and outcome, written once by the classification scripts; diagnostics accept `--tag` instead of path lists | the fidelity scripts select their run sets by tag |

## 5. Phase 2: fault models (cheap, high value)

| # | Item | Done when |
|---|---|---|
| 2.1 | `boardsim` realistic link faults, config driven and seeded: **Gilbert-Elliott** chunk/frame loss (rate + mean burst length), **stall windows**, **disconnect/reconnect** (EIO, optional new device name), **reader starvation** (buffer overflow drops oldest), asymmetric delay. Keep bit flips for decoder tests | PARTLY DONE: bursty loss, random and scripted stalls, board reboot, per-direction stats, all in `board_sim.toml` (shipped OFF until measured). Disconnect with a renamed device and reader starvation move to phase 4, where the pty exists |
| 2.2 | Scripted failsafe tests asserting "drive at zero within X ms": link stall of 499 ms and 501 ms around the 500 ms board watchdog, host link timeout around 1 s, reconnect mid-command | DONE: short stall keeps driving, long stall stops at the 500 ms timeout (measured 499 ms) and recovers, board silence reported as link lost, board reset reconfigured, bursty loss converges. Exact 499/501 ms edges need the sim clock (3.1) |
| 2.3 | **Quick win before the virtual robot:** configurable command delay/drop and sensor staleness inside the in-process world sim (`harness`), then a corpus **delay sweep** | DONE: `harness.TransportConfig` (command delay, drop, emulated board watchdog, scan delay; zero is the old behaviour, verified identical), sim-runner flags, `scripts/transport-sweep.sh`. Results in section 13 |
| 2.4 | NATS fault shim: per subject drop / delay / freeze (repeat last) / reorder / noise, from TOML, seed logged into the MCAP | DONE: `nats.Faults` on `nats.Config`, applied on the receiving side of every `Subscriber` (bursty loss, delay + order-keeping jitter, Poisson freezes that repeat the last message, reorder, Gaussian noise on float fields), seeded per subject. Config `src/config/hardware/nats_faults.toml` (ships with no subjects; enable per experiment from a profile), loader `hwconfig.NATSFaults`. Wired into `cmd/pi5` (every subsystem) and nav, which writes the plan as the `nats_faults` MCAP metadata record. Not wired yet: the standalone `cmd/*-node` binaries and `cmd/pi-zero` (do it with the virtual robot, phase 4) |
| 2.5 | Host-side robustness from the research: udev symlink by serial number for the board; close the fd on error before reopening | reconnect test passes with a renamed device |
| 2.6 | **Stale command burst (found by 2.2):** `boardlink.Command` carries no send time, so after a stall the board applies the whole backlog as if fresh (24 queued commands after a 1.2 s stall). Add a host send time to Command and a board-side maximum command age, or have the host drop queued commands on a stall | MOSTLY DONE: the board applies only the newest command of each Step (1 applied, 23 superseded after a 1.2 s stall, was 24 applied). Residual: a host that stopped sending during the stall leaves a stale newest command, applied once until the watchdog stops the car again; closing it needs a send time on Command plus a board-side clock reference |
| 2.7 | **Vision latency in the sim (found by 2.3):** the measured 0.85 s from camera to detection is the largest latency in the robot | DONE: `DetectionDelayS` (seen from the old pose, placed through the current one) and `DetectionDropRate`, sim-runner flags; results in section 13. Also fixed a one-tick boundary error in all latency emulation |
| 2.8 | **Decision needed (found by 2.3):** the corpus baseline assumes a command acts in the tick it is computed. Once phase 1 measures the real command delay, decide whether to re-baseline the corpus at it (every existing number moves) | decision recorded in ADR 0087 |
| 2.9 | **Blind laps (found by 2.7):** in `--blind` Obstacles completes 0 laps in 256/256 scenarios while driving and discovering signs | DIAGNOSED, NOT FIXED (section 14). Two confirmed defects in how the sim meets the blind stack; a naive fix made blind worse and was reverted. Needs a frame-contract decision (2.10) |
| 2.10 | **Frame contract for blind rounds:** decide which frame every blind consumer works in (pose, lap detector, path, sign discovery, parking, pass-side scorer), how the real `natsgw` + localizer produce it, and make the sim present exactly that. Compare against the Python oracle, which reportedly rotates the believed pose into the canonical frame | ADR; blind Open and Obstacles count laps without regressing collisions or wrong-side passes |
| 2.11 | **Solid walls in the Go sim (found by the section 15 study):** call `collision.AllowedStep` from `SimHardwareGateway.Advance` behind `contact_slides_along_surfaces` (same key as Python); add per-tick **physics invariants** (teleport: displacement and rotation within the kinematic limits for `dt`; penetration: depth above a tolerance outside a declared exemption) that fail the run as "invalid sim". Run before re-running any sweep, since it moves every Go baseline | Go and Python agree on blocked steps on the fixtures; invariants pass with the key on and fail with it off; corpus delta recorded; reuse doc section 1.4 corrected (it says Go slides) |
| 2.12 | **Port Python's measured sensor models to the Go harness:** LIDAR chassis occlusion and self-returns (model the chassis in the raycast, exclude it, add the measured returns explicitly), vision colour flips / bearing scatter / range falloff / confidence quantiles, IMU budget on by default, tick jitter, reverse-run rule 9.21 | ADR 0068 parity runs on equally realistic worlds |
| 2.13 | **Command lease** (section 16): each Command carries its host send time (synced clock, 1.5), a validity window chosen by the host from its situation (short near a wall or pillar, longer on a clear straight) and an expiry action (hold, ramp to stop, straighten and stop). The board executes a command only while it is valid and runs the expiry action after. Leases can only **tighten** the board's hard limits, never relax them. Closes the 2.6 residual | `boardsim` tests: a stall near an obstacle stops within the short lease, the same stall on a straight coasts to a controlled stop; a lease longer than the board maximum is clamped; stale commands after a stall are never applied |
| 2.14 | **Link health report** board → host: command age at apply (p50/p99), gaps, lease expiries, in Status. The host adapts (speed cap and larger margins when the link is degraded) | visible in MCAP; a netem/`boardsim` degradation lowers the host speed cap |
| 2.15 | **Setpoint horizon** (after 2.13, only if 1.3 shows link jitter matters): the host sends the next ~200-500 ms of timed speed/steering setpoints instead of one; the board plays the one due at its clock, so jitter does not reach the actuators and a stall follows the plan and then the expiry action | jitter sweep in `boardsim`: actuator timing error bounded by the board loop period, not by link jitter |
| 2.16 | **Board reflexes** (optional, hardware-dependent): local detection the board can do faster than the host round trip, such as encoder speed far below command (blocked wheel) or motor current if the hardware can sense it, with a bounded local reaction (limit or stop) reported as an event to the host | bench test on the car; the reflex never fires in clean corpus-equivalent runs |

## 6. Phase 3: platform seams

| # | Item | Done when |
|---|---|---|
| 3.1 | Injectable **Clock** across all Go nodes (real, and sim driven by a `sim.clock` subject); refuse to run on an uninitialized clock | nodes run on sim time in a test |
| 3.2 | ADR "the challenge is a plugin" + inventory; move WRO 2026 to `internal/challenge/wro2026/` | corpus **identical** before and after |
| 3.3 | Board role vs chip: `board.toml` `kind` names the protocol (`boardlink` or NATS/Zero), the chip is a separate field; `picolink` becomes a generic `boardlink` host session; `firmware/pico2` is one adapter among possible others | ADR 0098 amended; `cmd/pi5` and profiles updated |
| 3.4 | Track described in data; remove `[4]Section` | a non-square test track generates and runs |
| 3.5 | **World/state split with snapshot and restore** (MuJoCo `mjModel`/`mjData`, `mjSTATE_INTEGRATION`): immutable `World` (track, obstacles, robot geometry, config) vs `WorldState` (kinematics, clock, every RNG stream position, delay queues, contact / pass-side / lap bookkeeping, navigator state via a `Snapshot` interface). Written as `/sim/state` in the sim MCAP with scenario and resolved config as attachments. Fallback if the navigator cannot snapshot: resume by deterministic replay to tick N | `sim-runner --resume <bag> --at-tick N` continues bit-identically; a knob can be changed at resume to branch knife-edge verdicts |
| 3.6 | **Determinism check** (robomimic `playback_dataset.py`): replay a sim bag by re-applying `/ackermann_cmd` from the recorded initial state, compare every tick's pose, report the first divergence | in CI on one Open and one Obstacles fixture |

## 7. Phase 4: virtual robot (SITL)

| # | Item | Done when |
|---|---|---|
| 4.1 | World process: publishes LIDAR, IMU, (vision detections) on the **same NATS subjects** as the real nodes, owns `sim.clock`, consumes actuation | real `cmd/pi5` runs against it unchanged except driver selection |
| 4.2 | `sim` driver kind in the hardware profiles (as the synthetic camera already does) | selected by a profile, no code branches in nodes |
| 4.3 | Actuation through `boardsim` on a **pty** that `picolink` opens like the USB port; optional virtual Zero (motor node with fake drivers) only if the Zero stays a supported build | full loop: sensor -> nav -> link -> board -> kinematics |
| 4.4 | **Lockstep mode** for CI (publish sensors, wait for the tick's command or count a miss, advance; N x real time) and **real-time mode** for timing work | the corpus runs through the real binaries |
| 4.5 | **Timing and noise profiles** (TOML, seeded) applied at the bridge: measured latency distributions **with tail spikes**, vision latency 0.85 s, LIDAR loss 25%, staleness; metrics reported per profile | nominal and degraded profiles both in CI |
| 4.6 | Scenario files with explicit pass/fail predicates (laps, wrong-side pass = round ends, contact, time limit) and several seeds per scenario | gating uses predicates, not only an aggregate |
| 4.7 | MCAP of every simulated run, analyzable with the same tools as real bags; live Foxglove | a sim bag passes through the phase 1 tool |
| 4.8 | **Visualization in Foxglove Studio**: the sim (in-process harness and virtual robot) publishes Foxglove `SceneUpdate` entities (walls, pillars, parking lot, robot box, planned path, pursuit target, detections) plus scan and pose; `.proto` schemas generated into Go with `buf` | a simulated run is watchable live and on MCAP replay, same layout as a real bag |
| 4.9 | **Live parameter tuning** through Studio's Parameters panel: add the protocol's parameter capability to `pkg/foxglove`; the bridge maps get/set to NATS (`vtitan.param.*`); nodes hold a parameter store over the TOML config and read a snapshot at the **start of a tick** (deterministic in lockstep); a key is tunable only if its schema says `x-tunable`, bounded by the schema's `minimum`/`maximum`; every change (key, old, new, tick) is recorded in the MCAP; "save as overlay" writes a TOML profile overlay, nothing is written back automatically | a value changed in Studio shows its effect in the running sim, is in the bag, and can be saved as an overlay |
| 4.10 | Hardware rules for 4.9: failsafe keys (command timeout, watchdogs, link timeout) never tunable live; on the real car changes apply only while stopped or to a short allow-list | enforced in the store, tested |
| 4.11 | **Spike (right after phase 0, before the rest of phase 4):** in-process sim + Foxglove scene + one tunable knob (e.g. lookahead) end to end | a short demo: tweak in Studio, see the car's behaviour change |
| 4.12 | **Sensor profile descriptors** for 4.5, one per sensor (LIDAR, IMU, camera detections, command link): source rate, corrupter (noise, dropout, bursts), filter, **FIFO** delay with distribution and tail, seed salt (shape from robosuite `Observable`; semantics from MuJoCo 3.5 delays, which are true queues and part of the saved state; **not** robosuite's delay, a phase offset capped under one sampling period, nor its global RNG). TOML under `src/config/navigation/simulation/profiles/`, schema-validated; CLI flags stay as overrides; add `--seed` to native runs | every Go transport and sensor-error knob reproducible from a config file |
| 4.13 | **Sweeps as generated overlays** (robomimic config locking and `hyperparam_helper.py`): one overlay TOML per arm in the run directory, run against `--config-root` plus overlay, unknown keys rejected, resolved config hashed into the `sweep_results.py` row. Replaces the Python sweeps that sed-edit the shipped TOML in place | shipped tree untouched during a sweep (checked with `git status` in the script) |
| 4.14 | **Scoring ownership in SITL:** when the stack runs out of process (4.1), the world process owns scoring and invariants (principle 8); the stack cannot write to them. Also report continuous margins per run (minimum clearance, contact time, time to finish) next to the pass/fail predicates, so that "all passed" still ranks | predicates, invariants and margins computed only by the world process |
| 4.15 | Smaller adoptions: named start states in scenarios (MuJoCo keyframes: in-bay, section starts) with 4.6; contact points, normals and surface id as Foxglove markers (MuJoCo viewer) with 4.8 | with those items |

Workflow rule for 4.9: live tuning builds intuition, the corpus decides. A value
found live is saved as an overlay and goes through the corpus sweep; it is kept
only if the separate pass/fail predicates improve over several seeds. History
here shows single-run impressions mislead (one perception knob moved 21 scenario
verdicts for a net +1; scenario 0005 is a knife edge).

Optional, only if Studio's one-way control proves limiting: an Ebitengine
top-down developer viewer with pause, single step and speed control for the
lockstep sim (2D, pure Go). For native 3D later, raylib-go (cgo) over g3n.

## 8. Phase 5: CPU contention and hardware in the loop

| # | Item | Done when |
|---|---|---|
| 5.1 | Contention on the dev box: `stress-ng --cpu 4 --cache 2 --vm 1`, per-process systemd slices with `CPUQuota=` and a short `CPUQuotaPeriodSec`; compare per-stage p99 | report; note Go sets GOMAXPROCS from the cgroup limit |
| 5.2 | **MCAP replay under impairment**: replay a real run through the stack (recorded clock), with netem/cgroup stress, and diff the decisions | replay tool + diff report |
| 5.3 | **HIL bench**: real Pi 5 with unmodified binaries, sim on a PC over **wired Ethernet** (not WiFi), the Pico running real firmware with actuators blocked, or `boardsim` | smoke and timing runs on a schedule, not per commit |
| 5.4 | Pico bench verification: pins, PWM, watchdog, encoder interrupts; GPIO timing probe to measure link asymmetry once; replace `board_sim.toml` estimates with measurements | TOML values cite the measurement |
| 5.5 | CPU pinning: control and serial reader on an isolated core, vision on the rest; PREEMPT_RT **only** if cyclictest shows scheduler latency matters next to 1 ms USB frames and ~0.85 s vision | decision recorded with data |

## 9. Phase 6 and later: generic map and rulebook-dependent work

| # | Item | When |
|---|---|---|
| 6.1 | `Map` interface designed for 3D, implemented in 2D (`RayCast`, `Occupied`, `SurfaceAt`), with the parking lot and fixed objects modelled | after 3.4 |
| 6.2 | Localization with disambiguation (breaks the 180 deg twin) | with 6.1 |
| 7.x | Motion-aware planner with `R(v)`; generic mission/state machine; 2.5D (IMU pitch/roll measured first, LIDAR compensated); full 3D after a sensor ADR | only once a new rulebook says which scenario (A-H) applies |

Any time, not blocking: sim fidelity (escape rotation, steering saturation,
vision emulator rate/latency/colour errors).

- **MuJoCo contact-fidelity spike (F.1)**
  - **Scope:** time-boxed to 1-2 weeks, in the Python `sim` pixi
    environment only (`mujoco` ~27 MB wheel, pinned version), after 2.11.
  - **Model:**
    - The chassis is a 0.30 x 0.194 m, 1.3 kg box on planar joints
      (slide x, slide y, hinge yaw).
    - `velocity` actuators track the 4WS kinematics' body velocity, with
      `forcerange` from the measured stall force, so contact saturates
      the force and the solver slides, rotates or pushes. The template is
      robosuite's `null_mobile_base.xml` (kv 1000, forcerange ±600,
      frictionloss 250, sized for a Tiago; re-measure for our car).
    - No wheel contacts, and never write the chassis `qpos` directly.
    - Pillars are free 50 x 50 x 100 mm boxes. **Measure their mass and
      mat friction first.**
    - Contact settings: `condim=3`, elliptic cone, `implicitfast`,
      timestep 0.002 s.
  - **Experiments against bags:**
    - wall-slide progress at 10-30 deg
    - escape yaw (the 40-50% under-rotation)
    - pillar displacement per contact
    - the reverse pivot at about 1 rad/s
  - **Done when:** a report recommends one of three outcomes:
    - (a) a friction coefficient added to `AllowedStep` closes the gap:
      port it to Go
    - (b) it needs real dynamics: evaluate a cgo layer or a MuJoCo side
      process (no Go bindings exist)
    - (c) MuJoCo does not close it either: the gap is in the drive or
      servo model
- **MuJoCo for 3D physics:** only if a rulebook adds ramps or terrain, and
  only after F.1 shows it matches this chassis. Gazebo keeps the
  camera-image role.

## 10. Decisions needed

1. ~~ADR 0068 parity target (0.3).~~ Decided 2026-09-24: functional parity
   per outcome (`7f21e65f`).
2. Whether the Pi Zero stays a supported build (drives 4.3's virtual Zero).
3. ~~Where this plan lives.~~ Decided: `other/docs/reference/`.
4. ~~Envelope format for 1.1.~~ Decided: a common typed timing message in
   the protos, not NATS headers.
5. Which keys are tunable live (4.9): start with a small set of navigation
   tuning keys in sim only, or mark broadly and restrict on hardware.
6. Whether to re-baseline the corpus at the measured command delay once
   phase 1 has it (2.8): the zero-delay baseline is optimistic by one tick.
7. Where 2.11 (solid walls) goes in the build order. It moves every Go
   baseline, so the recommendation is before any further sweep and before
   re-running section 13, and together with decision 6, so the corpus is
   re-baselined once rather than twice.
8. Whether to run F.1 (MuJoCo spike) at all, and when. It needs the pillar
   mass and mat friction measured on the bench first.
9. Context-aware board (section 16).
   - The recommendation is to build 2.13 and 2.14 now: they are small, and
     2.13 closes a known defect.
   - Gate 2.15 on phase 1 data showing link jitter matters next to the
     rest of the pipeline.
   - Gate 2.16 on what the hardware can sense.
   - Decide also whether the lease fields go into `boardlink.Command`
     (protocol version bump, ADR 0098 amended) or into a separate
     message.

## 11. Not adopting (and why)

- Gazebo/Isaac/CARLA-class 3D worlds and photoreal cameras: emulate detections
  at the detector output instead. Gazebo stays only as a later validation option
  for 3D (reuse doc 4.8).
- A full OpenSCENARIO DSL or a web evaluator platform: TOML/Go scenarios with
  predicates are enough.
- Randomized dynamics for RL.
- HIL as a gate on every commit: scheduled bench runs instead.
- Bramble emulator and PIO encoder: parked until something needs them.
- PREEMPT_RT by default: only with data (5.5).
- Rerun for visualization: excellent tool, but (as far as known on 2026-09-24,
  unverified) no Go SDK; Foxglove is already integrated.
- g3n for native 3D: development has slowed; raylib-go if native 3D is ever
  needed.
- MuJoCo inside the Go corpus loop: no Go bindings exist, a cgo layer
  would be ours to maintain, and bit-exactness holds only per MuJoCo
  version and architecture. Revisit only if F.1 ends in outcome (b).
- robosuite as a dependency: manipulation-centric. Its saved state
  (`time, qpos, qvel`) is not bit-exact, its delay is not a latency
  queue, and it draws noise from the global RNG. Its designs are adopted
  (4.12, 6.1), not the code.
- robomimic, behaviour cloning, offline RL: no learning problem on the
  plan. The would-be demonstrations come from our own policy, there are
  hundreds of runs rather than tens of thousands, and deployment would
  need ONNX in Go. JCIIOT's BC grasp only worked from its trained poses.
  Revisit only with trusted contact physics (2.11, F.1), 3.5 and 4.12 in
  place, and a behaviour that resists hand tuning (the escape / K-turn
  family is the candidate). Its designs are adopted (3.6, 4.13, 1.6).
- Domain randomisation as a training tool. Sampling physical parameters
  from **measured** ranges per seed is fine as a robustness sweep.
- HDF5 datasets: MCAP stays the single recording format.

## 12. Research sources

SITL/HIL:
- PX4 simulation, HITL, SIH, MAVSDK CI: https://docs.px4.io/main/en/simulation/ ,
  https://docs.px4.io/main/en/simulation/hitl.html ,
  https://docs.px4.io/main/en/sim_sih/ ,
  https://docs.px4.io/main/en/test_and_ci/integration_testing_mavsdk.html
- ArduPilot SITL JSON backend: https://ardupilot.org/dev/docs/sitl-with-JSON.html
- f1tenth_gym: https://f1tenth-gym.readthedocs.io/en/latest/ ; F1TENTH
  sim-to-real: https://arxiv.org/html/2410.07447v1
- Duckietown: https://github.com/duckietown/gym-duckietown
- ROS 2 clock and time: https://design.ros2.org/articles/clock_and_time.html
- Autoware driving_log_replayer_v2: https://tier4.github.io/driving_log_replayer_v2/ ;
  scenario_simulator_v2: https://tier4.github.io/scenario_simulator_v2-docs/
- CARLA leaderboard: https://leaderboard.carla.org/

Latency, contention, faults:
- tc-netem: https://man7.org/linux/man-pages/man8/tc-netem.8.html ; Toxiproxy:
  https://github.com/shopify/toxiproxy
- CFS bandwidth control: https://docs.kernel.org/scheduler/sched-bwc.html ; Go
  container-aware GOMAXPROCS: https://go.dev/blog/container-aware-gomaxprocs
- Pi 5 thermals: https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/
- Gilbert-Elliott (NTIA): https://its.ntia.gov/publications/download/TM-23-565.pdf
- CDC-ACM on Linux: https://michael.stapelberg.ch/posts/2021-04-27-linux-usb-virtual-serial-cdc-acm/
- ros2_tracing: https://arxiv.org/abs/2201.00393 ; cause-effect chains:
  https://dl.acm.org/doi/10.1145/3703630
- Asymmetric delay in NTP: https://blog.meinbergglobal.com/2014/05/12/asymmetric-network-delay-ntp/
- ROS-RVFT fault injection: https://ros-rvft.github.io/guidelines/guideline-mta1

Reference study (section 15; full list of URLs in the comparison doc):
- MuJoCo docs: https://mujoco.readthedocs.io/en/latest/ (computation,
  modeling, APIreference, changelog); repo https://github.com/google-deepmind/mujoco
- robosuite: https://robosuite.ai/docs/ ; https://github.com/ARISE-Initiative/robosuite
- robomimic: https://robomimic.github.io/docs/ ; https://github.com/ARISE-Initiative/robomimic
- JCIIOT 2026 platform: https://github.com/JCIIOT2026/JCIIOT2026 (studied at `48ab492`)

Research findings came from two delegated passes; the specific figures quoted
from them (for example Pi 5 PREEMPT_RT latencies, netem options) should be
rechecked against the source before they drive a decision.

## 13. Results: latency sweeps (2.3, 2.7)

Conditions: native runner, `src/python/.corpus/{obstacles,open}` (256
scenarios each), shipped TOML tree, profiles
`270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm`, 20 Hz control, runs
deterministic. Resolution is one control tick (50 ms): every delay in
(0, 0.05] s gives the same result (checked at 0.005 to 0.04).

CORRECTION 2026-09-24: the first version of this section (in `59b5ed78`)
was measured with exact time comparisons on an accumulated sim clock, which
delivered delays that are exact multiples of the tick one tick late. Those
rows were too pessimistic (Obstacles 0.2 s: 66 collisions, now 31; Open
0.3 s: 186, now 32). Fixed with a tolerance (`harness.TimeEpsilonS`) and a
regression test; the tables below are re-measured.

Obstacles, command delay:

| delay s | succeeded | collided | timed out | wrong side | contact runs |
|---|---|---|---|---|---|
| 0 | 176 | 11 | 56 | 12 | 27 |
| 0.05 | 154 | 9 | 72 | 21 | 33 |
| 0.1 | 166 | 13 | 59 | 18 | 31 |
| 0.15 | 174 | 22 | 49 | 11 | 51 |
| 0.2 | 189 | 31 | 27 | 9 | 52 |
| 0.3 | 108 | 129 | 1 | 18 | 171 |

Open, command delay: 256/256 succeed with no collision up to 0.25 s; at 0.3 s
224 succeed and 32 collide.

Obstacles, LIDAR scan delay: collisions 11 / 7 / 3 / 32 / 36 / 75 at 0 / 0.05
/ 0.1 / 0.2 / 0.3 / 0.5 s, while timeouts fall 56 -> 0 and mean run time
130 -> 86 s.

Obstacles, command drops with the board watchdog at 0.5 s: collisions 11-14
up to 50% drops (successes 176 -> 157-159, mostly extra timeouts), 44 at 70%,
65 at 90%.

Obstacles, detection (camera) latency, 2.7:

- **Sighted (default) corpus: no effect at all.** 0 to 1.2 s give
  byte-identical results, because the sighted runner hands the sign router
  the true layout and the camera only confirms it. The default Obstacles
  corpus cannot measure anything about vision.
- **Blind (`--blind`), where the robot must discover the signs:**

| detection delay s | collided | timed out | wrong side | contact runs |
|---|---|---|---|---|
| 0 | 19 | 229 | 8 | 34 |
| 0.25 | 30 | 212 | 14 | 40 |
| 0.5 | 40 | 194 | 22 | 60 |
| 0.85 (measured) | 74 | 140 | 42 | 91 |
| 1.2 | 76 | 108 | 71 | 111 |

- Frame drops alone barely matter (collisions 16-21, wrong side 8 at 0 to 90%
  drops): detections still arrive often enough. With the hardware-like drop
  rate 0.79, delay 0.85 s gives 72 collisions and 41 wrong-side passes.
- **Blind runs complete 0 laps in all 256 scenarios**, even at zero latency,
  although the robot drives and discovers 4.6 of 5 signs on average. Laps
  come from the navigator's own `LapsCompleted()`, so either the Go
  navigator never counts a lap in blind mode (the mode a real round runs in)
  or the blind harness misses what it needs. "succeeded" is therefore 0 in
  every blind row; collisions and wrong-side passes remain meaningful.
  Tracked as 2.9.

Readings:

- Collisions are the clean signal for command delay: roughly flat to 0.1 s,
  rising from 0.15 s, steep between 0.2 and 0.3 s in Obstacles; Open is
  clean to 0.25 s.
- "Succeeded" is not monotonic because collisions end runs that would
  otherwise have timed out: exactly why outcomes are kept separate.
- One tick of command delay alone costs Obstacles 22 successes and 9 more
  wrong-side passes; the zero-delay baseline is an idealization (2.8).
- Vision latency is the largest measured delay on the robot and, in blind
  mode, multiplies wrong-side passes (8 -> 42 at 0.85 s), which end a round.
  Any vision-dependent work must be judged in blind mode, not on the default
  corpus.

## 14. Diagnosis: blind runs do not count laps (2.9)

Investigated 2026-09-24 on `17ff7ce1`, native runner, shipped config.

What happens (traced from the recorded navigator debug of Obstacles
scenario 0): the blind robot drives complete laps, its waypoint index wraps
about four times in 200 s, but `laps_completed` never moves. Sighted runs
count laps through the waypoint-only fallback; blind runs install a
`racetracker.LapDetector` when direction inference settles, so a wrap only
arms it and the geometric start/finish crossing must confirm the lap. It
never does in Obstacles.

Confirmed defects:

1. **The sim applies a belief correction to the physical chassis.**
   `Navigator.ApplyBelievedStart` corrects the heading through
   `CorrectHeadingForDirectionChange`. The real gateway (`natsgw`) adds it to
   the estimator's heading offset; `harness.SimHardwareGateway` instead
   rotates the simulated body (`state.Yaw += delta`). A blind round starting
   in the North corridor is physically spun 180 degrees the moment it infers
   its direction (visible as about 20 s of manoeuvring right after the start).
2. **The lap detector and the pose live in different frames.**
   `adoptDirection` measures the start and builds the detector assuming the
   canonical South start section (`trackmodel.South` hardcoded, per ADR 0053).
   Logged values: scenario 0 measures the start at (2.05, 0.51), the true
   start (1.05, 2.50) rotated 180 degrees, while the sim reports the pose in
   the true frame, in the North corridor. The detector's section guard and
   finish line then rarely line up with where the robot actually crosses.

Refuted:

- **"Rotate the reported frame" as the fix.** Making the sim rotate the pose
  it reports (and place detections through the same rotation) instead of
  the body removes the spin and puts the pose in the canonical frame, and
  leaves the sighted corpus byte-identical, but blind gets worse: Open
  successes 41 -> 26, Obstacles wrong-side passes 8 -> 67, still 0 laps in
  Obstacles. Other blind consumers (the provisional-direction path, sign
  lanes) evidently depend on the current mixed frames; in scenario 5 the
  robot then followed its path backwards from the start. Reverted, not
  committed.
- **"Only South starts count laps."** At HEAD, blind Open counts at least
  one lap in 19/86 East, 6/48 North, 8/67 South and 13/55 West starts: no
  section is special.
- **The parking controller**, built from the true metadata in blind mode:
  removing it changed nothing.

Consequence: every blind number in section 13 comes from a sim that spins
the chassis on direction inference and cannot count Obstacles laps. The
relative latency effects there are still informative (same sim for every
arm), but absolute blind outcomes are not. Whether the real robot shares
defect 2 depends on the frame the real localizer reports after the heading
correction, which the sim cannot answer: that is 2.10.

## 15. Reference study: MuJoCo, robosuite, robomimic, JCIIOT 2026 (2026-09-25)

Full analysis, with verified library facts, sources and the per-concern
comparison: `2026-09-25-mujoco-robosuite-robomimic-comparison.md` (same
folder). This section keeps the conclusions and where each one landed in
the plan.

### 15.1 What the three are

- **MuJoCo** (DeepMind, Apache-2.0, 3.14.0 on 2026-09-22): a physics
  engine with a plain C API and Python bindings.
  - Constraint-based contact with a friction cone.
  - `mjModel`/`mjData` split.
  - Full-state save/restore (`mjSTATE_INTEGRATION`), deterministic per
    version and architecture.
  - Batched ray casting that can exclude the own body.
  - Since 3.5, native FIFO actuator and sensor delays saved with the
    state.
  - No simulator-side noise.
  - **No Go bindings** of any kind.
  - The official car model is a differential drive.
- **robosuite** (ARISE, MIT, v1.5.2): MuJoCo environments for
  manipulation.
  - A task is composed of an arena, robots and objects.
  - Per-sensor `Observable` pipeline (sensor, corrupter, filter,
    delayer, sampling rate).
  - Placement samplers with collision rejection.
  - Domain-randomisation and data-collection wrappers.
  - v1.5 adds mobile bases.
- **robomimic** (ARISE, MIT, v0.5.0 from source; PyPI still ships 0.3.0):
  imitation learning and offline RL (BC, BC-RNN, BC-Transformer,
  Diffusion Policy, IQL, CQL...).
  - HDF5 demonstration datasets that store sim states, actions,
    observations and filter masks.
  - Two-way playback with a divergence report.
  - Locked JSON configs and a sweep generator.

They are layers of one stack. vTitan already has its own upper layers:
scenarios with separate predicates, sensor and transport emulation, MCAP
and bag tooling, sweeps. What it lacks is the bottom layer's strengths:
contact dynamics and a restorable state.

### 15.2 Findings about vTitan

1. **The Go sim has no solid walls.** Only tests call
   `collision.AllowedStep`, and the gateway always moves to the candidate
   pose. Python slides. This confounds ADR 0068 parity for any
   non-terminal contact, and section 13's sweeps ran without contact
   response. The reuse doc's section 1.4 says Go slides; it does not.
   → 2.11.
2. **No snapshot/restore in either sim.** Knife-edge verdicts can only be
   replayed from tick 0. → 3.5, 3.6.
3. **Go's sensor world is easier than Python's** (section 2). → 2.12.
4. **Go knobs are CLI-only.** There is no `--seed` for native runs. The
   `cmd/sim-runner/main.go` header still says "Python oracle
   orchestrator". → 4.12.
5. **Python sweeps sed-edit the shipped TOML in place**, restored by a
   trap. A crash or a parallel session can leave `src/config` modified.
   → 4.13.

### 15.3 Findings from the JCIIOT 2026 platform

The official platform of the JCIIOT 2026 "RunningRobot" competition: a
simulated Tiago mobile manipulator doing pick-and-place in five factory
scenes. An LLM planner turns a prompt into skills, and grasping is a
robomimic BC policy. It is the only project found that integrates all
three libraries.

1. The default drive mode writes the base `qpos` directly each step.
   Collisions are printed, then "navigation continues". Contacts with
   tables, conveyors, shelves, containers and similar are ignored by
   substring. This is the same defect as finding 15.2.1.
2. Because the base is teleported, a held object does not follow it. A
   second hack pins the object's pose to the base during transport.
3. The scorer reads the last frame of a trajectory JSON written by the
   contestant's pipeline, not the sim state.
4. Result: 15 of 20 teams scored 100/100 automatically. The ranking came
   from human review deducting for collisions, teleportation or wall
   pass-through, and grasps without contact. → principle 8, 2.11
   invariants, 4.14.
5. The BC grasp runs in a separate, freshly built environment. The robot
   heading is forced to the trained poses, and the demonstrations come
   from a scripted teacher using privileged object positions. BC
   generalises only near its training data. → section 11 (robomimic).
6. robosuite's mobile base uses planar joints driven by force-limited
   velocity actuators. → template for F.1.

### 15.4 Adopted as design (code stays ours)

| Pattern | Source | Plan item |
|---|---|---|
| Constraint-style contact response, physics invariants, declared exemptions | MuJoCo; JCIIOT counter-example | 2.11, principle 8 |
| Port measured sensor models, own-body ray exclusion | MuJoCo `mj_ray` `bodyexclude` | 2.12 |
| World/state split, full snapshot and restore | MuJoCo `mjModel`/`mjData`, `mjSTATE_INTEGRATION` | 3.5 |
| Two-way playback determinism check | robomimic `playback_dataset.py` | 3.6 |
| Per-sensor descriptor with FIFO delay | robosuite `Observable` (shape), MuJoCo 3.5 delays (semantics) | 4.12 |
| Locked config, generated sweep overlays | robomimic `Config`, `hyperparam_helper.py` | 4.13 |
| World process owns scoring; continuous margins beside predicates | JCIIOT counter-example | 4.14 |
| Keyframe start states; contact markers in Foxglove | MuJoCo keyframes, viewer | 4.15 |
| Run tags | robomimic filter keys | 1.6 |
| Arena / objects / robot composition | robosuite `Task` | 6.1 |
| Planar base with force-limited velocity actuators | robosuite `null_mobile_base.xml` | F.1 |

Suggested order, pending decision 7:
1. 2.11 first, because it re-baselines.
2. Then 3.5 + 3.6, since everything after benefits from resume and
   determinism.
3. Then 2.12 + 4.12 together (the same code).
4. 4.13 and 1.6 whenever convenient.
5. F.1 once the pillar mass and mat friction are measured.

## 16. Context-aware board (proposal 2026-09-25)

**Idea (operator).** Tell the actuation board what is happening, so it can
react to latency, jitter and link loss according to the situation instead
of with one fixed rule.

**Today.**
- `boardlink.Command` is only `{SpeedMPS, SteeringAngleRad}`.
- The board applies the newest command, and stops the car after a fixed
  500 ms of silence (`CommandTimeoutMS`) plus the hardware watchdog.
- That single rule is too loose near a pillar: at 0.5 m/s, 500 ms is
  0.25 m of blind travel. It is too strict on a clear straight, where a
  short stall does not need a full stop.
- After a stall, the newest command may be stale and is applied once
  (2.6 residual).

**Design rules.**
1. **Constraints, not semantics.**
   - The board stays challenge-agnostic (principle 7). The host translates
     its situation into numbers the board can enforce: a validity window
     for each command, an expiry action, and speed/steering caps.
   - Wall, pillar and escape knowledge stays on the host.
   - No WRO vocabulary crosses the link.
2. **Monotonic safety.**
   - Host constraints can only **tighten** the board's compiled-in limits.
   - A lease longer than the board maximum is clamped. A missing or
     corrupt lease means the board default.
   - A bug or stale message on the host can therefore make the car more
     cautious, never less.
3. **Everything is time-stamped against the synced clock** (1.5), so the
   board judges freshness itself.
4. **Context is itself subject to latency.** The host's view of the world
   is up to 0.85 s old on the vision path. The host must size the lease
   from the **data age** of what it saw (1.3), not from the moment it
   sends.

**What it fixes and what it does not.**
- It fixes:
  - link stalls and jitter
  - stale commands after a stall
  - the one-size timeout
- It does **not** fix perception latency. The 0.85 s camera path and the
  one-tick command delay (section 13) happen before the command exists.
- For those the matching item is **host-side latency compensation**:
  propagate the pose forward by the measured data age before planning.
  That belongs with phase 1 numbers, and should be considered alongside
  this proposal.

**Items:**
- 2.13 command lease with expiry action
- 2.14 link health reported back to the host, so the host adapts too
- 2.15 setpoint horizon, only if jitter is shown to matter
- 2.16 local reflexes the board can detect faster than a host round trip

**Testing.** All of it is testable without the car: `boardsim` with
scripted stalls, bursts and asymmetric delay (2.1), on the sim clock
(3.1). The in-process harness can emulate the lease by giving each
emulated command an expiry.

**Prior art to check before designing** (not yet researched, verify
first):
- ROS 2 QoS deadline and lifespan
- PX4 offboard-mode timeouts and failsafe actions
- ArduPilot GCS and throttle failsafe actions
- AUTOSAR E2E protection (counter plus timeout on each message)
