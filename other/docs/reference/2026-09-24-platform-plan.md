# vTitan platform plan

Date: 2026-09-24. Status: accepted 2026-09-24; phase 0 done except 0.5 (deferred); open decisions in section 10.

Context: the team is not competing again in WRO 2026 (did not win the national).
The goal is to turn vTitan into a platform ready for whatever the next challenge
is: reusable, measurable, and testable without the car. This plan consolidates:

- the reuse analysis (`2026-09-24-reutilizacion-si-cambia-el-reto.md`, same
  folder): what is reusable, what to generalize, scenarios A-H, 3D maps;
- the Pico / board work already shipped (`948f49dc`, `55d854f8`, `233f94ae`,
  `f6e67f89`);
- two research passes on how other projects do SITL/HIL and latency / fault
  testing (sources in section 12).

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

## 2. Where we are

| Area | State |
|---|---|
| Virtual actuation board | `pkg/boardsim`: real `boardloop` on in-memory hardware, link latency/jitter/bit flips from `board_sim.toml`, end-to-end tests against `picolink`. On `origin/master` (`233f94ae`, `f6e67f89`) |
| In-process world sim | `internal/sim/harness` + `kinematics` + `collision` + `sensorerrors`: drivetrain lag and servo slew, **no transport or compute delay** |
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

## 5. Phase 2: fault models (cheap, high value)

| # | Item | Done when |
|---|---|---|
| 2.1 | `boardsim` realistic link faults, config driven and seeded: **Gilbert-Elliott** chunk/frame loss (rate + mean burst length), **stall windows**, **disconnect/reconnect** (EIO, optional new device name), **reader starvation** (buffer overflow drops oldest), asymmetric delay. Keep bit flips for decoder tests | each mode unit-tested; values in `board_sim.toml` marked as estimates until phase 5 measures them |
| 2.2 | Scripted failsafe tests asserting "drive at zero within X ms": link stall of 499 ms and 501 ms around the 500 ms board watchdog, host link timeout around 1 s, reconnect mid-command | tests in CI |
| 2.3 | **Quick win before the virtual robot:** configurable command delay/drop and sensor staleness inside the in-process world sim (`harness`), then a corpus **delay sweep** | curve "corpus score vs added latency" for Open and Obstacles |
| 2.4 | NATS fault shim: per subject drop / delay / freeze (repeat last) / reorder / noise, from TOML, seed logged into the MCAP | usable by tests and by the virtual robot |
| 2.5 | Host-side robustness from the research: udev symlink by serial number for the board; close the fd on error before reopening | reconnect test passes with a renamed device |

## 6. Phase 3: platform seams

| # | Item | Done when |
|---|---|---|
| 3.1 | Injectable **Clock** across all Go nodes (real, and sim driven by a `sim.clock` subject); refuse to run on an uninitialized clock | nodes run on sim time in a test |
| 3.2 | ADR "the challenge is a plugin" + inventory; move WRO 2026 to `internal/challenge/wro2026/` | corpus **identical** before and after |
| 3.3 | Board role vs chip: `board.toml` `kind` names the protocol (`boardlink` or NATS/Zero), the chip is a separate field; `picolink` becomes a generic `boardlink` host session; `firmware/pico2` is one adapter among possible others | ADR 0098 amended; `cmd/pi5` and profiles updated |
| 3.4 | Track described in data; remove `[4]Section` | a non-square test track generates and runs |

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

## 10. Decisions needed

1. ~~ADR 0068 parity target (0.3).~~ Decided 2026-09-24: functional parity
   per outcome (`7f21e65f`).
2. Whether the Pi Zero stays a supported build (drives 4.3's virtual Zero).
3. ~~Where this plan lives.~~ Decided: `other/docs/reference/`.
4. ~~Envelope format for 1.1.~~ Decided: a common typed timing message in
   the protos, not NATS headers.
5. Which keys are tunable live (4.9): start with a small set of navigation
   tuning keys in sim only, or mark broadly and restrict on hardware.

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

Research findings came from two delegated passes; the specific figures quoted
from them (for example Pi 5 PREEMPT_RT latencies, netem options) should be
rechecked against the source before they drive a decision.
