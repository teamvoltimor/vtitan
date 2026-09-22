# WRO Future Engineers — Engineering Guide (Go + NATS)

A reference for rebuilding the WRO Future Engineers car as a modular, reusable, testable system: architecture, algorithms and their tradeoffs, simulation and Monte Carlo testing, implementation roadmap, package strategy, and libraries.

> **Note:** Rules (mat layout, parking, scoring, documentation requirements) change between seasons. Always check the current rules on wro-association.org. The architecture below is designed so a rule change only affects the `mission` layer.

---

## Table of contents

1. [Goals](#1-goals)
2. [Hardware architecture](#2-hardware-architecture)
3. [Why Go + NATS (and what it costs)](#3-why-go--nats-and-what-it-costs)
4. [Software architecture](#4-software-architecture)
5. [Should a neural network drive the car?](#5-should-a-neural-network-drive-the-car)
6. [Algorithm catalog](#6-algorithm-catalog)
7. [Tradeoffs of each algorithm](#7-tradeoffs-of-each-algorithm)
8. [Kalman filter and particle filter explained](#8-kalman-filter-and-particle-filter-explained)
9. [Estimating gyro drift](#9-estimating-gyro-drift)
10. [Testing methodology](#10-testing-methodology)
11. [Simulation](#11-simulation)
12. [Monte Carlo simulation](#12-monte-carlo-simulation)
13. [Implementation roadmap](#13-implementation-roadmap)
14. [Package design and reuse](#14-package-design-and-reuse)
15. [Third-party libraries and tools](#15-third-party-libraries-and-tools)
16. [Learning checklist](#16-learning-checklist)
17. [Open questions and things to verify](#17-open-questions-and-things-to-verify)

---

## 1. Goals

- **Reusable:** challenge rules are isolated in one layer; everything else survives rule changes.
- **Testable:** every algorithm is a pure package, testable with synthetic data, replayed logs, and simulation.
- **Measurable:** every design choice is decided by metrics from Monte Carlo runs, not impressions.
- **Educational:** the codebase is a platform for learning estimation, control, perception, and testing.
- **Shareable:** generic packages can be published or contributed to other projects.

---

## 2. Hardware architecture

### 2.1 Board roles

| Board | Language | Role |
|---|---|---|
| **Raspberry Pi 5** | Go (+ Python/C++ YOLO service) | Brain: perception, estimation, missions, control, **NATS server** |
| **Raspberry Pi Zero 2 W** | Standard Go | I/O bridge: LiDAR and IMU drivers, NATS client |
| **Raspberry Pi Pico / Pico 2** | TinyGo | Real-time: servo and ESC PWM, encoder counting, watchdog safety stop |

> Question worth testing: is the Zero needed at all? A Pi 5 plus a Pico is simpler. Measure before committing to the three-board split. [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) is how you answer it with numbers instead of opinion.

> **This table is the proposed rebuild, not the car as built.** vTitan today runs LiDAR, camera and the BNO085 (through the MCP2221 bridge) on the Pi 5, with the Pi Zero 2 W doing motor, encoder, button and OLED (`src/go/cmd/pi-zero`), and no Pico at all. Both designs are defensible; the guide should say which one it documents. See [§17](#17-open-questions-and-things-to-verify).

### 2.2 Sensors and actuators

- **LiDAR:** primary sensor for walls, corners, lateral position, and pillar ranges.
- **Camera + YOLO:** pillar color (red/green), parking markers. Colored corner lines can use HSV thresholding instead.
- **IMU (BNO085):** heading.
- **Servo:** steering (Ackermann).
- **Motor + encoders:** drive and speed feedback.

### 2.3 BNO085 interface choice

| Mode | What you get | Downsides |
|---|---|---|
| **UART-RVC** | Fused yaw/pitch/roll + acceleration at fixed 100 Hz; trivial driver | No raw gyro rate, no configuration, no calibration control |
| **I²C (SHTP)** | Full feature set: rotation vectors, calibrated/uncalibrated gyro, configurable rates, tare | Relies on clock stretching, which the Zero 2 W's Broadcom I²C handles poorly |
| **SPI (SHTP)** | Full feature set, faster, no clock-stretching issue | More wires (incl. interrupt pin), more driver work |

**Recommendation:** start with **UART-RVC** to get driving; move to **SPI** when you need raw gyro data for your own EKF, ZUPT, and Allan variance. If using I²C, connect it to the **Pico** (RP2040 handles clock stretching properly). A clean Go SHTP driver would be a valuable open-source contribution.

**Write the SHTP driver once, run it on both boards.** The driver needs exactly two things from its host:

```go
type Bus interface{ Tx(w, r []byte) error }                  // SPI transfer
type IntPin interface{ WaitLow(timeout time.Duration) bool }  // data-ready
```

periph.io implements both on Linux; TinyGo's `machine` package implements both on the Pico. Header parsing, Set Feature commands and report decoding then live in one package with no build tags, tested with plain `go test`.

- **Develop it on Linux first.** Logging, Delve, fast edits, no flash cycle. Port the same package to the Pico once it works.
- **Record raw SHTP traffic to files** and use them as parser fixtures, so decoding is tested without hardware. The same captures double as the [§9.5](#95-characterize-the-gyro-first) characterization recordings.
- Same constraint as [§14.5](#145-tinygo): no reflection, so it compiles under TinyGo.

### 2.4 Inter-board link: USB gadget Ethernet

- Pi Zero OTG port in **gadget mode** (use **ECM**, not RNDIS), Pi 5 as host.
- Expected NATS round trip: **~0.5–2 ms**, with spikes under load. Wi-Fi: a few ms with spikes of **50–100+ ms** — avoid it for control.

| Latency source | Typical cost |
|---|---|
| USB 2.0 link | ~0.3–1 ms ping round trip |
| NATS routing | tens of µs, but two hops (publisher → server → subscriber) |
| Pi Zero CPU | the main source of jitter |
| Protobuf serialization | negligible |

**Setup tips**
- Run the NATS server on the Pi 5.
- Set the Zero's CPU governor to `performance`; disable unneeded services.
- Keep messages small; process data locally where possible.
- Go sets `TCP_NODELAY` by default (Nagle is off).

**Measure**
1. `ping -c 1000 -i 0.01 <pi5-usb-ip>` — link baseline.
2. `nats rtt` and `nats bench` — NATS round trip and throughput.
3. A Go ping-pong with timestamps under real traffic — record **p50, p99, max**. The tail decides control stability.

If p99 is too high, keep the time-critical loop on one board and send only slower, high-level data across the link.

### 2.5 Determinism on Linux and the failsafe contract

[§3](#3-why-go--nats-and-what-it-costs) says "Go is not hard real-time" and [§2.1](#21-board-roles) hands the Pico a "watchdog safety stop". Both are right and neither is specified. Two questions are left open: how much jitter a Linux board actually has (which decides whether the Zero can hold a control loop, and therefore whether it is needed at all), and what the system does when something stops answering.

**Measure the right thing.** With hardware PWM the pulse train is generated in silicon and does *not* jitter with the scheduler. What jitters is the moment the duty cycle is updated, the timestamp of an encoder edge, and the wake-up of the control loop. Those are the numbers; PWM width on a hardware channel is not.

**The tuning ladder.** Climb it in order, and measure after each rung. Most of the win is in rungs 1-3, at a fraction of the cost of rung 4.

| Rung | What it does | Cost |
|---|---|---|
| **0. Baseline** | `cyclictest -m -p 80 -i 1000 -l 100000`, nothing tuned | none |
| **1. Housekeeping** | `performance` governor, Wi-Fi power save off, swap off, unused services off | minutes |
| **2. Process priority** | `SCHED_FIFO` (`chrt -f 80`), `mlockall`, `runtime.LockOSThread()` on the loop | a few lines |
| **3. Core isolation** | `isolcpus=3 nohz_full=3 rcu_nocbs=3` in `cmdline.txt`, IRQ affinity off that core, pin the loop to it | one boot edit |
| **4. PREEMPT_RT kernel** | preemptible kernel (mainline since 6.12); worst case typically drops from milliseconds to tens or low hundreds of µs | build and maintain a kernel |

Record the result of each rung, so the guide reports measurements rather than claims:

| Rung | p50 | p99 | max | Notes |
|---|---|---|---|---|
| 0. Baseline | | | | |
| 1. Housekeeping | | | | |
| 2. Priority | | | | |
| 3. Isolation | | | | |
| 4. PREEMPT_RT | | | | |

**Go-specific work.** The runtime adds its own pauses on top of the kernel's.

- Preallocate everything; keep the steady-state loop at **zero allocations**, and prove it with `testing.AllocsPerRun` and a `-benchmem` gate in CI rather than assuming it.
- `GOGC=off` with a `GOMEMLIMIT` backstop in the control process, or keep allocation at zero and leave GC on.
- `runtime.LockOSThread()` so the loop keeps the thread that carries the `SCHED_FIFO` priority.
- Avoid channels and `select` in the hot path; a preallocated ring or an atomic snapshot is cheaper and more predictable.
- If none of this gets the p99 inside budget, that is the argument for moving the loop to the Pico (or to C/Rust), not for more tuning.

**The failsafe contract.** Latency is the smaller half of this section. A Linux board offers no hard guarantee, so the design has to make the *failure* safe. Write these down as requirements with actual numbers, then test them on the bench rig ([§10.5](#105-firmware-bench-rig-pico-in-the-loop)):

| Failure | Detected by | Required response |
|---|---|---|
| Brain stops publishing commands | command age > `T_cmd` on the actuator board | steering to center, drive to neutral, within a stated time |
| Inter-board link drops | connection loss on the subscriber | same, and do **not** resume on reconnect until a fresh command arrives |
| Brain alive but sending stale or invalid commands | schema validation + staleness check per command | reject and treat as missing, rather than acting on it |
| Actuator process dies | hardware watchdog | reboot restores control, but PWM holds its last value through the reset unless the H-bridge enable is tied to a signal that dies with the process |
| Battery brownout | undervoltage reporting | log it; a reset mid-round is a scored failure, so this is a design constraint, not a runtime response |

> **Open question:** the value of `T_cmd`, whether "neutral" means coast or active brake for the drive motor, and whether the H-bridge enable is actually tied to something that dies with the process. None of the three is decided. See [§17](#17-open-questions-and-things-to-verify).

**Why this decides [§2.1](#21-board-roles)'s question.** If rungs 0-3 hold a 100 Hz loop with p99 inside the control budget, the Zero can own actuation and the Pico is optional. If they do not, the Pico is not optional. One measurement, both answers.

---

## 3. Why Go + NATS (and what it costs)

### Gains
- Static binaries, easy cross-compilation (`GOARCH=arm64`, or `GOARCH=arm GOARM=7`).
- Low memory use (important on the Zero).
- Memory safety compared with C++.
- Simple deployment compared with a ROS2 workspace.
- Sub-millisecond local NATS latency.
- **The simulator runs the exact same code as the car.**

### Losses you must replace

| ROS2 feature | Replacement |
|---|---|
| rosbag | NATS JetStream recording + MCAP files |
| RViz | Foxglove (MCAP) or a small web/ebiten viewer |
| tf2 | Your own `geom` package |
| Existing drivers | Port or write drivers for your LiDAR/IMU |

### Caveats
- **YOLO in Go is awkward** (ONNX Runtime via cgo). Prefer a separate Python/C++ inference service publishing detections over NATS.
- **Go is not hard real-time.** GC pauses are small but PWM belongs on hardware PWM or the Pico.
- **The link between boards is a new failure point.** Use a cable (USB gadget), not Wi-Fi.

### Go vs Python for simulation
- Tight loops (physics step, 360-ray raycasting, controller) are typically **10–100× faster** in Go than in pure Python.
- Vectorized NumPy, Numba, or JAX narrows the gap considerably.
- Headless, fixed-step, faster-than-real-time design matters more than the language.
- Both can parallelize Monte Carlo runs; Go makes it simpler (goroutines, no GIL).

---

## 4. Software architecture

### 4.1 Hexagonal (ports and adapters)

The core knows nothing about NATS, hardware, or the current season's rules.

```go
type RangeSensor interface{ Scan(ctx context.Context) (Scan, error) }
type HeadingSensor interface{ Yaw() (float64, time.Time) }
type Detector interface{ Detections() <-chan []Detection }
type Actuator interface{ Command(steer, throttle float64) error }
type Clock interface{ Now() time.Time }

// The rules of this season's challenge.
type Mission interface {
    Step(world WorldState) (Command, MissionState)
}
```

| Layer | Contents |
|---|---|
| **Domain** | Track model, pose estimation, controllers, state machine primitives — pure and testable |
| **Mission** | `OpenChallenge`, `ObstacleChallenge`, `Parking` — replaced when rules change |
| **Adapters** | NATS pub/sub, hardware drivers, simulator, replay-from-log — interchangeable behind ports |

- **Inject the clock** so tests, replays, and simulations are deterministic.
- **Explicit schemas** with Protobuf (or FlatBuffers).
- **Hierarchical subjects:** `car.sensor.lidar.scan`, `car.sensor.imu`, `car.perception.detections`, `car.cmd.drive`, `car.state.mission`.

### 4.2 Processing pipeline

```
perception  →  localization/estimation  →  decision (state machine)  →  control  →  actuators
```

### 4.3 State machine

```
START → DETECT_DIRECTION → STRAIGHT ⇄ TURN → (12 turns = 3 laps) →
   Open:     STOP_IN_START_SECTION
   Obstacle: FIND_PARKING → PARK → STOP
```

- Obstacle round adds an **AVOID** sub-state inside STRAIGHT (red → pass right, green → pass left).
- Record pillar positions on lap 1; they don't move, so laps 2–3 can be driven from memory and faster.
- Direction (CW/CCW) from LiDAR asymmetry at the first corner or from which colored line is seen first.

### 4.4 Localization without SLAM

- **Heading:** gyro yaw, snapped to 0/90/180/270° after each corner.
- **Section counter:** which straight you're on (12 turns = 3 laps).
- **Lateral offset:** LiDAR distance to the outer wall.

### 4.5 Config-driven algorithm selection

```yaml
estimator: complementary   # | ekf
controller: pid            # | purepursuit | stanley
lines: ransac              # | splitmerge
```

Swapping an algorithm is a flag, not a code change — essential for comparisons.

### 4.6 Time and timestamping

The injected `Clock` in [§4.1](#41-hexagonal-ports-and-adapters) buys determinism in tests. It says nothing about the harder problem: with two or three boards, every measurement crosses a clock boundary before anything acts on it.

- **Stamp at the source.** A measurement carries the time it was *taken*, in the driver, closest to the hardware event. The time it was received is a different (and less useful) number; log both if you want to see transport delay.
- **Bound the offsets between boards.** chrony over the gadget link, Pi 5 as server, for the two Linux boards. The Pico has no RTC and counts from boot, so estimate its offset and drift from the round trip of its own link protocol (an SNTP-style exchange over the serial link) and re-estimate periodically. State the residual offset you achieve; do not assume it is zero.
- **Set a budget, in the units that matter.** At 1 m/s, 10 ms of clock offset is 1 cm of apparent position error. Decide what error is tolerable and derive the sync requirement from it. The same budget governs ground truth ([§11.7](#117-ground-truth-on-the-mat)).
- **Age is what the controller cares about.** Log `now - measurement.stamp` for every input at the moment of the control decision, and put its p99 in [§10.2](#102-metrics)'s table. That single number explains most of the gap between "works in simulation" and "wanders on the mat".
- **Monotonic for durations, wall clock for correlation.** Go's `time.Time` carries both, but the monotonic reading is dropped when the value is serialized, so send explicit fields across NATS rather than relying on it surviving the hop.
- **A LiDAR scan is not an instant.** A 10 Hz scan smears 100 ms of motion across its points. Carry the scan start time and per-point angle or time so the distortion can be undone with the pose estimate. [§11.2](#112-realism-models) models this distortion in simulation; nothing described so far corrects for it on the car.

> **Open question:** does the LiDAR driver for the chosen model expose per-point timing, or only a scan boundary? That decides whether de-skewing is possible at all. See [§17](#17-open-questions-and-things-to-verify).

---

## 5. Should a neural network drive the car?

**No — use neural networks for perception only** (YOLO), and keep driving classical.

**Why end-to-end is a poor fit**
- The problem is well-structured: fixed 3×3 m track, measurable walls, simple rules. Hand-written logic solves it exactly.
- Randomized layouts require hundreds of recorded runs to generalize, and unseen layouts can still fail.
- A single collision or missed lap loses the round; PID fails predictably, networks fail silently.
- Debugging: a state machine is fixed by changing a threshold; a network by collecting data and retraining.
- The engineering documentation is scored; a clear architecture is easier to justify.

**Where networks help**
- Object detection (pillars, parking markers).
- Lighting robustness (a small CNN classifier vs. HSV thresholds).
- Optional experiment: a model predicting the avoidance offset, compared against the rule-based version.

---

## 6. Algorithm catalog

Not everything runs on the car. Algorithms fall into three relationships:

- **Pipelines (combine by design):** median → EMA → gating; YOLO → debounce → bearing fusion.
- **Alternatives (swap in the same slot):** complementary vs EKF; PID vs pure pursuit vs Stanley; RANSAC vs split-and-merge.
- **Supporting pieces (feed any choice):** startup bias and ZUPT improve both the complementary filter and the EKF.

### 6.1 Core stack (what runs on the car first)

| Layer | Use | Why it's enough |
|---|---|---|
| Signal | Median on LiDAR + EMA on encoder speed + range gating | Cheap; removes most noise |
| Perception | RANSAC wall lines + YOLO with N-frame debouncing | Heading, wall distance, pillar color |
| Fusion | YOLO bearing → LiDAR range | Accurate pillar position for little effort |
| Estimation | Complementary filter + startup bias + ZUPT | Handles drift over 3 laps |
| Decision | Hierarchical state machine | Required regardless |
| Control | PID on heading/offset + slew limiting | Proven, easy to tune |
| Parking | Scripted maneuver with LiDAR checks | Deterministic |

> **The estimation row depends on [§2.3](#23-bno085-interface-choice).** Startup bias and ZUPT both need the raw gyro rate. UART-RVC does not expose it, so on an RVC-only build the core stack is the complementary filter against wall angle ([§9.3](#93-lidar-wall-angle-measurement)) alone. See [§9](#9-estimating-gyro-drift).

### 6.2 Alternatives (pick one per slot)

- **Complementary vs EKF:** start complementary; switch to EKF only if logs show drift or noise problems.
- **PID vs pure pursuit vs Stanley:** compare in simulation, keep the winner.
- **RANSAC vs split-and-merge:** RANSAC is more robust with pillars in the scan.
- **Debouncing vs IoU/Kalman tracker:** debouncing usually suffices; add the tracker if detections flicker at speed.

### 6.3 Exploration only (learning, not needed to win)

- **Particle filter** — wall lines + section counter already localize on this track.
- **Reeds–Shepp parking** — elegant, but scripts are more reliable on a fixed layout.
- **DBSCAN** — RANSAC plus residual points already separates walls from pillars.
- **Allan variance** — offline analysis, not onboard.

### 6.4 Full list by layer

**Signal filtering**
- Median filter (LiDAR spikes, reflections)
- EMA / low-pass (encoder speed, gyro rate)
- Range and validity gating
- Complementary filter (gyro + wall angle)
- Debouncing (color lines, detections)

**Perception**
- LiDAR segmentation (breakpoint detection or DBSCAN)
- Line extraction (RANSAC or split-and-merge) → heading, wall distance, corners
- Object tracking (IoU + per-object Kalman, SORT-style)
- Camera–LiDAR fusion (box x-center → bearing → LiDAR range)
- HSV thresholding for orange/blue lines

**Estimation**
- EKF (pose + gyro bias)
- Zero-velocity updates (ZUPT)
- Particle filter (global localization)

**Planning and decision**
- Hierarchical FSM (mission → section → maneuver)
- Target lateral offset from pillar color/position
- Scripted parking; Reeds–Shepp as exploration

**Control**
- PID with anti-windup, filtered derivative, output limits
- Pure pursuit / Stanley
- Slew-rate limiting; speed profile (slow for corners and pillars)

**Rule of thumb:** add an algorithm only when logs show a specific failure it fixes.

---

## 7. Tradeoffs of each algorithm

### Signal filtering

| Algorithm | Pros | Cons |
|---|---|---|
| **Median** (core) | Removes spikes, keeps edges sharp | Adds window/2 samples delay; wide windows erase thin pillars |
| **EMA / low-pass** (core) | One line, almost no CPU | Adds lag; heavy smoothing makes control sluggish |
| **Range gating** (core) | Free; removes impossible readings | Thresholds need per-sensor tuning; too strict drops valid data |
| **Debouncing** (core) | Kills one-frame false positives | N frames of delay before reacting |

### Perception

| Algorithm | Pros | Cons |
|---|---|---|
| **RANSAC lines** (core) | Robust to pillars and outliers | Randomized, slight run-to-run variation; iterations trade speed vs reliability |
| **Split-and-merge** | Deterministic, fast | Outlier-sensitive; a pillar near a wall can split or bend the line |
| **DBSCAN** | Clusters of any shape | Two parameters to tune; overkill here |
| **IoU/Kalman tracker** | Stable IDs; predicts through missed frames | More code; ID mismatches cause confusing bugs |
| **Bearing → LiDAR fusion** (core) | Precise range from a cheap camera | Needs camera–LiDAR calibration; fails if the ray misses the pillar |

### Estimation

| Algorithm | Pros | Cons |
|---|---|---|
| **Complementary** (core) | ~10 lines, one tuning constant | Fixed trust ratio; no bias or uncertainty estimate |
| **Startup bias + ZUPT** (core) | Simple; fixes most gyro drift | Only corrects while stopped; misses warm-up drift during a run |
| **EKF** | Estimates bias, weights sensors by noise, gives uncertainty | Jacobians and `Q`/`R` tuning; diverges if badly tuned |
| **Particle filter** | Handles ambiguity and nonlinearity; global localization | Heavy CPU; needs a (partly unknown) map; noisy output |

### Control

| Algorithm | Pros | Cons |
|---|---|---|
| **PID** (core) | Universal, easy to reason about | Reactive only; gains depend on speed |
| **Pure pursuit** | Smooth, geometric, one main parameter (lookahead) | Cuts corners; needs a path |
| **Stanley** | Precise lateral tracking | Oscillates at low speed; needs accurate heading |
| **Slew limiting** (core) | Prevents jerks and skids | Limits reaction speed to sudden pillars |

### Decision and parking

| Algorithm | Pros | Cons |
|---|---|---|
| **Hierarchical FSM** (core) | Explicit, debuggable, adaptable to rule changes | Can tangle without disciplined transitions |
| **Scripted parking** (core) | Deterministic, quick to build | Brittle if start position varies |
| **Reeds–Shepp** | Optimal paths from any start pose | Complex math; needs accurate pose |

**Pattern:** each step up in sophistication trades simplicity and predictability for accuracy and flexibility. On a small, known track, simple usually wins until logs say otherwise.

---

## 8. Kalman filter and particle filter explained

Both maintain a **belief** about the car's state, **predict** how it changes as the car moves, and **correct** it when sensor data arrives. They differ in how the belief is represented.

### 8.1 Kalman filter

Belief = a Gaussian: state vector `x` (e.g. `[x, y, θ, gyroBias]`) and covariance `P`.

1. **Predict** (every control tick): motion model applied to inputs (encoder speed, gyro rate). Uncertainty grows:
   `P = F·P·Fᵀ + Q` (Q = process noise: slip, drift).
2. **Update** (when measurement `z` arrives): compare with expected `H·x`.
   Kalman gain `K = P·Hᵀ(H·P·Hᵀ + R)⁻¹` (R = sensor noise) decides trust.
   `x += K(z − H·x)`, and `P` shrinks.

The bicycle model is nonlinear (sin/cos of θ) → use an **Extended Kalman Filter**, linearizing with Jacobians around the current estimate.

**WRO example:** include gyro bias in the state. On straights, LiDAR wall distance measures lateral position and wall angle measures heading, so the filter learns the drift automatically.

### 8.2 Particle filter

Belief = N particles, each a candidate pose `(x, y, θ)`. Can represent several hypotheses at once.

```go
for {
    for i := range ps { ps[i].Move(v, yawRate, dt, noise) }             // predict
    for i := range ps { ps[i].W = likelihood(scan, raycast(m, ps[i])) } // weigh
    ps = resample(ps) // clone high-weight particles, drop low ones
    pose := weightedMean(ps)
}
```

- **Weighing:** raycast the map from each particle's pose, compare the expected scan with the real one. The simulator's raycaster is reused here.
- **WRO complication:** inner walls move between rounds. Match only outer walls, or add inner-wall offset to each particle's state.
- **Cost:** ~300–500 particles × 30 rays is fine on the Pi 5 in Go. Resample only when the effective sample size drops.

### 8.3 When to use which

| | Kalman (EKF) | Particle filter |
|---|---|---|
| Belief | Single Gaussian | Arbitrary, multi-modal |
| Best at | Fast, smooth fusion (gyro + encoders) | Global position against a map |
| Cost | Very low | N × raycasts |
| Fails when | Initial guess badly wrong | Too few particles |

**Combination:** EKF at control rate for heading and velocity; particle filter at LiDAR rate providing pose measurements to the EKF.

**Learning path:** 1-D Kalman (heading + gyro bias) on recorded data → EKF on pose → particle filter in the simulator.

---

## 9. Estimating gyro drift

Drift = **bias** (nonzero reading when not rotating) + **scale error** (reads 88° for a real 90°).

**Three of the four mechanisms below need the raw gyro rate, so [§2.3](#23-bno085-interface-choice) decides which of this section you can build.** UART-RVC emits fused yaw only: no raw rate, no configuration. On an RVC build, [§9.1](#91-startup-calibration) startup bias, [§9.2](#92-zero-velocity-updates-zupt) ZUPT and [§9.4](#94-bias-as-a-kalman-state) bias-as-a-state are all unavailable, [§9.3](#93-lidar-wall-angle-measurement) wall angle is the only correction left, and [§9.5](#95-characterize-the-gyro-first) narrows to the scale error, which is measurable from fused yaw by turning a known number of times.

**What unlocks the rest: putting the BNO085 on the Pico over SPI.** This is a real option, not a closed door, and it is the main reason the three-board layout in [§2.1](#21-board-roles) might earn its place. What it requires, concretely:

- **Wiring:** SCK/MOSI/MISO/CS plus INT, RST and PS0/WAKE, 3.3 V logic throughout. More wires than RVC's two, and a mount that keeps them short.
- **Firmware:** an SHTP driver in TinyGo. Write it against the `Bus`/`IntPin` interfaces in [§2.3](#23-bno085-interface-choice) so the parsing package is developed and debugged on Linux and then compiled for the Pico unchanged.
- **A link and a time base:** a framed protocol to the brain plus the clock-offset exchange in [§4.6](#46-time-and-timestamping), because the Pico has no RTC and counts from boot. Raw gyro with an unbounded timestamp is worth less than fused yaw with a good one.
- **A bench rig:** [§10.5](#105-firmware-bench-rig-pico-in-the-loop) stops being optional. A board in the control path that has never been tested electrically is the wrong trade.
- **A third board's costs:** one more failure point, one more thing to power and provision, and whatever compute-split reasoning put the sensors where they are must be revisited rather than assumed.

**What it buys:** startup bias, ZUPT and a bias state become possible; Allan variance characterization gets real raw data instead of a fused output; and the interrupt is timestamped in firmware rather than through Linux, which removes the 100 µs to 1 ms of jitter a Linux INT handler adds.

**Decide it with a measurement, not a preference.** Measure the scale error and the per-lap drift against ground truth ([§11.7](#117-ground-truth-on-the-mat)) first. If heading error is inside budget on an RVC build, the Pico buys precision nobody needs; if it is the binding constraint, this is the path that fixes it and the cost is justified.

### 9.1 Startup calibration

```go
var sum float64
for i := 0; i < n; i++ {
    sum += gyro.RawYawRate()
    time.Sleep(5 * time.Millisecond)
}
bias := sum / float64(n)
// later: yaw += (gyro.RawYawRate() - bias) * dt
```

Removes most error, but bias changes as the sensor warms up.

### 9.2 Zero-velocity updates (ZUPT)

When encoders say the car is stopped, true yaw rate is zero → the reading is the current bias:
`bias = 0.95*bias + 0.05*raw`.

### 9.3 LiDAR wall-angle measurement

- Fit a line to side-wall points (least squares or RANSAC to reject pillars).
- Its angle relative to the car = true heading relative to the track (a multiple of 90°).
- Difference from integrated gyro heading = accumulated drift.
- **Scale factor:** 4 corners = 360°; compare with what the gyro measured over a lap.

### 9.4 Bias as a Kalman state

- State `x = [θ, b]`
- Predict: `θ += (ω_raw − b)·dt`; `b` constant with small process noise.
- Update: wall angle corrects both `θ` and `b`.
- Bias is only **observable** because heading is measured regularly.

### 9.5 Characterize the gyro first

Record the car still for 30–60 minutes:
1. **Integrate and plot the angle** → drift rate (°/min); curvature shows temperature effects.
2. **Allan variance** → angle random walk and bias instability → measured values for `Q`.
3. **Log temperature** → fit a linear correction if bias tracks it.

---

## 10. Testing methodology

### 10.1 Ablation: one change at a time

1. **Freeze a baseline** (the core stack).
2. **Change one slot at a time**, everything else identical.
3. **Run on the same inputs:**
   - Unit tests with synthetic data and known answers
   - Replay of recorded real runs
   - Simulator Monte Carlo (100+ randomized layouts with noise)
   - Real mat, only for finalists

### 10.2 Metrics

| Metric | Measures |
|---|---|
| Success rate over N layouts | Reliability (most important) |
| Heading error at end of lap 3 | Drift correction |
| RMS lateral error | Tracking quality |
| Lap time | Speed |
| Detection-to-steer latency | Responsiveness |
| CPU % and loop jitter | Fits on the Pi |

### 10.3 Catch: slots interact

A different estimator changes controller behavior. **Re-tune controller gains after swapping the estimator**, otherwise you compare a tuned setup with an untuned one.

### 10.4 Test types

- **Table-driven tests** for the state machine.
- **Property-based tests** (e.g. "steering never exceeds limits").
- **Scenario tests** in the simulator, run in CI.
- **Benchmarks** for filter and raycast loop timing.

### 10.5 Firmware bench rig (Pico in the loop)

> **Conditional on there being a Pico.** On a two-board build this section is inert. It becomes mandatory the moment anything safety-critical or timing-critical moves to a microcontroller, including the BNO085-over-SPI path in [§9](#9-estimating-gyro-drift).

[§11.4](#114-hardware-in-the-loop-and-replay-over-nats) tests the *brain* against a simulator. Nothing so far tests the firmware, which by [§2.1](#21-board-roles) is the only component holding a safety-critical duty. This rig measures the Pico's outputs electrically instead of trusting them.

**Wiring.** A Linux host with GPIO stands in for the brain and instruments the Pico at the same time. A spare Pi Zero is convenient for this, but it is bench equipment, not a board on the car.

- Host USB → Pico USB, carrying the same protocol as the real link.
- Host GPIO → servo and ESC PWM pins, to measure the outputs.
- Host GPIO → encoder pins, to generate quadrature pulses.
- Common ground, 3.3 V logic on both sides, no servo power drawn from the host.

**Tests to automate.**

| Test | Pass condition |
|---|---|
| Drive command → PWM | Pulse width matches the command within tolerance |
| Watchdog | Stop sending commands; PWM returns to neutral within `T_cmd` ([§2.5](#25-determinism-on-linux-and-the-failsafe-contract)) |
| Command latency | Time from frame sent to PWM change, reported as p50/p99 |
| Encoder counting | Generated pulses match reported counts, both directions |
| Protocol robustness | Random bytes and corrupted frames are counted as errors; no crash, and the stream resyncs |
| Reboot recovery | Reset the Pico mid-run; hello/version exchange and telemetry resume cleanly |
| Stress | Max command rate plus max telemetry; loop time stays inside budget |

Write them as Go tests behind a build tag (`go test -tags hil ./hil/...`) and run them after every firmware flash, or automatically from a self-hosted CI runner on the bench host. This is also where the failsafe rows of [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) stop being a table and become tests.

**Limit.** Linux GPIO timestamps carry jitter. That is fine for 1-2 ms servo pulses and slow encoder rates, and not fine for precise microsecond measurement. Use a logic analyzer or a second microcontroller when the number has to be exact.

---

## 11. Simulation

Use layers, each with a different job.

### 11.1 Custom 2D simulator in Go (main tool)

Headless, faster than real time, used for Monte Carlo.

**Vehicle — kinematic bicycle model** (fixed time step, injected clock):

```go
func (c *Car) Step(steer, v, dt float64) {
    c.X += v * math.Cos(c.Theta) * dt
    c.Y += v * math.Sin(c.Theta) * dt
    c.Theta += v / c.Wheelbase * math.Tan(steer) * dt
}
```

- **World:** walls as line segments, pillars as circles, layouts randomly generated under the current rules.
- **LiDAR:** raycast against segments and circles at the real angular resolution and scan rate.
- **Camera:** don't render images — **synthesize detections** (pillars in the field of view become boxes with jitter, misses, false positives, and latency). YOLO accuracy is tested separately on real images.

### 11.2 Realism models

| Model | Add |
|---|---|
| LiDAR | Gaussian range noise, dropouts, motion distortion within a scan |
| Gyro | Bias + random walk in the bias, scale error (use Allan variance numbers) |
| Servo | Delay, rate limit, deadband |
| Motor | First-order lag, speed-dependent slip |
| Detections | Miss rate rising with distance, box jitter, pipeline latency |

**Domain randomization:** randomize all of these per run.

### 11.3 Calibrate against real logs

Feed the same commands from a real run into the simulator, compare trajectory and sensor traces, adjust the model until they match. Without this, the simulator only tests itself.

### 11.4 Hardware-in-the-loop and replay over NATS

The simulator publishes on the same subjects as real sensors and consumes `car.cmd.drive`. The real `brain` binary runs on the real Pi 5, catching timing, CPU, and serialization issues a pure simulation hides.

**Replay onto the real bus.** The same setup with a recorded bag in place of the simulator: republish a run's LiDAR, IMU and encoder messages on the real subjects, let the real brain decide, and compare its decisions against what the recording shows it did before. Keep the wheels off the ground or the actuator board disconnected, so nothing moves. This is the cheap rung between simulation and the mat: real binary, real CPU, real serialization, no battery and no track. [§10.1](#101-ablation-one-change-at-a-time) calls for "replay of recorded real runs" and [§14.1](#141-module-layout) reserves a `replay` adapter for it; this is that mode.

### 11.5 3D simulators (optional)

- **Webots:** easy setup, good camera rendering; useful for camera placement.
- **Gazebo:** familiar from ROS2 but heavier.

Too slow for Monte Carlo; use only for camera-in-the-loop tests.

### 11.6 Visualization

Write runs to **MCAP** and open them in **Foxglove** (plots, 2D scene, timeline scrubbing). Works for both simulated and real runs.

**Build order:** bicycle model + raycast → synthetic detections → noise models → ground truth → calibration → HIL. Ground truth comes before calibration, because without it [§11.3](#113-calibrate-against-real-logs) has nothing to calibrate against.

### 11.7 Ground truth on the mat

**The hole this fills.** [§10.2](#102-metrics) asks for RMS lateral error and heading error at the end of lap 3. [§11.3](#113-calibrate-against-real-logs) says to compare trajectories against real logs. On the mat, the only trajectory available is the estimator's own output, which is the thing under test: the comparison is circular, and those metrics are really simulator-only. An external measurement breaks the circle.

**Setup.** A camera above the mat, an ArUco marker flat on the roof of the car. OpenCV detects the marker's pose each frame and publishes position and heading.

- **Calibrate once:** lens intrinsics and distortion (checkerboard), then a homography from image to mat coordinates using markers at known positions on the mat. Record the residual reprojection error. That residual *is* the accuracy of your ground truth, and it belongs in the documentation next to every number derived from it.
- **Publish on its own subject** (`truth.pose`) so the MCAP recorder writes it into the same bag as the run. Foxglove then plots estimate against truth on one timeline, with no extra tooling.
- **Sync the clocks** per [§4.6](#46-time-and-timestamping), with the offset budget stated. Ground truth stamped in the wrong time frame biases every comparison silently, in the direction of travel.
- **Host it off the car.** It is bench equipment with its own `cmd/`, running wherever the camera is convenient, not a car binary.

**What it unlocks.**

- Real estimator error: complementary filter vs EKF, heading and position, measured against truth instead of against each other.
- [§11.3](#113-calibrate-against-real-logs) stops being circular: same commands, simulated trajectory against measured trajectory.
- Gyro drift per lap measured directly ([§9](#9-estimating-gyro-drift)), rather than inferred from wall angles that the estimator is already consuming.
- True lap times and wall-clearance margins, which are among the strongest numbers an engineering document can carry.

**Limits.** Single-camera marker pose is weakest in heading, at long range, and near the frame edges. Expect roughly millimetres to a centimetre of position error and around a degree of heading over a 3x3 m field with a decent lens, and quantify your own from the reprojection residual rather than quoting that range.

> **Open question:** camera, lens and mount; which host runs detection at frame rate; and the accuracy actually achieved. Undecided. See [§17](#17-open-questions-and-things-to-verify).

**If you build only one of [§10.5](#105-firmware-bench-rig-pico-in-the-loop), the replay mode in [§11.4](#114-hardware-in-the-loop-and-replay-over-nats), and this, build this.** It turns every practice run into measured data.

---

## 12. Monte Carlo simulation

### 12.1 Concept

Run the same system many times with randomized inputs, and use the statistics of outcomes to answer questions like "how often does the car finish?" You can't enumerate all layouts, placements, directions, and noise combinations, so you sample them.

### 12.2 Procedure

1. **Define random variables and distributions** (layout uniform over legal ones; LiDAR noise σ = 1 cm; gyro bias from Allan variance; servo delay 20–40 ms).
2. **Draw one sample** using a seed.
3. **Run the full simulation.**
4. **Record metrics** (success, lap time, final heading error, failure cause).
5. **Repeat N times** and aggregate.

```go
func runBatch(cfg Config, n int) []Result {
    results := make([]Result, n)
    var wg sync.WaitGroup
    for i := 0; i < n; i++ {
        wg.Add(1)
        go func(seed int) {
            defer wg.Done()
            rng := rand.New(rand.NewPCG(uint64(seed), 0))
            results[seed] = Simulate(cfg, RandomScenario(rng))
        }(i)
    }
    wg.Wait()
    return results
}
```

Use a worker pool (e.g. `errgroup` with a limit) if memory becomes an issue.

### 12.3 Statistics

- 190/200 successes → 95%. Standard error = √(p(1−p)/n) ≈ 1.5% → **95% ± 3%** (95% confidence).
- 95% vs 80% is a real difference; 95% vs 96% at n = 200 is noise.

### 12.4 Key practices

- **Seed every run** → failures are reproducible (rerun seed 137, watch in Foxglove).
- **Common random numbers:** compare configs on the same seeds; count seeds where one succeeds and the other fails (paired comparison) to detect small differences with fewer runs.
- **Classify failures** ("hit wall in corner", "wrong side of pillar", "parking missed") and plot a histogram — it tells you what to fix next.
- **Parameter sweeps** show where performance breaks down and how much margin you have.

### 12.5 Launching hundreds of configurations

**Budget:** runs = configs × seeds. Example: 200 × 200 = 40,000 runs; at ~0.1 s per 3-lap run ≈ 4,000 CPU-seconds ≈ **~8 minutes on 8 cores**. Time a single run first. Run batches on a PC or cloud VM, not on the Pis.

**Search strategies**

| Method | How it works | Use when |
|---|---|---|
| **Random search** | Sample configs within ranges | Default; beats grids when few parameters matter |
| **Latin hypercube** | Random but evenly spread per parameter | Small budgets |
| **Successive halving** | Run all on 20 seeds, keep top half, add seeds, repeat | Hundreds of configs; ~5–10× fewer runs |
| **Bayesian optimization** | Past results choose the next config | Tuning continuous gains (PID, lookahead) |

Workflow: **coarse to fine** — wide random search → successive halving → fine sweep around the winner.

**Infrastructure**
- Generate configs in Go from ranges, each with an ID and hash.
- **NATS JetStream work queue**, one job per (config, seed); workers are the same `sim` binary, add machines by starting workers.
- Store results in **SQLite** or **DuckDB** (config ID, seed, success, failure cause, metrics).
- Analyze with SQL.

**Traps**
1. **Winner's curse:** the best of 200 is partly lucky — re-run finalists on fresh, unseen seeds.
2. **Overfitting to the simulator:** prefer configs robust across the whole noise range; confirm the top 2–3 on the real mat.

---

## 13. Implementation roadmap

Always keep something running end to end; improve one layer at a time.

> **Status check against the repo.** This roadmap reads as if starting from zero, and it is not where the project stands. The Go track already has NATS transport, Protobuf schemas with protovalidate, MCAP recording, a Foxglove bridge, a simulator with sensor-error models and a real-log corpus, parameter sweep scripts, the USB gadget link provisioned in Ansible, and `cmd/pi-zero` running motor, encoder, button and OLED in production, alongside the ROS2/Python stack it is replacing. Phases 0, 1, 3 and 5 are substantially built. Either rebase the phases on what exists, or label the table explicitly as the path for a team starting from a ROS2 car. See [§17](#17-open-questions-and-things-to-verify).

| Phase | Work | Exit criterion |
|---|---|---|
| **0. Preserve & set up** | Record rosbags from the current ROS2 car (LiDAR, gyro, encoders, commands). Go module layout, `geom`, config, Protobuf schemas, subject naming, CI | Repo builds, tests run in CI |
| **1. Minimal simulator** | Bicycle model, layout generator, LiDAR raycast, sim clock, MCAP logging | Drive with fixed commands, view in Foxglove |
| **2. Open challenge in sim** | Median + EMA, PID + slew, FSM, `OpenChallenge` | 3 laps noise-free, both directions |
| **3. Monte Carlo harness** | Seeds, batch runner, results DB, failure tags | One command → success rate over 200 layouts |
| **4. Realism & drift** | Noise models; startup bias, ZUPT, complementary filter, RANSAC wall angle | Reliable with noise; small heading error after lap 3 |
| **5. Real hardware** | NATS on Pi 5, `io-bridge` on Zero, USB gadget, adapters, latency measurement | Open challenge on the real mat, logged to MCAP |
| **6. Sim ↔ reality** | Calibrate sim against real logs and Phase 0 recordings; HIL | Sim and real trajectories match closely |
| **7. Obstacle challenge** | Synthetic detections → avoidance + pillar memory → YOLO service → bearing fusion | High Monte Carlo success, then real mat |
| **8. Parking** | Scripted maneuver with LiDAR checks, randomized start poses | Reliable parking in sim and on the mat |
| **9. Exploration** | EKF, pure pursuit/Stanley, tracker, particle filter, parameter search, Reeds–Shepp | Each compared against baseline via Phase 3 harness |

**Ordering rationale**
- Simulator before hardware — debug logic without batteries or a mat.
- Monte Carlo harness before fancier algorithms — every improvement is measured.
- Open challenge before obstacle challenge — strict subset of the work.

**Migration from ROS2:** optionally bridge ROS2 ↔ NATS and move one node at a time (control and missions first, drivers last). Prove parity by replaying the same recordings through both versions.

---

## 14. Package design and reuse

### 14.1 Module layout

```
wro/
  pkg/geom/                 // Vec2, Pose, transforms, angle wrap
  pkg/filter/               // median, ema, complementary (generic)
  pkg/estimation/ekf/
  pkg/estimation/pf/
  pkg/perception/lidar/     // segmentation, ransac, lines, corners
  pkg/perception/track/     // IoU + Kalman tracker, camera-lidar fusion
  pkg/control/              // pid, purepursuit, stanley, slew
  pkg/fsm/
  mission/                  // open, obstacle, parking (WRO-specific)
  internal/adapters/        // nats, hardware, replay
  sim/                      // bicycle model, raycast, layouts, noise
  cmd/brain/  cmd/io-bridge/  cmd/sim/
```

Composable filter interface:

```go
type Filter[T any] interface {
    Update(x T) T
    Reset()
}
```

**Build order:** `geom` + `filter` → `control/pid` with sim → LiDAR lines → complementary → EKF with bias → tracker + fusion → particle filter / Reeds–Shepp.

### 14.2 What can be shared publicly

A package can be shared if it contains **no WRO rules, no NATS, no hardware specifics**.

| Package | Useful to | Notes |
|---|---|---|
| **control** | Any robotics/automation project | Anti-windup and filtered derivative done properly |
| **filter** | Anyone handling sensor data | Small, easy to maintain |
| **estimation** (EKF, PF) | Robotics, drones | Depend only on gonum |
| **lidar** | 2D LiDAR users | Few Go libraries exist |
| **track** | Vision-detection users | Go take on SORT |
| **sim2d** | Ground-robot developers | Keep WRO track generator out |
| **montecarlo** | Any simulation/tuning work | Not robotics-specific |
| **Drivers** (LiDAR, BNO085) | Go robotics users | Real ecosystem gaps; build on periph.io |
| **natsbus** | People wanting a lightweight ROS alternative | Most interesting, most work |

**Keep in the WRO repo:** missions, track/layout generator, pillar rules, parking scripts, config and wiring. (A WRO-specific package can still help other teams — FE repos are public anyway.)

**Extraction rules**
1. Start inside the monorepo; extract once a package has **two real users** (car + sim) and a stable API.
2. Check existing projects first (gonum, foxglove/mcap, periph.io, TinyGo drivers).
3. Minimize dependencies (stdlib + gonum for algorithms).
4. Stay at v0.x while the API changes.
5. Release with license (MIT/Apache-2.0), runnable examples, tests, benchmarks, and a README stating scope.

Highest value for least maintenance: **control**, **lidar**, **montecarlo**.

### 14.3 Repo strategy

**Recommendation:** two repos; the library as a single Go module to start.

```
github.com/ralvarezdev/wro-fe      ← app: missions, wiring, cmd/, WRO layouts
github.com/ralvarezdev/<robokit>   ← library: control, filter, estimation, lidar, sim2d, montecarlo…
```

| Option | Pros | Cons |
|---|---|---|
| Everything in one repo under `pkg/` | Simplest; atomic changes | Library tied to a competition repo |
| Library repo, single module (recommended) | One `go.mod`, one tag, one CI | One shared version for all packages |
| Multi-module / one repo per package | Independent versions, minimal deps for users | Prefix tags, more CI, painful cross-package changes |

**Split a package into its own module when:** it brings heavy dependencies (`natsbus` → nats.go; drivers → cgo/periph.io), needs a different release pace, or has a stable standalone API.

**Local development across repos:**

```
go work init ./wro-fe ./robokit
```

Don't commit `go.work`; tag releases and update `go.mod` when ready. Start with the library as a folder inside `wro-fe` and move it once `control`, `filter`, and `montecarlo` stabilize.

### 14.4 Contributing to existing frameworks

| Your package | Possible home | Fit |
|---|---|---|
| LiDAR / IMU drivers | periph.io, `tinygo.org/x/drivers` | Strong |
| LiDAR / IMU drivers | Gobot | Weak, see below |
| control, estimation, lidar, sim2d | Viam RDK (Go) as external modules | Good |
| Generic Kalman filter | gonum | Unlikely (scope) |
| MCAP helpers | foxglove/mcap Go library | Small fixes only |
| natsbus | Own project, listed among NATS community tools | Too opinionated for nats.go |
| montecarlo | Own library | No obvious host |

- **Gobot is the weakest of those homes.** It is a framework, not a driver library: `Adaptor` per board, `Driver` per device, a `Robot` with `Work()`, `Start()/Halt()` and an event bus. A driver contributed there implements Gobot's interfaces and is reusable only by Gobot users, whereas periph.io and the TinyGo drivers take a plain driver over a bus interface, which is what [§2.3](#23-bno085-interface-choice) and [§14.2](#142-what-can-be-shared-publicly) already assume. Rank it third on that row.
- **Contributing:** real users and expert review, but you follow their conventions. **Open an issue before a big PR.**
- **Own framework:** full freedom and strong portfolio value, but heavy maintenance and hard adoption. Worth it only once packages form a coherent whole proven on the car.

**Path:** drivers → periph.io/TinyGo; algorithms → own library (optionally Viam modules); framework later, if at all.

### 14.5 TinyGo

- **Not for the Pi Zero 2 W.** It runs full Linux on a quad-core Cortex-A53; standard Go gives the full stdlib, real parallelism, and working `nats.go`:
  ```
  GOOS=linux GOARCH=arm64 go build ./cmd/io-bridge          # 64-bit OS
  GOOS=linux GOARCH=arm GOARM=7 go build ./cmd/io-bridge    # 32-bit OS
  ```
- **Good for the Pico** (RP2040/RP2350): hardware PWM, interrupt-driven encoders, deterministic timing, watchdog stop when commands stop arriving. Link to the Zero or Pi 5 over UART/USB serial with a small binary protocol.
- Keep `control` and `filter` free of reflection so they compile under both Go and TinyGo — the same speed PID can run on the Pico and in the simulator. The same rule applies to device drivers; see the shared SHTP driver in [§2.3](#23-bno085-interface-choice).

---

## 15. Third-party libraries and tools

**(cgo)** = needs C libraries; complicates cross-compiling. Check each project's recent activity before depending on it.

### Math and estimation
- `gonum.org/v1/gonum` — `mat` (EKF, least squares), `stat`, `floats`, `optimize`
- `gonum.org/v1/plot` — drift, Allan variance, results plots

### Messaging and serialization
- `github.com/nats-io/nats.go` + `jetstream` subpackage — pub/sub, work queue, recording
- `github.com/nats-io/nats-server/v2` — embedded server for integration tests
- `google.golang.org/protobuf` + **buf** CLI — schemas and codegen

### Logging and visualization
- `github.com/foxglove/mcap/go/mcap` — MCAP recording
- `log/slog` (stdlib) — structured logs
- `github.com/hajimehoshi/ebiten/v2` — live 2D sim window

### Hardware
- `periph.io/x/conn/v3`, `periph.io/x/host/v3`, `periph.io/x/devices/v3` — GPIO, I²C, SPI, PWM
- `go.bug.st/serial` — UART (LiDAR, BNO085 RVC, Pico link)
- `tinygo.org/x/drivers` — Pico-side drivers
- LiDAR driver for your model — search first; be ready to write your own
- `gobot.io/x/gobot` - **evaluated, not used.** A robotics *framework* (Adaptor + Driver + Robot lifecycle + event bus) that would own the process structure [§4.1](#41-hexagonal-ports-and-adapters) and [§14.1](#141-module-layout) already define, while bottoming out in the same periph.io and serial primitives listed above. It has no driver for the parts that actually cost time here (RPLiDAR framing, BNO085 SHTP, MCAP, the Protobuf contracts). Worth reading its device drivers as reference implementations; not worth the dependency.

### Vision (Pi 5 only)
- `github.com/yalue/onnxruntime_go` **(cgo)** — YOLO from Go
- `gocv.io/x/gocv` **(cgo)** — OpenCV (HSV thresholding)

### Config and results
- `github.com/knadh/koanf` — config (YAML, env, flags)

> **Conflicts with the repo as built,** which standardizes on Viper plus TOML (`pelletier/go-toml/v2`) with schema-driven codegen into `internal/config/generated`. Pick one deliberately rather than by accident. Related: the note below about `gopkg.in/yaml.v3` is stale in context, since the repo already depends on its maintained successor `go.yaml.in/yaml/v3`. See [§17](#17-open-questions-and-things-to-verify).
- `github.com/goccy/go-yaml` — YAML (`gopkg.in/yaml.v3` is unmaintained)
- `modernc.org/sqlite` — pure-Go SQLite, easy cross-compiling
- DuckDB Go driver **(cgo)** — analytical queries on the PC

### Concurrency and randomness
- `golang.org/x/sync/errgroup` — bounded worker pools
- `math/rand/v2` (stdlib) — seeded PCG

### Testing
- `pgregory.net/rapid` — property-based tests
- `github.com/google/go-cmp` — readable struct comparisons
- `testing` benchmarks (stdlib)

### Tools
- **Foxglove** — MCAP viewer
- **nats CLI** — `nats rtt`, `nats bench`, subject inspection
- **golangci-lint** — CI linting
- **Optuna** (Python) — Bayesian optimization driving the Go simulator (no mature Go equivalent)

---

## 16. Learning checklist

- [ ] Record reference data from the ROS2 car
- [ ] Bicycle model and LiDAR raycaster
- [ ] PID with anti-windup, filtered derivative, slew limiting
- [ ] Hierarchical state machine for missions
- [ ] Monte Carlo harness with seeds, results DB, failure tags
- [ ] Confidence intervals and paired comparisons
- [ ] Sensor noise models and domain randomization
- [ ] Gyro characterization: drift plot, Allan variance, temperature
- [ ] Startup bias, ZUPT, complementary filter
- [ ] RANSAC line extraction and corner detection
- [ ] USB gadget link and NATS latency (p50/p99/max)
- [ ] Hardware adapters; open challenge on the real mat
- [ ] Sim calibration against real logs; hardware-in-the-loop
- [ ] Synthetic detections, avoidance logic, YOLO service, bearing fusion
- [ ] Scripted parking
- [ ] EKF with bias state vs complementary filter
- [ ] Pure pursuit and Stanley vs PID
- [ ] IoU/Kalman tracker
- [ ] Particle filter in simulation
- [ ] Parameter search: random, successive halving, Bayesian (Optuna)
- [ ] BNO085 SHTP driver over SPI
- [ ] Latency ladder measured on the actuation board (cyclictest p50/p99/max at every rung)
- [ ] Control loop proven allocation-free, gated in CI
- [ ] Failsafe contract written with numbers and tested on the bench rig
- [ ] Timestamp at source; cross-board clock offset bounded and logged
- [ ] Measurement age at the control decision, p99, in the metrics table
- [ ] LiDAR scan de-skew (or a recorded decision not to)
- [ ] Pico firmware bench rig, `hil` tests run after every flash
- [ ] Bag replay onto the real bus with the real brain
- [ ] Overhead ArUco ground-truth tracker calibrated, publishing into the run bags
- [ ] Estimator error measured against ground truth rather than against itself
- [ ] Extract and publish `control`, `lidar`, `montecarlo`

---

## 17. Open questions and things to verify

Nothing above is settled just because it is written down. Every `> **Open question:**` marker in the guide is indexed here, together with the places where the guide contradicts the car as built. An item leaves this table when it has an answer *and* the section that raised it has been updated.

| # | Question | Where | Why it matters | Settled by |
|---|---|---|---|---|
| 1 | Does this guide document the shipped two-board car or the proposed three-board rebuild? | [§2.1](#21-board-roles) | Every board-role claim downstream inherits the answer | A decision, then one editing pass |
| 2 | Is the Pi Zero needed at all, between a Pi 5 and a Pico? | [§2.1](#21-board-roles), [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) | Removes a board, a link and a failure mode if the answer is no | The latency ladder measurements |
| 3 | What is `T_cmd`, the command-staleness timeout? | [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) | Too short stutters on a latency spike; too long keeps driving into a wall | Link p99 from [§2.4](#24-inter-board-link-usb-gadget-ethernet) plus stopping distance |
| 4 | Does "neutral" mean coast or active brake for the drive motor? | [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) | Decides where the car ends up after a failsafe trip | A bench measurement of both |
| 5 | Is the H-bridge enable tied to a signal that dies with the process? | [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) | Without it, a watchdog reboot leaves the last PWM applied through the reset | Reading the wiring; possibly a hardware change |
| 6 | Stop at rung 3, or build and maintain a PREEMPT_RT kernel? | [§2.5](#25-determinism-on-linux-and-the-failsafe-contract) | Rung 4 is a standing maintenance cost for every OS update | Whether rungs 0-3 already meet the budget |
| 7 | How is the Pico's clock related to the Linux boards'? | [§4.6](#46-time-and-timestamping) | Its measurements are unusable in a shared time frame until it is | Designing the offset exchange into the link protocol |
| 8 | Does the chosen LiDAR expose per-point timing? | [§4.6](#46-time-and-timestamping) | No per-point timing means no motion de-skew, at any speed | Reading the protocol spec for the model |
| 9 | What clock-offset budget do the estimator and ground truth need? | [§4.6](#46-time-and-timestamping), [§11.7](#117-ground-truth-on-the-mat) | Sets the sync requirement; 10 ms is 1 cm at 1 m/s | Backing it out of the position error you accept |
| 10 | Which camera, lens, mount and host for the overhead tracker, and what accuracy does it reach? | [§11.7](#117-ground-truth-on-the-mat) | Ground truth less accurate than the estimator proves nothing | Building it and reporting the reprojection residual |
| 11 | Rebase the roadmap on what is already built, or label it as the from-ROS2 path? | [§13](#13-implementation-roadmap) | Four of ten phases are largely done; as written the table misleads | A decision, then one editing pass |
| 12 | Koanf and YAML as written, or the repo's Viper and TOML with codegen? | [§15](#15-third-party-libraries-and-tools) | Two config stacks in one project is the worst outcome | A decision |
| 13 | UART-RVC only, or the BNO085 on a Pico over SPI? | [§2.3](#23-bno085-interface-choice), [§9](#9-estimating-gyro-drift) | RVC forecloses three of the four drift mechanisms; SPI reopens them at the cost of a third board, a link protocol, a time base and a firmware bench rig | Measure the scale error and per-lap drift against ground truth first, then decide whether heading is actually the binding constraint |

**Where the vTitan-specific answers live.** This guide describes a rebuild; the
repository it sits in is mid-migration on different hardware. Items 1, 2, 6, 11
and 12 above are answered for vTitan in
`src/go/docs/internal/2026-07-21-go-migration-plan.md` (status section dated
2026-09-21), which also records which parts of this guide were carried into that
plan and which were deliberately dropped, with reasons. Settle a question there
first, then update the section here that raised it.

**Unmeasured tables.** [§2.5](#25-determinism-on-linux-and-the-failsafe-contract)'s latency table is deliberately empty. It stays empty until the numbers come off the real board; filling it with plausible values would be worse than leaving it blank.
