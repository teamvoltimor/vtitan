# MuJoCo, robosuite and robomimic: comparison with the vTitan simulator

Date: 2026-09-25. Companion to `2026-09-24-platform-plan.md` (item numbers
such as 4.4 refer to that plan) and `2026-09-24-reutilizacion-si-cambia-el-reto.md`.

## 0. Summary

- **What the three are.** MuJoCo is a physics engine. robosuite is a
  manipulation-learning framework built on MuJoCo. robomimic is a library
  that learns policies from demonstration datasets, usually recorded in
  robosuite. They are layers of one stack; only the bottom layer (physics)
  addresses a problem vTitan actually has.
- **Overlap with vTitan.**
  - vTitan already owns its own versions of the upper layers:
    - scenarios, corpora and scoring predicates
    - sensor noise and latency emulation
    - MCAP recording and bag analysis
    - sweeps
  - What it lacks is exactly what MuJoCo is best at:
    - contact dynamics (sliding, pushing, rotating against a surface)
    - a complete, restorable simulation state
- **New findings from this pass** (section 5):
  - **The Go sim has no solid walls.** `collision.AllowedStep` is ported
    but never called, and the chassis passes through walls and pillars
    (`internal/sim/harness/gateway.go:233`). The Python oracle does slide.
    Every Go-vs-Python parity comparison of anything that touches contact
    is therefore confounded.
  - Neither sim can snapshot and restore its state mid-run.
- **Case study (section 10):** the JCIIOT 2026 competition platform uses
  all three libraries together.
  - It has the same pass-through defect as our Go sim: by default the
    base pose is written directly and collisions are only logged. Its
    automatic scorer could not see it; 15 teams scored 100/100 and human
    reviewers had to rank them.
  - Its robosuite mobile base (planar joints with force-limited velocity
    actuators) is a ready template for our P5.
  - Its behaviour-cloning grasp only works from the poses it was trained
    at, which confirms section 8.
- **Recommendation.**
  1. Do not take any of the three as a runtime dependency of the Go stack
     or the corpus.
  2. Adopt their **designs**:
     - MuJoCo's model/data split, full-state save/restore, FIFO delay
       semantics and the constraint-style contact response
     - robosuite's per-sensor observable pipeline and placement samplers
     - robomimic's two-way playback determinism check, config
       locking/sweep generation and dataset filter keys
  3. Use MuJoCo itself only as an **offline contact-fidelity oracle** in
     Python, time-boxed, judged against bags.
  4. Keep robomimic parked: there is no learning problem here that the
     project has decided to take on.

## 1. Scope and method

- Library facts were checked on 2026-09-25 against the official docs,
  source on `main`/`master`, the GitHub API and PyPI. URLs are in section 11.
  Anything not confirmed is marked *unverified*.
- vTitan facts come from the code at `35316a96` with file paths. The one
  load-bearing claim (Go walls are not solid) was re-read directly in
  `gateway.go`.
- The question asked: what each tool is, where it overlaps with vTitan,
  what is worth adopting (as a dependency or as a design reference), and a
  concrete proposal.

## 2. What each tool is

### 2.1 Project facts

| | MuJoCo | robosuite | robomimic |
|---|---|---|---|
| Kind | Physics engine | Robot-learning environment framework | Imitation learning / offline RL library |
| Latest release | 3.14.0 (2026-09-22), releases every 2-3 weeks | v1.5.2 (2025-12-24) | v0.5.0 (2025-06-27); **PyPI still ships 0.3.0 (2023)**, 0.5 needs a source install |
| License | Apache-2.0 | MIT | MIT |
| Language | C/C++ core, plain C API; Python 3.10+ bindings | Python | Python + PyTorch |
| Maintainer | Google DeepMind | ARISE Initiative (Stanford SVL, UT Austin RPL) | ARISE Initiative |
| Activity | Very active (commit 2026-09-25) | Maintained, slow (2026-07) | Maintained, slow (2026-08) |
| Footprint | ~27 MB wheel; numpy, glfw, pyopengl, absl | mujoco>=3.3, numba, scipy, opencv, mink, qpsolvers | torch, torchvision, h5py, tensorboard, **pinned** transformers 4.41 / diffusers 0.11 / huggingface_hub 0.23 |
| Go support | **None**, official or community (0 Go repos found). A cgo layer over `mujoco.h` is feasible (flat C API) but would be ours to write and maintain | n/a (Python only) | n/a (Python only) |

### 2.2 MuJoCo

A general rigid-body physics engine designed for contact-rich robotics. You
describe the world in MJCF XML (or build it procedurally with `mjSpec`,
since 3.2), compile it into an `mjModel`, and advance an `mjData` with
`mj_step`. The pieces relevant here:

- **Model/data split.** `mjModel` is constant (geometry, masses,
  parameters); `mjData` is everything that changes. `mj_copyData` clones a
  state; you can keep one `mjData` per thread.
- **Stepping.** `mj_step` = checks + `mj_forward` + integrate. `mj_step1`
  runs the part before the control is set and `mj_step2` the part after,
  so the caller inserts its controller in between (single-step
  integrators only). Default `timestep` is 0.002 s. The docs recommend the
  `implicitfast` integrator for most models.
- **Full-state save/restore.**
  - `mj_getState`/`mj_setState` take a bit mask (`mjtState`).
  - `mjSTATE_INTEGRATION` captures everything needed for bit-exact
    continuation: physics, time, user inputs and solver warmstart.
  - The docs promise determinism for that full state, within one version
    and one architecture.
- **Contact.**
  - Contacts are soft constraints solved as an optimisation.
    - `solref` sets stiffness and damping, `solimp` the impedance curve.
    - `condim` picks the friction model: 1 = frictionless, 3 = sliding
      friction, 4 = plus torsional, 6 = plus rolling.
    - The friction cone is pyramidal (default) or elliptic ("a better
      model of reality").
  - Sliding along a wall falls out of the solver: the normal impulse stops
    penetration, and tangential motion continues, opposed by Coulomb
    friction.
  - Soft contacts allow slow slip. `impratio` (elliptic cones) and
    `noslip_iterations` tighten that.
  - `margin`/`gap` semantics changed in 3.9.0 (2026-05).
- **Ray casting.**
  - `mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, ...)` and
    the batched `mj_multiRay` (since 2.3.6).
  - Both can exclude a body or a geometry group, which is how you keep a
    LIDAR from hitting its own chassis.
  - 3.5.0 changed the signatures by adding a surface normal output.
  - The `rangefinder` sensor wraps the same thing.
- **Sensors.**
  - Available types include gyro, accelerometer, rangefinder, framepos,
    distance and others.
  - **The simulator adds no noise.** Native noise was removed in 3.1.4
    because it was not seedable or thread-safe; the `noise` attribute is
    now only metadata for the user.
- **Delays (new in 3.5.0, 2026-02).**
  - Actuators and sensors accept `delay`, `nsample` (ring buffer) and
    `interp`; sensors also take `interval` (sub-rate sampling).
  - The history buffer is part of `mjSTATE_PHYSICS`, so delayed signals
    are saved and restored with the state.
  - This is a true FIFO latency, not a phase offset.
- **Actuators.**
  - `motor`, `position`, `velocity`, `general`, `pid`, `dcmotor` (3.7.0:
    inductance, cogging, thermal effects, LuGre friction).
  - Activation dynamics (`dyntype filter` is a first-order lag), plus
    `ctrlrange` and `forcerange`.
  - No slew-rate attribute was found (*unverified*); a rate limit would be
    built from an integrator activation with a bounded `ctrlrange`.
- **Modelling.**
  - `<default>` classes, `<include>`, keyframes (named initial states).
  - Heightfields (terrain, ramps), engine plugins, sleeping islands (3.4).
- **Vehicles.**
  - The official `model/car/car.xml` is a **differential drive**: two
    wheels mixed through fixed tendons, plus a frictionless caster.
  - There is no Ackermann or 4WS example.
  - Community Ackermann models exist (MuSHR, stale since 2022) but none is
    mainstream; F1TENTH simulators are not MuJoCo-based.
  - There is no tyre/slip model beyond the Coulomb cone (*unverified*).
- **Tooling.**
  - The `simulate` viewer and the Python passive viewer (contact points
    and forces can be drawn).
  - Offscreen rendering and `mujoco.rollout` (multithreaded batch
    rollouts).
  - MJX (JAX) and MuJoCo Warp (NVIDIA GPU) for massive parallelism.

### 2.3 robosuite

A framework of MuJoCo environments for **manipulation**: robot arms,
grippers and, since v1.5, mobile and legged bases and humanoids (GR1). It
exposes a gym-style `reset`/`step` API with dictionary observations.

- **Composition.**
  - Class chain: `MujocoEnv` → `RobotEnv` → `ManipulationEnv`.
  - A `Task` merges an **Arena** (table, floor, bins), **Robots** and
    **Objects** into one MJCF model.
- **Observables** (`robosuite/utils/observables.py`).
  - Each observation is an `Observable`: a `sensor` function, then a
    `corrupter` (noise), a `filter`, a `delayer` and a `sampling_rate`.
  - Helpers include Gaussian and uniform corrupters, and deterministic,
    uniform and Gaussian delayers.
  - **Semantics to note:**
    - The "delay" is a phase offset inside one sampling period: a fresh
      value is read `delay` before the period ends. It is capped below one
      period (it warns and reads immediately otherwise).
    - It is **not** a latency queue, so it cannot represent a 0.85 s
      camera delay at a 10 Hz sensor rate.
    - Noise and delay draw from the global `np.random`, not a per-env
      stream.
- **Placement samplers.**
  - `UniformRandomSampler` and `SequentialCompositeSampler` (chains
    samplers, can place relative to other objects).
  - Collisions are rejected by 2D radius and z overlap, with up to 5000
    retries per object; failure raises `RandomizationError`.
- **Timing.** `control_freq` sets the action period, and each `step` runs
  `control_timestep / model_timestep` MuJoCo substeps. Observables update
  every substep.
- **Controllers.** Joint torque, velocity and position, OSC, and v1.5
  composite (whole-body) controllers.
- **Wrappers.**
  - `DomainRandomizationWrapper`: textures, colours, lights, cameras and
    dynamics (mass, inertia, friction, solref/solimp, damping), with its
    own seed.
  - `GymWrapper`.
  - `DataCollectionWrapper`: saves the model XML plus per-step states and
    actions.
- **State.** `MjSim.get_state()` returns only `(time, qpos, qvel)`: no
  activation, control, warmstart or delay history. By MuJoCo's own
  reproducibility note that is **not a bit-exact state**.

### 2.4 robomimic

A training and evaluation library for learning policies from offline
demonstrations.

- **Algorithms.**
  - Imitation learning:
    - BC (plus GMM and VAE variants)
    - BC-RNN
    - BC-Transformer
    - Diffusion Policy (new in v0.5)
    - HBC
  - Offline RL: IRIS, BCQ, CQL, IQL, TD3-BC.
- **Dataset format** (HDF5).
  - The `data` group holds `env_args`, stored as JSON.
  - Each `demo_N` group holds:
    - `actions`, `rewards` and `dones`
    - `states`, the flattened simulator states
    - `obs/<key>` and `next_obs/<key>`
    - a `model_file` attribute with the MJCF
  - `mask/<filter_key>` holds named subsets of demos, such as `train` and
    `valid`.
- **Scripts.**
  - `playback_dataset.py` replays a dataset two ways: forcing the recorded
    states, or re-applying the actions open loop. In action mode it
    compares each step with the recorded state and prints "playback
    diverged by {err} at step {i}", which makes it a built-in determinism
    check.
  - Also: `get_dataset_info.py`, `split_train_val.py`,
    `dataset_states_to_obs.py`.
  - `hyperparam_helper.py` generates a sweep of configs plus a shell
    script of runs.
- **Config.** `Config` objects serialise to JSON and lock at two levels:
  `lock_keys()` rejects new keys, `lock()` rejects any change.
- **Evaluation.** Periodic rollouts (`n`, `horizon`) log a success rate
  to TensorBoard or W&B.
- **Custom environments.** Implement the `EnvBase` abstract class (about
  20 methods, including `reset_to`, `get_state` and `is_success`) and
  register an `EnvType`. That is moderate effort.
- **Neighbours.** MimicGen (demonstration augmentation), LIBERO, RoboCasa.

## 3. Where vTitan is today

| Concern | Go sim (`src/go/internal/sim`) | Python sim (`src/python/src/simulation`) |
|---|---|---|
| Role | The migration target; must meet or beat Python per outcome (ADR 0068) | **Frozen oracle** and competition stack |
| Tick | `scenario/native_runner.go:514`: nav step → record → `gw.Advance(dt)` → checks. Fixed `dt`, sim clock owned by the gateway, single thread | Same loop shape (`simulator.py:1055`) plus measured tick-period jitter |
| Kinematics | 4WS counter-phase, speed lag, acceleration clamp, servo slew, 5 substeps (`kinematics.go:101`) | Same plus min-turn-radius curve and standstill scrub |
| Contact response | **None.** Oriented-rectangle test only; the body always moves to the candidate pose (`harness/gateway.go:233`). `collision.AllowedStep` exists but only tests call it | Solid walls and obstacles with **slide** (`collision_stepping.py:80`, `_slide_along`:34), optional pushable obstacles |
| Contact scoring | `contact_tracker.go`: forbidden-surface rules, start window and grace, fins unforgivable; pillar-nudge budget from the 85 mm circle | `scoring.py`: same plus reverse-run rule 9.21 |
| LIDAR | Raycast walls and boxes from the mount; i.i.d. dropout 1%, noise 0.03; **no chassis occlusion or self-returns** | Measured occlusion band 120-160 deg (dropout 0.689, self-return 0.021 m), invalid rate 0.095 |
| IMU | `sensorerrors`: bias, drift, scale, noise, start error; **perfect by default** | Same model; measured budget on by default |
| Vision | Visible if in HFOV and range; optional delay and drop (`sim_camera.go`), off by default | 0.85 s latency, 30% miss, colour flips, bearing scatter, range falloff, measured confidences |
| Transport | `harness/transport.go`: command delay/drop/watchdog, scan and detection delay; one-tick resolution; **CLI flags only** | n/a |
| Randomness | PCG streams from one seed, salted per subsystem; bit-reproducible. `sim-runner` has no `--seed` flag for native | Seeded |
| Snapshot/restore | **None** | **None** (only `apply_disturbance`) |
| Recording | MCAP per run (`scenario/record.go`), same topics as real bags | None |
| Scenarios | JSON metadata from `simgen`; corpora of 256 at seed 2026; Open space enumerated (`opencorpus`, 640 cases, MT19937 parity) | `scenario_catalog.py`, `scenario_builder.py` |
| Sweeps | `go/scripts/transport-sweep.sh` over one flag, TSV | `scripts/sim/sweeps/*.sh` sed-edit TOML in place, results in SQLite (`src/tools/sweep_results.py`) |
| 3D | `simgen` writes Gazebo SDF; Gazebo used **only for vision training data** (`other/apps/gazebo`) | n/a |
| Learning | Only the YOLO sign detector (Hailo); no RL or imitation code | n/a |

## 4. Side-by-side comparison by concern

Each row says what the reference does, what vTitan does, and the verdict.
The verdicts are: **adopt design** (copy the pattern, our code), **use tool**
(depend on the library), **keep ours** (ours is already better for our
needs), **skip**.

### 4.1 Time and stepping

- **MuJoCo:** the caller owns the loop. The `step1`/`step2` split leaves a
  slot for the controller inside the physics step.
- **robosuite:** control-rate actions over physics substeps.
- **vTitan:** the gateway owns a pure sim clock, the controller runs
  once per tick, and kinematics sub-step 5x. That is the same shape.
- **Verdict: keep ours.** Principle 1 of the plan is already met in the
  in-process sim. The lesson for 4.4 (lockstep through real binaries) is
  MuJoCo's explicit contract: sensors are produced, then the command is
  read, then the world integrates. Write that order into the world
  process protocol.

### 4.2 State, snapshot and determinism

- **MuJoCo:** `mjSTATE_INTEGRATION` covers everything needed to continue
  bit-exactly, including the delay history and the solver warmstart.
- **robosuite:** saves `(time, qpos, qvel)` only. That is a
  counter-example: its restored runs are not guaranteed to match.
- **robomimic:** stores `states` per step and checks determinism by
  replaying actions against them.
- **vTitan:** seeded and bit-reproducible, but only from tick 0. There is
  no way to resume a run at tick N.
  - That matters here. Many verdicts are knife-edges (scenario 0005, the
    `rev_speed` 25 → 13 knife edge, one knob moving 21 scenarios for a net +1).
  - Today a question like "what if the escape triggered one tick later"
    means re-running 200 s of sim and hoping the prefix is identical.
- **Verdict: adopt design.** Proposal P2 and P3.

### 4.3 Contact and physics

- **MuJoCo:** constraint-based contact with a friction cone. Sliding,
  pushing a free body and rotating against a wall are all consequences of
  one solver, tuned by `friction`, `solref`, `solimp`, `condim` and
  `cone`.
- **vTitan Go:** no response at all. Contact is only scored.
  - Terminal contacts end the round, so pass-through only matters for
    *non-terminal* contact: allowed wall touches in the grace window,
    pillar nudges, and escapes in progress.
  - Those are exactly the regimes where the measured fidelity gaps live:
    rotation during escapes is 40-50% too low, and the sim never slides.
- **vTitan Python:** a geometric slide (bisection plus tangential
  projection). This is a kinematic approximation with no mass or
  friction, which is why the "contact never slides" and "under-rotates in
  escapes" findings exist at all.
- **Verdict:**
  - **adopt design now:** wire `AllowedStep` in Go (P1).
  - **use tool offline:** MuJoCo as a contact oracle to learn what the
    right response looks like (P5).
  - **Not** a MuJoCo dependency in the corpus loop: there are no Go
    bindings, a cgo layer would be ours to maintain, and the Go stack is
    the migration target.

### 4.4 Sensors, noise and latency

- **MuJoCo:** clean geometry and a native FIFO delay with sub-rate
  sampling, saved with the state. Noise is left to the user on purpose,
  for seedability.
- **robosuite:** a clean per-sensor pipeline shape (sensor → corrupter →
  filter, plus delayer and rate). Its delay semantics (phase offset under
  one period) and global RNG make it the wrong *implementation* for
  0.85 s camera latency.
- **vTitan:**
  - One-tick-resolution FIFO delays in `harness/transport.go`, seeded per
    subsystem. Better than robosuite on semantics and seeding.
  - But:
    - it is configured by CLI flags only
    - Go ignores the measured LIDAR occlusion and vision corruption that
      Python has
    - the chassis self-return is not modelled in Go
- **Verdict: adopt design.**
  - robosuite's per-sensor descriptor *shape*
  - MuJoCo's delay *semantics* (FIFO, `interval`, part of the state)
  - MuJoCo's ray *exclusion* idea (exclude the own-body group, then
    re-add measured self-returns explicitly)
  - Proposal P4.

### 4.5 Scenarios, placement and randomisation

- **robosuite:** composable samplers with collision rejection, and
  `Task = Arena + Robots + Objects`.
- **vTitan:**
  - The WRO 36-entry table (`simgen/generate/scenarios.go:25`) plus legal
    start cells.
  - The eight duplicated double-pillar entries are expected: the
    operator confirmed 28 distinct scenarios in 36 entries.
  - `opencorpus` enumerates the Open space exhaustively. That is stronger
    than random sampling for a finite rulebook.
- **Verdict:**
  - **keep ours** for WRO.
  - **adopt design** for the generic map (6.1): arena / objects / robot as
    separate model parts is the clean way out of `[4]Section` and the
    4-fold symmetric wall model. Samplers with rejection become relevant
    only if a new rulebook stops enumerating layouts.
- **Domain randomisation.** Skip as a training tool. The same wrapper
  shape, sampling physical parameters from *measured* ranges per seed, is
  a fair robustness sweep (plan principle 2), but only for parameters
  that have a measured range.

### 4.6 Recording, datasets and replay

- **robomimic:** one self-describing file per dataset (model, env args,
  states, actions, observations, masks), plus two-way playback.
- **vTitan:**
  - MCAP for real and sim runs, 100+ bag diagnostic scripts, 413 real
    runs.
  - No tags: runs are classified after the fact by scripts.
  - The Go sim MCAP carries topics but not the world state or the
    scenario, so a sim bag cannot be restored or re-simulated from itself.
- **Verdict: keep MCAP** (it is the standard across the stack and
  Foxglove), and **adopt design** for the content:
  - a `/sim/state` channel and the scenario and config as MCAP attachments
    (P2, P3)
  - named filter keys over the run inventory (P7)

### 4.7 Config and sweeps

- **robomimic:** locked JSON configs and a generator that expands a
  parameter grid into runnable configs.
- **vTitan:**
  - A strong base: TOML plus JSON Schema plus generated DTOs, checked in
    CI.
  - The sweeps are fragile:
    - Python sweeps **sed-edit the shipped TOML in place** (restored by a
      trap), so a crash or a parallel session can leave a modified tree.
      The index-sharing incident in memory is the same class of risk.
    - Go sweeps cover one CLI flag.
- **Verdict: adopt design.** Sweeps become generated overlay files under
  the run directory, never edits of the shipped tree. The config is locked
  and hashed into the result row (P6).

### 4.8 Evaluation

- **robomimic:** success rate over N rollouts.
- **vTitan:** separate predicates, several seeds, failure-set diffs in
  SQLite. That is already richer and follows plan principle 6 (a single
  score gets gamed).
- **Verdict: keep ours.**

### 4.9 Visualisation

- **MuJoCo:** a viewer with pause, single step and contact
  points/forces.
- **vTitan:** plans Foxglove `SceneUpdate` (4.8).
- **Verdict: adopt design.** Draw contact points, contact normals and the
  surface id in the Foxglove scene. Pause and step belong to the lockstep
  control (4.4).

### 4.10 Learning

- **robomimic:** mature behaviour cloning and offline RL.
- **vTitan:**
  - No learning problem is on the plan. "Randomized dynamics for RL" is
    explicitly not adopted.
  - The would-be demonstrations (real bags) come from the current
    hand-built policy, flaws included, and behaviour cloning copies them.
  - There are hundreds of runs, not the tens of thousands of transitions
    per task these methods expect.
  - Deployment would be Go on a Pi 5, which means an ONNX export and a
    new runtime path.
- **Verdict: skip.** Revisit only under the conditions in section 8.

### 4.11 3D

- **MuJoCo:** heightfields, ramps and meshes; a lighter 3D engine than
  Gazebo.
- **vTitan:** Gazebo is installed and used for vision data only. The
  reuse doc keeps "own simulator or Gazebo" for 3D maps.
- **Verdict: defer.** If a new rulebook needs 3D physics, MuJoCo is the
  stronger candidate for *physics* (determinism, state API, stepping
  control). Gazebo keeps the camera-image role. Decide then, not now.

## 5. Findings

1. **The Go sim passes through walls and pillars.**
   - `harness/gateway.go:233-236`: "Here the body always moves to the
     candidate pose". `collision.AllowedStep` (`allowed_step.go:49`) has no
     caller outside tests.
   - The Python oracle runs with `contact_slides_along_surfaces=true`.
   - Consequences:
     - Any Go-vs-Python parity comparison (ADR 0068) involving
       non-terminal contact compares different physics.
     - The Go transport sweeps (plan section 13) were run without contact
       response. Their contact and collided columns count overlaps, not
       blocked motion.
     - The reuse doc (section 1.4) says Go collision "slides along the
       surface": that is only true in Python.
2. **No snapshot/restore in either sim.** Knife-edge investigations can
   only replay from tick 0.
3. **Go sensor models are cleaner than Python's.** The Go sim lacks:
   - LIDAR chassis occlusion and self-returns
   - vision corruption beyond delay and drop
   - the default IMU error budget
   - tick jitter
   - the reverse-run rule 9.21

   Parity on the sighted corpus is therefore partly parity of an easier
   world.
4. **Go knobs are CLI-only.**
   - `sim-runner` has no `--seed` for native runs, and the transport and
     sensor-error knobs have no TOML.
   - A sweep result can't be reproduced from a config file alone.
   - The header comment of `cmd/sim-runner/main.go` still calls the tool a
     "Python oracle orchestrator", although it now also runs the native
     Go sim (the default is still `--runner python`).
5. **Python sweeps edit the shipped config in place.** A crash between
   the edit and the trap, or a parallel session, can leave a modified
   `src/config`.
6. **External:**
   - MuJoCo 3.5+ has native FIFO actuator and sensor delays that are part
     of the saved state. That is the semantics vTitan's transport layer
     already implements, and it is a good reference for making the delay
     queues part of a snapshot.
7. **External:**
   - robosuite's "delay" is a phase offset capped under one sampling
     period, not a latency. Do not copy its implementation.
   - robosuite's saved state is incomplete. Do not copy its snapshot
     scope either.
8. **External:**
   - MuJoCo has no Go bindings of any kind.
   - Its official car is a differential drive, and there is no maintained
     Ackermann or 4WS reference model.
   - A vTitan MuJoCo model would be ours, and should model the chassis
     rather than the wheels (P5).

## 6. What to adopt

| # | Pattern | Source | vTitan target | Plan link |
|---|---|---|---|---|
| A1 | Constraint-style contact response: cancel normal motion, keep tangential with friction | MuJoCo | Wire `AllowedStep` in the Go gateway; later a friction coefficient | fidelity, ADR 0068 parity |
| A2 | Model/data split | MuJoCo | `World` (immutable: track, obstacles, robot geometry, config) vs `WorldState` (poses, velocities, clock, RNG states, transport queues, tracker state) | 4.1, 4.4 |
| A3 | Full-state save/restore with a flag set | MuJoCo (`mjSTATE_INTEGRATION`) | `WorldState.Snapshot()/Restore()` including RNG streams and delay queues | new, with 4.4 |
| A4 | Two-way playback divergence check | robomimic | `sim-runner verify-determinism`: replay a sim bag by commands and by states, report the first diverging tick | 4.7 |
| A5 | Per-sensor pipeline descriptor (source → corrupter → filter → delay → rate) | robosuite (shape), MuJoCo (FIFO semantics) | TOML sensor profiles consumed by the harness; one seeded stream per sensor | 4.5 |
| A6 | Ray exclusion of the own body | MuJoCo (`bodyexclude`, `geomgroup`) | Model the chassis in the Go raycast, exclude it, add measured self-returns explicitly (ports Python's occlusion band) | 4.5, fidelity |
| A7 | Locked config plus sweep generator | robomimic | Sweeps as generated overlays, config hash in each result row | 4.9 workflow rule |
| A8 | Dataset filter keys | robomimic | Named tags over `other/data/live/runs` (direction, challenge, deploy commit, outcome) | phase 1 tooling |
| A9 | Arena / objects / robot composition | robosuite | `Map` interface parts | 6.1 |
| A10 | Contact visualisation | MuJoCo viewer | Contact markers in the Foxglove scene | 4.8 |
| A11 | Keyframes | MuJoCo | Named start states in the scenario (in-bay, section starts) | 4.6 |
| D1 | MuJoCo as an offline oracle | MuJoCo (dependency, Python `sim` env only) | Contact-fidelity spike | fidelity |

**Not adopting:**
- robosuite as a dependency (manipulation-centric, incomplete state,
  global RNG)
- robosuite's delay implementation
- robomimic and domain randomisation for training
- MuJoCo inside the Go corpus loop (no bindings; the cgo cost is not
  justified until P5 proves a gain)
- replacing MCAP with HDF5

## 7. Proposal

Ordered by value per effort. Each item has a "done when" in the style of
the platform plan. None of it changes navigation behaviour; P1 changes the
sim and therefore the corpus baseline.

### P1. Solid walls in the Go sim (days)

- Call `collision.AllowedStep` from `SimHardwareGateway.Advance`, behind
  a config key (`contact_slides_along_surfaces`, same name as Python).
  Record the surface it blocked on.
- Run the 256+256 corpus with the key off and on, and diff the outcome
  sets. Re-run the Go-vs-Python parity check with the key on in both.
- Add two **physics invariants** to the harness. They run every tick and
  fail the run as an "invalid sim" outcome, separate from the challenge
  predicates. This is the automated version of what JCIIOT's human
  reviewers had to catch (section 10):
  - **teleport:** per-tick displacement and rotation within what the
    kinematic limits allow for `dt` plus a tolerance
  - **penetration:** chassis-surface penetration depth above a tolerance
    outside a declared exemption
- Exemptions (grace windows, pillar-nudge budget) are declared in config
  with a reason, never a silent ignore list.
- **Done when:**
  - Go and Python agree on blocked-step behaviour on the committed
    fixtures.
  - The invariants pass on the whole corpus with the key on, and fail
    with it off.
  - The corpus delta is recorded.
  - The reuse doc section 1.4 is corrected.
- **Risk:** the Go baseline moves (the latency sweeps in section 13 would
  need a re-run). That is the point: today's baseline is optimistic in
  the wrong physics.

### P2. `WorldState` snapshot and restore (about a week)

- Split the harness into an immutable `World` and a `WorldState`. The
  state contains:
  - kinematic state and clock
  - every RNG stream position
  - transport and camera delay queues
  - contact tracker, pass-side and lap bookkeeping
  - the navigator's own state, via a `Snapshot` interface on the
    navigator (the hard part: the navigator must expose its state or be
    re-creatable from it)
- Write it as a `/sim/state` MCAP channel every N ticks, with the scenario
  JSON and the resolved config as MCAP attachments.
- **Done when:** `sim-runner --resume <bag> --at-tick N` continues a run
  bit-identically to the original from tick N (checked by P3). A knob can
  be changed at resume to branch.

### P3. Determinism check (days, after P2)

- Port robomimic's playback idea:
  - replay a sim bag by re-applying `/ackermann_cmd` from its recorded
    initial state
  - compare every tick's pose with the recorded pose
  - print the first diverging tick and the error
- Run it in CI on one Open and one Obstacles fixture.
- **Done when:** CI fails on any nondeterminism, for example from map
  iteration order or an unseeded RNG.

### P4. Sensor and transport profiles in TOML (about a week)

- One descriptor per sensor (LIDAR, IMU, camera detections, command
  link):
  - source rate
  - corrupter (noise, dropout, bursts)
  - filter
  - FIFO delay with distribution and tail
  - seed salt
- Store descriptors under `src/config/navigation/simulation/profiles/`,
  schema-validated. Keep the CLI flags as overrides.
- Port Python's measured models into Go:
  - LIDAR occlusion and self-returns via chassis modelling (A6)
  - vision colour flips, bearing scatter, range falloff and confidence
    quantiles
  - IMU budget on by default
  - tick jitter
- Add `--seed` to native runs.
- **Done when:**
  - `nominal` and `measured` profiles both run in CI.
  - Every value cites its bag or bench measurement (principle 2).
  - Go's sensor realism matches Python's, making the ADR 0068 comparison
    fair.

### P5. MuJoCo contact-fidelity spike (time-boxed, 1-2 weeks)

- **Purpose:** learn what correct contact response looks like for this
  chassis, *then* decide whether a friction-aware rule in Go is enough.
- **Where:** Python `sim` pixi environment only. `mujoco` is a ~27 MB
  wheel with light dependencies; no Go change.
- **Model (MJCF):**
  - **Chassis:** a 0.30 × 0.194 m box, 1.3 kg, on planar joints
    (`slide x`, `slide y`, `hinge z`). Do not model the wheels: wheel
    contact models are the fragile part of MuJoCo cars, and the official
    car is differential drive anyway.
  - **Drive:**
    - Each control tick, the existing 4WS kinematics compute the desired
      body velocity.
    - `velocity` actuators on the three joints track it, with a
      `forcerange` equal to the measured drive and stall limits.
    - When the chassis presses on a surface the force saturates and the
      solver resolves the motion (slide, rotate, push).
    - Template: robosuite's `null_mobile_base.xml` uses exactly this
      shape. It has joints `joint_mobile_forward`/`side` (slide) and
      `joint_mobile_yaw` (hinge) with `frictionloss=250`, and `velocity`
      actuators with `kv=1000` and `forcerange=±600`. Those values are
      sized for a Tiago; ours must come from the 1.3 kg car's measured
      stall force.
    - Never write the chassis `qpos` directly. JCIIOT's direct mode does
      that, and it broke physical carrying and needed a pinning hack
      (section 10).
  - **Walls:** static boxes from `trackmodel`.
  - **Pillars:** free boxes 50 × 50 × 100 mm. Their mass and friction on
    the mat must be **measured** (not known today).
  - **Contact:** `condim=3`, elliptic cone, `implicitfast`, timestep
    0.002 s (25 substeps per 20 Hz tick).
- **Experiments**, against bags and existing diagnostics
  (`diag_bag_fidelity_axes.py`, `sim/diag_fidelity_axes.py`):
  1. Wall slide progress at 10-30 deg approach (the "56x less progress"
     finding).
  2. Yaw gained during escape episodes versus the hardware (the 40-50%
     under-rotation).
  3. Pillar displacement per contact versus the scored 59.4 mm derived
     displacement.
  4. The reverse pivot at about 1 rad/s.
- **Deliverable:** a short report with fitted `friction`/`solref` values
  and one decision:
  - (a) the gap closes with a simple friction coefficient added to
    `AllowedStep` → port that rule to Go and stop there
  - (b) it needs real dynamics → evaluate a cgo layer or a MuJoCo side
    process
  - (c) MuJoCo does not close it either → the gap is elsewhere (drive
    model, servo), drop the approach
- **Done when:** the report exists and the decision is recorded. Stop at
  the time box whatever the state.

### P6. Sweeps as generated overlays (days)

- The sweep generator writes one overlay TOML per arm into the run
  directory, runs against `--config-root` plus overlay, and never edits
  `src/config`.
- The resolved config is hashed and stored in the SQLite row (extend
  `src/tools/sweep_results.py`). Unknown keys are rejected (robomimic's
  `lock_keys`).
- **Done when:** the Python `sed` sweeps are migrated, and the shipped
  tree is untouched during a sweep (checked with `git status` in the
  script).

### P7. Run tags / filter keys (days)

- A small index file per run directory (or one inventory file) with tags:
  - challenge, direction
  - deploy commit
  - hardware profile
  - outcome
- The classification scripts write it once. Diagnostics accept `--tag`
  instead of path lists.
- **Done when:** the fidelity scripts select their run sets by tag.

### Later, conditional

- **A9 arena / objects / robot composition** with the `Map` interface
  (6.1).
- **A10 contact markers** with the Foxglove scene (4.8).
- **MuJoCo for 3D physics** only if a new rulebook adds ramps or 3D
  terrain, and only after P5 has shown MuJoCo matches this chassis.

## 8. When to revisit robomimic

All of these must hold:
- A rulebook or goal where a hand-built behaviour is clearly the
  bottleneck and hard to specify: for example the K-turn/escape family,
  which memory shows resisting parameter fixes (multiple refuted arms).
- A sim whose contact physics is trusted (P1, and P5's outcome).
- P2 and P4 in place, so demonstrations can be generated in sim at scale
  with measured noise, not only taken from the few hundred real bags.
- An agreed deployment path: ONNX in Go on the Pi 5 CPU. A small MLP is
  cheap; a diffusion policy likely is not at 20 Hz next to vision.

The narrowest sensible experiment would then be a behaviour-cloned
**escape manoeuvre**, gated by the same corpus predicates as any other
change. Until then, robomimic's value to this project is its design
patterns (A4, A7, A8).

## 9. Risks and costs

- **P1 moves every Go baseline.** Plan the re-run of the latency sweeps
  (plan section 13) right after.
- **P2's hard part is the navigator.** Snapshotting the sim world is
  mechanical; snapshotting the navigator's internal state (maps, lap
  detector, escape machines) requires every stateful component to expose
  or rebuild it. A fallback is "resume by deterministic replay to tick N":
  it costs the prefix time but needs no navigator change, and P3 already
  guarantees it is exact.
- **P5 can eat time.** MuJoCo contact tuning is open-ended. The time box
  and the three-way decision are the guard. Measuring the pillar mass and
  mat friction is a prerequisite, not an optional extra.
- **MuJoCo's API moves.**
  - Ray signatures changed in 3.5.
  - Margin semantics changed in 3.9.
  - Releases arrive every 2-3 weeks.
  - Pin the version in the `sim` environment.
- **Determinism caveats.** MuJoCo is bit-exact only per version and
  architecture. vTitan's own sim is pure Go with seeded PCG streams and
  should stay that way; that is a reason to keep MuJoCo out of the
  corpus loop.

## 10. Case study: the JCIIOT 2026 competition platform

Repository: https://github.com/JCIIOT2026/JCIIOT2026 (folder `JCIIOT`),
studied at commit `48ab492` (2026-09-11). It is the official platform for
the JCIIOT 2026 "RunningRobot" competition and the only real-world project
found that uses **MuJoCo + robosuite + robomimic together**, so it shows
what the three look like as an integrated stack.

### 10.1 What it is

- **Task.** A simulated Tiago mobile manipulator in five factory scenes
  (L1-L5, worth 10/15/20/25/30 points) picks a material bin from an input
  station and places it at an output station.
- **Pipeline.** A natural-language prompt goes to an LLM planner (Ollama
  or an OpenAI-compatible API). The planner produces JSON steps validated
  against a skill registry, and skills drive the simulation. Standard
  operating procedure (SOP) Word documents are turned into a Markdown
  knowledge base with a VLM.
- **Skills** (the only code contestants may change, besides
  `robot_params.json`):
  - `move`: A* navigation
  - `pick_up`: BC grasp plus lift verification
  - `place_down`: turn, lower, release
  - `record_trajectory`: writes the JSON the scorer reads
- **Repository shape.**
  - 1.8 GB and about 5,800 files.
  - Forked copies of robosuite (with five per-level environment files of
    about 1,700 lines each) and robomimic, with model checkpoints in Git
    LFS.
  - A 3,274-line Streamlit `app.py` that also contains the scorer.

### 10.2 How it uses each library

- **MuJoCo:**
  - The physics and rendering engine.
  - Scene timestep 0.005 s.
  - Two cameras: `birdview` for navigation replay, and
    `robot0_robotview` for the 128 × 128 policy input.
- **robosuite:**
  - Environments (`FactorySorting*` on `ManipulationEnv`).
  - The Tiago robot with a planar mobile base (see P5 for its joints and
    actuators).
  - OSC controllers.
  - `DataCollectionWrapper` for demonstrations.
- **robomimic:**
  - A BC grasp policy (`model_epoch_150/500.pth`) on image and low-dim
    observations.
  - Demonstrations come from a **scripted** collector
    (`load_factory_sorting_1_3fo3erfhisem_collect.py`):
    - it uses privileged object-site positions and OSC delta control
    - 20 rollouts by default
    - only successful episodes are written to robomimic HDF5
  - Only the L1 collector is provided.
- **Their own code** (not from the libraries):
  - scene JSON → occupancy grid with rotated rectangles inflated by robot
    radius plus safety margin → A* → path simplification (`get_map.py`,
    `core/navigation.py`)
  - a P-controller path follower

### 10.3 Findings

1. **Direct pose writes plus logged-only collisions, the same defect as
   our Go sim.**
   - `navigation.drive_mode` ships as `"direct"`. `_follow_path_direct`
     (`environments/robosuite_backend.py`) adds up to
     `max_linear / control_freq` to the base `qpos` each step and calls
     `sim.forward()`. The physics never gets a chance to block the base.
   - Collisions are read from `sim.data.contact`, printed, and then
     "navigation continues".
   - Contacts with the floor and with any geometry whose name contains
     `table_top`, `conveyor`, `shelf`, `container`, `tote`, `cardbox` or
     `plastic_crate` are ignored entirely.
   - The velocity-actuator mode (`"action"`) exists but is not the
     default.
2. **Direct writes break physics elsewhere, and a hack papers over it.**
   A free object held by the grippers does not follow a base whose `qpos`
   is overwritten. `transport_attachment.py` therefore pins the carried
   object's free joint at a fixed offset from the base during transport.
   One kinematic shortcut forced a second one.
3. **The scorer trusts an artefact written by the code under test.**
   - `_score_steps` (`app.py`) reads the last frame of the trajectory
     JSON produced by the contestant's pipeline, plus a `grasp_end` event,
     not the simulator state.
   - The rules: 50% for moving the object more than 1 m from the source,
     50% for ending within 0.8 m of the target table, −5 per collision.
   - A code comment gives different weights (30/30/40); unresolved.
4. **The automatic score saturated, and humans had to review the runs.**
   - 15 of the 20 listed teams scored 100/100 on the program. The ranking
     was decided by a manual "process score".
   - That review deducts 5 points per collision, the whole level for
     "teleportation or passing through walls", and half the level for
     "picking up objects without proper contact".
   - Those are exactly the physics violations that findings 1-3 let
     through automatically.
5. **The BC grasp only works where it was trained.**
   - The grasp does not run in the navigation environment. A **fresh**
     environment is built at the navigation pose, the policy runs there,
     and the objects are synced back (`grasp_object_physics`,
     `_sync_objects`).
   - The yaw is forced from `task_config.json`'s trained grasp poses
     whatever the navigator achieved, because the policy was trained from
     those poses.
   - The policy imitates a scripted teacher that had privileged state.
6. **The first-prize repository is named `jciiot2026_bc`.** Winners seem
   to have invested in the learned grasp, which is where the platform
   left room. (The winners' repositories were not studied.)

### 10.4 What vTitan takes from it

| Lesson | Action |
|---|---|
| Direct pose writes and logged-only contact produce runs that look perfect to the scorer (findings 1, 4) | P1 gains teleport and penetration invariants that fail the run automatically, so no human review is needed to catch it |
| Silent substring ignore lists hide contacts (finding 1) | Contact exemptions are declared in config with a reason and counted in the result, never dropped |
| One kinematic shortcut forced another (finding 2) | In any physics model (P5), drive through actuators and never write body poses |
| A scorer that reads the stack's own output can be fooled (finding 3) | Scoring stays in the harness on ground truth. When the stack moves out of process for SITL (4.1), the **world process** owns scoring and the stack cannot write to it |
| A binary score saturates and forces subjective tie-breaks (finding 4) | Keep separate predicates over several seeds (principle 6). Also report continuous margins per run (minimum clearance, contact time, time to finish) so that "all passed" still ranks |
| Planar joints with force-limited velocity actuators are the standard mobile-base model | Template for P5 (parameters to be re-measured for our car) |
| BC generalises only near its training poses and needs a separate environment (finding 5) | Confirms section 8. If ever tried, the pattern is a privileged scripted teacher in sim → a sensor-only student, evaluated off the training distribution |
| Vendored library forks, per-level copy-paste and a scorer inside the UI make a 1.8 GB repo that is hard to review | Keep ours: no vendored forks, one generic scenario format, scoring as a tested package |

Not relevant to vTitan: the LLM planner, the SOP/VLM knowledge base and
the Streamlit UI.

## 11. Sources

MuJoCo:
- Simulation and stepping: https://mujoco.readthedocs.io/en/latest/programming/simulation.html
- API functions (`mj_step1/2`, `mj_ray`, `mj_multiRay`, state): https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html
- Types (`mjtState`): https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html
- Computation (contact, cones, integrators, reproducibility): https://mujoco.readthedocs.io/en/latest/computation/index.html
- Modelling (sensors, delays, contact parameters): https://mujoco.readthedocs.io/en/latest/modeling.html
- XML reference: https://mujoco.readthedocs.io/en/latest/XMLreference.html
- Model editing (`mjSpec`): https://mujoco.readthedocs.io/en/latest/programming/modeledit.html
- Python bindings and viewer: https://mujoco.readthedocs.io/en/latest/python.html
- Changelog: https://mujoco.readthedocs.io/en/latest/changelog.html
- Official car model: https://github.com/google-deepmind/mujoco/blob/main/model/car/car.xml
- README (bindings list): https://github.com/google-deepmind/mujoco/blob/main/README.md
- MuJoCo Warp: https://github.com/google-deepmind/mujoco_warp
- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground
- MuSHR MuJoCo: https://mushr.io/tutorials/mujoco/

robosuite:
- Docs: https://robosuite.ai/docs/
- Observables: https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/utils/observables.py
- Placement samplers: https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/utils/placement_samplers.py
- Base environment: https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/environments/base.py
- State bindings: https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/utils/binding_utils.py
- Wrappers: https://github.com/ARISE-Initiative/robosuite/tree/master/robosuite/wrappers
- License: https://github.com/ARISE-Initiative/robosuite/blob/master/LICENSE

robomimic:
- Algorithms: https://robomimic.github.io/docs/introduction/implemented_algorithms.html
- Datasets: https://robomimic.github.io/docs/datasets/overview.html
- Configs: https://robomimic.github.io/docs/modules/configs.html
- Results and rollouts: https://robomimic.github.io/docs/tutorials/viewing_results.html
- Scripts: https://github.com/ARISE-Initiative/robomimic/tree/master/robomimic/scripts
- `EnvBase`: https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/envs/env_base.py
- Install: https://robomimic.github.io/docs/introduction/installation.html

JCIIOT 2026 (at `48ab492`):
- Repository and leaderboard: https://github.com/JCIIOT2026/JCIIOT2026
- Platform code: https://github.com/JCIIOT2026/JCIIOT2026/tree/master/JCIIOT
- Files read: `JCIIOT/README.md`, `JCIIOT/app.py` (`_score_steps`),
  `JCIIOT/src/robot_agent/environments/robosuite_backend.py`,
  `JCIIOT/src/robot_agent/core/navigation.py`,
  `JCIIOT/robosuite/robosuite/environments/factory_sorting/{get_map,transport_attachment,load_factory_sorting_1_3fo3erfhisem_collect}.py`,
  `JCIIOT/robosuite/robosuite/models/assets/bases/null_mobile_base.xml`,
  `JCIIOT/knowledge/robot_params.json`

vTitan (at `35316a96`):
- `src/go/internal/sim/harness/gateway.go`, `transport.go`, `config.go`
- `src/go/internal/sim/collision/allowed_step.go`, `track_model.go`
- `src/go/internal/sim/scenario/native_runner.go`, `contact_tracker.go`, `record.go`
- `src/python/src/simulation/collision_stepping.py`, `simulated_hardware_gateway.py`, `scenario_simulator/scoring.py`
- `other/docs/adr/0068-go-parallel-track-single-cutover.md`
