# Sensor / Topic Verification Runbook

Progressive, per-sensor validation of `platform/robot`'s ROS2 topics on real hardware, done
**before** running the full robot (state machine + all nodes together). Validating each sensor in
isolation catches bad readings that would otherwise get masked once everything runs together.

## Order and rationale

Test passive/observation-only sensors first, motion-capable ones next, and the most
setup-intensive one (vision) last regardless of its own passive/active nature — it needs more
work to stand up (Hailo runtime, model file, framing a target) than the other passive sensors, so
there's no benefit blocking on it before everything cheaper to test is already validated:

1. **`/imu/data`** (BNO08x, Pi 5) — passive, no motion required.
2. **`/scan`** (Slamtec C1 LIDAR, Pi 5) — passive.
3. **`/button/event`** (Pi Zero) — physical button press.
4. **`/ui/oled_mirror`** (Pi Zero) — visual check against the physical display.
5. **Motor/encoder topics** (Pi Zero) — first real motion risk. Do this with wheels off the
   ground / robot secured.
6. **State machine integration** (`/robot_state`, `/ackermann_cmd`, `/system_status`,
   `/race_metrics`) — only after 1–5 pass individually.
7. **Telemetry bridge → backend/frontend** — confirm the dashboard reflects live values
   end-to-end. Note: this exercises the `/hailo/detections` forwarding path too, but without real
   vision data behind it until phase 8 passes — re-check this once vision is validated.
8. **`/hailo/detections`, `/hailo/fps`** (vision, Pi 5) — passive, but deferred to last given the
   extra setup work (Hailo runtime, model file, framing a target in view).

Each node also has an isolated `pixi run -e dev run-<node>` task (see `pixi.toml`) that runs it
standalone, bypassing systemd and the rest of the node graph — use this when a topic looks wrong
and you need to debug that node alone.

## Prerequisites (per Pi, easy to forget on a fresh checkout)

- **`ros2_ws` must be built**: `~/.pixi/bin/pixi run -e dev build-ws`. Symptom if missing:
  `bash: ros2_ws/install/setup.bash: No such file or directory` when running any `run-<node>`
  task. A fresh `git clone` has `ros2_ws/src` but no `install/` until this runs.
- **`.env` must exist**: `cp platform/robot/.env.example platform/robot/.env` (per-Pi, gitignored,
  not templated by any task). Symptom if missing: pydantic `ValidationError` for whatever field
  the driver's `Config` needed first. Nothing auto-loads `.env` unless the entry point imports
  something that pulls in `src/logger/config.py` or `src/env.py` — `EnvironmentFile=-.../.env` in
  the systemd units is belt-and-suspenders for the same reason.
- Check `systemctl is-active voldemorbot-pi5.service` / `-pi-zero.service` before assuming a node
  is running — a Pi can have the repo cloned and built but the service never installed/enabled.

## General SSH/shell gotchas hit during testing

- **`pkill -f pattern` can kill itself.** If `pattern` is a literal substring of the `pkill`
  command's own invocation (e.g. `pkill -f sllidar` run as `bash -c "pkill -f sllidar; ..."`),
  `-f` matches against the full command line — including its own — and the whole SSH session dies
  before printing anything (looks exactly like a network drop: exit 255, no output). Use the
  bracket trick to avoid self-matching: `pkill -f '[s]llidar'`.
- **`cd dir && cmd &` only changes directory inside the backgrounded subshell.** The parent
  shell's CWD is unaffected, so a *second* backgrounded command later in the same SSH invocation
  needs its own explicit `cd` — it does not inherit the first command's directory change.
- This particular Pi's WiFi link drops mid-session often enough that a command can fail with exit
  255 for no reason related to the command itself. If a command that worked moments ago suddenly
  returns exit 255 with zero output, retry once or twice before assuming it's a real bug.

## Phase 1 — `/imu/data` (BNO08x UART RVC, Pi 5)

### Bugs found and fixed

1. **`ros2_ws` never built on Pi 5** — see prerequisite above. Not a code bug, just a missing
   provisioning step; `build-ws` fixed it.
2. **`Driver.__init__` config bug** (`src/hardware/imu/bno08x/mcp2221/uart_rvc.py`): built the
   inner `UARTRVCConfig` from raw ternary fallbacks (`quaternion=config.quaternion if config else
   None`) instead of resolving `config = config or Config()` first, like every other driver in
   `src/hardware/*/`. `QuaternionConfig` has no valid `None` state, so the node crashed on startup
   with a pydantic `ValidationError` before ever reading the `BNO08X_UART_RVC_QUATERNION__*` env
   vars. Fixed in commit `be6e6e0`.

### Validation method: blind rotation test

Rate (`ros2 topic hz`) and a single `ros2 topic echo --once` only prove the topic is alive — they
don't prove the orientation math is *correct*. The useful test is a **before/after snapshot around
a known physical rotation**, checked two ways:

1. **Angle/axis check**: capture a quaternion, physically rotate the robot by some amount (a
   known amount if validating, an unknown amount if double-checking — see below), capture another
   quaternion, then compute the relative rotation:
   ```python
   from scipy.spatial.transform import Rotation as R
   rel = R.from_quat(q2) * R.from_quat(q1).inv()
   rotvec = rel.as_rotvec(degrees=True)   # axis * angle, in degrees
   ```
   For a flat spin (yaw only), `rotvec` should be dominated by the Z component, with magnitude
   matching the physical rotation.
2. **Physics cross-check (stronger)**: gravity is fixed in the world frame, so rotating each
   snapshot's `linear_acceleration` into the world frame with its own quaternion should give the
   *same* vector both times:
   ```python
   R.from_quat(q1).apply(a1)  # should ≈ R.from_quat(q2).apply(a2)
   ```
   This catches inconsistencies the angle/axis check alone can miss (e.g. it would have caught a
   real axis-mapping bug even without knowing the "expected" axis in advance).
3. Cross-check the **raw** yaw/pitch/roll too, not just the derived quaternion, using the driver
   directly (bypasses ROS and the topic layer entirely):
   ```python
   from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver
   d = Driver(); d.connect(); d.start_polling()
   data = d.get_data()   # data.yaw_deg / .pitch_deg / .roll_deg / .quaternion
   ```
   If raw yaw swings ~N° while pitch/roll stay near their own baseline noise, and the quaternion
   delta computed from the *live* driver output is Z-dominant with magnitude ≈ N°, the conversion
   is validated end-to-end.

**A "blind" variant is useful**: have a second person rotate the robot by an amount they don't
tell you, then calculate the angle from the IMU data alone and have them confirm it against what
they actually did. This caught nothing wrong here (one anomalous early result didn't reproduce on
retest — likely the robot wasn't fully settled that one time), but it's a good trust-but-verify
habit before relying on this data for navigation.

### Live-streaming test scripts don't work over this SSH setup

Don't use a script that prints a countdown then streams readings for N seconds while asking the
other person to react to a "GO" printed mid-script — if you (the one running the script over SSH)
are the one watching the terminal, the other person **can't see that output live** and has no way
to time their action against it. Every attempt at this (5s buffer, 15s buffer, countdown, even
unbuffered `python3 -u`) produced zero captured motion — the window always elapsed before they
could react to a signal they never actually saw.

**Use the before/after snapshot method instead**: capture a reading, ask them to move it and say
"done" whenever ready (no time pressure), capture a second reading. This is what actually worked.

### Known limitation to keep in mind

BNO08x RVC mode computes yaw/pitch/roll internally via Euler angles, which are subject to gimbal
lock near ±90° pitch (yaw and roll become coupled/ambiguous). If a future test shows a rotation's
magnitude captured correctly but landing on the wrong axis, check whether pitch approached ±90°
during the motion before assuming it's a code bug — verify with the raw-angle driver script above.

## Phase 2 — `/scan` (Slamtec C1 LIDAR, Pi 5)

### Bugs found and fixed

1. **`sllidar_ros2` never fetched on Pi 5** — the third-party driver package isn't part of this
   repo; it's fetched into `ros2_ws/src` via the `fetch-lidar-driver` pixi task (idempotent),
   which must run once **before** `build-ws`. Symptom if skipped: `ros2_ws/src/` only has
   `voldemorbot_robot`, and the LIDAR launch fails looking for the `sllidar_ros2` package /
   `sllidar_node` executable.
2. **Three different launch paths used three different `frame_id` values** for the same physical
   sensor: the standalone path (`run-lidar` pixi task / `voldemorbot-lidar.service`, which launches
   the third-party `sllidar_ros2/launch/sllidar_c1_launch.py` directly) defaulted to `laser`; the
   integration path (`voldemorbot_robot/launch/lidar_launch.py`, included by
   `wro_state_machine_launch.py`) hardcoded `laser_frame`; the static TF (below) published a
   transform into `lidar_link`. None of these matched, so TF lookups for the actual published scan
   frame would have failed regardless of which path was running. Standardized all three on
   `lidar_link` (matching the URDF/SDF naming convention already used for `camera_link`/`imu_link`).
3. **`static_tfs.launch.py` existed in source but was never installed** — missing from
   `setup.py`'s `data_files`, so `ros2 launch voldemorbot_robot static_tfs.launch.py` failed with
   *"file 'static_tfs.launch.py' was not found in the share directory"* — and critically,
   `wro_state_machine_launch.py` also includes it via `get_package_share_directory(...)`, so the
   **full integration bringup was silently missing all sensor-frame transforms**, not just the
   standalone LIDAR test. Added to `data_files` and rebuilt.
4. **`static_tfs.launch.py` used the wrong `static_transform_publisher` CLI convention for this
   ROS2 distro.** It passed positional arguments (`arguments=["0.14", "0", "0.10", "0", "0", "0",
   "base_link", "camera_link"]`), but this distro's `static_transform_publisher` only accepts
   **named** arguments (`--x`, `--yaw`, `--frame-id`, `--child-frame-id`, etc. — confirmed via
   `ros2 run tf2_ros static_transform_publisher --help`). The old positional form would have failed
   outright once install was fixed. Rewritten to use named args.
5. **C1 mounted inverted** — needs two independent corrections, not one:
   - **180° yaw offset** on the `base_link -> lidar_link` static TF (a pure rotation) — fixes
     where "front" lands. Made configurable via `LIDAR_YAW_OFFSET_DEG` (default `180`) in `.env`,
     read through `src.env.EnvVar` (same mechanism every pydantic-settings driver already uses —
     the launch file just imports it, which triggers `load_dotenv()` as a side effect).
   - **`inverted:=true`** on the `sllidar_node` launch parameter (both `run-lidar`'s pixi task and
     `lidar_launch.py`'s `Node` parameters) — fixes a **left/right mirror** that the yaw offset
     alone cannot correct. A pure Z-axis rotation can shift where front/back land but can never
     swap left and right; that requires a reflection, which is exactly what mounting a 2D LIDAR
     upside-down causes and exactly what the driver's own `inverted` flag exists to correct.
     Discovered by validating against real measured distances in four directions (see below) —
     front/back matched immediately with just the yaw offset, but left and right came out swapped
     until `inverted:=true` was added.

### Validation method: measure real distances in known directions

Rate (`ros2 topic hz`, ~10 Hz for the C1's Standard mode) and range plausibility (values within
`range_min`/`range_max`, `.inf` for no-return) only prove the topic is alive with sane-looking
numbers — they don't prove the angle mapping is correct. The useful test: have someone stand next
to the robot, tell you the actual measured distance to a wall/object in each of front/left/right/
back, then compare against the scan data bucketed into narrow sectors around each of those
directions (0°/+90°/-90°/±180° in the frame you expect to be robot-relative):

```python
# after applying whatever yaw offset the static TF applies, to get robot-frame angles:
base_angle = ((raw_angle_deg + yaw_offset_deg + 180) % 360) - 180
```

This is what actually caught the left/right mirror bug above — a structural check (rate, range
bounds, "front has the biggest numbers") would have passed even with left/right swapped, since
swapping two sectors doesn't break any of those invariants. Only checking against real,
independently-known distances in specific directions surfaces a mirror/reflection bug.

### Self-occlusion from the robot's own chassis

Some angular range will read implausibly short distances that are the robot's own parts, not real
obstacles — figure out which range by physically clearing everything else out of range (or
knowing the true distances are large) and treating anything under some threshold (25cm here, since
the real walls were never that close) as self-occlusion. It came out as **~73 scattered small
clusters** (2–10 points each, likely thin structural elements — wiring, brackets, screw heads) —
not one solid block — spanning from about **+117° to +172°** on one side and **-122° to -177°** on
the other (i.e., concentrated around the rear, symmetric about 180°/back). Don't try to mask each
tiny cluster individually (fragile, overfits to one measurement — thin parts can clip a different
exact point on a re-scan); mask the full contiguous extent with margin instead, e.g. exclude
`abs(angle) > 115°` as a single check. Front/left/right (everything within ±115°) had zero
readings under the threshold.

Note the two rearward clusters don't quite meet at dead-back — they stop at `172.2°` and start at
`-176.7°`, leaving an ~11° window (`172°` through `180°` to `-177°`) with no recorded occlusion in
this particular scan. Mask through that gap too rather than treating it as a confirmed-clear
notch: the occlusion pattern is already sparse (thin parts, not a solid panel), so a gap between
two clusters more likely means a thin element missed a ray at that exact angle than a genuinely
reliable clear sightline — a re-scan could easily register a point there instead. A single
contiguous `abs(angle) > 115°` mask already covers this for free; don't special-case a "clear"
notch out of it.

## Phase 3 — `/button/event` (Pi Zero)

### Bugs found and fixed

1. **`get_state()` never reset `_last_event` after reading it**, in both the GPIO driver
   (`src/hardware/button/gpio/driver.py`) and the MCP2221 variant
   (`src/hardware/button/mcp2221/driver.py`). `button_node._poll()` runs at 20 Hz and publishes
   whenever `state.last_event` is truthy — since the field was never cleared, the very first real
   event would have republished forever, once per poll, flooding `/button/event` with the same
   stale message indefinitely. Fixed by having `get_state()` read-and-clear the field atomically
   under the driver's lock (the GPIO driver's lock existed but was never actually used to protect
   this state — also added locking around the `_on_pressed`/`_on_released` callbacks themselves,
   since without it a callback could write a new event in the narrow window between `get_state()`'s
   read and its clear, silently losing that event).
2. **`BUTTON_GPIO_PIN` was configured for GPIO 17, but the button is physically wired to GPIO 4.**
   `gpiozero.Button(17)` connects with no error — nothing about a missing physical connection
   raises an exception — so the driver reports "Button connected successfully" and the node runs
   fine forever, just never sees a real edge transition no matter how many times the physical
   button is pressed. This is the same class of bug as Phase 1's `Config` issue in spirit (silent
   "success" hiding a real problem), but a hardware-wiring mismatch instead of a code defect.
   Fixed by changing `BUTTON_GPIO_PIN` from `17` to `4` in `.env`/`.env.example`.

   Notably, `setup-pi-architecture.md` (marked "historical design spec — partially superseded")
   had GPIO 4 listed all along — its header explicitly calls out *"Pin assignments and hardware
   architecture below are still accurate"* even though other parts of that doc have drifted from
   the real implementation. Don't discount a whole doc as stale just because part of it is —
   check what it specifically claims is still current. The current `.env.example`, despite being
   live code rather than a doc, was the one that had actually gone stale here.

### Validation method: escalating bisection through the stack

For a discrete, momentary event (as opposed to a continuous stream like `/imu/data` or `/scan`),
a background listener with a generous window (60s) and "press whenever ready, tell me when done"
worked better than trying to synchronize a live countdown — same lesson as Phase 1's
live-streaming problem, just with a longer passive window instead of a snapshot pair.

When the full `/button/event` topic kept showing zero events even after both fixes above, the
useful technique was bisecting top-down through the stack, one layer at a time, until finding
where the signal actually disappeared:

1. **Raw GPIO read**, bypassing the driver class entirely — plain `gpiozero.Button(pin).is_pressed`
   polled in a loop. Confirms the physical wiring and pin number are actually correct.
2. **The driver class directly**, bypassing ROS/`button_node` — instantiate `Driver()`, call
   `.connect()`, poll `.get_state()` in a loop. Confirms the driver's callback registration and
   event/state logic work correctly in isolation.
3. **The full ROS topic**, via `ros2 topic echo` — confirms the node wrapper and publish path
   work end-to-end.

In this case, layers 1 and 2 both worked cleanly on the first proper attempt, and a subsequent
retest of layer 3 also succeeded — meaning the earlier "zero events" results were most likely
timing misses (the same "did I actually press it inside the window" issue seen throughout this
runbook), not a real remaining bug. The bisection was still valuable: it positively confirmed the
wiring and the driver logic before writing off the ROS layer as broken, rather than guessing.

Don't conclude a topic is broken from one "zero events" result on a momentary/discrete signal —
unlike a continuous stream (where you can just wait longer), a single missed press looks identical
to a real failure. Retry at least once, and bisect down a layer if it keeps failing, before
treating it as a genuine bug.

## Template for future phases

For each new sensor, document here: what bugs were found, what prerequisite gaps existed, and
what validation method actually proved the data correct (not just "topic exists and publishes").
