# Sensor / Topic Verification Runbook

Progressive, per-sensor validation of `src`'s ROS2 topics on real hardware, done
**before** running the full robot (state machine + all nodes together). Validating each sensor in
isolation catches bad readings that would otherwise get masked once everything runs together.

## Order and rationale

Test passive/observation-only sensors first, motion-capable ones next, and the most
setup-intensive one (vision) last regardless of its own passive/active nature - it needs more
work to stand up (Hailo runtime, model file, framing a target) than the other passive sensors, so
there's no benefit blocking on it before everything cheaper to test is already validated:

1. **`/imu/data`** (BNO08x, Pi 5) - passive, no motion required.
2. **`/scan`** (Slamtec C1 LIDAR, Pi 5) - passive.
3. **`/button/event`** (Pi Zero) - physical button press.
4. **`/ui/oled_mirror`** (Pi Zero) - visual check against the physical display.
5. **Motor/encoder topics** (Pi Zero) - first real motion risk. Do this with wheels off the
   ground / robot secured.
6. **State machine integration** (`/robot_state`, `/ackermann_cmd`, `/system_status`,
   `/race_metrics`) - only after 1–5 pass individually.
7. **Telemetry bridge → backend/frontend** - confirm the dashboard reflects live values
   end-to-end. Note: this exercises the `/hailo/detections` forwarding path too, but without real
   vision data behind it until phase 8 passes - re-check this once vision is validated.
8. **`/hailo/detections`, `/hailo/fps`** (vision, Pi 5) - passive, but deferred to last given the
   extra setup work (Hailo runtime, model file, framing a target in view).

Each node also has an isolated `pixi run -e dev run-<node>` task (see `pixi.toml`) that runs it
standalone, bypassing systemd and the rest of the node graph - use this when a topic looks wrong
and you need to debug that node alone. Exception: `run-vision` needs `-e vision` instead of `-e
dev` - ultralytics/hailort are feature-gated out of `dev` so the Pi Zero's environment doesn't
have to install them (see `pixi.toml`'s `[feature.vision...]` sections).

## Prerequisites (per Pi, easy to forget on a fresh checkout)

- **`ros2_ws` must be built**: `~/.pixi/bin/pixi run -e dev build-ws`. Symptom if missing:
  `bash: ros2_ws/install/setup.bash: No such file or directory` when running any `run-<node>`
  task. A fresh `git clone` has `ros2_ws/src` but no `install/` until this runs.
- **`.env` must exist**: `cp src/.env.example src/.env` (per-Pi, gitignored,
  not templated by any task). Symptom if missing: pydantic `ValidationError` for whatever field
  the driver's `Config` needed first. Nothing auto-loads `.env` unless the entry point imports
  something that pulls in `src/logger/config.py` or `src/env.py` - `EnvironmentFile=-.../.env` in
  the systemd units is belt-and-suspenders for the same reason.
- Check `systemctl is-active vtitan-pi5.service` / `-pi-zero.service` before assuming a node
  is running - a Pi can have the repo cloned and built but the service never installed/enabled.
- **Passwordless `sudo` must be set up per Pi** before the service unit files can be installed or
  managed remotely (`systemctl enable`/`start`/`restart` need root, and there's no way to supply a
  password over a non-interactive SSH session). Raspberry Pi OS's own imager sets this up for the
  default user automatically on first boot (`/etc/sudoers.d/010_pi-nopasswd`), but it isn't
  guaranteed on every image/provisioning path - verify with `sudo -n true` (silent exit 0 = already
  passwordless). If it prompts for a password, set it up once interactively:
  ```bash
  echo 'ralvarezdev ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/010_pi-nopasswd
  sudo chmod 0440 /etc/sudoers.d/010_pi-nopasswd
  ```

## General SSH/shell gotchas hit during testing

- **`pkill -f pattern` can kill itself.** If `pattern` is a literal substring of the `pkill`
  command's own invocation (e.g. `pkill -f sllidar` run as `bash -c "pkill -f sllidar; ..."`),
  `-f` matches against the full command line - including its own - and the whole SSH session dies
  before printing anything (looks exactly like a network drop: exit 255, no output). Use the
  bracket trick to avoid self-matching: `pkill -f '[s]llidar'`.
- **`cd dir && cmd &` only changes directory inside the backgrounded subshell.** The parent
  shell's CWD is unaffected, so a *second* backgrounded command later in the same SSH invocation
  needs its own explicit `cd` - it does not inherit the first command's directory change.
- This particular Pi's WiFi link drops mid-session often enough that a command can fail with exit
  255 for no reason related to the command itself. If a command that worked moments ago suddenly
  returns exit 255 with zero output, retry once or twice before assuming it's a real bug.

## Phase 1 - `/imu/data` (BNO08x UART RVC, Pi 5)

### Bugs found and fixed

1. **`ros2_ws` never built on Pi 5** - see prerequisite above. Not a code bug, just a missing
   provisioning step; `build-ws` fixed it.
2. **`Driver.__init__` config bug** (`src/hardware/imu/bno08x/mcp2221/uart_rvc.py`): built the
   inner `UARTRVCConfig` from raw ternary fallbacks (`quaternion=config.quaternion if config else
   None`) instead of resolving `config = config or Config()` first, like every other driver in
   `src/hardware/*/`. `QuaternionConfig` has no valid `None` state, so the node crashed on startup
   with a pydantic `ValidationError` before ever reading the `BNO08X_UART_RVC_QUATERNION__*` env
   vars. Fixed in commit `be6e6e0`.

### Validation method: blind rotation test

Rate (`ros2 topic hz`) and a single `ros2 topic echo --once` only prove the topic is alive - they
don't prove the orientation math is *correct*. The useful test is a **before/after snapshot around
a known physical rotation**, checked two ways:

1. **Angle/axis check**: capture a quaternion, physically rotate the robot by some amount (a
   known amount if validating, an unknown amount if double-checking - see below), capture another
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
retest - likely the robot wasn't fully settled that one time), but it's a good trust-but-verify
habit before relying on this data for navigation.

### Live-streaming test scripts don't work over this SSH setup

Don't use a script that prints a countdown then streams readings for N seconds while asking the
other person to react to a "GO" printed mid-script - if you (the one running the script over SSH)
are the one watching the terminal, the other person **can't see that output live** and has no way
to time their action against it. Every attempt at this (5s buffer, 15s buffer, countdown, even
unbuffered `python3 -u`) produced zero captured motion - the window always elapsed before they
could react to a signal they never actually saw.

**Use the before/after snapshot method instead**: capture a reading, ask them to move it and say
"done" whenever ready (no time pressure), capture a second reading. This is what actually worked.

### Known limitation to keep in mind

BNO08x RVC mode computes yaw/pitch/roll internally via Euler angles, which are subject to gimbal
lock near ±90° pitch (yaw and roll become coupled/ambiguous). If a future test shows a rotation's
magnitude captured correctly but landing on the wrong axis, check whether pitch approached ±90°
during the motion before assuming it's a code bug - verify with the raw-angle driver script above.

## Phase 2 - `/scan` (Slamtec C1 LIDAR, Pi 5)

### Bugs found and fixed

1. **`sllidar_ros2` never fetched on Pi 5** - the third-party driver package isn't part of this
   repo; it's fetched into `ros2_ws/src` via the `fetch-lidar-driver` pixi task (idempotent),
   which must run once **before** `build-ws`. Symptom if skipped: `ros2_ws/src/` only has the
   `vtitan_*` packages (at the time, a single `vtitan_robot` - since split into
   `vtitan_drivers`/`vtitan_navigation`/`vtitan_vision`/`vtitan_state_machine`/
   `vtitan_bringup`), and the LIDAR launch fails looking for the `sllidar_ros2` package /
   `sllidar_node` executable.
2. **Three different launch paths used three different `frame_id` values** for the same physical
   sensor: the standalone path (`run-lidar` pixi task / `vtitan-lidar.service`, which launches
   the third-party `sllidar_ros2/launch/sllidar_c1_launch.py` directly) defaulted to `laser`; the
   integration path (now `vtitan_bringup/launch/lidar_launch.py`, included by
   `wro_state_machine_launch.py`) hardcoded `laser_frame`; the static TF (below) published a
   transform into `lidar_link`. None of these matched, so TF lookups for the actual published scan
   frame would have failed regardless of which path was running. Standardized all three on
   `lidar_link` (matching the URDF/SDF naming convention already used for `camera_link`/`imu_link`).
3. **`static_tfs.launch.py` existed in source but was never installed** - missing from
   `setup.py`'s `data_files`, so `ros2 launch vtitan_bringup static_tfs.launch.py` failed with
   *"file 'static_tfs.launch.py' was not found in the share directory"* - and critically,
   `wro_state_machine_launch.py` also includes it via `get_package_share_directory(...)`, so the
   **full integration bringup was silently missing all sensor-frame transforms**, not just the
   standalone LIDAR test. Added to `data_files` and rebuilt.
4. **`static_tfs.launch.py` used the wrong `static_transform_publisher` CLI convention for this
   ROS2 distro.** It passed positional arguments (`arguments=["0.14", "0", "0.10", "0", "0", "0",
   "base_link", "camera_link"]`), but this distro's `static_transform_publisher` only accepts
   **named** arguments (`--x`, `--yaw`, `--frame-id`, `--child-frame-id`, etc. - confirmed via
   `ros2 run tf2_ros static_transform_publisher --help`). The old positional form would have failed
   outright once install was fixed. Rewritten to use named args.
5. **C1 mounted inverted** - needs two independent corrections, not one:
   - **180° yaw offset** on the `base_link -> lidar_link` static TF (a pure rotation) - fixes
     where "front" lands. Made configurable via `LIDAR_YAW_OFFSET_DEG` (default `180`) in `.env`,
     read through a pydantic-settings `BaseSettings` class (same mechanism every hardware driver
     uses; `.env` itself is loaded as a side effect of importing `src.logger.config`).
   - **`inverted:=true`** on the `sllidar_node` launch parameter (both `run-lidar`'s pixi task and
     `lidar_launch.py`'s `Node` parameters) - fixes a **left/right mirror** that the yaw offset
     alone cannot correct. A pure Z-axis rotation can shift where front/back land but can never
     swap left and right; that requires a reflection, which is exactly what mounting a 2D LIDAR
     upside-down causes and exactly what the driver's own `inverted` flag exists to correct.
     Discovered by validating against real measured distances in four directions (see below) -
     front/back matched immediately with just the yaw offset, but left and right came out swapped
     until `inverted:=true` was added.

### Validation method: measure real distances in known directions

Rate (`ros2 topic hz`, ~10 Hz for the C1's Standard mode) and range plausibility (values within
`range_min`/`range_max`, `.inf` for no-return) only prove the topic is alive with sane-looking
numbers - they don't prove the angle mapping is correct. The useful test: have someone stand next
to the robot, tell you the actual measured distance to a wall/object in each of front/left/right/
back, then compare against the scan data bucketed into narrow sectors around each of those
directions (0°/+90°/-90°/±180° in the frame you expect to be robot-relative):

```python
# after applying whatever yaw offset the static TF applies, to get robot-frame angles:
base_angle = ((raw_angle_deg + yaw_offset_deg + 180) % 360) - 180
```

This is what actually caught the left/right mirror bug above - a structural check (rate, range
bounds, "front has the biggest numbers") would have passed even with left/right swapped, since
swapping two sectors doesn't break any of those invariants. Only checking against real,
independently-known distances in specific directions surfaces a mirror/reflection bug.

### Self-occlusion from the robot's own chassis

Some angular range will read implausibly short distances that are the robot's own parts, not real
obstacles - figure out which range by physically clearing everything else out of range (or
knowing the true distances are large) and treating anything under some threshold (25cm here, since
the real walls were never that close) as self-occlusion. It came out as **~73 scattered small
clusters** (2–10 points each, likely thin structural elements - wiring, brackets, screw heads) -
not one solid block - spanning from about **+117° to +172°** on one side and **-122° to -177°** on
the other (i.e., concentrated around the rear, symmetric about 180°/back). Don't try to mask each
tiny cluster individually (fragile, overfits to one measurement - thin parts can clip a different
exact point on a re-scan); mask the full contiguous extent with margin instead, e.g. exclude
`abs(angle) > 115°` as a single check. Front/left/right (everything within ±115°) had zero
readings under the threshold.

Note the two rearward clusters don't quite meet at dead-back - they stop at `172.2°` and start at
`-176.7°`, leaving an ~11° window (`172°` through `180°` to `-177°`) with no recorded occlusion in
this particular scan. Mask through that gap too rather than treating it as a confirmed-clear
notch: the occlusion pattern is already sparse (thin parts, not a solid panel), so a gap between
two clusters more likely means a thin element missed a ray at that exact angle than a genuinely
reliable clear sightline - a re-scan could easily register a point there instead. A single
contiguous `abs(angle) > 115°` mask already covers this for free; don't special-case a "clear"
notch out of it.

## Phase 3 - `/button/event` (Pi Zero)

### Bugs found and fixed

1. **`get_state()` never reset `_last_event` after reading it**, in both the GPIO driver
   (`src/hardware/button/gpio/driver.py`) and the MCP2221 variant
   (`src/hardware/button/mcp2221/driver.py`). `button_node._poll()` runs at 20 Hz and publishes
   whenever `state.last_event` is truthy - since the field was never cleared, the very first real
   event would have republished forever, once per poll, flooding `/button/event` with the same
   stale message indefinitely. Fixed by having `get_state()` read-and-clear the field atomically
   under the driver's lock (the GPIO driver's lock existed but was never actually used to protect
   this state - also added locking around the `_on_pressed`/`_on_released` callbacks themselves,
   since without it a callback could write a new event in the narrow window between `get_state()`'s
   read and its clear, silently losing that event).
2. **`BUTTON_GPIO_PIN` was configured for GPIO 17, but the button is physically wired to GPIO 4.**
   `gpiozero.Button(17)` connects with no error - nothing about a missing physical connection
   raises an exception - so the driver reports "Button connected successfully" and the node runs
   fine forever, just never sees a real edge transition no matter how many times the physical
   button is pressed. This is the same class of bug as Phase 1's `Config` issue in spirit (silent
   "success" hiding a real problem), but a hardware-wiring mismatch instead of a code defect.
   Fixed by changing `BUTTON_GPIO_PIN` from `17` to `4` in `.env`/`.env.example`.

   Notably, `setup-pi-architecture.md` (marked "historical design spec - partially superseded")
   had GPIO 4 listed all along - its header explicitly calls out *"Pin assignments and hardware
   architecture below are still accurate"* even though other parts of that doc have drifted from
   the real implementation. Don't discount a whole doc as stale just because part of it is -
   check what it specifically claims is still current. The current `.env.example`, despite being
   live code rather than a doc, was the one that had actually gone stale here.

### Validation method: escalating bisection through the stack

For a discrete, momentary event (as opposed to a continuous stream like `/imu/data` or `/scan`),
a background listener with a generous window (60s) and "press whenever ready, tell me when done"
worked better than trying to synchronize a live countdown - same lesson as Phase 1's
live-streaming problem, just with a longer passive window instead of a snapshot pair.

When the full `/button/event` topic kept showing zero events even after both fixes above, the
useful technique was bisecting top-down through the stack, one layer at a time, until finding
where the signal actually disappeared:

1. **Raw GPIO read**, bypassing the driver class entirely - plain `gpiozero.Button(pin).is_pressed`
   polled in a loop. Confirms the physical wiring and pin number are actually correct.
2. **The driver class directly**, bypassing ROS/`button_node` - instantiate `Driver()`, call
   `.connect()`, poll `.get_state()` in a loop. Confirms the driver's callback registration and
   event/state logic work correctly in isolation.
3. **The full ROS topic**, via `ros2 topic echo` - confirms the node wrapper and publish path
   work end-to-end.

In this case, layers 1 and 2 both worked cleanly on the first proper attempt, and a subsequent
retest of layer 3 also succeeded - meaning the earlier "zero events" results were most likely
timing misses (the same "did I actually press it inside the window" issue seen throughout this
runbook), not a real remaining bug. The bisection was still valuable: it positively confirmed the
wiring and the driver logic before writing off the ROS layer as broken, rather than guessing.

Don't conclude a topic is broken from one "zero events" result on a momentary/discrete signal -
unlike a continuous stream (where you can just wait longer), a single missed press looks identical
to a real failure. Retry at least once, and bisect down a layer if it keeps failing, before
treating it as a genuine bug.

## Pi Zero deployment: never run `pixi install`/`build-ws` directly on it

The Pi Zero 2 W has ~415MB usable RAM. Resolving/installing the full `dev`
pixi environment (ROS2 Kilted base + opencv/scipy/pydantic/etc. via
robostack/conda) or rebuilding `ros2_ws` there directly is heavy enough to
swap-thrash the SD card into double-digit load averages, make SSH
unresponsive for many minutes at a stretch, and - confirmed the hard way -
crash the board with an unclean shutdown (fsck found a dirty bit and a
corrupted systemd-journald file on the next boot; no reported ext4 data
corruption that time, but repeat hard resets are a real risk to the
filesystem, not just an inconvenience).

**Two compounding traps found doing this:**
1. **`pixi run` silently self-repairs a stale/broken env before running
   anything.** If `vtitan-pi-zero.service` is `enabled` and the Zero
   reboots (e.g. mid-recovery) with an incomplete `dev` env, the service
   auto-starts on boot and its `pixi run -e dev launch-rpi-zero` immediately
   re-triggers a full install in the background - silently fighting any
   manual recovery attempt for the same disk I/O. **Always
   `sudo systemctl disable vtitan-pi-zero.service` before doing any
   maintenance on the Zero's pixi env**, and only re-enable once it's
   confirmed working standalone.
2. Aggressively retrying SSH connections against an already I/O-starved board
   compounds the problem - each connection spawns a new sshd session with its
   own overhead. Prefer long, sparse, patient checks over tight retry loops
   when the board is already struggling.

### Setting up a freshly-flashed Pi Zero

Automated via `pixi run -e dev bootstrap-fresh-zero`, run **on Pi 5** (see
`scripts/provisioning/bootstrap-fresh-zero.sh`). Idempotent - safe to re-run on a Zero
that's already set up. Covers, in order:

1. Trusting Pi 5's SSH key into the Zero's `~/.ssh/authorized_keys` (needs
   *some* existing trusted path in first - e.g. Raspberry Pi Imager's own SSH
   key customization on first boot - the script can't bootstrap first contact
   from nothing).
2. Verifying passwordless sudo on both boards.
3. Copying `src/python` (which nests `shared/`, so a single copy covers
   both) onto the Zero via `git archive HEAD -- src/python | ssh ... tar -x`
   from Pi 5's own checkout, piped straight over the already-trusted SSH
   link. This sidesteps needing GitHub auth on the Zero at all (the repo is
   private; Pi 5 authenticates via `gh`, which isn't worth replicating on a
   throwaway/resource-constrained board). Before `shared/` moved inside
   `src/python/`, it was a separate sibling copy step and easy to forget:
   `pixi.toml` installs it as an editable dependency
   (`vtitan-shared = { path = "./shared" }`), so anything importing
   `shared.*` (e.g. `oled_display_node`'s `shared.config.constants`) would
   fail with `ModuleNotFoundError: No module named 'shared'` if it were
   missing - confirmed the hard way testing the merged `pi_zero_node` on a
   freshly re-imaged Zero, back when it needed its own copy step.
4. `cp .env.example .env` on the Zero, **only if `.env` doesn't already
   exist** - never overwrites a hand-tuned one.
5. Installing the `pixi` CLI itself on the Zero (just the ~20MB binary via
   `curl -fsSL https://pixi.sh/install.sh | sh` - NOT `pixi install`, which
   is the heavy conda/mamba resolve step this whole deployment approach
   exists to avoid running on the Zero; see below).
6. Enabling I2C - **off by default on a fresh Raspberry Pi OS image**,
   required for the SSD1306 OLED display. `dtparam=i2c_arm=on` in
   `/boot/firmware/config.txt` loads the `i2c_bcm2835` bus driver, but the
   `/dev/i2c-*` character device nodes additionally need the `i2c-dev` kernel
   module (`echo i2c-dev | sudo tee -a /etc/modules` to persist across
   reboots, not just a one-off `modprobe`). Confirmed via
   `i2cdetect -y 1` showing the display responding at `0x3c`, matching
   `.env`'s `SSD1306_I2C_ADDRESS`.
7. Templating `systemd/vtitan-pi-zero.service`'s `__TARGET_USER__` /
   `__TARGET_HOME__` placeholders and installing it to
   `/etc/systemd/system/` - left **disabled**. Don't `systemctl enable` it
   until you've confirmed `sudo systemctl start
   vtitan-pi-zero.service` works standalone (same reasoning as the
   "two compounding traps" above - an enabled service auto-starting mid
   troubleshooting fights you for the same constrained resources).

A reboot is needed after step 6 for the I2C change to take effect - use
`safe-shutdown-zero` (below), never pull power directly.

**The fix: build on Pi 5, ship the result to the Zero as tarballs.** Pi 5 has
15GB+ RAM and does the same `pixi install -e dev` + `colcon build` in under a
minute. Both boards are `linux-aarch64` with the same username and identical
absolute repo path (`~/vtitan/src/...`), so a straight copy
of `.pixi/envs/dev` and a `ros2_ws` build produced with `-e dev` works without
any conda/mamba resolution happening on the Zero at all.

Automated via `pixi run -e dev deploy-dev-env-to-zero`, run **on Pi 5**
(see `scripts/provisioning/deploy-dev-env-to-zero.sh`). One-time prerequisite: Pi 5's own
SSH key needs to be in the Zero's `~/.ssh/authorized_keys` (they don't trust
each other by default) - generate one with `ssh-keygen -t ed25519` on Pi 5 if
`~/.ssh/id_ed25519.pub` doesn't already exist, then append it on the Zero.
Transferring over the USB-gadget link (`192.168.250.1`, wired) rather than
the Zero's own WiFi radio is markedly more reliable - the ~850MB compressed
`dev` env transfer is large enough that WiFi drops repeatedly restart it from
scratch (`scp` doesn't resume).

The script extracts under `_new`-suffixed names and atomically renames them
into place (`dev` → `dev_old_<timestamp>`, `dev_new` → `dev`, same for
`ros2_ws/build`/`install`) rather than deleting the old broken env first -
`rm -rf` on a large, deeply-nested conda env (e.g. bundled Qt/WebEngine
license trees with huge file counts) can itself hang for a very long time on
the Zero's slow SD card. Clean up the timestamped `*_old_*` leftovers
manually once the new env is confirmed working.

### Always shut the Zero down cleanly before cutting power

Pulling power on a running Zero (instead of shutting it down first) leaves the ext4 journal
uncommitted. Confirmed the hard way: this showed up on next boot as free-block/inode count
mismatches and a stale orphan-file flag (`e2fsck -f` fixed it), and the board wouldn't come back
up on its own because boot was hung waiting on an interactive fsck prompt with no display
attached. Repeat occurrences risk worse (real ext4 data-structure corruption, not just accounting).

Run `bash scripts/provisioning/safe-shutdown-zero.sh` **on Pi 5** before ever removing power from the Zero - it
stops `vtitan-pi-zero.service`, syncs, issues a clean `shutdown -h now`, and polls until the
Zero is actually offline before telling you it's safe to unplug it.

To power both boards down in one command (e.g. before switching the robot from wall/USB power to
battery), run `bash scripts/provisioning/safe-shutdown-both.sh` **on Pi 5** instead - it runs the same Zero
shutdown first, then syncs and shuts Pi 5 itself down last, once the Zero is confirmed offline. Also
available as `task robot:zero ACTION=shutdown-both` from the repo root.

If it's already wedged in a dirty-fsck state with no display attached: pull the microSD card, plug
it into another Pi (or a USB reader) that can mount ext4 natively, and run
`sudo e2fsck -n -f /dev/<partition>` read-only first to see what's wrong, then `sudo e2fsck -f -y
/dev/<partition>` to fix it.

### Merged `pi_zero_node` - verification and resource baseline

Verified end-to-end on a freshly-flashed Zero (bootstrap → deploy →
`systemctl restart vtitan-pi-zero.service`): the motor, button, and OLED
nodes that used to run as 3 separate `ros2 run` processes now construct and
activate inside a single `pi_zero_node` process
(`ros2_ws/install/lib/vtitan_drivers/pi_zero_node`). `ps`/`systemctl
status` cgroup output confirms one Python PID owns all three lifecycle
nodes; the `ros2 launch` parent process is normal launch-file overhead, not
a second node.

Idle resource baseline (no driving, no LIDAR/vision - just motor+button+OLED
holding steady state), sampled ~19 minutes after boot on the Pi Zero 2 W's
415MB usable RAM:

- **CPU**: ~65-70% of one core sustained (fluctuates 50-75%), i.e. roughly
  16-19% of total system CPU capacity on the quad-core Zero 2 W. Higher than
  it might look at a glance - likely GPIO/encoder polling loops rather than
  a leak; not yet root-caused, worth profiling if it becomes a bottleneck
  once LIDAR/vision are running concurrently.
- **Memory**: ~106MB RSS for the node process (~25% of total RAM), ~247MB
  used system-wide (~59%), ~167MB available. Comfortable headroom at idle,
  but worth re-checking once the full stack (LIDAR + vision + navigation) is
  running on the same board simultaneously.

### USB-gadget link: end-to-end verification (`verify-zero-integration.sh`)

Run **on Pi 5** via `bash scripts/provisioning/verify-zero-integration.sh` (or `pixi run -e dev
verify-zero-integration`), after bootstrap + deploy have shipped code to the Zero and its service is
running. Checks, in order: SSH reachability over both the USB-gadget IP (`192.168.250.1`) and WiFi;
`usb0` carrier state, ping, and `dmesg` for `cdc_ether` TX-watchdog faults; the systemd service's
active/enabled state and `--as-is` `ExecStart`; ROS2 topic discovery across the two boards; and a
real `AckermannDriveStamped` publish → motor driver → feedback round-trip (safe with motors
unpowered - it only checks the software position-tracking value, not physical motion).

#### Bug found and fixed

**`set -uo pipefail` + sourcing `ros2_ws/install/setup.bash` kills the whole script.** ROS2/colcon's
generated setup scripts reference unset variables internally and are not `set -u` safe - sourcing
one under `-u` throws an "unbound variable" error that terminates the entire script immediately,
silently, with no useful output (looks exactly like the script hanging, not erroring). Fixed by
wrapping just the `source` line in `set +u` / `set -u`. Any script in this repo that sources a ROS2
setup file needs the same guard if it also uses `set -u`.

#### Isolating "over USB" from "over WiFi" - the duplicate-node false alarm

Both boards can have `usb0` and `wlan0` up simultaneously, which makes it easy to *think* something
is working over USB when WiFi is silently carrying the traffic instead. Two things worth knowing:

- **To actually prove USB-only operation**, disable WiFi on the Zero (`sudo nmcli radio wifi off`)
  and re-run discovery - don't just trust that the USB path "looks" reachable while WiFi is also up.
  All topics (`/ackermann_cmd`, `/motor/*`, `/button/event`, `/ui/oled_mirror`, etc.) stayed fully
  discoverable and functional with the Zero's WiFi fully off, confirming the ROS2 graph genuinely
  works over the USB-gadget link alone, not just alongside WiFi.
- **`ros2 node list` showing `/pi_zero_node` three times, with a "nodes share an exact name"
  warning, is a DDS multi-locator discovery artifact, not three real processes** - this persisted
  even with the Zero's WiFi off, because the *Pi 5* side still had both `usb0` and `wlan0` active,
  so CycloneDDS advertises/discovers the same single participant via multiple network paths. Verify
  with `ps aux | grep pi_zero_node` (one PID) and `ros2 topic info <topic> --verbose` (publisher
  count: 1, one `Node name` entry) before assuming duplicate nodes are actually publishing
  duplicate/racing messages - they aren't; `ros2 topic hz` on affected topics showed a single clean
  steady rate with no doubling.

#### Measured latency

- Raw USB-gadget link (ICMP ping, Pi 5 → Zero): **avg 0.22ms, min 0.15ms, max 0.28ms**, 0% loss over
  20 pings - effectively negligible for control-loop purposes.
- ROS2-level round-trip (publish `/ackermann_cmd` → observe the matching value land on
  `/motor/steering_position`, averaged over 5 distinct steering angles) at the default 20Hz feedback
  rate: **avg 41.15ms, min 22.12ms, max 60.57ms**. The gap between this and the raw ping time
  confirms the bottleneck is the motor node's own feedback publish cadence, not the USB transport.

### `AckermannMotorNode` owns both steering and drive

`ros2_ws/src/vtitan_drivers/vtitan_drivers/motors/ackermann_motor_node.py`'s
`AckermannMotorNode` is a single node/process responsible for **both** the steering servo
(`self.steering`) and the drive motor (`self.drive`), each independently backend-configurable
(`steering_backend`, `drive_backend` params). One shared `_publish_feedback` timer, running at
`PUBLISHER_RATE_HZ` (module constant, default `20.0`), publishes both `/motor/steering_position` and
`/motor/drive_speed` together - bumping that one constant affects both feedback streams at once. On
the BuildHAT backend specifically, a single `CombinedDriver` object satisfies both the
`SteeringDriver` and `DriveDriver` interfaces, so steering and drive can be literally the same
underlying hardware object accessed through two different type interfaces.

### Feedback-rate tuning: CPU/latency tradeoff (tested, not currently adopted)

`pi_zero_node` already runs `ackermann`/`button`/`oled` under one `MultiThreadedExecutor`
(`pi_zero_node.py`) rather than a single-threaded spin - each node gets its own default callback
group, so e.g. a slow OLED I2C write can't block the motor feedback timer. This gives real
concurrency for I/O-bound hardware calls (most driver-level GPIO/I2C/serial calls release the GIL
while blocked on the bus), but **not** genuine CPU-bound multicore parallelism - it's still one
Python process/one GIL, so pure-Python compute across all three sub-nodes is still serialized
regardless of the Zero 2 W's 4 cores. True multicore parallelism would require splitting back into
separate OS processes, which is exactly what the `pi_zero_node` merge undid (fewer DDS participants,
one systemd unit, simpler ops) - not worth reverting unless a specific sub-node becomes CPU-starved.

Measured on live hardware, `PUBLISHER_RATE_HZ` 20 vs 30 (steady-state, `pi_zero_node` idle otherwise):

| Metric                      | 20Hz (default)      | 30Hz                |
|------------------------------|---------------------|----------------------|
| CPU (`pi_zero_node` process) | ~50% of one core     | ~78% of one core      |
| Round-trip latency avg       | 41.15ms              | 28.45ms (-31%)        |
| Round-trip latency max       | 60.57ms              | 49.01ms (-19%)        |

CPU scales roughly linearly with the rate. 30Hz leaves only ~22% headroom on that core, which is
shared with button and OLED callback threads - under real competition load (driving + button
presses + OLED refresh concurrently) this is close enough to saturation to risk jitter or a delayed
watchdog check (`_watchdog_check` runs on a fixed 500ms timer looking for stale commands), which is
a worse failure mode than the current ~41ms latency. **Decision: reverted to 20Hz for now** - the
~30% latency win isn't worth the headroom risk without also splitting the motor node out.

**If lower feedback latency is needed later**, the concrete next step is splitting
`AckermannMotorNode` (steering+drive, since they're already merged as one responsibility) into its
own process, separate from button+OLED - giving it a dedicated core - rather than raising
`PUBLISHER_RATE_HZ` on the current shared-process setup. Not yet implemented.

### Drive-motor characterisation on battery power (2026-07-25)

First tests with the drive motor actually powered - it is wired **only** to the battery, so every
prior session's "motor" testing had exercised the software path with no current reaching the motor
at all. Run with `scripts/hardware/test_motors.py` (see below); all figures are `/motor/drive_speed` encoder
feedback averaged over the steady-state portion of a 5s hold.

#### Direction was inverted (fixed via config, not rewiring)

A positive commanded speed drove the robot **backwards**. Fixed by setting
`MOTOR_DRIVE__REVERSED=true` (note the double underscore - it is a nested pydantic-settings field;
the single-underscore `MOTOR_REVERSE_DRIVE` named in the node's docstring never existed and was
silently ignored. Docstring corrected). No rewiring needed. `.env.example` now ships `true`, since
`.env` is gitignored and a fresh bootstrap would otherwise silently drive the robot in reverse.

#### Deadband: nothing moves below ~0.7 m/s commanded

Stepping upward from 0.02 m/s, **every** command from 0.02 through 0.6 m/s produced zero encoder
movement; 0.7 m/s was the first to move the wheels. With `MOTOR_DRIVE__SPEED_SCALE=30.0` that is
`int(0.7 * 30) = 21`, i.e. roughly a **21% duty-cycle stiction threshold**. Commands below this are
not "slow", they are "nothing happens" - speed-control logic (navigator speed tables, any future
PID) must treat sub-0.7 m/s as a dead command rather than a small one. 0.7 m/s itself is marginal
and did not reliably sustain motion over a 5s hold; ~1.4 m/s was the lowest speed that moved
dependably.

#### Forward/reverse asymmetry: ~1.7x, and it is physical

At *identical* 100% duty (3.5 m/s commanded, which clamps to the `MOTOR_DRIVE__MAX_SPEED=100` cap):

| Direction | Mean       | Min    | Max    | Stdev | n   |
|-----------|-----------|--------|--------|-------|-----|
| Forward   | 336.6 deg/s | 252.3 | 486.8 | 40.2  | 370 |
| Reverse   | 193.7 deg/s | 77.3  | 343.6 | 42.2  | 384 |

The ~143 deg/s gap is >3x either sample's stdev, so it is not measurement noise. It is also **not**
a software artifact: `SpeedEstimator.update()` (`src/hardware/motors/encoder/control.py`) derives
speed from a signed `delta = counts - prev_counts` with no direction-dependent branch. And it is not
traction/weight-transfer, because the same asymmetry appeared in earlier **off-ground** runs
(~1.5-1.65x there). Most likely cause is brush timing advance in the brushed DC motor (brushes
optimised for one rotation direction), possibly compounded by gearbox drag.

Practical consequence: **reverse is ~40% slower than forward for the same command.**
`CoreNavigator`'s escape maneuvers (reverse bursts when stuck or too close to a wall) will travel
correspondingly less than a symmetric model would predict.

#### Known inconsistency: feedback sign vs command sign

With `MOTOR_DRIVE__REVERSED=true`, `_ackermann_callback` negates the command sent to the hardware
but `get_drive_speed()` still reports the encoder's raw *physical* direction - so commanding `+3.5`
publishes `-346 deg/s`. Harmless today (nothing subscribes to `/motor/drive_speed`;
`ROS2HardwareGateway` takes position from LIDAR and yaw from the IMU only), but it must be resolved
before any consumer - telemetry, or the encoder-feedback ideas discussed for the navigator - relies
on that topic, since an inverted feedback sign in a closed loop is a runaway.

#### Measure steady-state, not a single sample

The first version of `test_motors.py` reported one end-of-hold feedback sample and produced wildly
inconsistent numbers (e.g. 48 deg/s at 2.8 m/s but 144 deg/s at 3.0 m/s). The signal is noisy enough
(stdev ~40 deg/s) that a lone sample says almost nothing. The script now records every sample via
the subscription callback, discards a `--spinup-s` acceleration window, and reports mean/min/max/
stdev - which is what made the forward/reverse asymmetry above legible rather than looking like
scatter.

### Drive encoder calibration -- counts_per_rev was 3.5x wrong (2026-07-25)

`_DEFAULT_COUNTS_PER_REV` was `194.0`, derived from the datasheet as 11 PPR x4 quadrature x an
assumed ~4.4 gear ratio. Measured against tape, the real figure is **676** (~15.4:1 gearing), so
`counts_to_distance()` was under-counting revolutions and over-reporting distance by ~3.5x.

Calibrated from **raw quadrature counts** (`get_drive_counts()`), now exposed as `encoder_counts` in
`/motor/status` -- it wasn't published anywhere before, which is why the first attempt had to
integrate `/motor/drive_speed` instead. Don't do that: that feedback is exponentially smoothed and
rate-derived, and integrating it gave counts ~2.4x low (683 vs a true 1650), which in turn produced
a confidently-wrong CPR estimate of ~349-369.

| Run | Raw counts | Measured | Implied CPR |
|-----|-----------|----------|-------------|
| 90% duty, 5s | 1650 | 54 cm | 672 |
| 100% duty, 5s | 2447 | 79 cm | 681 |

The two agreeing to **1.3%** is the important part, not either number alone: wheel slip only ever
inflates the count for a given distance, so agreement across a 10-point duty spread means slip is
negligible and this is the true geometric ratio. Validated afterwards on an **independent** run not
used to derive it -- 1341 counts predicted 43.6 cm, tape said 44 cm (0.9%).

Re-measure with `scripts/hardware/calibrate_encoder.py` if the drivetrain changes.

#### Why duty-based speed calibration was abandoned

The original plan was to calibrate commanded speed against duty cycle. That is worthless here,
because duty->speed depends on battery voltage. Demonstrated accidentally: the *identical* command
(3.0 = 90% duty, 5 s) travelled **43 cm on a nearly-flat battery and 54 cm on a fresh one** -- 26%
apart. The encoder->distance ratio is geometric and holds regardless.

The consequence is that `MOTOR_DRIVE__SPEED_SCALE` (`motor_speed = velocity * 30`) is not a unit
conversion at all -- it claims 100% duty is 3.33 m/s when the robot actually tops out near 0.13 m/s.
Commanded "m/s" values are therefore meaningless today; anything above ~3.33 simply saturates.

**Not yet done at the time of this entry** (closing the loop; since done). `run_drive_at_rpm()` and a
`PIDController` existed in `dc_encoder/driver.py` (since split into `base.py`'s `ClosedLoopDrive` +
`encoder/control.py`, 2026-08-25) but at the time nothing called them -- `ackermann_motor_node` used the open-loop
`run_drive_forward/reverse`. Switching to the closed-loop path would make commands true m/s and
self-correct for battery droop. Two caveats when that happens: the PID gains
(`kp=0.002, ki=0.004, ff=1/max_rpm`) have never run on hardware, and both `run_drive_at_rpm()` and
`get_drive_rpm()` hardcode `dt=0.02`, so the loop must be driven at 50 Hz or the gains silently mean
something else.

### Steering servo jitter -- fixed by moving to hardware PWM (2026-07-25)

The servo twitched continuously, including while holding a fixed angle. Bisected on hardware by
elimination: it persisted with a **single** command and zero PWM rewrites (ruling out command
traffic), and on a **fresh battery** (ruling out supply sag). Cause was gpiozero's pin factory --
the Zero resolves to `LGPIOFactory`, whose `PWMOutputDevice` generates the pulse train in software,
so kernel scheduling jitter lands straight on the servo's pulse width. No pin choice fixes that;
gpiozero never touches the SoC PWM peripheral here.

Fixed by rewriting the driver onto `/sys/class/pwm` and pointing the overlay at the servo's pin.
The overlay was `pwm-2chan` with no parameters, which defaults to GPIO 18/19 -- pins nothing on this
robot uses -- while the servo sits on GPIO 12, so both hardware channels were idle. Now
`dtoverlay=pwm,pin=12,func=4` (`func=4` is ALT0, GPIO 12's PWM function), applied by
`bootstrap-fresh-zero.sh`. At the time this was deliberately single-channel: the two-channel
variant's second pin would claim GPIO 13, which the drive motor used via gpiozero, and a DC motor
is indifferent to PWM *jitter* specifically.

**Confirmed fixed on hardware** -- the servo now holds a commanded angle steady. Steering travel was
also verified across the full range in both directions, and the direction convention is correct
(positive = left, per REP-103), so `SERVO_REVERSED` stays `false`.

Access is group-based, not root: Raspberry Pi OS's `99-com.rules` chgrps `/sys/class/pwm` to `gpio`
and the service user is in that group. The driver fails loudly if the overlay is missing rather than
falling back to software PWM, so a re-flash that loses the config surfaces as an error instead of
silently reintroducing the twitch.

### Drive motor moved to hardware PWM too (2026-07-28)

The single-channel decision above only accounted for PWM *jitter*, which a DC motor tolerates fine.
It didn't account for gpiozero's software PWM being a continuous background thread regardless of
jitter tolerance -- on the Pi Zero (`LGPIOFactory`), that thread was a real, constant CPU cost, found
while investigating `ackermann_motor_node` pegging ~80% CPU on an already-overloaded board (only
153MB free of 415MB, already swapping). The drive motor's PWM pin (GPIO 13, `_L298nPins.pwm_pin`
in `ackermann_motor_node.py` at the time -- pin dataclasses were split per-backend 2026-08-25) is the
SoC's other hardware PWM channel (PWM1) alongside the servo's GPIO 12 (PWM0), so both now share one
`pwm-2chan` overlay: `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`. The H-bridge enable line
(then `dc_encoder/driver.py`, now `l298n/driver.py`) writes `/sys/class/pwm` the same way the servo
driver does (`l298n/config.py` holds `pwmchip`/`pwm_channel=1`/`frequency_hz=1000`); direction pins
stay on gpiozero since those are plain digital I/O with no PWM involved. The quadrature encoder is
now a separately-wired component (`encoder/driver.py`), unaffected by this section either way.

**Not yet confirmed on hardware** -- this needs a fresh `bootstrap-fresh-zero.sh` run (or a manual
config.txt edit + reboot) and a re-check of CPU usage and PID behavior on the Zero.

### USB-gadget link is intermittent across Zero reboots (2026-07-25)

The `cdc_ether` TX-watchdog failure is **not** a cable, power, or config fault, despite looking like
all three. Established by elimination on hardware:

- **Not config.** `config.txt` diffed clean against a pre-edit backup; both sides had matching
  subnet, distinct MACs, `LOWER_UP` carrier, `g_ether` loaded, offloads already disabled.
- **Not signal integrity / speed fallback.** The `new full-speed ... new high-speed` pairs in
  `dmesg` are ordinary USB enumeration (devices chirp up to high-speed after connecting), not
  degradation. The link sat at **high-speed for the entire 24-minute failure window**.
- **Not power.** `vcgencmd get_throttled` reported `0x0` -- no undervoltage, no throttling -- even
  with the Zero drawing its power through the same micro-USB port carrying the data.
- **Not the port.** Already on a Pi 5 USB 2.0 port.

What it actually is: the gadget link comes up either working or dead, decided at enumeration time,
and each power-cycle re-rolls it. Same `config.txt`, same cable, same port -- dead after one boot,
0.3 ms round-trip after the next.

**A soft `reboot` is the bad case.** `shutdown -r now` re-initialises the gadget without the host
seeing a true disconnect/reconnect, and dwc2 does not reliably recover from that -- the link
enumerates, reports carrier, and silently passes no traffic in either direction (both sides' ARP
stays `INCOMPLETE`/`FAILED`). A full power-cycle recovered it every time.

Practical rule: **after any Zero reboot, run `verify-zero-integration.sh`. If the USB link is dead,
power-cycle the Zero rather than rebooting it again.** Note the Zero is currently powered *through*
the Pi 5's USB port, so "power-cycle" means unplugging that cable -- which is a hard power cut, so
shut the Zero down cleanly first (`ZERO_HOST=<wifi-ip> bash scripts/provisioning/safe-shutdown-zero.sh`, since
the USB path is by definition unusable at that point).

Things tried that did **not** help, so don't burn time on them again: `ethtool -K usb0 tx off rx off
tso off gso off gro off`, bouncing `usb0` down/up, reloading `g_ether` on the Zero, and dropping MTU
to 1000 on both sides.

### `scripts/hardware/test_motors.py`

Hardware smoke test driving the real `ackermann_motor_node` over ROS2, run **on Pi 5**
(`pixi run -e dev test-motors`, or `python3 scripts/hardware/test_motors.py` for the flags below):

- Steering sweep (left/center/right/center) checking `/motor/steering_position` converges to each
  commanded angle. Safe with the robot stationary.
- Drive hold at `--drive-speed` for `--drive-duration-s`, reporting steady-state statistics, then a
  stop check. **Spins the wheels** - prompts for typed confirmation unless `--yes` is passed;
  `--skip-drive` omits it.
- `--find-min-speed` steps through `--min-speed-candidates` (ascending) and reports the first that
  actually moves the motor - how the deadband above was found.

Two gotchas it encodes, both learned the hard way:

1. **The motor node has a 1s command watchdog.** A single publish followed by `sleep` lets the
   watchdog fire mid-test and stop the motor ("No Ackermann commands received for 1.27s - stopping
   motors for safety" in the journal, wheels never move). The drive hold republishes at 10Hz
   throughout, like a real controller would.
2. **Don't trust the first feedback message after a command.** The feedback topic streams
   continuously, so the subscription queue can already hold pre-command samples the instant you
   publish; reading the "next" message reports the *previous* state. The script spins until the
   reading actually satisfies the expected condition instead.

## Template for future phases

For each new sensor, document here: what bugs were found, what prerequisite gaps existed, and
what validation method actually proved the data correct (not just "topic exists and publishes").
