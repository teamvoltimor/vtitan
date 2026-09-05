# Why FastDDS's shared-memory transport is disabled (2026-07-28)

## Symptom

Over a single day of hardware testing/iteration (many `systemctl restart`
cycles on both the Pi 5 and Pi Zero as code changed), ROS2 topics would
intermittently stop delivering between two nodes that were both alive,
both correctly configured, and both subscribed with compatible QoS. No
exception, no crash, no error on the subscribing node -- messages simply
never arrived. Concretely, on the same robot, in the same session:

- `state_machine_node` would see `/scan` and `/imu/data` fine (reach
  `READY`), while `telemetry_bridge_node` -- a second, independent
  subscriber to the *same* topics, started in the *same* launch, with
  the *same* QoS profile -- never received a single message, so the
  OLED's LIDAR/IMU readout stayed at zero indefinitely.
- `oled_display_node` on the Pi Zero would keep rendering a stale page
  (e.g. a challenge-mode fault) long after `state_machine_node` on the
  Pi 5 had moved on, because its subscription to a topic published by a
  *restarted* instance of that node never re-matched.
- A freshly published test topic (`ros2 topic pub` / `ros2 topic echo`,
  nothing to do with this robot's own nodes) intermittently failed to
  connect too, ruling out a bug in this codebase's own pub/sub wiring.

Directly implicated in the logs:

```
RTPS_TRANSPORT_SHM Error: Failed init_port fastdds_port7003: open_and_lock_file failed
```

## Root cause

ROS2 on this robot uses `rmw_fastrtps_cpp` (FastDDS) as its RMW
implementation. FastDDS's default transport configuration layers a
**shared-memory (SHM) transport** on top of UDP for same-host
communication (lower latency than looping traffic through the kernel
network stack). To coordinate SHM segments between processes, it creates
lock files and semaphores under `/dev/shm`:

```
/dev/shm/fastdds_<participant-guid>
/dev/shm/fastdds_port<N>
/dev/shm/sem.fastdds_port<N>_mutex
```

These are meant to be cleaned up when a participant shuts down cleanly.
In practice, on this robot's dev/test workflow -- `systemctl restart`
(which sends SIGTERM/SIGINT, but under `TimeoutStopSec` can fall through
to SIGKILL if a node doesn't exit fast enough) many times in a session --
that cleanup doesn't reliably happen. Stale entries accumulate:
`/dev/shm` went from a handful of files to over 100 within a single day
of iteration. A stale port-lock file then makes a *new* participant's
attempt to bind that SHM port fail. Depending on exactly which port a
given participant's fastdds instance happens to land on, that failure
can silently degrade or block discovery/delivery for that participant's
subscriptions -- while a different participant on the same host, using a
different port, works fine. This is exactly the pattern observed: some
subscriptions healthy, others dead, no pattern visible from the
application code because the failure is entirely inside FastDDS's
transport layer, several layers below anything this codebase controls.

Manually clearing `/dev/shm/fastdds_*` and `sem.fastdds_*` before each
restart worked every time it was tried -- confirming the diagnosis -- but
the files reaccumulated within roughly an hour of continued
restart-heavy iteration, making it a recurring manual chore rather than
a fix.

## Why UDP-only, not "just clean up /dev/shm more carefully"

Options considered:

1. **Clean `/dev/shm` before every restart.** This is what was done
   during the session that found this bug, repeatedly. It works, but
   it's a manual step someone has to remember every time, on both
   boards, and it doesn't survive an unclean shutdown (power loss,
   `SIGKILL`) -- exactly the case it's most needed for.
2. **A cleanup script/systemd `ExecStartPre=` that clears stale FastDDS
   files automatically.** More robust than remembering to do it by
   hand, but still just papering over the underlying unreliability --
   and riskier than it looks: a blanket `rm -f /dev/shm/fastdds_*`
   immediately before start could race an *still-running* participant
   from a slow-to-exit previous instance (e.g. `ExecStop=SIGINT` with a
   grace period) and rip out a file a live process still holds a
   reference to, which is a bug in itself even if usually harmless on
   Linux (unlinking an open file doesn't invalidate existing
   descriptors, but a *new* participant racing to recreate the same
   named segment while the old one hasn't released it yet is exactly
   the kind of TOCTOU issue this class of bug already demonstrated).
3. **Switch RMW implementation** (e.g. to Cyclone DDS, which has a
   different shared-memory story). A much bigger change -- different
   dependency, different behavior/tuning surface, needs re-validating
   this whole robot's QoS assumptions against a different
   implementation. Disproportionate to the actual problem.
4. **Disable the SHM transport, force UDP-only, via a FastDDS XML
   profile.** Removes the failure mode at its source: if SHM is never
   used, there is nothing under `/dev/shm` to go stale. This is what was
   implemented.

The deciding factor for (4) over (1)/(2): this robot's actual traffic
doesn't need SHM's latency advantage to begin with.

- Every message this robot publishes at any real rate (`/scan` at
  10Hz, `/imu/data` at 100Hz, `/ui/telemetry_summary` at 10Hz,
  `/ackermann_cmd`, motor feedback, `/vision/detections`) is small --
  well under a kilobyte -- and the control loop rates involved (tens of
  Hz) are not latency-sensitive at the microsecond scale where
  loopback UDP vs. SHM would matter. UDP on `lo` on a Raspberry Pi is
  still sub-millisecond.
- **Inter-board traffic (Pi 5 <-> Pi Zero) already crosses the
  USB-gadget Ethernet link as UDP** -- SHM is a same-host-only
  optimization and was never in play for that path regardless. Only
  same-host topics (state_machine_node <-> telemetry_bridge_node on
  the Pi 5; oled_display_node <-> button_node/ackermann_motor_node on
  the Pi Zero) were ever using SHM at all.
- So the correctness cost of dropping SHM is effectively zero for this
  application, and the reliability upside (no more silently-dead
  subscriptions after a restart) is exactly the bug this session spent
  the most time chasing.

## What was actually changed

New file: `platform/config/fastdds_udp_only.xml` -- a FastDDS XML
profile that defines a single UDPv4 transport descriptor, sets
`useBuiltinTransports=false` (which is what pulls in SHM by default),
and applies it as the default participant profile.

Wired in via the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable,
added to all three systemd unit files that launch ROS2 nodes on this
robot:

- `vtitan-pi5.service` (state machine, vision, IMU, telemetry
  bridge)
- `vtitan-lidar.service` (LIDAR driver)
- `vtitan-pi-zero.service` (button, OLED, ackermann motor)

Each got one line added right after its existing `ROS_DOMAIN_ID=0`:

```
Environment=FASTRTPS_DEFAULT_PROFILES_FILE=/home/ralvarezdev/vtitan/platform/config/fastdds_udp_only.xml
```

No code changes were needed -- this is entirely a transport-layer
configuration change, invisible to every node's Python source.

## How to verify it's working

- `/dev/shm` should never accumulate `fastdds_*` / `sem.fastdds_*`
  files again, regardless of how many times services are restarted --
  there's nothing under `/dev/shm` for a UDP-only participant to
  create in the first place.
- No more `RTPS_TRANSPORT_SHM Error` lines in `journalctl` for any of
  the three services.
- Subscriptions that previously went silently stale after a same-host
  publisher restart (the `/system_status` -> OLED case, the
  `/scan`/`/imu/data` -> `telemetry_bridge_node` case) should now
  reconnect reliably every time.

## How to revert

Delete the added `Environment=FASTRTPS_DEFAULT_PROFILES_FILE=...` line
from each of the three unit files, `sudo systemctl daemon-reload`, and
restart. `platform/config/fastdds_udp_only.xml` can stay in place
unused, or be deleted -- it has no effect unless something points
`FASTRTPS_DEFAULT_PROFILES_FILE` at it.
