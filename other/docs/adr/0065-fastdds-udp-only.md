# 0065. FastDDS shared memory is disabled and transport is UDP only

- Status: accepted
- Date: 2026-09-15

## Context

FastDDS's default SHM transport leaves lock files and segments under `/dev/shm`.
Across the frequent node restarts this workflow involves, stale entries
accumulated: `/dev/shm` went from a handful of files to over 100 within a day of
iteration, and a stale port-lock file makes a new participant's SHM port bind
fail. That silently blocked discovery or delivery for whichever participant
landed on the wedged port, while other subscriptions kept working, so the symptom
looked like a random node failure rather than a transport fault.

## Options considered

- (a) Switch the RMW to Cyclone DDS.
- (b) Clear `/dev/shm` manually.
- (c) An `ExecStartPre=` cleanup script.
- (d) Disable the SHM transport and force UDP only, via a FastDDS XML profile.

## Decision

(d). `src/python/config/fastdds_udp_only.xml` declares a UDPv4 transport with
`useBuiltinTransports=false` (which is the switch that removes SHM) and a
user-transport whitelist, and each ROS2 unit sets
`FASTRTPS_DEFAULT_PROFILES_FILE` to it. The units are `vtitan-pi5.service`,
`vtitan-lidar.service` and `vtitan-pi-zero.service`.

The deciding factor over (a) to (c) is that this robot's traffic does not need
SHM's latency advantage: UDP loopback is effectively instant for it, and
inter-board traffic (Pi 5 to Pi Zero) already crosses the USB-gadget link as UDP,
where SHM was never in play. If SHM is never used there is nothing under
`/dev/shm` to go stale.

## Consequences

- The stale-lock failure mode is removed at its source rather than managed.
- Every ROS2 process must inherit the environment variable; a unit that misses it
  silently falls back to the builtin transports.
- `interfaceWhiteList` must use exact addresses, not CIDR.

## History

- bf064e1f 2026-07-28: disable the FastDDS shared-memory transport, force UDP
  only. Adds the XML and the doc.
- 9d281afb 2026-08-09: actually wire in the documented transport. The
  `Environment=` line had been written up but never added to any of the three
  units; reproduced live 2026-08-09.
- 2ac67f60 2026-08-09: record a failed whitelist attempt. CIDR
  `192.168.250.0/24` is NOT valid for FastDDS; it filtered out every interface
  ("All whitelist interfaces were filtered out") and broke `DATA_READER` creation
  entirely, worse than the original bug. Reverted.
- 1533bb25 2026-08-09: replace it with a working exact-address whitelist
  (`127.0.0.1`, `192.168.250.1`, `192.168.250.2`), but the whitelist alone did not
  resolve the Pi5/Zero delivery failure.
- 3299b25c and 39d8e679 2026-09-05/10: path moves into `src/python/config/`.
- 7d4c69d0 2026-08-28: give every `/ackermann_cmd` publisher a compatible QoS
  deadline. DEADLINE is two-sided: `state_machine_node` subscribed at 200 ms so
  `/race_metrics` reflected real commands, while `ros2_hardware_gateway` and
  `joy_teleop_node` offered `QOS_STREAM`'s Infinite deadline and DDS delivered
  nothing between those pairs, so it was never actually receiving live driving
  commands. `QOS_ACKERMANN_CMD` (RELIABLE + 200 ms) now serves every publisher
  and subscriber on the topic.

## Refuted

- Cyclone DDS: disproportionate to the problem.
- Manual `/dev/shm` clearing and an `ExecStartPre=` script: a recurring chore, and
  the script risks a TOCTOU race against a still-running participant.
- CIDR in `interfaceWhiteList`: confirmed broken on hardware.

## Cross-references

- 0067 owns the USB-gadget link the inter-board UDP traffic crosses.
