---
title: 'cmd: glue the dashed program names and rename the generic server'
priority: p3
section: vtitan
topics:
    - repo-tooling
---
`src/go/cmd/` holds 11 dashed programs and one generic `cmd/server` in the legacy `other/` tree. `cmd/` stays flat here
(ADR 0007 in ralvafi: nesting is for the vendor-shaped trees like plc-platform, not for a repo with a handful of programs),
so the fix is to glue the two-token names.

- `imu-node` → `cmd/imunode`, `lidar-node` → `cmd/lidarnode`, `motor-node` → `cmd/motornode`,
  `telemetry-node` → `cmd/telemetrynode`, `capture-node` → `cmd/capturenod`. Check first whether the `-node` suffix is
  load-bearing (a container or a tool may match the binary name); if `capture` alone reads better, `cmd/capture` is fine
  and the other four stay as they are.
- `sim-runner` → `cmd/simrunner`; `bench-harness` → `cmd/benchharness`; `track-navigator` → `cmd/tracknavigator`;
  `state-machine` → `cmd/statemachine`; `foxglove-bridge` → `cmd/foxglovebridge`; `pi-zero` → `cmd/pizero`.
- `other/apps/backend/cmd/server` → `cmd/apiserver`.
- `other/apps/backend/gen/telemetry/v1` is generated protobuf (`package telemetryv1`) and stays.

Omitted from the ralvafi sweep at the owner's request; tracked here instead. Whatever name wins, the artifact (`bin/<name>`)
and any Dockerfile `-o` follow it.
