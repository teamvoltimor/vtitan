# 0095. `cmd/pi5` composes the Pi 5 board, and each subsystem's loop is shared

- Status: accepted
- Date: 2026-09-21

## Context

0068 defines the cutover as one swap of `vtitan-robot@go`, and 0066 splits
compute into a perception board and a real-time control board. Both assume a
binary per board. `cmd/pi-zero` was that for the Zero. `cmd/pi5` was not: its
own doc comment promised IMU, LIDAR, vision, telemetry, state machine,
track-navigator and capture as supervised goroutines, and it supervised capture
alone. The rest of the Pi 5 ran as separate systemd units, which is the
node-per-process shape the Go stack moved away from.

That is not a cosmetic gap. There was no Go binary composing the Pi 5 race
plane, so there was nothing for the cutover to swap to.

The obstacle was that each subsystem's run loop lived inside its own
`cmd/*/main.go`. Composing them meant either duplicating those loops into
`cmd/pi5` or extracting them first. `internal/node/motor` had already
established which: it was extracted from `cmd/motor-node` when `cmd/pi-zero`
became a second caller, on the grounds that duplicating a loop containing a
safety watchdog was not acceptable.

## Options considered

- (a) Leave the Pi 5 as one systemd unit per node and cut over by swapping
      several units at once.
- (b) Extract each subsystem's loop into `internal/node/<x>` and compose them as
      supervised targets in `cmd/pi5`, following the `internal/node/motor`
      precedent.

## Decision

(b). `internal/node/{imu,lidar,telemetry,nav}` join the existing
`motor`/`button`/`capture`, and `internal/node/statemachine` gains its run loop
beside the sinks it already held. Each exposes a `Config` and a
`Run(ctx, cfg, logger)` that owns its own driver and NATS connection, so a
supervised restart re-opens both rather than inheriting a half-dead one. The
single-subsystem `cmd/*-node` binaries remain as bench tools and now build a
`Config` and delegate, so a bench run and a board run cannot drift apart.

Three wiring choices are deliberate:

- **Nav is last in the target slice.** It is the only target that commands the
  actuators, so on a clean start the sensors it reads are already publishing
  before it first steps.
- **The state-machine target registers only with `--robot-id`.** The backend
  command channel is how an operator starts a round remotely and a race can run
  without it; with no robot ID it would be a supervised goroutine redialling a
  backend nobody configured.
- **Vision is not a target.** 0068 keeps it in Python behind a sidecar, so this
  board's side of it is the detections subscription the nav loop already opens.

## Consequences

- The cutover has something to swap to: one unit per board.
- A subsystem crash-looping cannot take the board process down. Observed on a
  dev machine, where IMU and LIDAR back off 1s/2s/4s/8s against absent serial
  ports while capture, telemetry and nav keep running, and asserted in
  `test/smoke`.
- `cmd/pi5`'s target list is not importable (it is `package main`), so
  `test/smoke` composes the same node packages itself. The two must be kept in
  step by hand; if that drifts, the target list should move into a package both
  can call.
- The composition is verified without hardware but not on it. `test/bagreplay`
  drives the navigator domain packages rather than this composition, so what is
  still unknown is whether the real serial drivers behave as the composition
  expects, not whether the composition holds together.

## History

- b7b58e94 2026-09-21: the extraction and the six targets. `cmd/track-navigator`
  went from 680 lines to 144, `telemetry-node` 266 to 106, `lidar-node` 212 to
  108, `imu-node` 196 to 108, `state-machine` 157 to 124.
- d5e73f3e 2026-09-21: `test/smoke`. Its first version asserted that a drive
  command arrives and passed with the scan subject deliberately wrong, because
  the navigator publishes every tick regardless and commands a standstill with
  no data. It now asserts speed > 0, measured at 0.133 m/s with scans against
  exactly 0 without.
- 7e974f2f 2026-09-21: `natsgw.Gateway.Close` deleted. Running the composed
  binary showed four ERRORs on every clean shutdown, from a gateway closing a
  connection it had merely been handed.

## Cross-references

- 0066 (the two-board compute split this implements on the Pi 5 side)
- 0068 (the cutover this unblocks)
- 0094 (the `pkg/` boundary the extraction sits above)
