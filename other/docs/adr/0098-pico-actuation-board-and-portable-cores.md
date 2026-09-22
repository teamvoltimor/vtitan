# 0098. A Pico 2 is an alternative actuation board, and the logic both boards share lives in `pkg/portable`

- Status: accepted
- Date: 2026-09-22

## Context

0066 splits the robot across a Pi 5 and a Pi Zero 2 W, and the Zero owns the
actuators: drive motor, steering servo, wheel encoder, start button, OLED. It
does that on Linux, which has cost this project real bugs: a released GPIO line
floats HIGH through the level shifter and the BTS7960 reads it as full reverse,
the command watchdog stops running while the process restarts, and encoder
edges arrive through a kernel event queue with no latency bound.

A Raspberry Pi Pico 2 (RP2350) avoids all three by construction: pins it drives
stay driven, a hardware watchdog resets it, and PIO counts quadrature in
hardware. `go-future.md` section 18 had already costed a Pico, but as a THIRD
board added for raw gyro over SPI, and found the extra board the largest cost.
The question here is different: a Pico instead of the Zero, chosen per build,
with the Zero kept.

Measured before deciding (2026-09-22):

- TinyGo 0.42.0 builds against this module's `go 1.26.0` and has `pico2` and
  `pico2-w` targets.
- A `pico2` program inside `src/go` importing `pkg/geom` built to 18 KB of
  flash. A `//go:build tinygo` directory is skipped cleanly by `go build`,
  `go vet`, `go test` and golangci-lint over `./...`.
- Of the 17 `pkg/` packages, only `geom`, `control`, `backoff` and the
  `Driver[T]` contract were free of Linux-only or reflection-heavy imports.
  Every piece of logic the firmware needs (the H-bridge Fast-Brake ordering,
  servo pulse mapping, the wheel-to-servo conversion, the command watchdog,
  quadrature decoding, the BNO085 RVC parser) sat in a package that also
  imported go-gpiocdev, go.bug.st/serial, `os`, protobuf or NATS.

## Options considered

- (a) Replace the Zero with a Pico.
- (b) Add a Pico as a third board (`go-future.md` section 18).
- (c) Keep both, selected per build, behind the same interface to the Pi 5.

For where the firmware and shared code live:

- (i) A separate `src/tinygo` module, with the shared code copied or vendored.
- (ii) The existing `src/go` module, with the shared code in `pkg/` and the
      firmware behind the `tinygo` build tag.

## Decision

(c) and (ii).

**One interface above the board.** Everything above the actuation board talks
to it through the NATS subjects it already uses: `ackermann_cmd` in,
`motor_status`, `joint_states` and `button_event` out. The Zero serves them
itself, as today. The Pico cannot (nats.go and protobuf-go depend on sockets and
reflection TinyGo does not provide), so it speaks a framed serial protocol over
USB or UART to a Pi 5 target that republishes the same subjects. The navigator,
state machine, recorder and Foxglove bridge never learn which board is fitted.
The choice is a hardware-profile axis, next to the servo and motor profiles
(0021).

**The link is wired, and the radio stays off.** A radio in the control path
adds latency and jitter, and competition rules restrict wireless use during a
round. A Pico 2 is sufficient; a Pico 2 W may be used with its radio disabled in
race firmware, noting it reserves GPIO 23, 24, 25 and 29.

**The failsafe moves into the firmware.** Command timeout centres the steering
and zeroes the drive, as the Zero's loop does; the RP2350 hardware watchdog
covers a hung firmware. A lost link is a silent command stream, which the same
timeout already handles. The LPWM pull-down resistor is still fitted: it
protects the Zero and the power-on window of either board.

**Shared logic lives in `pkg/portable`, and must build under TinyGo.** It holds
what both boards run, moved out of the Linux drivers without behaviour change:
`hbridge` (Fast-Brake ordering), `servo` (pulse mapping), `actuation` (speed to
duty, wheel to servo angle, the finite-command rule, the command watchdog),
`quadrature` (decode and odometry) and `bno085rvc` (the RVC frame parser). The
`pkg/driver` packages keep only their Linux backends and delegate. Firmware
lives in a `firmware/<board>` directory of the same module, tagged
`//go:build tinygo`. One module means a change to shared logic and both of its
consumers is one commit, and there is no copy to drift.

**Enforcement, two halves.** A depguard rule, `portable-must-build-on-tinygo` in
`src/go/.golangci.yml`, denies `pkg/portable/**` the imports TinyGo cannot build
or a microcontroller cannot host (`os`, `syscall`, `net`, `reflect`, `unsafe`,
`log/slog`, the Linux GPIO, serial and peripheral libraries, NATS, protobuf,
`internal/`). The `robot-go-tinygo` CI job runs `src/go/scripts/tinygo-check.sh`,
which discovers every package under `pkg/portable`, runs its tests under TinyGo
and builds it for `pico2`. Discovery rather than a list means a package added
later is covered without editing either.

## Consequences

- `pkg/portable` is the first part of `pkg/` with a real second consumer, which
  is the trigger 0094 set for growing `pkg/`. The nav, statemachine and track
  packages still have none and stay internal.
- Firmware is not linted by golangci-lint (its `machine` import only exists
  under TinyGo). The shared logic is, which is why the logic lives in
  `pkg/portable` and the firmware stays thin.
- The link protocol (framing, integrity check, sequence numbers, version
  handshake, and the timestamp echo that yields the cross-board clock offset of
  `go-future.md` section 4.6) needs its own decision when it is built. This ADR
  fixes only that it exists and what it must carry.
- TinyGo becomes a pinned tool (`mise.toml`), and its version moves in step with
  the module's Go version.
- Raw gyro over SPI (the reason section 18 wanted a Pico) becomes an
  incremental addition to a board that already exists, not a third board. It is
  still gated on measuring heading drift first, per section 18.3.
- Nothing here is on the Go cutover's critical path (0068). The Zero remains the
  race board until the Pico passes the same bench criteria.

## History

- 2026-09-22: spike on TinyGo 0.42.0, then the extraction and its enforcement.

## Cross-references

- 0021 (hardware profiles as stackable overlays; the board becomes one more axis)
- 0066 (the two-board split this extends)
- 0068 (the migration this does not gate)
- 0094 (the `pkg/` boundary; `pkg/portable` is a stricter subset of it)
- 0095 (the Pi 5 composition root the link target joins)
