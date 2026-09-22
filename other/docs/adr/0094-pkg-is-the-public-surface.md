# 0094. `pkg/` is the Go module's public surface and may not import `internal/`

- Status: accepted
- Date: 2026-09-21

## Context

Every Go package in `src/go` lived under `internal/`, which is correct Go
default: nothing outside the module can import it, so nothing has to be
maintained as an API. But several of those packages are not the robot. The
BNO085 RVC parser, the RPLIDAR scan assembler, the BTS7960 controller, a
goroutine supervisor with systemd `sd_notify`, a typed publisher/subscriber over
nats.go, MCAP recording, angle and clamp helpers: none of them contains a WRO
rule, and 0068's own package-extraction note already named `foxglove` and
`simgen/sdf` as candidates.

Leaving them in `internal/` has a specific failure mode beyond tidiness. A
package nothing can import from outside is under no pressure to stay
importable, so it acquires repo-shaped dependencies quietly. Every driver had
already done exactly that: `imu.ConfigFor` read `internal/config/profile` and
the generated hardware DTOs, so the BNO085 driver could not be used without
vTitan's TOML tree. `internal/driver/camera/nats.go` had gone further and
imported the transport, breaking the layering rule the folder structure claims.

## Options considered

- (a) Leave everything under `internal/` and extract only when a consumer
      outside the module appears.
- (b) Move the packages that are infrastructure rather than robot into `pkg/`,
      and forbid `pkg/` from importing `internal/`.

## Decision

(b). `pkg/` holds `driver/{imu,lidar,motor,encoder,button,display/ssd1306}` and
the `Driver[T]` contract, `backoff`, `supervise`, `transport/nats`,
`protoschema`, `recording`, `foxglove`, and `geom` + `control` split out of
`nav/navutil`. `internal/` keeps the robot: all of `nav/*`, `sim/*`, `simgen/*`,
the `node/*` run loops, `config/*`, `telemetry/diag`, `statemachine/*`.

The test is not "has no repo imports". `statemachine/core`, `sim/corpus`,
`nav/trackmodel` and `nav/wallheading` have none either, and stayed: they are
self-contained because WRO geometry is self-contained, which is not the same as
being reusable. The test is whether the package would mean anything to someone
who is not running this robot.

A depguard rule in `.golangci.yml` denies `internal/` from `pkg/**`, so the
property is a build failure rather than a habit. Test files are exempt: an
external consumer never compiles `_test.go`, so a test reaching into
`internal/` constrains nobody, and `protoschema`, `recording` and `foxglove` all
use this repo's own message types as sample data.

Driver profile resolution moves to `internal/hwconfig` (`IMU`, `LIDAR`,
`Motor`, `Button`, `Display`, `Encoder`), because `pkg/` cannot read
`internal/config`. Each function keeps its driver's previous fallback
behaviour.

The camera moved last, and not as a file move: its NATS-backed source was
wired into `camera.New`'s own factory switch, so it first had to leave for
`internal/adapters/natscamera`. It is now `pkg/driver/camera`, and the one
driver in `pkg/` needing cgo via gocv.

## Consequences

- The drivers are the only packages in the tree that do not own their own
  `ConfigFor`. `navigator`, `waypoints`, `signrouter`, `controllers` and a dozen
  more still do, across 40+ call sites. The asymmetry is the price of the
  boundary and should be explained rather than "fixed".
- Changing a driver `Config` field is no longer a free refactor: `pkg/` is
  public API in intent even while it has one consumer.
- The depguard rule only bites where golangci-lint runs. The `robot-go` CI job
  runs it on every push, with the findings that predate it grandfathered by a
  pinned `--new-from-rev`; the rule was confirmed to still fire in that mode.
- Nothing is extracted to a separate repository. 0068 refused that and still
  does; if a driver ever leaves, upstream (periph.io, `tinygo.org/x/drivers`)
  beats a directory in a competition monorepo.

## History

- b7b58e94 2026-09-21: drivers and the `Driver[T]` contract to `pkg/driver`,
  profile resolution to `internal/hwconfig`.
- 4a36ebb7 2026-09-21: the infrastructure packages follow, `navutil` splits into
  `pkg/geom` + `pkg/control`, and the depguard rule lands. The rule's first
  version matched nothing (its glob missed relative paths) and was caught by
  planting a violating import, not by reading it.
- bc183bc1 2026-09-21: the camera's NATS source moves to
  `internal/adapters/natscamera`, and the camera follows the other drivers.
  This ADR's Decision kept saying the camera "did not move" for a day after
  it had, until the docs check (0097) flagged the dead path.
- d67946e1 2026-09-22: golangci-lint joins the `robot-go` CI job, so the
  depguard rule runs on every push.

## Cross-references

- 0068 (the parallel-track migration whose extraction note this settles)
