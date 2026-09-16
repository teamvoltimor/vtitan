# 0068. The Go stack is a parallel track with a single cutover

- Status: accepted
- Date: 2026-09-15

## Context

A second implementation in Go, on NATS instead of ROS2/DDS, was started as a
long-term replacement for the Python and ROS2 competition stack. A component-by-
component migration that ran both stacks at once would have double the failure
surface and neither stack's advantages.

## Options considered

- (a) Migrate component by component, running a hybrid system.
- (b) Migrate on a parallel track and cut over once, only on verified parity.
- (c) Split reusable modules into separate repositories.

## Decision

(b). Python and ROS2 remain the competition stack and are documented as the
rollback. Go is ported component by component and validated against real bags
(the harness is MCAP replay plus a native sim harness) and cut over in one move
with `vtitan-robot@go` when it reaches full verified parity. The criterion is not
"looks ready": it is verified full parity, and the Go navigator has not yet run a
full race loop on the robot, which is the missing criterion. A hybrid migration is
explicitly forbidden.

Both stacks read the SAME `src/config` TOML tree; Go has no separate config tree
and loads the same paths through its own profile loader.

(c) is refused for now: do not create repositories until there is a concrete
second consumer with no good existing library. Only `foxglove` and maybe
`simgen/sdf` qualify, and even then the recommendation is to move them to `pkg/`
in the monorepo, because separate repos buy only independent versioning and CI at
real maintenance cost.

## Consequences

- There is never a half-migrated system.
- Go lags Python in config coverage: it does not read `sign_discovery.toml` (its
  `MaxIngestRangeM` is 2.0 against the TOML's 1.5), has no equivalent for
  `sensor.toml` or `state_estimator.toml`, and ignores `navigation-challenges/`.
  A Go loader can also succeed without reading a key (a missing `mapstructure`
  tag), which is a known drift hazard.
- Vision has no Go implementation (no HailoRT Go bindings); the camera and NPU
  stay Python behind a sidecar that publishes detections for the Go navigator.

## History

- 258f1f9e 2026-08-28: scaffold the Go/NATS workspace as the long-term ROS2
  replacement, described as such and not as a parallel experiment.
- a0f25133 and 24c1db0b 2026-08-30: MCAP bag-replay harness and navigator parity
  gate.
- 83b082d6 2026-08-30: native sim harness, blind discovery, and the
  `vtitan-robot@` systemd template (python or go swap).
- 4b2d4d72 2026-09-03: port the bay-exit manoeuvre.
- 28724991 2026-09-03: extend the native runner to Obstacles with signs (parking
  not yet).
- d2573633 and 6e752a6b 2026-09-04: record sim runs to MCAP and make `/scan` CDR
  so bags render.
- 48f14281 2026-09-07: state which stack races and where Go stands; only the
  telemetry backend is Go in production.
- 1d3cb4db 2026-09-11: a Python vision sidecar publishing detections for the Go
  navigator.
- f0f13549 2026-09-11: wire the Go track navigator for blind-mode racing; add
  `--config-root` and `--profiles`; add the Go systemd units.
- d6f01332 2026-09-14: source every Go config from TOML, not code defaults.

## Cross-references

- 0069 owns the shared config tree and governance; 0046 (repo layout) stays
  separate and is the reason `src/python` and `src/go` are siblings.

## Evidence

- The entire Obstacles gap was ONE 2.0 s no-progress timeout against Python's
  30.0, not a control-law difference; the shared corner-escape loop (about 20
  percent) is the timeout cause on both stacks.
- The Go pass-side deficit was a scorer artefact: 0/256 by the truth scorer, and
  the bootstrap and green-bias decomposition is VOID.
- Every Go Obstacles baseline measured before 2026-09-05 is VOID: it ran the old
  contact-bounded bay exit at arc 0.3.
- The bay-exit drift cause was `ClearanceGuard=false`, `ArcSteerNorm=0.3` and a
  missing `SpeedScale` in Go, with no `bay_exit` key in any TOML; fixed by making
  the family config-driven.
- Loaders returned SUCCESS while never reading keys they needed (a class bug); the
  structural fix is a shipped-tree completeness test requiring every concrete
  default to appear in the TOML.
- `risk_ray_window` is settled (ship 1) but the key is absent from master, so the
  shipped tree does not express it.
- Go corpus validity is still open: `--blind` withholds no signs,
  `attempt_after_final_lap` is never read, the sim casts from body centre not the
  sensor, and the yaw gain drives sign collisions.
- NATS was chosen over Zenoh: `zenoh-go` is confirmed CGo on an unstable C API (a
  cross-compile blocker) and `go-zeromq/zmq4` is withdrawn (WIP); latency is a
  wash, not a factor.
- The process model is one binary per board, not one per node; `lidar-node` stays
  separate, and fault isolation is via `internal/supervise` recover().
- Protobuf uniformly (no two-tier JSON split), no gRPC or Connect (NATS
  request-reply), cobra/pflag/viper CLIs, and go-playground validator for config
  against protovalidate for the wire.
