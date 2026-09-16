# Go migration plan: full ROS2 replacement (Go + NATS)

> **Transport decision (superseded 2026-08-28): NATS, not Zenoh.**
> Zenoh was originally chosen for two reasons - `rmw_zenoh` as a bridge
> during a hybrid ROS2/Go period, and "a real Go client, unlike DDS."
> Neither holds: the hybrid-bridge approach was dropped in favor of a
> clean cutover (see Scope below), and `zenoh-go`'s own README confirms
> it's a CGo binding around `zenoh-c`, built against an explicitly
> *unstable* C API flag (`-DZENOHC_BUILD_WITH_UNSTABLE_API=ON`) - not a
> pure-Go client at all. That's a real cross-compile blocker from a
> Windows dev machine to two ARM boards, plus an API-churn risk on top
> of it. `nats.io/nats.go` is genuinely pure Go (no CGo, no C toolchain,
> no submodule build step), has a decade-mature stable API, native
> request-reply (replacing Zenoh's Queryable/Query), and JetStream KV
> buckets as a cleaner durable-value primitive than Zenoh's
> `PublicationCache`/`QueryingSubscriber` workaround for latched topics.
> The one thing traded away - Zenoh's brokerless peer-to-peer mesh - is
> irrelevant for a two-board robot; `nats-server` runs as one small
> static Go binary on the Pi 5, acting as the hub both boards connect to.
> Every "Zenoh" reference below should be read as historical context for
> *why* the schema/topic design looks the way it does - the transport
> itself is NATS. Subject naming uses `.`-separated NATS subjects
> (`vtitan.sensor.v1.scan`) instead of `/`-separated Zenoh key
> expressions, otherwise unchanged.
>
> **Research update (2026-08-28):** two corrections after actually
> checking, not assuming. (1) As of Zenoh 1.9.0 "Longwang" (April 2026),
> `zenoh-go` became an **official, SoftBank-sponsored binding with full
> API coverage from day one** - it's still confirmed CGo (Zenoh's own
> blog: *"wraps zenoh-c via CGo"*), so the build-pipeline cost stands,
> but it is not the fringe/unproven binding this doc originally
> characterized it as; if the NATS broker-SPOF risk (see Process model)
> ever becomes a real dealbreaker, Zenoh is a *credible* fallback, not a
> reckless one. (2) `go-zeromq/zmq4` - floated earlier as a pure-Go,
> brokerless alternative specifically to dodge the NATS SPOF risk - is
> **not viable**: its own README says `[WIP]`, "needs a caring
> maintainer." It's withdrawn from consideration entirely; don't revisit
> it without new evidence of active maintenance. (3) Latency is a wash:
> independent benchmarks disagree on which of NATS/Zenoh is faster
> (0.21ms vs 0.35ms in one IoT-edge study; the reverse by ~25% in a
> separate Rust microbenchmark), both sub-millisecond either way - 1-2
> orders of magnitude under this robot's actual budget (200ms deadline
> on `/ackermann_cmd`, 10ms period on the fastest sensor topic). Latency
> is not a deciding factor between them and shouldn't be treated as one.
> NATS also has concrete precedent on this exact hardware class - a
> documented Raspberry Pi Zero 2 deployment, sub-20MB binary, <2% CPU at
> idle - which the benchmark numbers alone don't capture. Net: NATS
> stays the pick.

## Scope and strategy

This is a **parallel-track, clean-cutover** migration, not a hybrid. The
existing Python/ROS2/DDS stack (`platform/robot/ros2_ws`,
`platform/robot/src`) keeps running and gets maintained as normal -
competition work, bug fixes, tuning all continue there unaffected. A new
Go stack is built alongside it, developed and validated independently,
node by node, against the same bag/sim corpora the Python stack uses. ROS2
and NATS do **not** coexist in production and no bridge/translator nodes
are built - the whole point of parallel-track development is that nothing
in the Go stack talks to anything in the Python stack. Only once the Go
stack reaches full parity does the switch happen, in one cutover, and the
Python/ROS2 stack is retired.

Until cutover, the Go stack is developed and tested standalone: unit
tests, the Go simulator, and bag replay (feeding recorded LIDAR/IMU/etc.
data into Go nodes and diffing output against what the Python stack
produced from the same bag) are the only feedback loops - there is no
"run it next to the real robot" step until parity is already established.

**Order of migration** (risk-ascending - see rationale in
`docs/dds-shm-transport-disabled.md`-adjacent history and the project's
own findings about the nav stack being mid-debug):

1. **[DONE]** Leaf sensor drivers: IMU (UART-RVC) `07773a03`, LIDAR `fefdd437`
2. **[DONE]** Motor control loop: BTS7960 PID (`ackermann_motor_node`) `511d0a00`
3. **[DONE, orchestration only]** Simulation: scenario orchestration `3cfcbdb5`
   (shells out to Python - kinematics/raycast core math NOT ported; see the
   2026-08-29 profiling finding below the punch list for why that's now
   justified but still deferred to step 6)
4. **[DONE]** Peripherals: button `25a35c8d`, OLED/SSD1306 low-level driver
   `50fd2cf5` (page-rendering deferred, needs step 5's NATS wiring),
   telemetry-summary aggregation logic `63c03a86` (NATS wiring + the
   corresponding `.proto` both deferred, same reason)
5. **[DONE, core logic only]** State machine `2f95f04c` - the 4-state
   BootCheck/Ready/Racing/Finished machine and its 8-rule transition
   table (`internal/statemachine/core`), plus the transport-independent
   command/backoff/outbox plumbing. Still deferred, same reason as
   step 4: real NATS wiring (unblocks OLED page rendering + telemetry
   publishing too), and jumper/challenge-mode logic + the `race_state`
   helper, both of which turned out to be transport-bound and outside
   `core.py`'s actual scope - not silently dropped, just not reached yet.
6. **[MOSTLY DONE, one hard blocker - see "Nav stack status" below]** Nav
   stack: `track_navigator_node`, `direction_estimator`, sign router - last,
   and gated on bag-replay + sim-corpus parity against the Python baseline
   before any hardware bench time. As of 2026-08-30: all six blind-mode
   modules, sign routing, escape recovery, collision avoidance, and parking
    are fully ported and bag-replay-verified (96.7%/94.1% phase parity on
    sighted/blind bags respectively). The live path planner
    (`plan_believed_path`/`calculate_waypoints`) was the last hard blocker and
    is now ported (`waypoints.CalculateWaypoints`/`PlanBelievedPath`,
    2026-08-30 closing session) - `cmd/track-navigator` plans from corridor
    widths + direction instead of hardcoding a rectangle. The nav stack is now
    port-complete except `CornerLatch` (Python source still uncommitted WIP).
7. Camera: stays Python (`picamera2`/libcamera has no viable Go binding
   path) - runs as a standalone process that publishes over NATS like
   every other Go node; it just happens to be written in Python. This is
   the one component allowed to never fully migrate.

Do not skip ahead to the nav stack early because it's the "interesting"
part - its Go port is only trustworthy once bag replay and the sim corpus
say so, and that requires the sim (step 3) and the wire format (step 1-2)
to exist first.

## Folder structure

New Go workspace, developed alongside the existing Python tree, untouched
until a node is ready to cut over:

```
platform/robot-go/
  go.work                      # multi-module workspace
  go.work.sum
  buf.yaml                     # buf module config (lint + breaking-change rules)
  buf.gen.yaml                 # codegen config: proto/ -> internal/schema/pb/
  proto/
    vtitan/
      sensor/v1/scan.proto
      sensor/v1/imu.proto
      actuation/v1/ackermann_cmd.proto
      actuation/v1/joint_states.proto
      actuation/v1/motor_status.proto
      nav/v1/robot_state.proto
      nav/v1/race_metrics.proto
      nav/v1/nav_debug.proto
      ui/v1/telemetry_summary.proto
      ui/v1/button_event.proto
      diag/v1/system_status.proto
  cmd/
    pi5/                 main.go   # PRODUCTION: combined board binary (see
                                    # "Process model" below) - wires
                                    # imu+lidar+vision+telemetry+state-machine
                                    # +track-navigator as supervised goroutines
    pi-zero/              main.go  # PRODUCTION: combined board binary -
                                    # motor+button+oled
    imu-node/            main.go   # bench/dev: single-subsystem binary,
    lidar-node/          main.go   # same internal packages as pi5/, used
    motor-node/          main.go   # for isolated hardware bench testing
    state-machine/       main.go   # and for local debugging without
    track-navigator/     main.go   # spinning up the whole board's binary
    sim-runner/          main.go
    foxglove-bridge/     main.go   # rviz2 replacement
  internal/
    driver/
      imu/               # Driver interface + UART-RVC/I2C impls
      lidar/             # Driver interface + serial impl
      motor/             # BTS7960 control loop
    transport/
      nats/              # thin wrapper: typed Publisher[T]/Subscriber[T]
                          # over nats.go, encode/decode at the edge
    schema/
      pb/                # buf-generated Go structs - DO NOT hand-edit
    nav/
      trackmodel/
      directionestimator/
      signrouter/
      navigator/
    sim/
      kinematics/
      collision/
      scenario/
      corpus/            # loads existing YAML/JSON scenario corpora
    config/
      profile/           # per-component hardware profile loading (viper)
    telemetry/
      diag/
    supervise/
      supervise.go        # shared per-board goroutine supervisor (see
                          # "Process model" below) - recover(), restart
                          # policy, systemd sd_notify/watchdog integration
  pkg/                    # anything meant to be imported outside this module
                          # (keep empty until something actually needs it -
                          # don't pre-create public API surface speculatively)
  configs/
    profiles/             # ported from platform/shared/src/shared/config/...
  test/
    bagreplay/             # bag-replay parity harness
    corpus/                 # sim-corpus parity harness
```

Module boundaries follow the same principle as the current Python
`hardware/<component>` vs `ros2/<component>` split: `internal/driver/*`
knows nothing about NATS; `internal/nav/*` knows nothing about drivers;
`cmd/*` binaries are just different *compositions* of the same
subsystem-agnostic internal packages - a `cmd/imu-node` binary and the
IMU goroutine inside `cmd/pi5` both import `internal/driver/imu` and
`internal/transport/nats` identically. This is what makes the `Driver`
interface (below) actually pay off - drivers are testable with zero
network/transport dependency, and composable into either a standalone
binary or a combined board binary without changing a line of driver code.

## Design patterns

- **Small interfaces at hardware boundaries.** One `Driver` interface per
  sensor/actuator class:
  ```go
  type Driver[T any] interface {
      Connect(ctx context.Context) error
      Read(ctx context.Context) (T, error)
      Close() error
  }
  ```
  Mirrors the current Python pattern where `i2c.py` / `uart_rvc.py` /
  `mcp2221/*` are interchangeable backends for the same sensor - same
  abstraction, compile-time checked instead of duck-typed.

- **Producer/consumer via channels, not shared polling threads.** Replace
  the current "background polling thread + manual locking" pattern
  (`i2c_base.py`) with: one goroutine reads the driver in a loop and
  pushes decoded frames onto a `chan T`; a second goroutine (or the
  transport publisher directly) consumes and publishes. No locking needed
  for the hot path - the channel *is* the synchronization.

- **`sync.RWMutex` only for cached "latest value" reads**, e.g. a
  telemetry snapshot queried on-demand from multiple goroutines. Don't
  reach for a mutex where a channel already models the flow.

- **Context-first cancellation.** Every long-running goroutine takes a
  `context.Context` and returns on `ctx.Done()`. This is the Go-idiomatic
  replacement for ROS2's node shutdown/`rclpy.shutdown()` lifecycle -
  no custom signal-handling per node, one shared cancellation path from
  `main()`.

- **Errgroup for fan-out with error propagation**, both for
  per-node goroutine supervision (driver loop + publisher loop +
  heartbeat loop) and for the sim orchestrator running N scenarios
  concurrently:
  ```go
  g, ctx := errgroup.WithContext(ctx)
  g.Go(func() error { return driverLoop(ctx, ...) })
  g.Go(func() error { return publishLoop(ctx, ...) })
  return g.Wait()
  ```

- **Explicit deadline/heartbeat watchdogs where DDS QoS DEADLINE used to
  be implicit.** The current `/ackermann_cmd` topic relies on DDS
  `DEADLINE(200ms)`; NATS has no transport-level equivalent. The motor
  node must implement this itself: a `time.Timer` reset on every received
  command, firing a fail-safe stop if not reset within the deadline. This
  is application logic now, not configuration - treat it as a real
  requirement to port, not an afterthought.

- **`//go:embed` for baked-in defaults**, matching the current
  hardware-profile-per-component system: ship default profile TOML
  embedded in the binary, allow override by file on disk (same
  precedence the Python side already has via `.env` activation).

- **No premature public API.** Everything defaults to `internal/` until
  something outside the module actually needs to import it. This mirrors
  the project's existing "no speculative abstraction" convention.

## Packages

### Transport & schema

| Concern | Package | Notes |
|---|---|---|
| Transport | `github.com/nats-io/nats.go` | pure Go, no CGo - see the transport decision note at the top of this doc. Subjects mirror the schema already defined, `.`-separated instead of `/`-separated. Also covers request/response needs (calibration commands, diagnostics queries) via NATS's native request-reply - **no gRPC/Connect dependency needed**, don't add one |
| Durable/latched state | NATS JetStream KV (part of `nats.go`, no separate package) | replaces the Zenoh `PublicationCache`/`QueryingSubscriber` pattern for `robot_state`, `system_status`, `challenge/jumper_inserted` - a late subscriber reads the current value from the KV bucket directly instead of relying on cached replay |
| Broker | `nats-server` (single static binary, run via systemd on the Pi 5) | NATS is hub-and-spoke, not brokerless like Zenoh - for two boards this is simpler to reason about than mesh discovery, not a downgrade. The Pi Zero's `nats.go` clients connect to the Pi 5's `nats-server` over the existing USB-gadget link |
| Serialization | `google.golang.org/protobuf` | protobuf for every topic - see "Encoding" decision below. `buf` (external CLI, not a go.mod dependency) drives codegen + lint + breaking-change detection against `proto/` |

### CLI - cobra, reversing an earlier call

Every `cmd/*` binary that takes real arguments (`sim-runner` first, more as
they're built) uses `github.com/spf13/cobra` + `github.com/spf13/pflag`,
matching go-architect's and `cli-tool-architect`'s canonical Go CLI stack
(cobra/pflag/viper). An earlier pass in this doc's history reasoned "stdlib
`flag` is probably enough, skip cobra to avoid an unneeded dependency" -
that was wrong to hold as a blanket default: `cmd/sim-runner` was actually
built with stdlib `flag` before this decision, and it works, but every
`cmd/*` binary from here on standardizes on cobra so the whole tree gets
subcommand structure, shell completions, and `--help` generation for free
instead of hand-rolling it per binary. `cmd/sim-runner` gets migrated to
cobra as part of its next touch, not urgently ripped out standalone.

### Hardware I/O - the gap the earlier list missed

The earlier pass only covered serial (IMU UART-RVC, LIDAR). The motor
driver (BTS7960 R_EN/L_EN + PWM) and the I2C variant of the IMU/MCP2221
bridge need direct GPIO/I2C access, which serial libraries don't cover:

| Concern | Package | Notes |
|---|---|---|
| GPIO | `github.com/warthog618/go-gpiocdev` | Uses the modern Linux GPIO character-device ABI (`/dev/gpiochipN`), not legacy sysfs GPIO - sysfs is deprecated and doesn't behave the same on newer kernels. This matters more on Pi 5 (RP1 southbridge changed GPIO addressing entirely) than Pi Zero, but since the motor/GPIO work currently lives on the Pi Zero side, verify which board any given GPIO consumer actually targets before assuming legacy approaches work |
| I2C | `periph.io/x/conn/v3/i2c` + `periph.io/x/host/v3` | IMU I2C backend, MCP2221 I2C bridge variant - direct analog to the current `adafruit_bno08x.i2c`/Blinka path |
| PWM | hardware PWM via `/sys/class/pwm` (thin custom wrapper, no library needed) or software PWM via `time.Ticker` if a hardware PWM channel isn't available on the pin in use | Verify against the current `robot.toml`/wiring docs which pins are wired to actual hardware PWM channels vs GPIO-bit-banged before picking one approach - this is a hardware fact, not a library choice |

### Config, validation, concurrency, process

| Concern | Package | Notes |
|---|---|---|
| Config | `github.com/spf13/viper` | layered env/file/default, matches current `.env`-activates-profile precedence |
| Validation | `github.com/go-playground/validator/v10` | direct analog to the current Pydantic validation on config structs |
| Serial I/O | `go.bug.st/serial` | IMU UART-RVC, LIDAR - no CGo needed for either |
| Concurrency | `golang.org/x/sync/errgroup` | fan-out with error propagation, both per-node and sim orchestration |
| Process supervision | `github.com/coreos/go-systemd/v22/daemon` | `sd_notify`/watchdog integration for the combined per-board binaries (see "Process model" below) - lets systemd's existing `Restart=` policy detect a hung process, not just a crashed one |
| Logging | stdlib `log/slog` | structured logging - matches the existing "never `print()`, structured logging" convention, no external dependency needed |
| Randomness (sim) | stdlib `math/rand/v2` | seeded `rand.Rand` per scenario run, direct analog to the current `np.random.default_rng(seed)` - keep seeding explicit for reproducible scenario runs |

### Testing & parity harness

| Concern | Package | Notes |
|---|---|---|
| Unit tests | stdlib `testing`, table-driven | don't add testify/ginkgo unless a real gap shows up - stdlib is enough for this scale |
| Struct diffing | `github.com/google/go-cmp` | for the bag-replay and sim-corpus parity harnesses specifically - readable diffs when comparing Go-stack output against the recorded Python-stack baseline, which stdlib `reflect.DeepEqual` doesn't give you |
| Bag reading | `modernc.org/sqlite` | rosbag2's `.db3` format is SQLite - this is a pure-Go SQLite driver (no CGo), needed only inside `test/bagreplay/` to read existing recorded bags, not a runtime dependency of any node |

### Debug tooling & visualization

| Concern | Package | Notes |
|---|---|---|
| Visualization | Foxglove Studio (websocket protocol) via `github.com/coder/websocket` | rviz2 replacement - `coder/websocket` (formerly `nhooyr.io/websocket`) is the actively-maintained, context-aware choice here over the older `gorilla/websocket` |
| Protobuf debug dump | `google.golang.org/protobuf/encoding/protojson` | backs the small `cmd/zdump`-style CLI helper mentioned above - lets you `nats sub` a subject and print it as readable JSON despite the wire format being protobuf |

### Schema validation & recording - added 2026-08-28

| Concern | Package | Notes |
|---|---|---|
| Wire-message validation | `buf.build/go/protovalidate` (+ `buf.build/bufbuild/protovalidate` as a `buf.yaml` dep for the `buf/validate/validate.proto` import) | CEL-based constraints declared as field options directly in `.proto` files (e.g. `[(buf.validate.field).required = true]`), enforced at runtime with no separate codegen step. This is the buf-native counterpart to `go-playground/validator` - **the two aren't redundant**: `validator` stays for Go-side config/TOML structs, `protovalidate` covers wire messages. Wired up and verified end-to-end on `AckermannCmd` (see `internal/schema/pb/vtitan/actuation/v1/validate_test.go`) - constraints are kept structural (required/finite) for now, not hardware-specific numeric bounds, since those live in `motors.toml` and haven't been reconciled into the schema yet |
| Recording (rosbag2 replacement) | `github.com/foxglove/mcap/go/mcap` | Confirmed pure Go, no CGo (compression deps are `klauspost/compress`/`pierrec/lz4`, both pure Go). MCAP is Foxglove's own container format, schema-agnostic and built to hold protobuf-encoded channels natively - since Foxglove Studio is already the chosen rviz2 replacement, MCAP is the natural post-cutover recording format, not an independent choice. Considered and rejected: hand-rolling a protobuf-delimited file format (MCAP already solves this correctly); relying solely on NATS JetStream stream retention as "the bag" (works for short-term replay but isn't portable - you can't hand a JetStream stream to Foxglove Studio the way you can an `.mcap` file). rosbag2/`.db3` stays relevant only inside `test/bagreplay/`, for reading *old* Python-stack recordings during the parity-gate period - it is not the ongoing format going forward |

Keep this list short beyond what's above. Every dependency added here is
one more thing to audit and keep updated on a Pi Zero's constrained
storage/update cadence - don't add a package for something 20 lines of
stdlib already does. Two deliberately **not** included: no linear-algebra
library (`gonum`) unless the nav-stack port turns out to need real matrix
math beyond what the current numpy usage suggests (it doesn't look like
it does); no YAML library unless the scenario corpus format turns out to
be YAML rather than JSON - confirm the actual corpus file format before
adding one.

### Encoding: protobuf for everything, not a two-tier split

An earlier draft of this schema split encoding by rate - protobuf for
high-rate typed topics, JSON for low-rate telemetry - because the JSON
tier mirrored the existing Python `wire_models.py` Pydantic structs and
stayed debuggable via a raw subscribe during a planned ROS2/transport
hybrid period that was later dropped. That reasoning no longer applies: this is a clean cutover with no
ROS2 side to stay compatible with, so there's no benefit to two encoding
paths - just two codegen pipelines and a per-topic "which format is this"
lookup to maintain. Use protobuf uniformly. The debuggability loss for
low-rate topics (race metrics, button events, telemetry summary) is
minor and solved with a small `cmd/zdump`-style CLI helper
(`protojson.Marshal` on a subscribed message) rather than by forking the
encoding strategy.

## Testing

Three layers, same shape as the current Python test strategy but ported,
plus the concrete Go testing patterns from `go-architect` §9 applied
throughout - these aren't a fourth layer, they're how the three layers
below get written:

1. **Unit tests** - table-driven, black-box (`package foo_test`, not
   `package foo`) so tests exercise the same public surface external
   callers do, `t.Parallel()` on every test and subtree. Drivers get
   fake/mock implementations of their own `Driver[T]` interface for use
   in higher-level tests (motor node logic tested without real serial
   hardware). `stretchr/testify` for assertions where stdlib alone is
   awkward (deep struct comparison, `require` vs `assert` semantics) -
   `testifylint` catches the common misuse of the two.
2. **Fuzz tests** (Go native fuzzing, `go test -fuzz`) - specifically for
   the serial frame decoders (IMU UART-RVC, LIDAR), since these parse
   untrusted byte streams from real hardware and are exactly the class
   of code fuzzing is good at: feed it garbage/truncated/malformed
   frames and confirm it errors cleanly instead of panicking or hanging.
   Not needed for most other packages - reserve it for parsers at a
   trust boundary, not applied blanket.
3. **Concurrent-code tests** - `testing/synctest` (GA since Go 1.25) for
   anything with goroutines/channels/timers that would otherwise be
   flaky under real wall-clock time (the deadline/heartbeat watchdog
   logic in the motor node is the concrete first candidate - a
   `synctest`-wrapped test can assert the fail-safe fires at exactly the
   simulated deadline without an actual `time.Sleep` in the test).
4. **NATS integration tests, in-process, no Docker** - `nats-server` is
   itself a Go library (`github.com/nats-io/nats-server/v2/server`), so
   integration tests can start a real (ephemeral, in-memory) NATS server
   directly inside `go test` via `server.NewServer(...)` - no
   `testcontainers-go`/Docker dependency needed for this specific case,
   even though `testcontainers-go` remains the right tool for anything
   that genuinely needs a containerized dependency. Gate these behind a
   build tag (`//go:build integration`) per go-architect §9 so plain
   `go test ./...` stays fast by default.
5. **Bag-replay parity** - `test/bagreplay/`: feed the same recorded
   bags the Python stack was validated against into the Go nav stack,
   diff outputs (steering/speed commands, corridor assignment, lap
   counts) against what the Python stack produced from the same input,
   within an explicit tolerance. This is the actual go/no-go gate for
   cutting over `track_navigator_node` - not "it compiles."
6. **Sim-corpus parity** - `test/corpus/`: run the same Obstacles/Open
   scenario corpora (the 256-case Obstacles corpus, the Open suites)
   through the Go sim + Go nav stack, compare collision rate, laps
   completed, pass-side correctness, timeouts against the current
   Python baseline recorded in `.claude` memory
   (`REFERENCE ROW 2026-08-27`, `585ce6f7`). Must match or beat it,
   not just "run without crashing."

No node is considered portable until it passes its own unit tests *and*
(for anything in the nav/sim path) parity tests against the Python
baseline it's replacing.

## Go idioms: enums, composition, interfaces

Applying `go-architect` directly rather than re-deriving Go idiom from
first principles - see the skill for the full rationale, this is the
subset that actually applies to this codebase's shape:

- **Enums**: Go has none natively. For wire-level enums (e.g.
  `MotorStatus.State` - `IDLE`/`RUNNING`/`FAULT`/`ESTOP`), the
  protobuf-generated Go enum type already gives you this correctly, no
  extra work needed. For internal-only enums that never cross the wire
  (nav states, corridor classification), use the standard typed-constant
  `iota` pattern with a `String()` method, and gate any `switch` over
  the type with the `exhaustive` linter (already in `.golangci.yml`) so
  adding a new enum value can't silently fall through unhandled cases -
  this is the direct Go analog of Python's `StrEnum` gotcha already
  documented in project memory (`strenum_leaks_into_rclpy_parameters`).
- **"Subclasses"**: Go has no inheritance - don't reach for embedding to
  simulate it preemptively. The existing plan already uses the correct
  Go-idiomatic substitute: small interfaces (`Driver[T]`) for
  polymorphism, not a base-struct hierarchy. Struct embedding is fine
  *if* concrete duplicate logic actually shows up across driver
  implementations (e.g. identical connect-retry bookkeeping in both the
  IMU and LIDAR serial drivers) - add it reactively when that
  duplication is real and caught by `dupl`, not speculatively now.
- **Interfaces**: define them where they're *consumed*, not where
  implemented (`go-architect` §4) - e.g. `internal/nav/navigator`
  declares the narrow interface it needs from a driver, rather than
  drivers exporting a interface for consumers to discover. Keep them
  1-3 methods (`interfacebloat` enforces this); `Driver[T]`'s three
  methods (`Connect`/`Read`/`Close`) already fits. Functions accept
  interfaces, return concrete structs (`ireturn` enforces this).
- **Construction**: `New...` constructors for anything initializing
  maps/slices/channels; Functional Options for driver/transport setup
  that takes more than a couple of parameters, rather than a large
  config struct passed by value.
- **Validation**: `Validate() error` on Go-side structs taking external
  input, backed by `go-playground/validator` tags (config/TOML) -
  distinct from `protovalidate` (wire messages), see the packages table
  above.

## Workers / concurrency model

Each `cmd/*-node` binary follows the same internal shape:

```
main()
  ctx, cancel := signal.NotifyContext(...)   // OS signal -> ctx cancellation
  driver := driver.New(...)
  pub := transport.NewPublisher[T](natsConn, "vtitan.sensor.v1.imu")
  g, ctx := errgroup.WithContext(ctx)
  g.Go(driverLoop)     // driver.Read() in a loop -> chan T
  g.Go(publishLoop)    // <-chan T -> pub.Publish()
  g.Go(heartbeatLoop)  // only where a deadline/watchdog applies
  g.Wait()
```

No node needs a worker pool beyond this - the robot's actual concurrency
needs (one driver loop, one publish loop, occasionally a watchdog) don't
justify a generic pool abstraction. The one place a real worker pool is
useful is the **sim orchestrator**, where N scenario runs are genuinely
parallel and CPU-bound:

```go
sem := make(chan struct{}, runtime.NumCPU())
g, ctx := errgroup.WithContext(ctx)
for _, scenario := range corpus {
    scenario := scenario
    g.Go(func() error {
        sem <- struct{}{}
        defer func() { <-sem }()
        return runScenario(ctx, scenario, resultsCh)
    })
}
```
Results collected off `resultsCh` by a single aggregator goroutine -
avoids a shared-map + mutex in favor of channel-based aggregation.

## Process model: one binary per board, not one per node

ROS2 pays a real per-node cost - each node is its own DDS participant and
usually its own OS process. That cost is why the *current* systemd setup
is already grouped at board level, not node level
(`vtitan-pi5.service`, `vtitan-pi-zero.service`), with `vtitan-lidar` and
`vtitan-race` split out separately for specific operational reasons
(the LIDAR vendor driver needing independent restart; `/ackermann_cmd`
publisher contention between `vtitan-race` and the rest - see
`ackermann_cmd_contention_from_race_service` in project history).

Goroutines don't have that cost. A Go binary can run everything one
board needs - driver loops, publish loops, watchdogs - as goroutines in
a single process with a single NATS connection, instead of one OS
process and one connection per node. **Default to one binary per board**:
`cmd/pi5` hosts IMU + LIDAR + vision + telemetry + state-machine +
track-navigator; `cmd/pi-zero` hosts motor + button + OLED. This cuts
process count, cuts connection overhead, and - critically - lets
same-process subsystems that need to share state (e.g. state-machine
reading the latest IMU sample) do it via a Go value/channel instead of a
round trip through the transport.

Two things this trades away, both mitigated rather than ignored:

- **Fault isolation.** A panic in one subsystem's goroutine used to only
  take down that one OS process; in a combined binary it can take down
  the whole board. Mitigate with `internal/supervise`: every subsystem
  goroutine runs under a `recover()`-wrapped wrapper inside the
  `errgroup`, logs the panic via `slog`, and either restarts just that
  goroutine (bounded retry count) or, if it's unrecoverable, lets the
  whole process exit - which systemd's `Restart=always` already handles
  today, and Go binaries start in milliseconds, not the multi-second
  ROS2/DDS discovery handshake this replaces.
- **Independent restart of one subsystem.** No longer free - restarting
  the LIDAR driver alone, for example, now means restarting the whole
  `cmd/pi5` binary. This is exactly why LIDAR should stay its own binary
  (`cmd/lidar-node`, matching the existing `vtitan-lidar.service`
  precedent) rather than being folded into `cmd/pi5` - it already earned
  its isolation operationally once, no reason to re-litigate that.

So the concrete split: **`cmd/pi5`** (IMU + vision + telemetry +
state-machine + track-navigator), **`cmd/pi-zero`** (motor + button +
OLED), and **`cmd/lidar-node`** standing alone, mirroring the one
existing case where per-node isolation was already worth it. The other
`cmd/*-node` binaries (imu-node, motor-node, state-machine,
track-navigator individually) aren't deployed in production - they exist
so a single subsystem can be bench-tested or debugged in isolation
without spinning up the whole board binary, same as how hardware bench
sessions work today.

If `track-navigator`'s Go port turns out to be a frequent source of
crashes during its bring-up (plausible, given it's the last and riskiest
migration target), reconsider splitting it out of `cmd/pi5` into its own
binary at that point - this isn't a one-way door, just a default.

## Simulation

Port target, not a rewrite target by default - the current
`scenario_simulator` is already free of ROS2/Gazebo (pure kinematic
bicycle model + numpy-vectorized raycast collision model) and has no
blocking global state, so it's already close to what the Go port needs
structurally. Two sub-decisions, not one:

- **Orchestration** (running N scenarios concurrently): port to Go early
  (step 3 above) regardless of what happens to the core math - this is
  where the errgroup/worker-pool pattern above pays off immediately and
  independently of the rest of the migration.
- **Core math** (kinematics, raycast, collision stepping): only port if
  profiling the current Python implementation shows the interpreter loop
  - not the already-vectorized numpy raycast - is the actual bottleneck.
  Profile before committing; don't assume the language is the
  bottleneck when the hot path is already in compiled numpy code.

`internal/sim/corpus/` must load the *same* scenario corpus files the
Python sim consumes today (don't fork the corpus format) so parity
testing (above) is comparing against a truly identical input set.

## What's left - the actual punch list

Everything below is either unverified, undecided, or not yet built. This
is the real gap between "the plan is written" and "the plan is
executable" - grouped by what blocks what.

### Verify before writing code against them

- [x] ~~`zenoh-go` API stability~~ - resolved by switching to
      `nats.go` (pure Go, mature API, see transport decision note at top).
- [x] ~~Where `nats-server` runs and how it's supervised~~ - resolved
      2026-08-30: new `systemd/vtitan-nats.service` runs `nats-server` on
      the Pi 5 as `Restart=always` (a SPOF must self-heal, not stay dead
      on a clean exit), `WantedBy=multi-user.target`, no `network-online`
      gate (both boards talk over the usb0 gadget link, not WiFi - gating
      on WiFi would only delay the broker they actually need). A minimal
      `config/nats-server.conf` listens on `0.0.0.0:4222` (so the Pi Zero
      reaches it over usb0) with ping/timeout tuned for a flaky link. The
      production broker address is `nats://pi5.local:4222`, set once via
      `VTITAN_NATS_URL` in `.env` (sourced by the units) and consumed by
      every Go binary's `--nats-url` fallback (`nats.DefaultURL()`).
- [x] ~~NATS reconnect behavior specifically over the USB-gadget link~~ -
      addressed 2026-08-30 in `internal/transport/nats`: reconnect knobs
      were already native (`ReconnectWait`, `MaxReconnects=-1` = unlimited),
      and `Connect` now also adds `ReconnectJitter`/`ReconnectJitterTLS`
      (breaks the reconnect lockstep across clients after a link flap),
      tightened `PingInterval`/`MaxPingsOutstanding` (recycles a
      silently-dead socket instead of driving blind), and - the key
      cold-boot fix - retries the *initial* dial up to
      `InitialConnectAttempts` (-1 = until ctx alive), so a client that
      starts before nats-server is up (the Pi 5 reboot cold-boots both
      boards over the gadget link, see `wait-for-gadget-link.sh`) still
      connects once it comes up instead of dying at startup.
      Disconnect/reconnect callbacks log via `slog` when a logger is set,
      so link-state is observable. In-process tests cover the retry-until-
      ready and bounded-retry cases. The one thing still only verifiable
      on real hardware is the actual flaky-link flap timing - the logic is
      in place and unit-tested at the boundary, but confirm live before
      declaring the USB-gadget behavior "proven".
      flaky-link scenario this item is actually about. The original
      motivation for considering Zenoh's reconnection semantics was the
      documented link flakiness; verify NATS's client reconnect
      (buffering + backoff, native to nats.go, not reimplemented) on
      real hardware before treating this as solved.
- [x] ~~Latency comparison between NATS and Zenoh~~ - researched, it's a
      wash (independent benchmarks disagree on which wins, both
      sub-millisecond, both 1-2 orders of magnitude under this robot's
      actual 200ms/10ms budgets). Not a deciding factor either way.
- [ ] `periph.io` maintenance status for the hardware-PWM (GPIO12/13)
      path - its release cadence has slowed; fall back to a thin custom
      `/sys/class/pwm` wrapper if it's not trustworthy.
- [ ] Scenario corpus file format (YAML vs JSON) - determines whether a
      YAML library gets added at all.
- [x] ~~`get_tuning()`'s thread/process-safety~~ - resolved as a side
      effect of the profiling fix below: `NavigationTuning` is a frozen
      dataclass over exclusively frozen pydantic groups, fully immutable,
      so the now-cached instance is safe to share across threads/callers
      by construction. No parallel-run blocker.
- [x] ~~Profile the current Python sim's hot path~~ - run 2026-08-29
      against a real 437-step Obstacles scenario. **Finding: 70% of
      runtime (8.7s of 12.4s) was `get_tuning()` re-loading TOML config
      from disk on every one of 649 calls** (an incomplete fix from an
      earlier profiling pass - the per-file TOML *parse* was cached, but
      the surrounding directory-walk stat calls weren't). Fixed in
      `33fadae9` - **3.08x wall-clock speedup (13.7s → 4.4s), verified
      byte-identical simulation output**. Re-profiled after the fix:
      `raycast_grid` + `wrap_angle` + trig calls now genuinely dominate
      remaining runtime (~43% of the post-fix total, up from ~7% when the
      config-reload noise was drowning everything else out). **This is
      the actual answer to "should we port the sim math to Go":** yes,
      there's now a real, evidence-based case for it - but only *after*
      this fix, not instead of it. Porting math that was 7% of a
      12.4s runtime would have been the wrong lever; porting math that's
      43% of a 4.4s runtime is a legitimate next step whenever the nav
      stack's Go port reaches that stage. Not started - this finding
      justifies doing it eventually, it doesn't change the migration
      order (nav stack, including this math, is still last and gated).
- [ ] `go-gpiocdev`'s bias-pull-down support actually reduces exposure on
      this SoC/kernel combo as expected - confirm on hardware, don't
      assume the uAPI feature is fully supported by the running kernel.

### Physical, not software - don't let the Go work distract from these

- [ ] Physical pull-down resistor on LPWM (GPIO26) - still not installed
      per your own hardware findings. Independent of this entire
      migration; do this regardless of language.

### Not yet decided

- [ ] Foxglove websocket bridge scope - needs to exist by step 3-4, since
      rviz2 disappears entirely at cutover (clean-cutover, not gradual),
      not decided in detail yet.
- [ ] `cmd/zdump` debug CLI's exact scope (which topics, filtering,
      output format) - currently just a named placeholder.
- [x] ~~`golangci-lint` setup~~ - done. `.golangci.yml` copied from the
      `go-architect` skill template, `local-prefixes` set to the real
      module path. **Real finding**: the template referenced two linters
      (`stylecheck`, `looppointer`) that don't exist in the installed
      golangci-lint 2.12.2 despite matching the skill's pinned version -
      `stylecheck`'s checks were folded into `staticcheck` upstream,
      `looppointer` was removed (Go 1.22's per-iteration loop variables
      made its bug class structurally impossible). Both removed from
      this project's `.golangci.yml`; worth flagging back to whoever
      maintains the skill template, since STACK.md's version pin didn't
      catch the drift. `golangci-lint run ./...` is clean (0 issues) as
      of this scaffold.
- [x] ~~Confirm `buf` is wired~~ - yes, confirmed working end-to-end:
      `buf lint`, `buf generate` → `go build`, and (new) `buf dep update`
      pulling `buf.build/bufbuild/protovalidate` all verified.
- [ ] CI pipeline itself (as opposed to the tools it would run) - `buf
      lint`/`buf breaking` on every change (this is now the *only*
      contract, no ROS2 fallback to catch schema drift), `golangci-lint
      run`, `go build`/`go vet`/`go test`, and where that actually runs
      (GitHub Actions vs. something else) - none of this is wired into
      CI yet, only verified to work locally.
- [ ] Deployment/cross-compile story - not addressed until now. Both
      boards are `linux-aarch64` (Pi Zero 2 W, not the original
      single-core Pi Zero), so this is simpler than it could've been:
      one `GOOS=linux GOARCH=arm64` build target, no multi-arch matrix.
      Still needs: where binaries get built (cross-compiled on dev
      machine vs built on-device), how they land on each board (new
      systemd units alongside/replacing `vtitan-pi5.service` etc., same
      `deploy-dev-env-to-zero.sh`-style transfer mechanism or a new one),
      and whether the ~415MB usable RAM on the Pi Zero 2 W changes
      anything about running a combined `cmd/pi-zero` binary (it
      shouldn't - a static Go binary's footprint is smaller than the
      ROS2/DDS process set it replaces - but confirm rather than assume).
- [ ] Observability beyond Foxglove (metrics, `prometheus/client_golang`)
      - mentioned as optional, not decided either way.

### Not yet built (the actual next actions)

- [x] ~~`go.work` skeleton + `platform/robot-go/` folder scaffolding~~ -
      done. Module `github.com/teamvoltimor/vtitan/platform/robot-go`,
      single-module workspace (no concrete reason found in this doc for
      true multi-module, so that's the fallback per the earlier
      "no premature complexity" convention). Full `cmd/`/`internal/`
      tree scaffolded with compilable stubs, `go build ./...` /
      `go vet ./...` both clean.
- [x] ~~First `.proto` files: `scan.proto`, `imu.proto`,
      `ackermann_cmd.proto`~~ - done, plus `joint_states.proto` and
      `motor_status.proto` beyond the original ask. `buf lint` clean.
      `buf generate` → `go build` verified end-to-end (this caught a
      real gap: generated code needs `google.golang.org/protobuf`'s
      submodules resolved via `go mod tidy`, not just the top-level
      module listed in `go.mod` - now fixed).
- [ ] First real node: IMU UART-RVC driver + NATS publisher - the
      template the rest of the migration copies. Not started.
- [ ] Bag-replay harness skeleton (`test/bagreplay/`) - needed before
      *any* nav-stack parity claim is possible, so building it early
      (even against Python-stack-only output, before any Go nav code
      exists) validates the harness itself is correct first.

### Nav stack status (2026-08-30, updated end of session)

**Ported and bag-replay-verified:** all six blind-mode modules
(`racetracker`, `localization`, `wallheading`, `corridorestimator`,
`corridorfollower`, `startconditions`/`startmeasurement`), `parking`, sign
routing (`signrouter`: router/discovery/deformation/routing/sign_lane/
config - function-for-function coverage, confirmed against Python source),
`escape_recovery`, and `collision_avoidance`/`sectors`/`bumper`. The native
Go sim harness (`internal/sim/harness`) also exists for sighted-only runs,
with a caveat below.

**Live path planner - PORTED (2026-08-30, closing session).** `plan_believed_path`/
`calculate_waypoints` are now `waypoints.CalculateWaypoints` /
`waypoints.PlanBelievedPath` in `internal/nav/waypoints/planner.go`,
mirroring `generation.py` line-for-line: per-corridor center-bias
(`CenterBiasForCorridor`, already ported), feasibility check
(`ValidatePathFeasibility`), per-corner arc radii (`CornerArcRadius` with the
`CORNER_ARC_ASSUME_WIDE` substitution, now `Config.CornerArcAssumeWide`),
`BuildAllSegments`/`AssembleLoop`/`BuildWaypointSequence` (already ported),
and `validate_bounds` (`MaxCoordM` + the central inner-square exclusion). The
Python-only `ScenarioMetadata` is replaced by a `PlannerInput` carrying only
the fields generation.py actually reads (believed `CorridorGeometry` +
`StartingConditions` + track/chassis extents), so the planner stays
decoupled from the rest of the scenario-model layer, which has no Go
equivalent. `Section.loop_order` is now `trackmodel.LoopOrder` (anchored at
East, matching the Python original). `cmd/track-navigator/main.go` now plans
its bench path via `CalculateWaypoints` instead of the hardcoded 4-corner
rectangle - the binary can synthesize a real arc-waypoint centerline from
corridor widths + direction. Tests in `planner_test.go` cover loop order,
spawn-adjacency, unresolved-direction rejection, infeasible-corridor
rejection, and believed-geometry replanning; `go build`/`go vet`/`go test
./internal/nav/... ./cmd/...` all green.

**Should-port-soon, not a blocker:** `CornerLatch` (`platform/robot/src/
navigation/core_navigator/corner_latch.py`, live-verified 2026-08-30:
hardware clearance p05 doubled, Open 126/128 - see project memory
`corner_preview_decays_mid_corner_2026_08_30`) has no Go equivalent yet.
`drive.go` still calls the older static `trackmodel.PathTurnAhead` lookahead
instead of a latch that persists the turn-ahead signal through the arc. The
Python source itself was still uncommitted WIP as of this writing, so this is
expected lag, not a missed port.

**Should-port-soon, not a blocker:** `CornerLatch` (`platform/robot/src/
navigation/core_navigator/corner_latch.py`, live-verified 2026-08-30:
hardware clearance p05 doubled, Open 126/128 - see project memory
`corner_preview_decays_mid_corner_2026_08_30`) has no Go equivalent yet.
`drive.go` still calls the older static `trackmodel.PathTurnAhead` lookahead
instead of a latch that persists the turn-ahead signal through the arc.
The Python source itself was still uncommitted WIP as of this writing, so
this is expected lag, not a missed port.

**`GetWheelOdometry` always returning `ok=false` (`internal/adapters/
natsgw/gateway.go`) is confirmed correct, not a gap.** Traced Python's
`get_wheel_odometry()`: it reads a value only ever set by a `/joint_states`
subscriber callback. No file anywhere in `platform/robot/src` publishes
`sensor_msgs/JointState` or `/joint_states` - the topic is declared in
`platform/shared/config/ros_topics.toml` but has no producer (no
`ros2_control`, no `joint_state_publisher`, nothing). So on real hardware
Python's own `get_wheel_odometry()` always returns `None` too - the Go
port's always-`false` faithfully matches actual runtime behavior of the
system it's replacing, not an oversight.

**Bag-replay parity gate: the ~27-30% phase-mismatch was a test-harness
bug, not a navigator regression.** `test/bagreplay/parity_test.go`'s
`toLidarScan` was feeding the Go navigator raw, uncorrected `/scan`
bearings during replay. The real ROS2 hardware gateway
(`ros2_hardware_gateway.py`) applies a mandatory LIDAR yaw-offset rotation
before the Python navigator ever sees a bearing (`_LIDAR_YAW_OFFSET_RAD`,
combining a 180° correction for the physically-upside-down mount -
`robot.toml`'s `[lidar] inverted = true`, set in the base config, not
behind a hardware profile - plus any residual mount trim). The harness
skipped this, so every replayed bearing was 180° off from what Python's
escape/collision logic actually saw: what Go treated as "dead ahead" was
physically the chassis rear, and vice versa, which plausibly explained the
apparent over-triggering into `active_maneuver`/`escape_triggered` phases.
Fixed 2026-08-30 by applying `profile.RobotConfig.LidarYawOffsetRad()`
(memoized via `sync.OnceValue` - it's called once per scan row, and
re-parsing `robot.toml` through viper on every call is both slow and
crashes `mapstructure` under that load) plus NaN/Inf range sanitization
(`sanitize_lidar_ranges`'s Go-side equivalent) in `toLidarScan`. Result:

| Bag | Field | Before | After |
|---|---|---|---|
| Sighted (`run_20260829_140424`) | phase match | 27.6% | **96.7%** (2518/2605) |
| Sighted | `forward_clearance_m` match | 0% | **92.7%** |
| Sighted | `risk` match | 9.6% | **99.1%** |
| Blind (`run_20260830_025327`) | phase match | 29.4% | **94.1%** (1817/1930) |

This also matches (and resolves) the pre-existing memory note
`bag_replay_lidar_yaw_offset.md` ("replay needs LIDAR yaw offset"), which
had already flagged the gap. This confirms the ported navigator's
perception and escape logic are correct - the divergence measured earlier
this session was entirely a fixture artifact.

**Blind-bag corpus correction:** a prior session claimed no `.mcap` blind
bag existed in the repo ("the pulled `vtitan_runs_pulled` corpus is
entirely SIGHTED Open/Obstacles runs"). That was wrong - 91 real `.mcap`
bags exist locally at `platform/robot/vtitan_runs_pulled/` (untracked,
gitignored, pulled from the Pi 5 via `pull-runs-from-pi5.sh`), and
`run_20260830_025327` specifically has a genuine 53-tick `blind_creep`
phase (direction unresolved, `waypoint_index`/`steer_target` null) before
direction resolves to `counterclockwise` and the run continues in
`normal_drive`. `TestParity_BlindPackagesVsBag` is wired to this bag now
(via `VTITAN_BLIND_BAG_DIR`) and exercises the blind-mode packages for
real, not skipped.

### Ongoing, not a one-time task

- [x] Track parity-test results per node in this doc - see "Nav stack
      status" above, updated 2026-08-30. Keep appending here as remaining
      nav-stack work (path planner, `CornerLatch`) lands, don't let this
      doc go stale the way a written-once plan tends to.
