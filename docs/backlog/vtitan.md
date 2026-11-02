# vtitan backlog

Queue for this repo, created 2026-11-01 from the [reuse audit](../../../../../libraries/go/identity/docs/reviews/2026-11-01-reuse-audit.md).
vtitan already takes five kits (`grpckit`, `resilience`, `svckit`, `termkit`, `tick`) and its own
`pkg/` tree is mostly domain (driver 4,313, recording 1,707, transport 831, boardsim 815), so the
rows below are small.

## Later

- [ ] One module host: the five kits are required from `github.com/ralvarezdev/*` while the
  `go.ralvarez.dev` vanity exists. Platform queue R153 decides the host; afterwards this is one
  sweep of `go.mod` and imports  #vtitan  P3
  plan: [platform queue](../../../../ralvarez/identity-service/docs/backlog/platform.md)
- [ ] `pkg/backoff` (75) → `resilience/backoff`: both are a reconnect policy with `Next`/`Reset`;
  the kit one is stateful, saturates instead of overflowing and takes an injectable `Rand` for the
  jitter. Check the gRPC-channel call sites in `internal/statemachine/robotcmd/client.go` and
  `internal/node/picolink`  #vtitan  P3
- [ ] `pkg/supervise` (262) is a wrapper over `resilience/supervise`: keep only what vtitan adds
  (the process naming and the shutdown ordering) or inline the kit calls; do not grow a second
  supervisor  #vtitan  P3
- [ ] No process observability yet: `internal/telemetry` is a **domain** package (it aggregates
  LIDAR, IMU and vision detections for the OLED/UI summary), not logging or metrics, so there is
  nothing to delete here. The repo has no Prometheus usage and configures `slog` by hand in each
  `cmd/*`. When the `obs` kit exists (platform queue R152) this repo is a new adopter, not a copy
  to remove  #vtitan  P3
- [ ] `internal/vtcli` (5,972) does not use `termkit`, which the other CLIs already take for
  tables and colour; assess it only if the commands stay  #vtitan  P3
