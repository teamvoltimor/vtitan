---
title: '`pkg/backoff` (75) → `resilience/backoff`'
priority: p3
section: vtitan
topics:
    - vtitan
---

both are a reconnect policy with `Next`/`Reset`; the kit one is stateful, saturates instead of overflowing and takes an injectable `Rand` for the jitter. Check the gRPC-channel call sites in `internal/statemachine/robotcmd/client.go` and `internal/node/picolink`
