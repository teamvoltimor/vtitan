---
title: 'Flatten `src/go/internal/adapters/` into `internal/nats/`'
priority: p3
section: vtitan
topics:
    - vtitan
---

`adapters/` is a role bucket: it says what the packages are (adapters) rather than what they talk to. Its three
entries are all one transport and already named for their job — `natscamera` (1 file), `natsgw` (5), `natsvision` (2).

`git mv internal/adapters/natscamera internal/nats/camera`, `natsgw` → `nats/gateway`, `natsvision` → `nats/vision`
(or keep the flat names `internal/{natscamera,natsgw,natsvision}` if the import path then reads better), and drop
`internal/adapters`. A protocol parent is allowed; a role parent is not. Rewrite the imports, then `go build ./...` and
the lint gate; no behaviour change.
