---
title: 'cmd: the mains hold their wiring; internal/statemachine/core is a catch-all'
priority: p3
section: vtitan
topics:
    - repo-tooling
---
Recorded only, no code change (owner's call).

- `src/go/cmd/sim-runner/main.go` is 598 lines, `pi-zero` 472, `pi5` 350, `foxglove-bridge` 274 and the `*-node` mains
  ~250 each: they hold their wiring (flags, NATS/ROS setup, loops) in `main`. Move the bodies into packages under
  `internal/` and leave thin mains, the shape ralvafi uses.
- `internal/statemachine/core` (`package core`, with its tests) is a catch-all inside a feature package; split it by what
  it holds (states, transitions, effects) and delete the directory name.
- Module path `github.com/teamvoltimor/vtitan/src/go` carries `/src/go`: a module belongs at the repo root
  (`github.com/teamvoltimor/vtitan`) unless the tree is deliberately nested — if it is, say so in the README, because a
  Go reader expects the root.
