---
title: 'deps: pymongo and strawberry-graphql hold their versions against the advisory'
priority: p2
section: vtitan
topics:
    - deps
---
`happy-dom` moved to 21.0.4 and `hydra-core` to 1.3.8 (three high and one high advisory), and the hailo lock was refreshed.
Two highs remain, and neither moves:

- `other/ml/hailo`: `pymongo` stays at **4.9.2** (needs ≥ 4.18.3) and `strawberry-graphql` at **0.316.0** (needs ≥ 0.327.2).
  `uv lock --upgrade-package <name>` and a full `uv lock --upgrade` both leave them, so something in that project's
  resolution pins them — find it (a direct pin in `pyproject.toml`/`uv.toml`, or a package that constrains the range)
  before trying again; relaxing it is the fix.
- `schemes/wiring/tscircuit`: `deepmerge` (low) has no release above 4.3.1, so the advisory cannot be closed.
