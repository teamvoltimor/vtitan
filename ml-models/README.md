# models/

Tracked, versioned archive of compiled Hailo models and the checkpoints they
were compiled from. This is the single source of truth for "what model is
this" - the ml-service's own `models/` directory
(`auto-annotator/ml-service/models/`) and the compiler's scratch directory
(`hailo/shared_with_docker/`) are both untracked working copies that get
populated *from* here, not the other way around.

## Layout

```
models/
  <name>/
    LATEST              # plain text, contains the current version dir name (e.g. "v1")
    v1/
      metadata.yaml      # what this version is, when it was promoted, what compiled it
      <name>.hef         # compiled model, ready to deploy
      *.onnx / *.pt       # the checkpoint(s) it was compiled from, if applicable
    v2/
      ...
```

Each version directory is immutable once promoted - never edit files inside
an existing `vN/`, only add a new `vN+1/` and update `LATEST`. `metadata.yaml`
records why the version exists (what changed, what training run or compile
produced it).

## Scripts

- `scripts/promote-from-hailo.sh <name>` - copies the compiled `.hef` (and
  any `.onnx`/checkpoint files it can find alongside it) from
  `hailo/shared_with_docker/` into a new `models/<name>/vN/`, and updates
  `LATEST`. Run this after compiling a new model with `task gmr:workflow` (in
  `hailo/`) that you want to keep.
- `scripts/deploy-to-ml-service.sh <name> [version]` - copies a tracked
  version (defaults to `LATEST`) into
  `auto-annotator/ml-service/models/<name>/`, where the ml-service and
  `platform/robot/scripts/provisioning/deploy-to-pi5.sh` actually read from.
  Run this to make a tracked version the one the ml-service (and, after a
  Pi 5 deploy, the robot) actually uses.

Both are also available as `task models:promote NAME=gmr` /
`task models:deploy NAME=gmr [VERSION=v1]` from the repo root.

## Why not Git LFS

These files are small (a few MB each) and change infrequently - plain git
tracking is fine at this scale. Revisit if a model's checkpoints grow large
enough that repo clone size becomes a real problem.
