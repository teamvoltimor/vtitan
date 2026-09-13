# Architecture Decision Records

One Markdown file per decision. These hold the *why*: the measurements, the
run ids, the options considered, and the premises that were later refuted.
The config values live in `src/config/`, their short descriptions live in the
JSON Schemas (`src/config/schemas/`), and each schema key points here through
`x-journal`.

## Why separate files

A single growing engineering log does not scale for per-value justification:
the log is a curated, regulation-shaped deliverable, and it would balloon.
Small per-decision files stay reviewable, blame-able, and conflict-free.

## Reference from a schema

```json
"collision_thickness": {
  "x-journal": "adr:0001-wall-collision-thickness"
}
```

- `adr:<stem>` resolves to `docs/adr/<stem>.md`.
- A plain string is a repo-root-relative path, optionally with `#anchor`.
- A list is allowed when genuinely independent decisions touched one value.
- One evolving story should be a supersede chain (see below), not a list.

`task config:check` fails if any reference does not resolve.

## Chain, do not pile up

When a later decision replaces an earlier one, the new ADR supersedes the old
and carries `Supersedes: 0001` in its header. The schema keeps only the current
ADR; the history is the chain.

## Naming and status

`NNNN-slug.md`, zero-padded, assigned in order. Status is `proposed`,
`accepted`, `superseded by NNNN`, or `deprecated`.

## Index

| ADR | Title | Status |
|---|---|---|
| [0001](0001-wall-collision-thickness.md) | Wall collision thickness matches the visual wall | accepted |
| [0002](0002-lidar-mount-forward-shift.md) | LIDAR collision geometry sits at the real mount | accepted |
