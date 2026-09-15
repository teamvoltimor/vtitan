# 0081. Every 3D part is published as STEP and STL with kebab-case names

- Status: accepted
- Date: 2026-09-15

## Context

SolidWorks part names (uppercase, spaces, `ñ`, decimal commas) break URLs, shell
globs and clones onto non-UTF-8 filesystems, and the repository could not tell
which of two parts came first because they were all published in one commit.
GitHub renders STL in an interactive viewer and does not preview STEP at all, so
vTitan's own CAD was the unviewable format.

## Options considered

- (a) Publish one format per part, with the original designer names.
- (b) Publish STEP for fabricating and STL for printing and viewing, with one
      normalized kebab-case stem per part.

## Decision

(b). Each part ships as `.step` (exact B-rep, editable in CAD, not previewed by
GitHub) and `.stl` (triangle mesh, printable, rendered by GitHub). A part that
exists in both formats carries the SAME stem in both, so they pair at a glance.
Names are kebab-case ASCII with a lowercase extension.

The suffix grammar is fixed: `-vN` is an ordered iteration (the last is the one
printed), `-vN.M` is a minor retouch of `vN`, and a descriptive suffix (`-cajera`,
`-arrastre`, `-con-eje`) is a coexisting variant, not an iteration. The old names
`-max`, `-ultimate`, `-nuevo`, `-editado`, `-ligero` were replaced by `-vN`
because they are not orderable.

## Consequences

- A judge can spin an STL in the browser; a builder can reopen the STEP.
- 17 of 29 `.step` files pair with a same-name `.stl`; the other 12 are purchased
  components.
- The print-parameter manifest (material, layer height, infill, supports) is still
  missing.

## History

- a76de71c 2026-09-06: import the old and current 3D models under SolidWorks names.
- 3a73ff3a 2026-09-08: move `3d-models/` to `models/` and detector weights to
  `ml-models/`.
- 3b330a1c 2026-09-14: rename 90 of 120 files to kebab-case (42 with `ñ`/`ó`, 2
  decimal commas, 72 extension case).
- 878b8485 2026-09-14: publish the STL exports renamed to match the `.step` stems;
  add the READMEs.
- 30e8d633 2026-09-14: version the filenames (`-vN`), delete a duplicate STL,
  correct counts.
- a9539e66 2026-09-14: name the model folders after the robots.
- bfdea298 2026-09-14: recompress the blueprints (3308 KB to 2496 KB).
- aae2ab4e 2026-09-14: identify the 50-tooth motor pulley.

## Refuted

- The "48 STL" count; "(PNG y WebP)"; the stale "15 cm/s" motor identity.

## Cross-references

- 0082 owns the wiring harness scheme, the other hand-authoring choice.
