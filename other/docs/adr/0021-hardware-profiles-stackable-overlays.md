# 0021. Hardware profiles are stackable partial overlays

- Status: accepted
- Date: 2026-08-11

## Context

A 270 degree servo gives roughly 85 deg of road-wheel travel against the
current build's bench-measured ~55 deg. Swapping the servo is trivial, but the
tuning built around that geometry (corner arc ceiling, speed ladder, pursuit
lookahead and thresholds) was hand-tuned and hardware-validated for 55 deg, and
must not be silently recomputed from the new geometry. The mechanism has to let
alternate tuning live beside the base config without branching on servo angle
anywhere in the code.

## Options considered

- (a) A full standalone copy of every config file per profile
      (`profiles/servo270/robot.toml`, `.../navigation/**`, ...).
- (b) A profile is a partial override holding only the keys that differ from
      base, selected as an ordered, stackable list.

## Decision

(b). `VTITAN_HARDWARE_PROFILE` names a comma-separated, ordered list of
profiles. Each profile is a file tree under `profiles/<name>/` containing only
the keys it changes, deep-merged left to right onto the base
(`base <- profile[0] <- profile[1] <- ...`, later names winning). The base TOML
is untouched in place. A profile combining a servo variant with an independent
track-geometry variant is just two names in the list.

Profiles whose numbers are not yet benchmarked are seeded empty or verbatim
from base, marked `# TODO(...): needs bench/track validation`, never with
invented values. The build uses one live combination at a time.

## Consequences

- Servo and motor limits can change without editing the chassis config, and no
  navigation code reads a servo angle to decide behaviour.
- A profile that omits a required component fact is a loud load-time error (ADR
  0011), not a silent zero.
- Overlay keys are merged by file path, so a profile cannot yet move a key
  between files without a loader change (see the config-constants audit,
  `other/docs/internal/audits/2026-08-22-config-constants-audit.md`, section 10).
