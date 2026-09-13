# 0042. Sign range fusion is gated and paired with cluster detection

- Status: accepted
- Date: 2026-09-11

## Context

Taking a sign's range from the LIDAR ray at the camera's bearing shipped TRUE
until 2026-09-06 and was measured to make the estimate WORSE: at the camera's
bearing the return is wall-shaped 51 percent of the time and pillar-shaped 27
percent, median implied chord 34 cm against a 5 cm sign. It fired on 92.5
percent of detections (the gate was 0.05 < r < 10.0, i.e. no gate), and a wall
behind a sign is always further, so it was half the outward bias that pinned
believed signs to the walls.

Replayed over 78 hardware bags (09-06 to 09-10), pass-side routing errors /
passes judged:

| arm | result |
|---|---|
| both off (baseline) | 194 / 654 (29.7 percent) |
| fusion UNGATED | 227 / 605 (37.5 percent) |
| fusion + cluster gate | 180 / 712 (25.3 percent) |

The ungated arm reproducing its own refutation is what validates the harness.
The gated arm is the only one that improves without shrinking the denominator.

## Options considered

- (a) Fusion off.
- (b) Fusion ungated.
- (c) Fusion gated by a free-standing cluster of pillar width whose range agrees
      with the pinhole.

## Decision

(c). `lidar_range_fusion` is back ON since 2026-09-11, but only together with
`lidar_range_fusion_cluster`: the gate requires a free-standing cluster of
pillar width at the camera's bearing whose range agrees with the pinhole within
`lidar_range_fusion_agreement`; failing either test, the pinhole stands. The two
are one mechanism; with the cluster gate false, the fusion is the version
measured harmful.

## Consequences

- The fusion can only improve a pinhole range it corroborates, instead of
  overriding it with a wall return.
- The pair is not independently tunable; the schema says so.
