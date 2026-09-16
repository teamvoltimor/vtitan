# 0047. The contact reverse ships disabled

- Status: superseded by 0088
- Superseded by: 0088
- Date: 2026-09-13

## Context

The contact reverse backs the chassis off when it has closed to `contact_dist`,
instead of creeping forward into it. It is ~0.4 s / 25 mm at creep, with a
cooldown that only counts down while the path is CLEAR so the back-off and the
approach cannot oscillate against the same obstacle.

It was believed (premise, later refuted) that a trail gate -- only reversing
when the pose trail confirms enough covered ground -- would rescue the
behaviour by stopping it firing against a surface the chassis had not actually
reached. The hardware case it was written for is real: `run_20260906_121254`
spent ten seconds at +0.152 m/s against a green pillar. That case is still
unaddressed, and this mount has no rear sensing to make a seeing reverse
possible.

Three back-to-back passes over the 256 corpus, 2026-09-06, differing only in
this value and the gate:

| metric | off(0) | ungated(8) | trail-gated(8) |
|---|---|---|---|
| in-time | 149 | 150 | 150 |
| laps>=3 | 150 | 151 | 151 |
| stuck | 26 | 15 | 24 |
| collisions | 8 | 13 | 11 |
| of which wall | 4 | 10 | 9 |
| timeouts | 65 | 70 | 66 |
| rev-run | 5 | 4 | 2 |

## Options considered

- (a) Enable the contact reverse ungated.
- (b) Enable it gated on the pose trail.
- (c) Ship it disabled.

## Decision

(c). Gating gave back the timeouts and most of the extra collisions, and gave
back the stall benefit with them -- the trail rarely confirms 6.1 cm of covered
ground at the moment the chassis is against something, which is precisely when
it has stopped moving and stopped laying trail. What survives is +5 wall
contacts against a FLAT headline, so the behaviour is not earned in either
form. The value stays 0.

## Consequences

- The code stays, measured and documented, so the behaviour can be revisited
  without rediscovering it.
- The hardware case that motivated it (`run_20260906_121254`, ten seconds of
  +0.152 m/s against a green pillar) remains real and still unaddressed: the
  sim says a blind reverse is not the answer, and this mount has no rear
  sensing to make a seeing one.

## Superseded by 0088

This decision was replaced by [0088](0088-refuted-config-knobs.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

Several mechanisms were built, measured and found not to earn their place. The
risk is re-proposing them from the same reasoning, or reading a `false` as
unfinished work. The contact reverse is the clearest case: on
run_20260906_121254 the robot held +0.152 m/s for ten seconds against a green
pillar, and the back-off was meant to address it.

### Options considered

- (a) Delete the refuted code so it cannot be re-enabled by accident.
- (b) Keep the code, ship it off, and record the measurement so it is not
      rediscovered.

### Decision

(b). The mechanisms stay in config at their off value, with the refutation in the
comment, so a `false` does not read as unfinished and the behaviour can be
revisited without rediscovering it.

`contact_reverse_ticks = 0` with `contact_reverse_cooldown_ticks = 20`: gating on
the pose trail gave back the timeouts and most of the extra collisions and gave
back the stall benefit with them, because the trail rarely confirms 6.1 cm of
covered ground at the moment the chassis is against something. What survives is
+5 wall contacts against a flat headline, so the behaviour is not earned in either
form. The hardware case remains real and unaddressed: this mount has no rear
sensing, so a seeing reverse is impossible, and the simulator says a blind reverse
is not the answer.

The other refuted singles ship off the same way: `retrace_escape = false` (halves
wall strikes 13 to 7 but gives back most of the sign gain 41 to 53);
`side_correction_blends = false` (the qualifying forward nudge is 0.8 percent or
less of hardware ticks, so it cannot move the alternation it was written for);
`tick_router_during_maneuver = false` (off and unvalidated, not refuted);
`replan_blend_ticks = 0` (refuted twice); `sign_lane_split_overlap = false`;
`sign_lane_skip_unsatisfiable = false`; `advance_past_passed_waypoint = false`;
`stale_target_rescue = false`; `sign_contact_evade`, `sign_lidar_align`.

### Consequences

- A refuted knob cannot silently become live; its measurement is one grep away.
- The contact-reverse hardware case is documented as unsolved rather than closed.

### History

- 6102224f 2026-08-17: retrace measured and rejected (wall 13 to 7, sign 41 to
  53).
- 2a0e9e28, dd0cb0c6, c94e8a31 2026-08-31: the unconfirmed-width bias and deferral
  that superseded the replan blend; confirm `replan_blend_ticks` refuted with both
  confounds removed.
- 946a83e0, bb32958c 2026-09-06: the contact reverse, then its trail gate; three
  arms off/ungated/trail-gated all leave the value 0.
- 40b79421 2026-09-05: write the code-only tuning constants into TOML, marked as
  refuted.
- aa54fb42 2026-09-11: price the reverse-to-buy-road manoeuvre and recommend
  against building it; notes `CONTACT_REVERSE_TICKS = 0` so the centred back-off
  never fires.
- b50b889c 2026-09-11: `side_correction_blends`, refuted by its own scoping.
- 88148831 2026-09-12: `tick_router_during_maneuver`, off and unvalidated.
- b982e109 2026-09-13: create ADR 0047.
- f5fa3a61 2026-09-14: correct what `retrace_escape` is and which evidence refuted
  it.

### Cross-references

- 0047 is superseded; its decision is carried above.
- 0055 and 0050 own the escape family the contact reverse belongs to; 0057 owns
  the corridor follow the replan blend belongs to; 0051 owns the sign lane.

