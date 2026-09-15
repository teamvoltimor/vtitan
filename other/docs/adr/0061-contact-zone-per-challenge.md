# 0061. The contact zone is per challenge

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0039

## Context

Obstacles was escaping from its own plan. A single contact distance for both
challenges meant the robot treated the pillars it was routing around as contact
threats: on the 256 corpus it ran 685 escapes per lap, 126 timeouts, and sign
collisions at 0.132 per lap. The fix is not a global threshold, because Open has
no pillars and nothing measured wants Open to differ.

## Options considered

- (a) One contact distance for both challenges.
- (b) A per-challenge contact zone that supersedes the base value on the challenge
      it names.

## Decision

(b). `obstacles_contact_dist` supersedes `contact_dist` whenever a sign router is
attached; Open keeps `contact_dist = 0.10`. There is deliberately NO
`open_contact_dist`: an unset knob nothing has ever moved reads as tuning that
exists. `obstacles_contact_dist = 0.07` since 2026-09-14. It is a pointer, so an
absent value means "leave `contact_dist` alone", not "the escape never fires". A
validator rejects an override at or above `slow_dist`, because the ladder is an
ordered if/elif chain and the override would delete the slow tier.

The value is set from hardware, not the corpus optimum. The 2026-09-14 corpus
(with the corrected LIDAR model) is monotonic and prefers smaller, down to 0.04,
but the simulator cannot price the absorbing-contact failure, so 0.07 takes most
of the K-turn reduction while keeping 7 cm of stopping room rather than 4 cm.

## Consequences

- Obstacles stops escaping from its own plan: laps>=3 11 to 82, in-time 3 to 52,
  timeouts 126 to 48, escapes/lap 685 to 44, sign collisions/lap 0.132 to 0.009,
  wall collisions flat at 17.
- The threshold must be verified on 256, never a subset; it passed on 16 fixtures
  and 128 scenarios and failed at 256.
- It is NOT hardware-validated and the stopping-distance bench is still unrun. If
  it shows standstill inside 4 cm, take 0.04; if 6 to 8 cm, 0.10 was accidentally
  right and the trigger needs rethinking rather than retuning.
- The narrow-observable-window argument for a larger value is refuted: on 18 bags
  and 4.9M rays the front trigger window is flat (11 at 0.04 against 10 at 0.10);
  `gap = range - 0.0278`, so 0.04 fires on a 6.8 cm return, above the C1's 4.5 cm
  floor.

## History

- 80631fe4 2026-09-01: per-challenge contact zone so Obstacles can stop escaping
  from its own plan. At 0.05: laps driven 68.0 to 434.0, laps>=3 11 to 82,
  escapes/lap 684.7 to 44.4, sign collisions/lap 0.132 to 0.009.
- 1de8094d 2026-09-02: ship the Obstacles contact zone at 0.05; the Go field is a
  pointer so absent means leave `contact_dist` alone.
- 35db7574 and adaf194b 2026-09-06/10: `min_valid_range_m` 0.05 to 0.044 paired
  with `obstacles_contact_dist` 0.05 to 0.04. Either alone measured worse.
- ced51207 2026-09-14: model the C1's chassis occlusion and real dropout rate.
  No-return 25.4 percent hardware against 1 percent sim; the contact_dist verdict
  was taken under the old model and had to be redone.
- 4dce7f17 2026-09-14: pin `obstacles_contact_dist` to 0.10 after a plumbing fix
  (`1a6a46c3`) silently promoted it to live. Per 141 K-turn latches: 0.10 fires
  112 (79 percent), 0.07 28 (20 percent), 0.04 20 (14 percent).
- d344b813 2026-09-14: set `obstacles_contact_dist` to 0.07. On the corrected
  model a 82-scenario file gave 0.04 to 9 failed, 0.05 to 12, 0.06 to 15, 0.07 to
  15, 0.10 to 28. The narrow-window argument refuted.
- 0312e172 2026-09-14: price the front trigger's observable window on real scans
  (18 bags, 27,271 forward scans, 4.9M rays). BLIND flat.
- 50a2c4c3 2026-09-14: move the contact_dist parity pin with the shipped value.
- 1a6a46c3 2026-09-14: re-resolve per-challenge params on the router swap. This
  is the root cause of the accidental promotion: params resolved only in `__init__`,
  so the first Obstacles race of each process ran Open config (13,852 Open-tier
  ticks in 9 of 9 rounds).

## Cross-references

- 0039 is superseded; its per-challenge decision is carried above.
- 0056 owns the raw/masked scan and the related `min_valid_range_m` and
  `forward_path_ahead_of_bumper`.
- 0055 owns the escape manoeuvre that consumes `contact_dist`.
