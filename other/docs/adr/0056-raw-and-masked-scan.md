# 0056. The scan is read twice, raw for speed and masked for the escape

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0040, 0041, 0048

## Context

The reactive layer had four ways to misread its own scan, and each one produced a
confident wrong answer.

One scalar self-detection threshold cannot describe the rear: the chassis
boundary runs from 0.137 m at the sector edges to 0.2722 m straight back, and the
old 0.08 m floor is inside the body everywhere in between. Measured on
run_20260906_192424 the rear minimum was the ROBOT on 100 percent of scans, so
`most_constrained_side` read BACK on 84 percent of driving ticks. There is no
BACK escape branch, so no manoeuvre was ever generated: 212 ticks of
`escape_risk=critical` across five pillar contacts produced zero escapes.

The no-return floor was set to 0.05 m while its comment claimed to "match" the
C1's rated 0.045 m minimum. Because the filter is `r > min_valid_range_m` and the
sensor REPORTS 0.045 for anything closer than it can measure, every floor reading
was discarded as invalid: a chassis nosed into a corner had its whole forward
cone thrown away and `AssessRisk` returned the unconditional `RiskCritical` for a
fully-invalid lane, a wedge it could not read its way out of.

The forward lane's short-return test took the bare minimum over the sweep, which
is an extreme-value statistic, not a clearance: with sigma = 0.03 m of range
noise across about 500 rays, the smallest reading in the lane sits 2 to 3 sigma
below the nearest real surface, so one noisy ray can read as an obstacle.

And a whole-cone no-return was read as open road, when it is the LIDAR failing to
see. Two wedged hardware rounds read forward clearance near 12 m and held 0.24
m/s for zero laps.

## Options considered

- (a) One scalar self-detection threshold; a floor at the rated minimum; a bare
      minimum over the forward lane; no-return read as clear.
- (b) Per-bearing chassis geometry for self-detection; a floor strictly below the
      rated minimum; adjacent-ray corroboration; whole-cone no-return is degraded.

## Decision

(b). `rear_self_detection_from_chassis = true` gates the rear sector by ray-vs-
rectangle geometry (`chassis_exit_range_m`, sensor at origin, body offset by the
LIDAR mount), per bearing, kept in the sensor frame so callers still convert with
`bumper_gap_behind`. It is a `default` tag, so an omitted TOML key reads the
shipped true rather than reverting to the scalar through the zero value.

`min_valid_range_m = 0.044` sits strictly below `RobotSpecs.LIDAR_MIN_RANGE`
(0.045): a floor reading is a measurement, not an absent one.

`risk_ray_window = 1` is a count of ADJACENT rays that must corroborate a short
return; 1 restores the bare minimum and is the no-op. The window that should ship
is still being measured, so no window above 1 ships.

`forward_no_data_is_degraded = true`: a forward cone that returns nothing is
degraded, not clear. Zero false positives on a clean bag replay
(run_20260830_182505, 0 of 847) against 62.7 and 66.1 percent on two wedged
rounds. `forward_path_ahead_of_bumper = true` selects the forward lane from the
bumper plane rather than the LIDAR; it removes 98 of 366 (27 percent) triggers at
or behind the front-bumper plane, where 47 percent of escapes fired more than 60
deg off the nose and only 11.5 percent dead ahead. The along-track selection
threshold and the radial `bumper_gap_ahead` are different axes; nothing is
applied twice.

The escape reads a second, masked scan: returns attributed to a routed sign the
router still intends to route around are withheld from the escape trigger, while
the raw scan caps speed. A routed sign is passed inside `contact_dist` by design,
so on the raw scan the escape would fire on every sign pass. The mask anchors on
the LIDAR cluster nearest each belief (`ranges_beyond_chassis`), not on the belief,
because p50 belief-to-return is 0.15 to 0.25 m against a 0.12 m radius.

`blind_wedge_left_min/max_deg = -155.0/-120.0` and `blind_wedge_right_min/max_deg
= 120.0/160.0` are the re-measured rear occlusion on the current mount: about a
40 deg slot straight back, asymmetric (35 deg blind left, 40 right). The
2026-08-04 numbers (about 25 deg) and the 2026-08-22 fully-masked pair are
superseded. Widening the wedges to cover the body was refuted: covering the
structure closes the slot, `rear_sector.measured` goes permanently false, and
that removes reverse authorization rather than fixing it.

## Consequences

- A fully-invalid lane is no longer indistinguishable from a near wall.
- A single noisy ray no longer reads as an obstacle.
- An unreadable forward cone no longer authorizes driving into it.
- The change is not hardware-validated, and the abeam 47 percent of escapes is
  not fixed by the bumper plane; the swept-corridor fix is still open.
- `contact_dist = 0.10` (Open) and `obstacles_contact_dist = 0.07` are set from
  hardware; the stopping-distance bench is still outstanding. `contact_dist`
  0.04 and `min_valid_range_m` 0.044 were a matched pair and 0.04 is 5 mm below
  the C1 rated minimum.

## History

- 672189e2 2026-08-02: stop the direction estimator's corner cone reusing
  `front_half_fov_deg`. `direction_arc_half_fov_deg = 8.0`.
- 79f871fc 2026-08-04: forward clearance uses min not mean; raise the floor toward
  spec. run_20260804_114500 logged forward clearance 5.15 m while the true sector
  minimum was 0.078 m.
- 95243956 2026-08-04: mask LIDAR blind wedges by angle instead of range.
- 7ef80375 2026-08-22: widen the wedges, closing the rear slot (superseded).
- 310ff30d 2026-08-28: forward-lane no-return no longer reads as clear road.
  run_20260828_224645 whole forward cone near 12 m at closest approach.
- fbb94eeb 2026-08-31: re-measure the rear blind wedges on the current mount.
  Refusals 54 to 0; 640-case 612 to 617 ok, collisions 3 to 1, one new collision
  (case 64). First time the reverse gates can fire since 2026-08-22.
- d638800b and bda6b10c 2026-08-31: an unreadable forward cone is degraded, not
  open road. run_20260831_205208 96/153 (62.7 percent), run_20260831_205235
  74/112 (66.1 percent).
- 8203fa39 2026-09-06: the robot's own rear structure suppressed every reactive
  escape. `most_constrained_side` BACK 361/612 (59 percent) to 4/612 (0.7
  percent); rear min at contacts 0.127 to 0.910 m; manoeuvre generated 0 to 100
  percent of ticks.
- 9b0a15e8 and 3f05e614 2026-09-06: corroborate a short LIDAR return before it
  counts as an obstacle. `risk_ray_window` ships 1.
- 35db7574 and adaf194b 2026-09-10: land the floor-reading fix. `min_valid_range_m`
  0.05 to 0.044, `obstacles_contact_dist` 0.04. Same-day hardware aborted two Open
  runs at pose (2.97, 0.58) reading exactly 0.045 m at near full lock.
- e37465ea 2026-09-10: the clearance-floor sweep axis; each half measured worse
  alone than either endpoint.
- b645e3bf 2026-09-11: price the rear sector's blindness, find there is none.
  78-bag replay: 64167/64258 = 99.9 percent, 803/803 at reverse launch.
- b7c2401a 2026-09-11: anchor the escape mask on the LIDAR cluster, not the
  belief. Associated 86.7 percent, self-returns 1.3 percent.
- 18ec157d 2026-09-14: select the forward lane from the bumper plane. Nine rounds,
  366 latches: 47 percent fired >60 deg off the nose, 98/366 (27 percent) at or
  behind the bumper.
- d344b813 2026-09-14: `obstacles_contact_dist = 0.07`. A 82-scenario
  real-sensor-error file was monotonic 0.04 to 9 failed, 0.07 to 15, 0.10 to 28;
  141 K-turn latches: 0.10 fires 79 percent, 0.07 20 percent, 0.04 14 percent.

## Cross-references

- 0040, 0041 and 0048 are superseded; their decisions are carried above.
- 0031 (clearance thresholds distance not time) stays separate as a proposed
  time-to-collision derivation, not implemented.
- 0047 (contact reverse ships disabled) stays separate.
- 0050 owns the escape side; 0055 owns the manoeuvre selection.
