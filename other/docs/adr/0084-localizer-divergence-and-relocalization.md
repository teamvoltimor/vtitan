# 0084. Pose divergence is detected off-track and recovered by armed global relocalization

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0024, 0025

## Context

A localizer that latches wrong does not recover within a round. On
run_20260907_205830 the estimate latched 1.5 to 3 m off at t=8.0 s and never came
back, driving the navigator into walls for 20 escape manoeuvres and 46 s of
net-zero travel. Of hardware runs where the localizer lost track (about 1 in 3),
19 of 19 scored zero laps, and the shipped rescue recovered 6 of 19. Prior to the
fix, the pose-covariance signal did not surface the divergence at all.

A separate defect silently froze the pose: the jump guard's speed gate sat BELOW
the speeds the robot actually reaches, so updates were thrown away whenever the
robot went quickly. On run_20260829_003233 encoder and pose agreed to 0.5 percent
(28.46 against 28.62 m) because both were under-reporting, the encoder from a
stale counts_per_rev and the pose from updates past 0.25 m/s being discarded. Two
suppressed measurements agreeing is not corroboration.

## Options considered

- (a) Keep the static jump-guard bound and a local search only.
- (b) Track the drivetrain with the jump bound, and arm a global re-solve when the
      pose stops explaining the scan.

## Decision

(b). The physical invariant: the car cannot leave the track, so a pose whose
predicted scan does not match the real one is provably wrong, and a match landing
off-track is evidence the search is lost. The divergence detector projects the
beams from the believed pose and counts how many land outside the track; a pose
that puts a third of its returns through a wall says so unambiguously where the
covariance does not.

Off-track scans and over-threshold costs count toward the SAME consecutive streak;
`relocalize_after_scans = 15` (about 1.5 s) re-solves over the whole free space,
not the plus/minus 0.15 m window. The global winner is accepted only when it beats
the local one by at least `relocalize_accept_ratio = 0.5`; otherwise the wall
model, not the pose, is presumed wrong and no jump happens. The cost threshold is
`relocalize_cost_threshold = 0.03`, grid step `relocalize_grid_step_m = 0.03`.
Across 1945 scans neither clean 3-lap run ever crossed the threshold; the failed
run sat at median 0.0432, and one relocalization took it to a 0.0101 fit and 0
percent off-track beams.

The jump guard's `max_speed_mps = 0.60` is a speed bound, not a quality threshold,
kept at roughly 1.5x headroom over the measured closed-loop ceiling and raised
whenever the drivetrain gets faster. A stale bound silently invalidates any
comparison of encoder and pose odometry.

At every race boundary the position, the heading reference and the yaw correction
are reset together, and `_commit_direction` discards position when direction
inference overturns the assumption, because the localizer takes yaw as given and
wrong-yaw creep corrupts position. `stale_timeout_sec = 0.5` is derived as 5 times
the slowest feed's scan period (LIDAR at 10 Hz).

## Consequences

- A diverged pose has a way back instead of a lost round.
- The detector is the limit, not the rescue: effort spent improving the rescue
  before detection improves is misdirected.
- The within-race drift itself is still open; this fix only stops it surviving a
  race boundary, giving future investigation a clean start.
- Go lacks the race-boundary reset in production; the re-seed is Python-only.

## History

- a677a71e 2026-07-04: `stale_timeout_sec` and stale-sensor gating.
- 3b882816 2026-07-05: the original `LidarLocalizer`, resolving a frozen-pose bug.
- 84974f88 and 67041694 2026-08-04: re-seed position on reset, and clear
  `_yaw_correction` (a prior race's direction correction survived).
- 47f6e61c 2026-08-04: discard position drift when direction inference overturns
  the assumption (implied speeds 2.8 to 6.4 m/s against a real 0.156).
- 874271fc 2026-08-05: the cost/margin ambiguity guard, reverted. Confirmed-bad
  and correct matches had median cost about 25 to 26 either way.
- 9e91981f 2026-08-05: replace it with the speed bound plus hysteresis (0.25 m/s
  against a measured 0.156).
- 626a0112 and 88312eaf 2026-08-06/09: measure the start pose, and retry a refused
  measurement instead of racing on the assumption.
- 9552445e 2026-09-07: global relocalization. Replay off-track beams 72.4 to 0.0
  percent in one relocalization; balanced-128 128/128 to 127/128 without the
  accept ratio.
- 1d8cf4e9 2026-09-07: publish `localizer_fit_cost` and the relocalization count
  on `/nav_debug`.
- 91b2ffe3 2026-09-10: derive the staleness gate from the scan rate.
- 392e9522 2026-09-10: port global relocalization to Go.
- fe4acf62 2026-09-11: refute the clipped-residual-saturation theory; the trigger
  separates cleanly (963 consecutive bad scans against a max streak of 7).
- 63c042bb 2026-09-11: the beam-outside-track divergence method rules the localizer
  out for the evening reversals; those were a frozen waypoint index.
- 1913975c 2026-08-29: `max_speed_mps` 0.25 to 0.60 (see 0076).
- 2026-08-04: a reset that did not re-seed position let `pose_x`/`pose_y` drift to
  hundreds of metres across a button-cycled race.

## Refuted

- The static 0.25 bound (froze the pose); the cost/margin ambiguity guard; pose
  covariance as a divergence signal; the "detector never tripped" theory; relaxing
  the direction gates; the speed-guard hole after a belief swap (examined and
  turned down, do not re-fix).

## Cross-references

- 0024 and 0025 are superseded; their decisions are carried above.
- 0076 owns the encoder calibration that the odometry comparison depends on;
  0053 owns the start pose and direction commit; 0086 owns the sim error budget.

## Evidence

- Never measure yaw rate from `pose_yaw`: it is the localizer's fused pose,
  published at about 5 Hz against the IMU's about 166 Hz, too coarse and too
  lagged to carry the achieved rate. Derive the rate from the orientation
  quaternion instead, since `/imu/data.angular_velocity` is all zeros (the BNO08x
  UART-RVC has no gyro). A re-measurement on one recorded run
  (`run_20260829_140424`, 13085 IMU samples against 2604 `pose_yaw` samples) did
  not reproduce the "understates by 4 to 6x" figure this ADR used to quote: over
  a 0.15 s finite difference the differentiated `pose_yaw` tracked the quaternion
  rate (mean 0.287 against 0.255 rad/s) and its total heading variation was
  slightly higher, so no single damping factor should be quoted.
- Blind mode seeds the believed start as the canonical section; on a
  four-fold-symmetric 1.0 m track the localizer locks to a clean 90/180/270 deg
  rotation of truth for the whole run.
- The rotational lock breaks two absolute-XY subsystems (the escape mask and
  sign-discovery association), not raw driving; `8fc832a0` and `f6f1278c` gate
  both on the ROBOT's own corridor (collisions 231 to 202, in-time 13 to 27).
- Seeding the position solve with encoder dead reckoning between scans was tried
  and rejected. The robot covers 0.8 cm between scans in simulation and about
  1.6 cm on the real C1 at full speed, against 3 cm of LIDAR noise, so the
  correction is smaller than the noise on the measurement it would seed. Measured
  over the 28 fixtures it changed the sighted peak error not at all and made the
  blind peak error 2.5x worse (20.8 -> 52.2 cm), because blind means the wall
  model itself is wrong and dead reckoning between poor fixes compounds drift
  rather than staying anchored to the last one.
- `max_speed_mps`: the current drivetrain measured about 0.58 m/s at max_duty 0.5
  and about 0.9 m/s open-loop (2026-08-29 bench), against the retired motor's
  0.156 m/s ceiling.
- `relocalize_accept_ratio`: without it the balanced-128 Open sweep went 128/128
  to 127/128 (scenario 94 turned into a reverse-run) and one case lost 17 s.
- `stale_timeout_sec`: past the timeout the gateway reports the sensor
  unavailable, so the navigator degrades safely instead of acting on frozen data.
- On hardware 23 to 30 percent of the sweep is fabricated max-range no-returns;
  counting them compresses the healthy-versus-lost fit-cost gap from 8x (0.006
  against 0.05) to 2x (0.023 against 0.051).
- The cost/margin ambiguity guard was reverted on 2026-08-05 after replaying
  against 22 real hardware runs (846 sampled ticks): confirmed-bad and correct
  matches had median cost about 25 to 26 and median margin about 0.02 percent
  either way.
- The global rescue of run_20260907_205830 recovered a 48 s pose divergence with
  a residual 10 to 15x lower at every sampled tick and no beams off-track.
- The forward-tracking invariant was born from a 2026-08-05 CCW run whose estimate
  tracked backwards along its heading (drive east, pose slid west), so pure
  pursuit never advanced the plan; sweeping every pulled bag, 9 of 24 violate it.
  The latest-run gate went green on the CW run that completed a lap and red on the
  CCW run that drove into a corner. Both real captures committed direction by
  t=1.2 s.
- The speed-bound guard does not bound anything: across every pulled bag,
  including `run_20260805_195501` and both runs after the guard shipped in
  `9e91981`, accepted estimates still imply multiples of the drivetrain maximum,
  and its hysteresis accepts a persistent pull at full magnitude.
- LIDAR is the only real position source on hardware (navigation review item
  NEW-1, 2026-07-05).
- Symmetric uniform layout: the wrong-corridor pose predicted a cost of 0.017
  against the 0.03 threshold, so the detector correctly stayed silent.
- Projecting scan returns into world coordinates with the believed pose identified
  a wedge as the parking lot's west fin, and found a pillar 0.38 m from the wall in
  one round and 0.53 m in its siblings: 15 cm of same-layout disagreement, an
  object that moved or a pose bias that no clearance metric surfaces.
- The width-spread gate is necessary, not sufficient, and it tests the wrong
  statistic. `run_20260915_160804` ran commit `98fd5a37` with a clean tree, so
  `relocalize_min_width_spread_m` was already shipped, and the round still lost its
  pose. The BELIEVED widths were 0.600 / 0.600 / 1.000 / 1.000: a spread of 0.400
  that passes the gate easily, while the model is EXACTLY mirror-symmetric about
  both axes, so every pose has twins explaining the scan as well as it does. The
  gate reads `max - min`, which is not the same question as "is this model
  symmetric". (0.63 / 0.955 / 0.958 are that round's TRUE spans measured off
  `/scan`; an earlier revision of this entry cited them as the belief. That was an
  attribution error and it is corrected here.)
- The event itself: the estimate teleported 2.06 m in 1.26 s, the cost fell 0.0316
  to 0.0100 so `relocalize_accept_ratio` was satisfied, the corridor label flipped
  east to west, the planner replanned the waypoint index 35 -> 9, `angle_error_rad`
  reached 2.96 (170 deg, target directly behind), and yaw swept -68 -> +145 deg,
  213 deg in 4.8 s, while the pose stayed inside an 8 x 26 cm box. That is the
  U-turn the operator saw on the round scored 6. It is also the only tick stall over
  0.5 s in the whole afternoon: the control loop blocked 1.256 s across it.
- THE REAL DEFECT IS THE SCAN/YAW PAIRING, not the search. Replaying the shipped
  `_relocalize_globally` over the 40 scans of t=20.5-24.5 s with the yaw interpolated
  across the stall NEVER produces the teleport: the winner stays at x~2.5, lag-1
  median 0.030 m, zero flips. It reproduces only when the POST-stall yaw (-87.55 deg,
  what production logged at t=22.907) is applied to PRE-stall scans, and then the
  winner lands within 6 cm of production's accepted pose. One DEGREE of yaw flips
  the winner between twins 2.0 m apart, at costs 6% apart. Pairing a scan with the
  yaw that belongs to it is the lever; `relocalize_confirm_dist_m` is not.
- `relocalize_confirm_dist_m` holds a winner further than 1.00 m and takes it only
  when a later global search lands within `jump_confirm_tolerance_m`. It was built
  on the local speed guard's reasoning -- a real correction reconverges from an
  independent scan, an ambiguous tie does not -- and REPLAYING THE REAL SEARCH OVER
  EVERY SCAN OF BOTH AVAILABLE BAGS REFUTES THAT REASONING. At one scan's lag the
  genuine rescue (`run_20260907_205830`) reconverges within 5 cm on 95.5% of 555
  pairs and the twin on 80-95% of its own; both would confirm, and on the next scan
  the guard would take the twin. There is no separation: inside 160804's clean
  window the correct family is 81.6% stable and the twin family 80.0%.
- What makes the guard work is the CADENCE, not the reasoning. `_relocalize_globally`
  zeroes the streak, so the second search is at least `relocalize_after_scans` later,
  and by then the winner has moved WITH the robot: at lag 15 the winner displaces a
  median 0.309 m against the robot's own 0.335 m, and `jump_confirm_tolerance_m`
  carries no motion compensation. At its real cadence this is a STATIONARITY test.
  It refuses the 160804 teleport because that twin episode lasts 3 scans while the
  car moved 0.3 m. Simulating a search every 15 scans over that bag, 2 of 30
  consecutive pairs confirm. DO NOT add motion compensation: it would restore the
  refuted reconvergence test and accept the twin.
- The cost is understated by about 7x if quoted as one re-arm. On the genuine rescue,
  15-scan-apart searches confirm on 5 of 37 pairs and the first confirmation lands at
  10.5 s, not ~1.5 s. Still far better than the 48 s divergence that rescue recovered,
  which is why it ships on.
- The confirm rule would NOT have protected the uniform-layout runs. On 140358 and
  161222 the robot is nearly still on 56% and 45% of scans, 15-scan-apart searches
  confirm on 19 of 39 and 6 of 10 consecutive pairs, and the lag-1 winner still flips
  between twins 1.4-2.8 m apart. Only `relocalize_min_width_spread_m` keeps the search
  off those runs.
- The one discriminator the replay did find, and which NEITHER guard tests, is the
  FLIP RATE over a window: 0 flips above 0.5 m in 555 pairs on the genuine rescue,
  against 4 of 457, 7 of 599 and 13 of 159 on the three ambiguous runs.
- The corpus cannot score any of this. Obstacles fixes all four corridors at 1000 mm,
  so the spread gate refuses the search there and the confirm rule sits downstream of
  it; measured, both arms return 14 failures with an identical set (fixed=0, broke=0).
- n CAVEAT: exactly two bags both pass the spread gate and contain the phenomenon,
  one event each. Every percentage above rests on a single run apiece.
- GO PARITY GAP, recorded rather than silently carried: `src/go/internal/nav/
  localization` has neither this guard nor `relocalize_min_width_spread_m`. The Go
  localizer is two guards behind the Python one.
