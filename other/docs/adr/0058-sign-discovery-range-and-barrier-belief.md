# 0058. Sign discovery takes the closest observation and bounds ingest by range

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0042

## Context

A sign's position came from the camera bbox height through the pinhole model, so
its error grows with range: at 1.5 m a sign spans about 42 px and a 1 px error is
3.6 cm, but by 3 m it spans 21 px and the same error is 14 cm, wider than the lane
gap. A distant reading seeds a track most of a metre out and the router then
routes around a phantom.

The magenta parking-lot barrier reads under the RED class when blurred (52 percent
of that class's detections on run_20260905_214920), so it was admitted to the sign
map as a pillar. Colour and confidence do not separate it; a real pillar is taller
than it is wide, but a box the frame has cut off has no shape measurement at all:
a pillar being approached runs out of frame, its height stops growing while its
width does not, and w/h crosses 1.0 with nothing about the pillar having changed.

## Options considered

- (a) Average all observations of a track; reject by colour/confidence; a bare
      pinhole range.
- (b) Take a track's closest observation; bound ingest by range; gate by aspect
      only for unclipped boxes; carry the LIDAR cluster as a gated range.

## Decision

(b). A track's position is its CLOSEST observation, never an average: the error
is monotone in range, so averaging would let distant readings pull a good
estimate off. `max_ingest_range_m = 1.5` (was 2.0) because the pinhole correction
holds to about 1.5 m (binned error -7 cm at 1.1 to 1.5 m, -77 cm beyond) and the
router's `activation_dist_m` is 1.40, so ingesting only inside 1.2 m would discover
a sign after the point it must already be routed around. `max_pillar_aspect = 1.0`
rejects a wider-than-tall box, and `frame_edge_tolerance_px = 2.0` exempts a
clipped box from the aspect test because its ratio is not a shape measurement: 61
percent of the reds the gate was rejecting were clipped (run_20260906_145546).
The exemption is by all edges, not the vertical axis only: the camera is tilted
down so boxes clip on the bottom, and vertical-only reaches 10 of 458 leaked boxes
(2 percent).

`lidar_range_fusion = true` and `lidar_range_fusion_cluster = true` are ONE
mechanism: the sign's range is taken from the LIDAR cluster at the camera's
bearing only when a free-standing cluster of pillar width agrees with the pinhole
within `lidar_range_fusion_agreement = 0.5`; otherwise the pinhole stands. The
ungated fusion is the version measured harmful.

The magenta barrier is no longer rejected by shape; it is carried as a belief
(`barrier_belief_min_sightings = 4`, `barrier_merge_radius_m = 0.35`,
`barrier_suppression_radius_m = 0.30`), and a RED detection landing within the
suppression radius of a believed barrier is dropped rather than seeded as a
pillar. `barrier_span_along_wall = true` treats the believed lot as the 0.45 m
SPAN between the fins rather than a point, because the belief's centroid sat 0.36 m
from the west fin and wall-shaped reds on that fin were suppressed 1/14 and 2/19
by the point model against 10/14 and 13/19 by the span. It is ON since 2026-09-15
for exactly that validation round and is UNVALIDATED until it runs; the simulator
cannot score it because it never emits a magenta detection.

## Consequences

- A far detection cannot seed a phantom track the router routes around.
- The barrier is believed rather than discarded; every magenta detection was
  previously thrown away (1,921 detections against 748 wall-shaped reds admitted
  over four rounds).
- Range is no longer the accuracy limit for a projected position: turning the
  fusion off moves the believed lot at most 0.17 m while the lot wanders 0.57 m,
  and the residual is zero-mean bearing scatter (median +0.4 and +3.2 deg, IQR
  about +/-12 deg), not an offset. A better position has to come from averaging
  sightings or carrying uncertainty into the planner.
- `max_signs_per_section = 0`, `snap_to_lattice_m = 0.0` and `colour_pool_radius_m
  = 0.0` ship off (refuted); the rules-constrained slot map ships off.

## History

- 0ca8f45b 2026-09-06: reject sign detections wider than tall. run_20260905_214920
  199/383 (52 percent) reds wider than tall; green w/h p90 0.85. `active_sign_count`
  5 to 50 on a track of at most 8.
- 8571fe2a 2026-09-06: do not judge the shape of a box the frame cut off. Adds
  `frame_edge_tolerance_px`. 304/500 rejected reds (61 percent) clipped against
  22/43 green.
- 6d3e3328 2026-09-06: the camera's bearing was mirrored, so every sign was placed
  on the far wall. 174 px error upright against 528 px mirrored; turns
  `lidar_range_fusion` off (wall-shaped 51 percent).
- 7f857e3c 2026-09-06: only test a box's shape where the barrier could be. Adds
  the corridor exemption.
- 648d4730 2026-09-07: decode detections against the pose the CAMERA saw from.
  `vision_latency_s = 0.85`, `range_scale = 1.95`, `max_ingest_range_m` 2.0 to
  1.5. Position error p50 47.1 cm / 19 percent inside association with neither
  fix, 15.3 cm / 61 percent with both.
- 41058de0 2026-09-07: distant signs are in frame; retraining will not close the
  1.4 m activation gap.
- a0c536e0 2026-09-07: LIDAR-proposes/camera-confirms. Recall 221/244 (91
  percent), lead p50 0.59 m; spread/chord filters refuted (47 to 49 percent).
- e2839383 2026-09-07: let the LIDAR propose WHERE while the camera decides WHAT.
- 6a8a958c 2026-09-09: range from LIDAR, colour from camera, only when
  qualified. Nearest-ray shift p10/p50/p90 -0.09/+0.50/+1.61 m; gated cluster
  -0.28/-0.14/+0.15 m.
- 0fc94942 2026-09-11: ship the cluster-gated fusion. Baseline 194/654 (29.7
  percent), ungated 227/605 (37.5), gated 180/712 (25.3). `colour_pool_radius_m`
  refuted.
- b3d7a038 2026-09-11: a corridor boundary is not a sight line for the barrier
  gate. Wall-shaped let through 58.4 to 18.8 percent.
- d7ad6f96 2026-09-11: find which hole the barrier comes through. Corridor
  exemption is 83 percent of the leak.
- 2026-09-11: the corridor exemption's numbers. 72 percent of wall-shaped red
  detections carry no corridor label at all, and over three rounds the shape gate
  admitted 293 wall-shaped reds on the strength of the corridor test, of which
  257 (88 percent) were taken from the corridor NEXT to the lot's. 55 of the
  barrier's own 220 magenta detections also come from that corridor, so the
  barrier is demonstrably visible from there and a corridor boundary is not a
  sight line.
- b9c4acc7 2026-09-11: advance the corridor debounce every tick, not every
  detection. Only 11.6 percent of ticks carry a detection.
- b9e5fffd 2026-09-11: a per-section publication cap, measured and REFUTED.
- dabd57c7 2026-09-11: build the rules-constrained sign map, off by default.
  Routing 23.3 to 15.0 percent, position changes 44.9 to 3.1.
- 17ed2c56 2026-09-14: believe the magenta barrier instead of discarding it.
  run_20260914_214824 pendulums 66.4 s; keeps 41 to 73 percent of the barrier out
  on 3 of 4 runs.
- c9358428 2026-09-15: range vs bearing refutation (above).
- e3e4dc27 2026-09-14: split frame-clipped red boxes by which axis clipped them;
  refutes the vertical-only narrowing.
- 1a3a9328 and 9729ca84 2026-09-15: believe the lot as a span, off then on for its
  validation round. Fins at x=0.94 and 1.42 (0.48 m span, centre 1.18) against a
  belief centroid of 1.30. Suppression 1/14 to 10/14, 2/19 to 13/19; cost real
  pillars refused 25.5 to 26.4, 2.7 to 4.9, 8.8 to 12.7 percent.

## Cross-references

- 0042 is superseded; its gated-fusion decision is carried above.
- 0051 owns the lane the discovered signs feed; 0064 owns the corridor a corner
  sign is filed under.
- 0062 owns the parking model the barrier belief protects.

## Evidence

- The lane is designed for 1.40 m of anticipation, but the camera sees 0.66 m and
  the router commits at 0.38 m, so the manoeuvre executes on a quarter of its
  shaped distance: a perception-range and commitment-latency problem, not a
  planner one.
- Duplicate tracks SPLIT the colour vote and the pass side is often decided at
  0.57 m, too late to act on. The duplication rate matches sim and hardware (2.0x
  against 2.2x) but the nearest-neighbour separation differs 17x (0.012 m sim
  against 0.21 m hardware).
- Duplication is a localization-drift artifact, not a lever: specs form metres
  apart under about 1.5 m of early-window pose error, then converge to about 12 mm
  and stay unmerged, and no safe association radius catches them because real
  signs are 1.00 m apart.
- Blind gap attribution is about 71 percent believed pose and layout and only 29
  percent sign discovery, so fixing discovery is bounded at about 12 collisions of
  the 41-collision gap.
- Four downstream dedup and fold fixes are already refuted on 256 (the fold
  variant worst at 231 against a 202 baseline); do not write a fifth.
- `range_scale = 1.95` corrects the pinhole range: the detector's boxes are about
  2x taller than a 0.10 m pillar projects to (implied height p50 19.8 cm against
  the assumed 10.0), so the raw pinhole under-reads by half. Median |range error|
  6.3 cm against 8 LIDAR-located pillars (run_20260906_232408 / _232748). It is
  not a true scale (log-log slope -0.40, not -1) and only works paired with the
  camera time alignment: 2D position error p50 and share inside association is
  47.1 cm / 19 percent with neither, no better with the scale or the alignment
  alone, and 15.3 cm / 61 percent with both.
- The ungated `lidar_range_fusion` was measured harmful: at the camera's bearing
  the return is wall-shaped 51 percent of the time and pillar-shaped 27 percent
  (median chord 34 cm against a 5 cm sign), it fired on 92.5 percent of
  detections, and it cost 28 cm of median position error with the corrected
  bearing. The 78-bag replay is the validation: both off 194/654 (29.7 percent),
  ungated 227/605 (37.5 percent, reproducing the refutation), gated 180/712
  (25.3 percent, fewer errors and more passes).
- `vision_latency_s = 0.85` was measured on run_20260906_232408 / _232748 (0.78
  and 0.95 independently). The unfitted check: the recovered cx-vs-bearing slope
  reads -309 px/rad at zero lag, which no real lens can produce (floor about
  620), and -679 at 0.85 s.
- `max_signs_per_section` ships off (0): the rulebook allows two per section and
  the map believed 8, 7, 12 and 5 on the 2026-09-11 rounds, but a cap lets an
  early phantom hold a slot the real pillar then cannot have, which is the
  failure mode of an earlier dedup attempt. The rulebook value is 2.
- The barrier span trade is not free: barrier share suppressed rises 60.8 to
  66.8, 23.5 to 27.9 and 68.3 to 75.7 percent, but real pillars refused rise 25.5
  to 26.4, 2.7 to 4.9 and 8.8 to 12.7 percent, and the benefit/cost ratio is
  slightly worse on two of three rounds. `colour_pool_radius_m` remains refuted.
- `sign_lidar_align`: measured on run_20260906_163641 and _163854, a LIDAR return
  exists at the camera's own bearing on 98 to 100 percent of red/green detections,
  and the object at that bearing measures 3.8 to 6.6 cm across at the median, a
  5 cm sign rather than the wall behind it. run_20260906_163854 passed a red on the
  wrong side because it was never detected at all.
- `snap_to_lattice`: measured 2026-09-09, within a LIVE commitment the believed
  position moves p50 7.7 cm, p90 60.9 cm, max 103 cm, which drops the sign out of
  the router's candidate filter and makes the commitment a 0.5 s duty cycle.
- `_clustered_range`'s qualifying tests: the camera focal disagrees with the one
  solved from bearings (measured 1034-1088 px against 545-645), and the ungated
  nearest-ray attempt, when a pillar stands in front of a wall, took the wall
  cluster and paid a range error p50 -69 cm.
- The cluster-shape test alone discriminated pillar from wall at 54 percent, near
  chance; it ships only paired with the agreement test.
- Distant red boxes carried the RED label at p50 confidence 0.79, so confidence
  cannot separate the blurred barrier from a pillar.
- `_detection_to_world` must project from the mount, not the body centre: casting
  from the chassis origin lands every sign short by the mount offset, pulling the
  estimate about 12 cm toward the robot and reading in RViz as sign estimates
  sitting 10-20 cm off the drawn signs. localization.py took the same correction
  on 2026-08-21; this consumer was missed then.
- `_SignTrack.corridor`'s continuous-distance gate was tried and measured WORSE
  across the whole range of reasonable thresholds, because it is not
  rotation-equivariant. A second, still rotation-equivariant attempt widened the
  gate near a corner boundary by a `corner_blend_m` face-distance slack on the
  full 256-scenario blind corpus: every nonzero threshold tried (0.03-0.20 m) was
  worse than 0.0 on collisions, laps>=3 and in-time, 190 (0.0) against 195-207
  across the range, non-monotonic, with the escape-maneuver rate moving in step,
  so it was reverted. All seven arms came from one sweep invocation, so that
  internal comparison is fair, but do not read the 190 as the corpus baseline: a
  standalone run reproduces 202/256 twice, byte-identical, and the 190 was never
  cross-checked.
- The per-section publication cap on an EXACT sim map should have been a
  byte-identical no-op; it fixed 7 and broke 6 instead.
- LIDAR proposals lead the camera by about 0.6 m, but the detector is only 84
  percent precise and the router's window is the last 0.3 m, so a proposal is
  safe only because it cannot publish without a camera colour vote.
- `SlotSignMap` against the shipped map on identical observations over 125 bags,
  and the four refuted publication-time repairs: `ObservedSignMap` believed 9 to
  25 signs on a track that physically holds at most 8, with about half of all
  believed positions matching no legal cell, on a 0.15-0.25 m position error
  against pillars 0.20 m apart across a lane pair and 0.50 m apart along a
  section. `snap_to_lattice_m = 0.40` appeared to halve routing errors but 264
  passes vanished from the denominator and the peak believed count rose 26 to 40.
  The rules-constrained slots map scored worst peak believed 24 to 7, runs over
  the physical max 32/125 to 0/125, re-points while committed 1062 to 0 and
  colour flips while committed 66 to 0. The lane partner has zero evidence 21
  percent of the time and the cut is clear 70 percent, so lane placement is near
  a coin flip; deliberately flipping every lane routes at 14.3 percent against
  15.8 percent correct, and lane error moves execution by +7 points. Colour
  pooling at radius 0.00/0.25/0.55 gives flip totals 157/159/166 (refuted). The
  simulator's exact map has 0.0 percent of ticks above the physical maximum
  against 86 percent on hardware.
- Re-pointing a slot whose router index is already PASSED would make an unpassed
  pillar inherit the "behind us" flag: measured on 168 of 708 re-points (24
  percent). Fresh slots recover those pillars at 23 passes over 125 runs and
  +1.0 point of routing error, newly measured exposure rather than newly created.
- `_claim` leaves a reading further than `slot_accept_radius_m` from every cell
  unclaimed rather than pulling it to the nearest: 12.2 percent of observations
  are in that class.
- `_one_per_depth`: the rulebook's 36-scenario table never puts two pillars on
  one depth line (every double is depth 1.0 plus 2.0, 1.5 alone; verified over
  all 24 doubles and all 514 same-section pairs in the 256 corpus). Measured on
  run_20260915_002408 (counter-clockwise, 3/3 laps) the east section's two slots
  were held by (2.4, 1.0) and (2.6, 1.0), one green pillar the LIDAR places at
  x=2.49-2.55, while the red pillar at (2.40, 1.88) was refused all round: 55
  fused observations landed on its cell and never displaced either twin, and the
  chassis escaped 30 times in one 0.5 m cell against a pillar its map did not
  contain. The same twin pair shows on run_20260914_215248.
- `_apply_section`: an earlier version opened a slot whenever no incumbent was
  displaceable, which let a section hold three. Measured on run_20260911_225646
  as a believed-sign peak of 12 against the physical maximum of 8, against the
  shipped map's 64.
- `_displaces` hysteresis: the cut between the last accepted and first rejected
  cell is clear (2x or better) in only 56.5 percent of section-runs, p10 ratio
  1.20, so about a third of assignments would flip on noise. At margin 1.5 the
  churn halves for a median 0.55 s of phantom hold (p90 9.2 s); above 2.0 the
  tail and the count of challengers that lead at the end and never get the slot
  grow faster than the churn falls.
- `set_committed`: freezing the committed slot takes colour flips while committed
  from 66 to 0 and re-points from 166 to 0, and routing improves from 15.8 to
  15.0 percent.
- The `retire` wiring landed 2026-09-11 in `dabd57c7`; the NOT-WIRED note landed
  two days after it.
- The `reset_for_new_lap` / `retire` asymmetry: measured 2026-09-15 over five
  rounds spanning two builds, zero ticks reading over 8 on lap 1 (the cap holds
  exactly while `_passed == _retired`) against 20-53 percent of ticks on laps 2
  and 3, peak 14. The design was measured as a package at 23 recovered passes
  over 125 runs and +1.0 point of routing error, so forwarding the lap reset is
  no longer a no-op and needs its own A/B.
- `lidar_proposer`: the camera stops resolving signs past about 1.1 m while the
  LIDAR picks up pillar-shaped clusters at a median 1.31 m, buying position early
  and colour in the last 0.3 m. The placement lattice raises the detector's
  precision from roughly half to the mid-eighties while giving up a little recall.
- Decoding a detection against the pose the CAMERA saw from, rather than the pose
  at receipt, takes the bearing residual from 20.2 deg to 5.4 deg. The corrected
  pairing is what makes `range_scale`'s pinhole correction usable at all; see the
  `vision_latency_s` measurement above.
- `max_pillar_aspect`: neither area nor confidence separates the blurred barrier
  from a real pillar, only aspect does.
- `range_scale`: without the pose alignment the scale doubled lateral error (9.8 to
  18.6 cm); aligned it goes 3.6 to 6.8 cm. Binned error is about +/-5 cm out to
  1.1 m. Prefer fixing the detector box convention and returning `range_scale` to
  1.0.
- `lidar_range_fusion` ungated: the gate was 0.05 < r < 10.0 m, i.e. no gate; because
  a wall behind a sign is always farther it supplied half the outward bias that
  pinned believed signs to walls. The ungated arm reproducing its own earlier
  refutation is what validates the 78-bag replay harness.
- `vision_latency_s`: `/vision/detections` is a headerless `std_msgs/String`, so
  before 2026-09-07 every detection was decoded against the pose at RECEIPT. At
  0.3 m/s through a corner the 0.85 s lag is most of a sign's lateral offset.
- `max_signs_per_section`: the rulebook allows two per section and eight on the
  track; unlike `snap_to_lattice_m`, which quantised position and left the count
  alone, a cap needs no opinion about which tracks are duplicates.
- `barrier_belief_min_sightings`: every magenta detection was dropped by
  `detection_to_observation`, which returns None for any non-routing colour.
  `barrier_merge_radius_m` is sized to absorb the pinhole range error rather than
  to resolve the object, because the lot is 0.20 m long. `barrier_suppression_radius_m`
  is sized from the pinhole position error, not from the lot.
- `barrier_span_along_wall`: `barrier_belief.py`'s own docstring records the same
  wedge at (0.75, 0.25) on 2026-09-14. The chassis spent 70 s and 172 s (46 percent
  of each round) fighting the west fin while the planner routed around a phantom
  0.45 m away, and the aggregate cannot see the west fin, which is the thing that
  ends rounds. Narrowing the merge/suppression radius to 0.22 or 0.18 alongside the
  span moves back down the same benefit/cost curve rather than off it.
- The barrier arrived as a RED detection at 60 percent in one round and 4 to 9
  percent in two others, located from the lot's own LIDAR cluster; the rate ships
  at 0.0 because 4 to 60 percent is a range, not a rate.
