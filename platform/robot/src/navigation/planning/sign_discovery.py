"""Build a persistent map of traffic signs from camera detections alone.

``SignRouter`` has always been handed its signs from scenario metadata, which
is knowledge the robot cannot have: WRO randomises the sign layout every round
and no scenario file exists on the mat. A blind run therefore currently ends up
with ``signs == []`` and no sign avoidance at all (``node.py`` only builds a
router when metadata supplies one) — the Obstacles Challenge equivalent of the
corridor-width problem ``corridor_estimator.py`` solves for the Open Challenge.

This closes that gap. Detections arrive per-frame and are individually noisy
and transient; routing needs a stable world-frame position that persists after
the sign leaves the camera's cone (``activation_dist`` is 0.80 m while the
chassis is 0.30 m long, so the robot is still steering around a sign well after
it has left the frame). So detections are associated into persistent tracks.

What the robot legitimately knows without a scenario file, and what it does not:

* Known: the pinhole geometry, the sign's real 0.10 m height, its own pose.
  Those are what ``_detection_to_world`` needs, and it already exists.
* Not known: how many signs there are, where, or what colour. All three are
  discovered here.

**Range is the accuracy limit.** Distance comes from the bounding box height
via the pinhole model, so its error grows with range: at 1.5 m a sign spans
~42 px and a 1 px error is ~3.6 cm, but by 3 m it spans ~21 px and the same
1 px error is ~14 cm — wider than the gap between two lanes of the WRO sign
grid, which would place the sign in the wrong lane and route the robot to the
wrong side. Observations beyond :data:`_MAX_INGEST_RANGE` are therefore
ignored, and a track's position is taken from its closest observation rather
than averaged over all of them, since the error is monotone in range and
averaging would let distant readings pull a good estimate off.

Layering note: this module sits *below* ``sign_router`` and must not import it.
It owns the geometric primitives both need — ``SignSpec`` and the pinhole
projection — which ``sign_router`` re-exports, so every existing
``from ...sign_router import SignSpec`` keeps working.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs, TrackDimensions, TrafficSignSpecs
from shared.domain.models import Pose, SignColor, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.lidar_proposer import ProposerParams, find_clusters
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.ports import LidarScan
from src.navigation.utils import _dist2d, _nearest_ray, wrap_angle

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.enums import Section
    from shared.domain.models import Detection

logger = logging.getLogger(__name__)

# Camera focal length in pixels — derived from HFOV and image width.
_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)


@dataclass(frozen=True)
class SignSpec:
    """Expected traffic sign location and color from scenario metadata."""

    x: float
    y: float
    color: SignColor


def detection_to_observation(
    det: Detection,
    robot_pose: Pose,
    tuning: NavigationTuning | None = None,
    lidar_ranges_m: Sequence[float] | None = None,
    lidar_angles_rad: Sequence[float] | None = None,
    barrier_possible: bool = True,
) -> TrafficSignObservation | None:
    """Convert a Detection (pixel bbox) to a TrafficSignObservation (world coords).

    This is the bridge between the ROS wire format (Detection) and the
    internal world-coordinate observation used by ObservedSignMap. Returns
    None for a non-sign detection (e.g. a MAGENTA parking-block class) --
    previously this fell through an ``else GREEN`` default that silently
    misclassified anything that wasn't literally "red", including a genuine
    magenta detection, as a green sign.

    Args:
        det: Single camera detection with bbox (x1, y1, x2, y2).
        robot_pose: Robot's current pose estimate.
        tuning: Navigation tuning instance. Defaults to the default tuning profile.
        lidar_ranges_m: The same tick's LIDAR sweep, robot-frame, for range
            fusion -- see :func:`_detection_to_world`. ``None`` (the default)
            uses the pinhole-only distance estimate.
        lidar_angles_rad: Matching robot-frame bearings for ``lidar_ranges_m``.
    """
    if det.class_name not in (SignColor.RED, SignColor.GREEN):
        return None
    tuning = get_tuning(tuning)
    # A pillar is taller than it is wide. Rejecting the rest is what keeps the
    # magenta parking-lot barrier -- which reads as RED under motion blur -- from
    # seeding a sign every frame it is in view. Colour and confidence cannot do
    # it: those boxes carry the RED label at p50 confidence 0.79. See
    # SignDiscoveryParams.MAX_PILLAR_ASPECT for the measurement.
    #
    # The test is SKIPPED for a box clipped by the frame, because the aspect
    # ratio of a clipped box is not a measurement of the object's shape. A
    # pillar the robot is closing on grows until it runs out of frame: its
    # height then stops increasing while its width keeps going, and the ratio
    # crosses 1.0 with nothing about the pillar having changed. Traced on
    # run_20260906_145546 at 23.4-24.6 s -- a red pillar detected at 0.47-0.84
    # confidence, x_max pinned at the frame edge for every frame, w/h climbing
    # 0.33 -> 1.27 as it approached, rejected exactly when it was nearest and
    # mattered most. Across two runs, 304 of the 500 red detections this gate
    # rejects (61%) are frame-clipped.
    bbox_for_shape = det.as_bbox()
    height_px = bbox_for_shape.y_max - bbox_for_shape.y_min
    width_px = bbox_for_shape.x_max - bbox_for_shape.x_min
    max_aspect = tuning.sign_discovery.MAX_PILLAR_ASPECT
    frame_w = float(RobotSpecs.CAMERA_WIDTH)
    frame_h = float(RobotSpecs.CAMERA_HEIGHT)
    edge = tuning.sign_discovery.FRAME_EDGE_TOLERANCE_PX
    clipped = (
        bbox_for_shape.x_min <= edge
        or bbox_for_shape.y_min <= edge
        or bbox_for_shape.x_max >= frame_w - edge
        or bbox_for_shape.y_max >= frame_h - edge
    )
    # The shape test only makes sense where the thing it is filtering can BE.
    # There is exactly one parking lot and it sits in the corridor the robot
    # started in; a wide red box seen from any other corridor cannot be the
    # barrier, and rejecting it only throws away a pillar. Measured across
    # run_20260906_145546 and _145909: wall-shaped reds sit at corridor `None`
    # 72% of the time -- the start, in and around the bay -- exactly where the
    # magenta barrier detections sit (67%), while pillar-shaped reds spread
    # across the driving corridors (south 62-73%, west 11-24%).
    if barrier_possible and max_aspect > 0.0 and not clipped and height_px > 0 and width_px / height_px > max_aspect:
        return None
    world = _detection_to_world(
        det,
        (robot_pose.x, robot_pose.y),
        robot_pose.yaw,
        tuning,
        lidar_ranges_m,
        lidar_angles_rad,
    )
    if world is None:
        return None
    bbox = det.as_bbox()
    return TrafficSignObservation(
        world_x_m=world[0],
        world_y_m=world[1],
        color=det.class_name,
        confidence=det.confidence,
        detected_at_timestamp=0.0,
        bbox_xmin=int(bbox.x_min),
        bbox_ymin=int(bbox.y_min),
        bbox_xmax=int(bbox.x_max),
        bbox_ymax=int(bbox.y_max),
    )


def legal_sign_positions() -> tuple[tuple[float, float], ...]:
    """Every world position a pillar may legally stand at. 24 of them.

    Built from the rulebook geometry rather than transcribed: the three depth
    rows are ``GRID_DEPTH_NEAR/MIDDLE/FAR`` (1.0/1.5/2.0 m), which coincide
    exactly with the inner square's own bounds, and the two width lines are the
    corridor's division lines ``GRID_WIDTH_OUTER/INNER`` (0.4/0.6 m). Six per
    section, four sections.

    Deliberately returns ALL of them together with no section label. Snapping
    to the nearest of the whole set needs no answer to "which corridor is this
    sign in", which matters because that label is the known-flaky one --
    ``current_corridor`` flips 37-39 times in a three-lap run holding twelve
    real corners. A snap keyed on it would inherit the flapping it exists to
    cure. The 24 points are far enough apart (0.20 m is the closest pair, the
    two width lines) that nearest-point is unambiguous well past the estimate
    error this corrects.
    """
    depths = (
        TrafficSignSpecs.GRID_DEPTH_NEAR,
        TrafficSignSpecs.GRID_DEPTH_MIDDLE,
        TrafficSignSpecs.GRID_DEPTH_FAR,
    )
    near = (TrafficSignSpecs.GRID_WIDTH_OUTER, TrafficSignSpecs.GRID_WIDTH_INNER)
    far = tuple(TrackDimensions.TRACK_SIZE - w for w in near)
    points: list[tuple[float, float]] = []
    for d in depths:
        points.extend((d, w) for w in near)   # SOUTH
        points.extend((d, w) for w in far)    # NORTH
        points.extend((w, d) for w in near)   # WEST
        points.extend((w, d) for w in far)    # EAST
    return tuple(points)


_LEGAL_SIGN_POSITIONS = legal_sign_positions()


def snap_to_lattice(x: float, y: float, max_snap_m: float) -> tuple[float, float]:
    """Pull a believed sign position onto the nearest legal lattice point.

    A pillar does not move during a round, and it can only have been placed at
    one of 24 positions. A believed position that wanders is therefore known to
    be wrong, and quantising it to the lattice removes the wander by
    construction rather than by damping it. Measured 2026-09-09: within a LIVE
    commitment the believed position moves p50 7.7 cm, p90 60.9 cm, max 103 cm,
    which is what drops the sign out of the router's candidate filter and makes
    the commitment a 0.5 s duty cycle.

    ``max_snap_m`` of 0 disables this. Above 0 it is also a REJECTION radius:
    an estimate further than that from every legal point is left alone rather
    than dragged onto one, because a wild estimate snapped confidently is worse
    than a wild estimate that still looks wild. The pillar may itself be nudged
    up to ``MAX_LEGAL_DISPLACEMENT_M`` (5.94 cm) and stay legal, so a radius
    below that would refuse to snap a sign the robot has legitimately bumped.
    """
    if max_snap_m <= 0.0:
        return x, y
    best = min(_LEGAL_SIGN_POSITIONS, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
    if math.dist(best, (x, y)) > max_snap_m:
        return x, y
    return best


def _clustered_range(
    lidar_ranges_m: Sequence[float],
    lidar_angles_rad: Sequence[float],
    bearing_rad: float,
    pinhole_m: float,
    tuning: NavigationTuning,
) -> float | None:
    """Range of the pillar-shaped, ISOLATED cluster at ``bearing_rad``, if any.

    The qualified half of the sensor split: the CAMERA has already said a sign
    is at this bearing and what colour it is, and the LIDAR is asked only how
    far away it is -- the one thing it measures directly, where the camera
    infers it from bbox height through a focal that does not agree with the one
    solved from bearings (1034-1088 px against 545-645).

    Returns None, leaving the pinhole estimate standing, unless BOTH tests
    pass. Two tests rather than one because each catches a different way the
    unqualified version failed:

    * SHAPE AND ISOLATION, via ``find_clusters`` -- a wall segment is
      contiguous but never steps away at both ends, and the first attempt took
      a wall on 51% of detections.
    * AGREEMENT with the pinhole -- a pillar standing in front of a wall offers
      two plausible clusters and the further one is the wall, which is why that
      attempt's range error was p50 -69 cm.

    The nearest cluster in ANGLE wins, not in range. The camera's claim is a
    BEARING, so bearing is what identifies the object it is talking about;
    choosing by range would re-import the very bias that broke the first
    attempt, since the wall behind a sign is always further away.
    """
    scan = LidarScan(ranges_m=tuple(lidar_ranges_m), angles_rad=tuple(lidar_angles_rad))
    clusters = find_clusters(scan, ProposerParams())
    if not clusters:
        return None
    best = min(clusters, key=lambda c: abs(wrap_angle(c.bearing_rad - bearing_rad)))
    if abs(best.range_m - pinhole_m) > tuning.sign_discovery.LIDAR_RANGE_FUSION_AGREEMENT * pinhole_m:
        return None
    return best.range_m


def _detection_to_world(
    det: Detection,
    robot_pos: tuple[float, float],
    robot_yaw: float,
    tuning: NavigationTuning | None = None,
    lidar_ranges_m: Sequence[float] | None = None,
    lidar_angles_rad: Sequence[float] | None = None,
) -> tuple[float, float] | None:
    """Project a bbox detection to an approximate world position.

    The camera alone gives bearing (accurate -- horizontal position in frame
    doesn't depend on depth) and colour (LIDAR has no notion of colour, so a
    detection is required regardless). Distance from bbox height alone grows
    less accurate with range -- ~3.6 cm error at 1.5 m, ~14 cm by 3 m, enough
    to misjudge which WRO sign lane a sign sits in (see
    docs/sign-avoidance-investigation.md). The LIDAR sees the same signs
    (confirmed on hardware) and measures range far more precisely at any
    distance, so when a scan is available this looks up the ray nearest the
    camera's own bearing and trusts ITS range instead of the pinhole
    estimate -- falling back to pinhole-only when that ray is not a
    plausible return (dropout, self-detection, or implausibly far to be the
    same object the camera is looking at).

    Args:
        det: Single camera detection with bbox (x1, y1, x2, y2).
        robot_pos: Robot (x, y) position (metres).
        robot_yaw: Robot heading (radians, 0 = east).
        tuning: Navigation tuning instance. Defaults to the default tuning profile.
        lidar_ranges_m: The same tick's LIDAR sweep, robot-frame, for range
            fusion. ``None`` skips fusion and uses the pinhole estimate alone
            -- the only behaviour available before this fusion existed, and
            still correct when no scan is available that tick.
        lidar_angles_rad: Matching robot-frame bearings for ``lidar_ranges_m``.

    Returns:
        Estimated world (x, y) of the sign, or None if bbox is too small.
    """
    tuning = get_tuning(tuning)
    bbox = det.as_bbox()
    pixel_height = bbox.height
    if pixel_height < tuning.sign_discovery.MIN_RELIABLE_BBOX_HEIGHT_PX:
        return None

    # Estimate distance using pinhole model: d = (f * real_h) / pixel_h.
    #
    # RANGE_SCALE ships at 1.0, so this is the raw pinhole -- deliberately,
    # even though it UNDER-reads by ~2x. Correcting the range with a scalar
    # DOUBLES the lateral error, because the residual ~12 deg bearing error is
    # angular and a longer ray lengthens the lateral miss in proportion. Lateral
    # is what the router acts on. See RANGE_SCALE's docstring; fix the bearing
    # error first.
    distance = _CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT / pixel_height * tuning.sign_discovery.RANGE_SCALE

    # Horizontal angle from image centre, POSITIVE TO THE LEFT to match the
    # robot frame (`LidarScan`: 0 = forward, +pi/2 = left, CCW positive).
    #
    # This was `(cx / W - 0.5) * HFOV` until 2026-09-06, which is positive for a
    # box on the RIGHT of the image -- the robot's right, i.e. a NEGATIVE CCW
    # bearing. Every sign was therefore reflected across the robot's heading
    # axis, which in a 1 m corridor lands it on the far wall. Nothing in-tree
    # could catch it: `vision_emulator` reproduces the true geometry directly
    # and the router's unit tests BUILD `cx` from this same formula, so both
    # were self-consistent with the error.
    #
    # Measured on run_20260906_192424 by predicting the box's centre column from
    # the true bearing to a LIDAR-located pillar: upright 174 px of error
    # against 528 px mirrored, reproduced at three robot headings spanning 165
    # degrees. Sign-position error p50 64 cm -> 48 cm, and the outward bias that
    # pinned believed signs to the walls falls from +40 cm (77% outward) to
    # -3 cm (47%).
    cx = bbox.center.x
    theta_h = (0.5 - cx / RobotSpecs.CAMERA_WIDTH) * RobotSpecs.CAMERA_HFOV

    # LIDAR range fusion, OFF by default -- it was measured to make the estimate
    # WORSE. A single ray at the camera's bearing is not the pillar: at that
    # bearing the return is wall-shaped (implied chord > 30 cm) on 51% of
    # detections and pillar-shaped on 27%, median implied chord 34 cm against a
    # 5 cm sign. It fired on 92.5% of detections and cost 5 cm of median
    # position error with the old bearing and 28 cm with the corrected one,
    # because a wall behind a sign is always FURTHER -- it was the second half
    # of the outward bias. Even restricted to pillar-shaped returns the range
    # error is p50 -69 cm, with only 16% within 10 cm.
    #
    # Kept rather than deleted because the idea is sound and the implementation
    # is what failed: the gate is `0.05 < r < 10.0`, which is no gate at all.
    # A version that required a small isolated cluster AND agreement with the
    # calibrated pinhole would be worth measuring -- but the cluster-shape test
    # alone discriminated pillar from wall at 54%, near chance, so that wants
    # its own evidence before it ships.
    if tuning.sign_discovery.LIDAR_RANGE_FUSION and lidar_ranges_m and lidar_angles_rad:
        lidar_range = (
            _clustered_range(lidar_ranges_m, lidar_angles_rad, theta_h, distance, tuning)
            if tuning.sign_discovery.LIDAR_RANGE_FUSION_CLUSTER
            else _nearest_ray(lidar_ranges_m, lidar_angles_rad, theta_h)
        )
        if (
            lidar_range is not None
            and tuning.lidar_sectors.MIN_VALID_RANGE_M < lidar_range < RobotSpecs.CAMERA_FAR_CLIP
        ):
            distance = lidar_range

    # Project from where the SENSOR is, not from the body centre. `distance` is
    # measured by the camera or the C1, and both sit 0.1222 m forward of centre
    # (robot.toml's [camera]/[lidar] mount_x_offset -- equal today, so the
    # bearing lookup above can treat them as coincident in the ground plane).
    # Starting the ray at the chassis origin therefore lands every sign short by
    # that offset, pulling the estimate ~12 cm toward the robot along its
    # heading. localization.py took this same correction on 2026-08-21, where
    # casting predicted rays from the centre was biasing the pose fit along the
    # corridor axis; this consumer was missed then. Visible in RViz as sign
    # estimates sitting 10-20 cm off the drawn signs.
    sensor_x = robot_pos[0] + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(robot_yaw)
    sensor_y = robot_pos[1] + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(robot_yaw)
    bearing = robot_yaw + theta_h
    wx = sensor_x + distance * math.cos(bearing)
    wy = sensor_y + distance * math.sin(bearing)
    return wx, wy


@dataclass(slots=True)
class _SignTrack:
    """One candidate sign, accumulated across frames."""

    x: float
    y: float
    best_range: float
    """Range of the closest observation so far — the one (x, y) is taken from."""

    corridor: Section
    """The ROBOT's own corridor (``corridor_for_position`` on its position, not
    the sign's) at the moment this track was created. Fixed for the track's
    lifetime -- a sign does not change corridor.

    Association is gated on this matching the robot's settled corridor at
    each new observation (see ``ObservedSignMap._nearest_track`` and
    ``_settle_robot_corridor``), not on the observation's own reprojected XY.
    This is what prevents a believed pose that is a wrong-but-consistent
    rigid rotation of the truth (the blind-mode rotational-lock failure) from
    folding one corridor's sign into a different corridor's track just
    because the rotation happens to land their reprojected positions close
    together: the two signs' own reprojected XY coincide by construction of
    that bug, so a gate keyed on the SIGN's position (including a corridor
    label derived from it) cannot tell them apart -- but the ROBOT's own true
    position when it observed each one still classifies to a different
    corridor, since it was standing in a genuinely different place.

    A continuous distance on the robot's own RAW WORLD-XY position (rather
    than this discrete corridor) was tried and measured WORSE across the
    whole range of reasonable thresholds -- see the sign_discovery.py module
    notes -- because it is not rotation-equivariant: it can alias two
    different real corridors' signs together exactly like the bug this gate
    exists to survive.

    A second attempt, still rotation-equivariant, widened this gate near a
    corner boundary to also accept the adjacent corridor within a
    ``corner_blend_m`` face-distance slack (via a since-reverted
    ``corridor_candidates_for_position``), on the theory that a sign visible
    from either of two adjacent corridors was being arbitrarily split into
    two tracks. Measured on the full 256-scenario blind corpus: EVERY
    nonzero threshold tried (0.03-0.20 m) was worse than 0.0 on collisions,
    laps>=3, and in-time -- 190 (0.0) vs 195-207 across the range, non-
    monotonic, with escape-maneuver rate moving in step -- so it was
    reverted rather than shipped. (All seven arms came from a single sweep
    invocation, so that internal comparison is fair, but do NOT read the
    190 as the corpus baseline: a standalone run reproduces 202/256
    twice, byte-identical, and the 190 was never cross-checked.) Read as: the corner-boundary ambiguity
    this targeted is not, in practice, the dominant remaining failure mode,
    and blending in a second corridor's tracks costs more (via bad merges)
    than the boundary misses it recovers."""

    hits: int = 0
    votes: dict[str, float] = field(default_factory=dict)
    """Confidence-weighted colour votes. A sign's colour decides which side the
    robot must pass it, so a single mislabelled frame must not be able to flip
    it; summing confidence across frames makes that need sustained disagreement."""

    published_index: int | None = None
    """Index in the router's sign list once published, or None while pending."""

    lidar_fixed: bool = False
    """Whether this track's position came from a LIDAR proposal.

    Once set, camera observations contribute COLOUR ONLY and never move the
    position. The LIDAR measures range directly; the camera infers it from bbox
    height through a pinhole model whose error put believed pillars ON THE WALLS
    on hardware. Letting a nearer camera reading overwrite a LIDAR fix would
    reintroduce exactly that error at the moment it matters most."""

    @property
    def color(self) -> SignColor:
        """The winning colour vote, or UNKNOWN while no camera has voted.

        ``votes`` is keyed by ``SignColor.value`` (see ``_fold_observation``),
        so the winner is always a valid member; converting here keeps the
        ``str`` keys the dict is built with while handing callers the enum
        ``SignSpec.color`` is declared as.

        A LIDAR-proposed track has a position and no votes. UNKNOWN is the
        honest answer for it -- ``max()`` over an empty dict would raise, and
        any default colour would be a coin flip on a round-ending rule.
        """
        if not self.votes:
            return SignColor.UNKNOWN
        return SignColor(max(self.votes, key=lambda name: self.votes[name]))

    snap_m: float = 0.0
    """Lattice snap radius this track publishes with. See ``snap_to_lattice``."""

    def as_spec(self) -> SignSpec:
        """The track as the router sees it, snapped to the legal lattice.

        Snapping HERE rather than on ingest keeps the raw estimate intact for
        association and for the range/colour votes, and quantises only the
        number the router acts on. An estimate that is drifting is still
        allowed to drift back onto a better lattice point; what it can no
        longer do is hand the router a position 60 cm from where any pillar
        could physically be.
        """
        x, y = snap_to_lattice(self.x, self.y, self.snap_m)
        return SignSpec(x=x, y=y, color=self.color)


class ObservedSignMap:
    """Persistent world-frame sign map accumulated from camera detections.

    Tracks are append-only once published: ``SignRouter`` keys its
    ``_passed``/``_engaged`` bookkeeping by list index, so reordering or
    removing a published sign would silently retarget that bookkeeping onto a
    different sign. Positions are refined in place, which keeps those indices
    valid while still letting a closer look correct an early estimate.
    """

    def __init__(
        self,
        min_confidence: float,
        max_ingest_range_m: float | None = None,
        association_dist_m: float | None = None,
        min_hits: int | None = None,
        robot_corridor_flip_ticks: int | None = None,
        tuning: NavigationTuning | None = None,
    ) -> None:
        """Start an empty map.

        Args:
            min_confidence: Detections below this confidence are ignored, so the
                map inherits the router's own detection threshold rather than
                introducing a second, independently-tuned one.
            max_ingest_range_m: Ignore observations further than this (m).
                Defaults to NavigationTuning.sign_discovery.MAX_INGEST_RANGE_M.
            association_dist_m: Two observations within this distance (m)
                are treated as the same sign.
                Defaults to NavigationTuning.sign_discovery.ASSOCIATION_DIST_M.
            min_hits: Observations required before a track is published as
                a real sign. Defaults to NavigationTuning.sign_discovery.MIN_HITS.
            robot_corridor_flip_ticks: Consecutive ticks the robot's own
                corridor must disagree with the settled value before the
                association gate accepts the change. Defaults to
                NavigationTuning.sign_discovery.ROBOT_CORRIDOR_FLIP_TICKS.
            tuning: Navigation tuning instance. Defaults to the default tuning profile.
        """
        tuning = get_tuning(tuning)
        sd = tuning.sign_discovery
        self._min_confidence = min_confidence
        self._max_ingest_range_m = max_ingest_range_m if max_ingest_range_m is not None else sd.MAX_INGEST_RANGE_M
        self._association_dist_m = association_dist_m if association_dist_m is not None else sd.ASSOCIATION_DIST_M
        self._min_hits = min_hits if min_hits is not None else sd.MIN_HITS
        self._snap_m = sd.SNAP_TO_LATTICE_M
        self._robot_corridor_flip_ticks = (
            robot_corridor_flip_ticks if robot_corridor_flip_ticks is not None else sd.ROBOT_CORRIDOR_FLIP_TICKS
        )
        self._tracks: list[_SignTrack] = []
        self._robot_corridor: Section | None = None
        """Settled robot corridor -- see ``_settle_robot_corridor``."""
        self._robot_corridor_flip_streak: tuple[Section, int] | None = None

    def _settle_robot_corridor(self, raw: Section) -> Section:
        """Debounce the robot's own corridor classification.

        Mirrors ``SignRouter._settled_corridor``, which debounces a sign's -- see
        ``ROBOT_CORRIDOR_FLIP_TICKS``'s docstring for why the robot's own per-tick
        classification needs it even though a sign's does not.
        """
        if self._robot_corridor is None or raw == self._robot_corridor:
            self._robot_corridor_flip_streak = None
            self._robot_corridor = raw
            return raw
        candidate, streak = self._robot_corridor_flip_streak or (raw, 0)
        streak = streak + 1 if candidate == raw else 1
        if streak < self._robot_corridor_flip_ticks:
            self._robot_corridor_flip_streak = (raw, streak)
            return self._robot_corridor
        self._robot_corridor_flip_streak = None
        self._robot_corridor = raw
        return raw

    def observe(
        self,
        observations: list[TrafficSignObservation] | None,
        robot_pos: Waypoint,
    ) -> None:
        """Fold one frame of world-coordinate observations into the map.

        Args:
            observations: World-coordinate traffic sign observations.
            robot_pos: Robot position, used for range gating.
        """
        if not observations:
            return

        robot_corridor = self._settle_robot_corridor(corridor_for_position(robot_pos.x, robot_pos.y))
        for obs in observations:
            if obs.confidence < self._min_confidence:
                continue
            if obs.color not in (SignColor.RED, SignColor.GREEN):
                continue

            world = Waypoint(obs.world_x_m, obs.world_y_m)
            observed_range = _dist2d(world, robot_pos)
            if observed_range > self._max_ingest_range_m:
                continue

            self._fold(world, observed_range, obs, robot_corridor)

    def propose(self, positions: list[tuple[float, float]] | None, robot_pos: Waypoint) -> None:
        """Fold LIDAR-proposed POSITIONS into the map, casting no colour vote.

        The LIDAR says an object is there, roughly 0.6 m before the camera can
        say what it is. A proposal therefore refines geometry and nothing else:
        it starts or updates a track, marks the position LIDAR-fixed so no later
        pinhole estimate can move it, and leaves ``votes`` empty so the track
        reports ``SignColor.UNKNOWN``.

        A track with no votes is never published (see ``newly_confirmed``), so
        proposals CANNOT reach the router on their own -- which is what makes
        this safe at the detector's measured 84% precision. What they buy is
        that when the camera finally votes, the position is already settled
        instead of being established from scratch inside the last 0.3 m.

        Args:
            positions: World ``(x, y)`` candidates from ``lidar_proposer.propose``.
            robot_pos: Robot position, used for range gating and corridor keying.
        """
        if not positions:
            return
        robot_corridor = self._settle_robot_corridor(corridor_for_position(robot_pos.x, robot_pos.y))
        for x, y in positions:
            world = Waypoint(x, y)
            observed_range = _dist2d(world, robot_pos)
            if observed_range > self._max_ingest_range_m:
                continue
            track = self._nearest_track(world, robot_corridor)
            if track is None:
                track = _SignTrack(
                x=world.x,
                y=world.y,
                best_range=observed_range,
                corridor=robot_corridor,
                snap_m=self._snap_m,
            )
                self._tracks.append(track)
            track.hits += 1
            # Closest LIDAR look wins, on the same monotone-error argument the
            # camera path uses -- but between LIDAR readings only.
            if not track.lidar_fixed or observed_range <= track.best_range:
                track.best_range = observed_range
                track.x, track.y = world.x, world.y
            track.lidar_fixed = True

    def _fold(
        self,
        world: Waypoint,
        observed_range: float,
        obs: TrafficSignObservation,
        robot_corridor: Section,
    ) -> None:
        """Merge one projected observation into the nearest track, or start one."""
        track = self._nearest_track(world, robot_corridor)
        if track is None:
            track = _SignTrack(
                x=world.x,
                y=world.y,
                best_range=observed_range,
                corridor=robot_corridor,
                snap_m=self._snap_m,
            )
            self._tracks.append(track)

        track.hits += 1
        track.votes[obs.color.value] = track.votes.get(obs.color.value, 0.0) + obs.confidence

        # A LIDAR-fixed position is not up for revision by a camera estimate --
        # see `_SignTrack.lidar_fixed`. The camera still votes on colour above,
        # which is the whole point of the split.
        if track.lidar_fixed:
            return

        # Closest observation wins outright: pinhole range error is monotone in
        # range, so a nearer reading is strictly better evidence than the
        # running estimate, and averaging would only let worse readings back in.
        if observed_range <= track.best_range:
            track.best_range = observed_range
            track.x, track.y = world.x, world.y

    def _nearest_track(self, world: Waypoint, robot_corridor: Section) -> _SignTrack | None:
        """The closest existing track within ``self._association_dist_m``, if any.

        Also requires the track to have been created from the same ROBOT
        corridor -- see ``_SignTrack.corridor``'s docstring. Distance alone is
        not enough under a wrong-but-consistent rigid rotation of the
        believed pose, which can reproject one corridor's sign close enough
        to another corridor's true position to fall inside
        ``association_dist_m``.
        """
        best: _SignTrack | None = None
        best_dist = self._association_dist_m
        for track in self._tracks:
            if track.corridor != robot_corridor:
                continue
            d = _dist2d(Waypoint(track.x, track.y), world)
            if d < best_dist:
                best_dist = d
                best = track
        return best

    def newly_confirmed(self) -> list[_SignTrack]:
        """Tracks that have crossed ``self._min_hits`` and are not yet published.

        The caller is responsible for assigning ``published_index`` once it has
        appended the returned specs to its own sign list.

        Two variants of a cross-corridor, position-proximity dedup were tried
        here, both targeting the SAME physical sign forking a brand-new track
        on a later lap (traced on go_obstacles_0003: 18 tracks published for
        6 physical signs over 3 laps, corrupting sign_lane.py's per-corridor
        plateau geometry at the collision tick) -- and both measured WORSE on
        the full 256-scenario blind corpus, so neither is shipped:

        1. Skip publishing a track within a small distance of an already-
           published one: 209/256 collisions against a 202/256 baseline,
           laps>=3 45 vs 70, escapes more than doubled. Discarding the duplicate also discarded
           whatever position/colour refinement IT was accumulating, freezing
           the original (often less mature, lap-1) estimate forever.
        2. FOLDING the duplicate into the target instead (hits/votes summed,
           closest-range position/colour evidence wins) rather than dropping
           it, specifically to keep that refinement: measured WORSE STILL --
           231/256 collisions against that same 202/256 baseline, laps>=3
           24 vs 70, in-time 7 vs 31, i.e. worse than variant 1 it was
           meant to improve on. Folding can
           suddenly move an already-published, already-converged sign's
           position or flip its colour vote mid-run the moment ANY
           position-proximate track merges in, even one that is not a clean
           re-detection of the same physical sign (estimate noise, a
           genuinely different nearby feature) -- yanking a stable, correctly
           routed sign into a discontinuous change is evidently more
           disruptive than either leaving it alone or silently dropping the
           duplicate.

        Both variants share the same underlying issue: from position alone,
        "same physical sign re-discovered under a different corridor label"
        and "a different observation that happens to land nearby" cannot
        always be told apart, and neither variant's failure mode was fixed by
        changing what happens to the duplicate's DATA -- it's the very act of
        letting a cross-corridor position match influence the published
        record at all that costs more than the duplication it targets. Any
        further attempt at this go_obstacles_0003 mechanism should look
        upstream of publication (e.g. resolving WHY the robot's settled
        corridor is unstable at a re-detection, not what to do once it
        already forked a track) rather than a third downstream dedup variant.

        A track also needs a COLOUR before it is published. A LIDAR-proposed
        track accumulates hits with no colour vote, and can cross ``min_hits``
        on geometry alone; publishing it would hand the router a sign with no
        pass side, whose every routing decision would then decline. Holding it
        back until the camera votes keeps the router's world exactly as it was
        while still letting the proposal refine the position in the meantime.
        """
        return [
            t for t in self._tracks if t.published_index is None and t.hits >= self._min_hits and t.votes
        ]

    def published(self) -> list[_SignTrack]:
        """Tracks already handed to the router, for in-place position refinement."""
        return [t for t in self._tracks if t.published_index is not None]
