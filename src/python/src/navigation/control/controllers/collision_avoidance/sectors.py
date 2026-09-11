"""LIDAR sector math for collision avoidance.

Pure functions over a scan: synthesize angles when absent, filter by bearing
window, blind wedges and self-detection, and attribute rays to mapped obstacles
so the reactive layer can withhold them. No controller state.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.domain.models import SectorRanges

from src.config.tuning_helpers import get_tuning
from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.domain.enums import Section
    from shared.domain.models import Pose, Waypoint

_NO_RETURN_MARGIN_M = 0.05
"""How far below ``RobotSpecs.LIDAR_MAX_RANGE`` still counts as a fabricated
no-return substitute, not a genuine long reading.

``ros2_hardware_gateway._lidar_callback`` sanitizes NaN/inf rays (the Slamtec
driver's real "no signal" value) to ``LIDAR_MAX_RANGE`` BEFORE any sector code
ever sees the scan, so by the time a ray reaches this module the distinguishing
information is already gone -- a finite 12.0m reading is indistinguishable from
a real 12.0m echo. ``np.isfinite`` alone (this module's historical no-return
filter) is dead code against a sanitized hardware scan: it only ever excludes
literal ``inf``, which nothing downstream of the gateway still emits. Measured
on hardware 2026-08-28 (a 60cm-corridor run with an enlarged centre wall):
``compute_forward_clearance`` and ``assess_risk`` both read the whole forward
cone as ~12m ("wide open") at the exact moment the chassis was closest to a
wall -- every ray in the cone was grazing-incidence no-return, sanitized to the
fabricated far value, and nothing downstream could tell that apart from a
genuinely clear corridor. On this track's scale (a few metres), no real echo
should ever land within a few cm of the sensor's 12m spec ceiling, so treating
anything in that band as unmeasured is safe."""


def mask_mapped_obstacles(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    robot_pose: Pose,
    mapped_xy: Sequence[tuple[Waypoint, Section]],
    radius_m: float,
    cluster_xy: Sequence[Waypoint] | None = None,
    assoc_m: float = 0.0,
) -> np.ndarray:
    """Blank the LIDAR returns that land on an obstacle the planner already owns.

    The reactive layer in this module exists for what the planner does *not*
    know about: walls it is drifting into, and unmapped returns. A traffic sign
    the ``SignRouter`` is actively routing around is the opposite case -- the
    planner has a deliberate plan for it, and that plan is to pass it at
    ``lateral_offset`` centre-to-centre -- a gap narrower than ``contact_dist``
    once the sign's own half-width is subtracted. So with the raw scan the escape
    maneuver fires on every single sign pass and reverses the robot out of a gap
    the planner aimed for on purpose. Measured over the 16 obstacles fixtures,
    that decides the run before the router's aim can matter at all: every
    planning-side knob reads flat because the reactive layer overrides it (see
    ``docs/sign-avoidance-investigation.md``, "The escape layer is the gate").

    So the split is by *provenance*, not by distance: a return attributable to a
    mapped, actively-routed sign is withheld from the escape trigger, while walls
    and genuinely unknown returns keep the full guard. This is deliberately
    narrower than blanking the whole obstacle class from perception
    (``lidar_sees_obstacles=False``), which is a diagnostic only -- the C1
    really does see the signs, and an unmapped one must still stop the robot.

    Masked rays are set to ``inf`` rather than dropped, so the returned array
    stays index-aligned with ``lidar_angles``. ``inf`` is already this module's
    no-return sentinel: ``sector_ranges`` filters it via ``np.isfinite`` and
    ``_forward_path_ranges`` rejects it via its lateral-offset test.

    Args:
        lidar_ranges: Array of LIDAR range measurements.
        lidar_angles: Per-ray bearings (radians, 0 = forward). Synthesised from a
            full ``[-pi, pi)`` sweep when omitted, matching the rest of this module.
        robot_pose: Robot pose in world frame, needed to place each ray's endpoint
            on the map.
        mapped_xy: World positions of the mapped obstacles to withhold, each
            paired with its own corridor
            (``SignRouter.routed_sign_positions_by_corridor``). A ray is only
            attributed to a sign if its endpoint is within ``radius_m`` AND
            ``robot_pose`` itself is currently in that sign's corridor --
            proximity alone is not trustworthy under a believed pose that is a
            wrong-but-consistent rigid rotation of the truth (the blind-mode
            rotational-lock failure), which can reproject a genuinely unmapped
            obstacle's ray onto a routed sign's coordinates purely by coincidence.
            Gated on the ROBOT's own corridor rather than the ray endpoint's: a ray
            endpoint can jitter across a hard corridor boundary between ticks from
            ordinary LIDAR angle quantisation even when it is legitimately close to
            a sign just inside that boundary, which would make an endpoint-keyed
            gate flap; the robot itself is normally well inside a corridor, not
            standing on its 1.0/2.0 boundary, whenever anything is close enough to
            mask. See ``_SignTrack.corridor`` for the matching guard on the
            discovery side.
        radius_m: How close a ray endpoint must be to the anchor to count as
            that obstacle. Must cover the obstacle's own half-diagonal plus
            localisation and mapping error, but stay well under the distance to the
            nearest wall behind it -- too large and a wall standing behind a sign
            is silently masked along with it.
        cluster_xy: World positions of pillar-shaped LIDAR clusters this tick.
            When given (with ``assoc_m`` > 0) each mapped position is first
            SNAPPED to its nearest cluster, and the mask is anchored on the
            cluster instead of on the belief.
        assoc_m: How far a belief may sit from its cluster and still be the same
            pillar. 0 disables the snap and restores belief-anchored masking.

    WHY THE SNAP EXISTS. ``radius_m`` was doing two jobs with incompatible
    requirements: covering the MAP'S ERROR (how far the belief can be from the
    thing) and covering the OBSTACLE'S EXTENT (how big the thing is). The first
    wants a large radius, the second a small one, and on hardware there is no
    value that satisfies both -- measured 2026-09-11 on the two Obstacles rounds
    that wedged at the same point, the nearest LIDAR return sat p50 0.248 m and
    0.154 m from the believed sign position while the shipped radius was 0.12 m,
    so the mask caught 0/209 and 60/316 of the ticks it existed for. Raising it
    is not available either: a wall behind a sign can be 0.15 m away, and
    masking that removes a guard nothing else replaces.

    Snapping separates them. ASSOCIATION (belief to cluster) can be generous
    because a wrong association only costs the mask; EXTENT (cluster to ray) can
    stay tight because the cluster is measured, not believed. The cluster finder
    is the proposer's own ``find_clusters``, already in production inside the
    gated range fusion and measured at 91% recall -- a free-standing run of
    pillar width, bounded on both sides by a step, which a flat wall cannot
    satisfy.

    Returns:
        A copy of ``lidar_ranges`` with attributed rays set to ``inf``. The input
        is returned unchanged (as an array) when there is nothing to mask.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    if ranges.size == 0 or len(mapped_xy) == 0 or radius_m <= 0.0:
        return ranges

    if lidar_angles is None:
        angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
    else:
        angles = np.asarray(lidar_angles, dtype=float)

    robot_x, robot_y, robot_yaw = robot_pose.x, robot_pose.y, robot_pose.yaw
    robot_corridor = corridor_for_position(robot_x, robot_y)
    # Inside a corner square the chassis straddles two legs, and
    # corridor_for_position tie-breaks to the nearest inner FACE rather than to
    # the leg being driven -- a robot 4 cm past x=2.0 while running west along
    # the south straight reads as EAST. The corridor gate below then refuses to
    # mask the south sign it is in the middle of passing, the reactive layer
    # sees a routed sign as an unmapped frontal threat, and the escape fires
    # into a forward/reverse limit cycle that never clears it. Measured over
    # the 256-scenario corpus: every lap-0 stall sat within 0.35 m of a sign,
    # and in 85-94% of them (both stacks, sighted and blind) the robot's
    # corridor disagreed with that sign's.
    #
    # So the gate is only relaxed where the classification is genuinely
    # ambiguous. Outside a corner it still applies in full -- measured
    # identical to dropping it entirely, which is what says the gate never
    # discriminated anywhere else.
    in_corner = (
        not TrackDimensions.CORNER_MIN <= robot_x <= TrackDimensions.CORNER_MAX
    ) and (not TrackDimensions.CORNER_MIN <= robot_y <= TrackDimensions.CORNER_MAX)
    # Only finite returns have an endpoint to attribute; inf rays are already
    # no-returns and feeding them through cos/sin yields inf-inf = nan.
    finite = np.isfinite(ranges)
    bearings = angles + robot_yaw
    end_x = robot_x + ranges * np.cos(bearings)
    end_y = robot_y + ranges * np.sin(bearings)

    attributed = np.zeros(ranges.shape, dtype=bool)
    for mapped_wp, mapped_corridor in mapped_xy:
        if not in_corner and mapped_corridor != robot_corridor:
            continue
        anchor_x, anchor_y = mapped_wp.x, mapped_wp.y
        if cluster_xy and assoc_m > 0.0:
            # Nearest cluster wins, and only within assoc_m: an unassociated
            # belief falls back to itself rather than borrowing some other
            # pillar's position, which would mask the wrong thing entirely.
            best = min(
                ((math.hypot(c.x - anchor_x, c.y - anchor_y), c) for c in cluster_xy),
                key=lambda pair: pair[0],
            )
            if best[0] <= assoc_m:
                anchor_x, anchor_y = best[1].x, best[1].y
        attributed |= np.hypot(end_x - anchor_x, end_y - anchor_y) < radius_m

    masked = ranges.copy()
    masked[attributed & finite] = np.inf
    return masked



def robust_min_range(path: np.ndarray, window: int) -> float:
    """Closest range in ``path`` that ADJACENT rays corroborate.

    The bare minimum over a forward cone is an extreme-value statistic, not a
    clearance: the sweep carries Gaussian range noise across ~500 rays, so the
    smallest of the few dozen inside the lane routinely sits two to three sigma
    below the true nearest surface. Measured on the 256-scenario Obstacles
    corpus (Go, whose model matches), that phantom was worth 30 runs.

    A percentile over the whole cone would be the wrong shape -- a 0.05 m sign
    pillar subtends only about four rays at 1 m, and a percentile discards it
    as readily as it discards noise. What separates them is ADJACENCY: a real
    surface produces a run of short returns, uncorrelated noise produces
    isolated dips. This slides a window over the lane and takes the smallest
    window MEDIAN, so a reading must be corroborated by its neighbours to
    count, while an object spanning a window still registers at its true range.

    At 500 samples over 360 deg a 0.05 m pillar spans ~8 rays at 0.5 m and ~15
    at 0.25 m, where it first matters, so the shipped window of 5 sits well
    inside what a real obstacle produces.

    ``window <= 1``, or a path shorter than the window, is the bare minimum.
    """
    if path.size == 0:
        return math.inf
    if window <= 1 or path.size < window:
        return float(np.min(path))
    strided = np.lib.stride_tricks.sliding_window_view(path, window)
    return float(np.min(np.median(strided, axis=1)))

def _forward_path_ranges(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    path_half_width: float,
    min_valid_range_m: float,
    ahead_of_bumper: bool = False,
) -> np.ndarray:
    """Ranges of points ahead of the robot inside its driving lane.

    A point at bearing ``theta`` (0 = forward) and range ``r`` sits at lateral
    offset ``r*sin(theta)`` from the robot's centreline. Only points that are
    ahead (``cos(theta) > 0``) and within ``path_half_width`` of the centreline
    are in the robot's path -- the side walls of a corridor are excluded, so a
    robot driving straight down a narrow corridor is not perpetually flagged just
    because a wall is 0.2 m off its shoulder.

    A literal +inf no-return ray is excluded for free here (its lateral offset
    is also inf, failing the ``path_half_width`` bound), but the hardware
    gateway's fabricated ``LIDAR_MAX_RANGE`` substitute (see
    ``_NO_RETURN_MARGIN_M``) is finite and dead-ahead lands well inside the
    lane -- it needs its own exclusion or a forward cone that is ENTIRELY
    no-return (grazing incidence off something very close, not "genuinely
    clear") reports as the single largest, safest-looking range in the path.
    Measured on hardware 2026-08-28: this is what let ``assess_risk`` report
    SAFE at the exact moment the chassis was closest to a wall.
    """
    ranges, mask = _forward_path_selection(
        lidar_ranges, lidar_angles, path_half_width, min_valid_range_m, ahead_of_bumper
    )
    return np.asarray(ranges[mask])


def _forward_path_selection(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    path_half_width: float,
    min_valid_range_m: float,
    ahead_of_bumper: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """The forward-lane ranges and the boolean mask selecting them.

    Split out so ``_forward_path_ranges`` and ``forward_path_nearest_ray``
    cannot disagree about what "in the path" means. They are answers to the
    same question -- how close is the nearest thing ahead, and which ray said
    so -- and a second copy of this mask would be free to drift from the one
    the risk decision actually used, which is precisely the drift the ray
    identity exists to rule out.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    if ranges.size == 0:
        return ranges, np.zeros(0, dtype=bool)

    angles = _angles_for(ranges, lidar_angles)

    lateral = np.abs(ranges * np.sin(angles))
    # "Ahead" of WHAT. The sensor is the historical answer and is wrong by the
    # mount offset: it counts a band alongside the chassis as forward path.
    # See ClearanceZones.FORWARD_PATH_AHEAD_OF_BUMPER.
    along_track = ranges * np.cos(angles)
    ahead = along_track > (RobotSpecs.LIDAR_TO_FRONT_BUMPER if ahead_of_bumper else 0.0)
    mask = (
        ahead
        & (lateral < path_half_width)
        & (ranges > min_valid_range_m)
        & (ranges < RobotSpecs.LIDAR_MAX_RANGE - _NO_RETURN_MARGIN_M)
    )
    return ranges, mask


def _angles_for(
    ranges: np.ndarray,
    lidar_angles: np.ndarray | tuple[float, ...] | None,
) -> np.ndarray:
    """Per-ray bearings, synthesised as a full even sweep when not supplied."""
    if lidar_angles is None:
        return np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
    return np.asarray(lidar_angles, dtype=float)


def forward_path_nearest_ray(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    path_half_width: float,
    min_valid_range_m: float,
    ahead_of_bumper: bool = False,
) -> tuple[float, float] | None:
    """Bearing and range of the closest in-path ray, or None if the lane is empty.

    The identity of the ray ``assess_risk`` minimised over. The risk decision
    reduces the whole forward lane to one scalar gap, which cannot distinguish
    "a sign is dead ahead" from "the outer wall has come round into the lane on
    a late corner commit" -- and those want opposite fixes. The bearing does:
    a threat near 0 rad is in front of the robot, one out near the lane edge is
    something the robot is turning into.

    Returns the raw ``(angle_rad, range_m)`` as measured, NOT a bumper gap: the
    caller comparing this against a threshold should convert, and folding the
    12.2 cm sensor offset in here would make the range disagree with the
    ``angle`` it is paired with.
    """
    ranges, mask = _forward_path_selection(
        lidar_ranges, lidar_angles, path_half_width, min_valid_range_m, ahead_of_bumper
    )
    if not bool(np.any(mask)):
        return None
    angles = _angles_for(ranges, lidar_angles)
    idx = int(np.flatnonzero(mask)[int(np.argmin(ranges[mask]))])
    return float(angles[idx]), float(ranges[idx])


def _forward_path_has_rays(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    path_half_width: float,
) -> bool:
    """Whether any ray at all falls geometrically inside the forward lane.

    Ignores validity entirely (no ``min_valid_range_m``/no-return exclusion) --
    this answers "did the scan sweep this lane" not "is the lane clear",
    letting a caller (``assess_risk``) tell a genuinely-empty scan window
    (nothing to judge, safe by construction) apart from a lane that had rays
    but every one of them was a no-return (something is very likely very
    close -- see ``_forward_path_ranges``'s no-return exclusion), which must
    NOT read the same as the first case.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    if ranges.size == 0:
        return False

    angles = _angles_for(ranges, lidar_angles)

    lateral = np.abs(ranges * np.sin(angles))
    ahead = np.cos(angles) > 0.0
    return bool(np.any(ahead & (lateral < path_half_width)))


def chassis_exit_range_m(angles_rad: np.ndarray) -> np.ndarray:
    """Distance from the LIDAR to the CHASSIS BOUNDARY along each bearing.

    Nothing outside the robot can return closer than this, so a shorter reading
    at that bearing is the robot seeing itself -- a geometric fact, not a tuned
    threshold. Ray-vs-rectangle exit distance, with the sensor at the origin and
    the chassis offset by ``LIDAR_MOUNT_X_OFFSET`` (the LIDAR sits 0.1222 m
    FORWARD of centre, so the body is mostly behind it).

    Why a scalar threshold cannot do this job: over the rear +/-45 deg sector the
    boundary runs from 0.137 m at the sector edges to 0.272 m straight back, a
    factor of two. Measured on run_20260906_192424, the chassis showed up at
    0.125 m near -157 deg AND at 0.187 m near -172 deg -- either side of any
    single value, so one number either leaks the first or rejects real obstacles
    around the second.
    """
    half_length = RobotSpecs.LENGTH / 2.0
    offset = RobotSpecs.LIDAR_MOUNT_X_OFFSET
    x_forward = half_length - offset
    x_rear = -(half_length + offset)
    y_side = RobotSpecs.WIDTH / 2.0
    cos = np.cos(angles_rad)
    sin = np.sin(angles_rad)
    with np.errstate(divide="ignore", invalid="ignore"):
        # Slab exit distance per axis; a ray parallel to an axis never leaves
        # through it, hence the infinities.
        along = np.where(cos > 0.0, x_forward / cos, np.where(cos < 0.0, x_rear / cos, np.inf))
        across = np.where(sin > 0.0, y_side / sin, np.where(sin < 0.0, -y_side / sin, np.inf))
    return np.minimum(along, across)


def ranges_beyond_chassis(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    margin_m: float,
) -> np.ndarray:
    """The scan with every self-return replaced by ``inf`` -- no range floor.

    A consumer that must look at returns RIGHT NEXT TO the chassis still has to
    reject the chassis itself, and a scalar floor cannot do both: it is either
    above the returns of interest or below the robot's own. ``chassis_exit_range_m``
    separates them per bearing, which is where the distinction actually lives.

    Measured 2026-09-11 on the two Obstacles rounds, at the 105 ticks a contact
    recovery engaged -- share whose committed belief found a cluster within
    ``ESCAPE_MASK_CLUSTER_ASSOC_M``, and how many of the admitted clusters were
    the robot seeing itself:

    | floor | associated | self-returns | recovery STILL fires |
    |---|---|---|---|
    | none (control) | 1.0% | 0.0% | 79.0% |
    | 0.30 m (the proposer's) | 1.0% | 0.0% | 71.4% |
    | 0.15 m (the first shipped) | 31.4% | 4.6% | 81.9% |
    | 0.08 m | 87.6% | 5.5% | 13.3% |
    | **this, per bearing** | **86.7%** | **1.3%** | **11.4%** |

    It dominates every scalar on BOTH intermediate axes at once: it associates
    55 points more often than the 0.15 m floor it replaces while admitting fewer
    self-returns than that floor did. The outcome column is the one that matters
    -- the 0.15 m floor left the recovery firing on 81.9% of those ticks, no
    better than masking NOTHING, so the mask shipped earlier the same day was
    inert exactly where the rounds were being lost.

    Read the control row first. Every tick counted DID engage on the robot, so
    79.0% is the replay's own ceiling and the other rows are differences against
    it, not absolutes; the 21% shortfall is the replay seeing only the committed
    belief and default tuning rather than the deployed overlay.

    ``inf`` rather than a large finite range so a consumer filtering on
    ``isfinite`` drops these as "no measurement outside the body along this
    ray", which is what they are.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    angles = _angles_for(ranges, lidar_angles)
    return np.where(ranges >= chassis_exit_range_m(angles) + margin_m, ranges, np.inf)


def sector_ranges(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    center_rad: float,
    half_fov_rad: float,
    filter_self_detection: bool = False,
    self_detection_threshold_m: float | None = None,
    min_valid_range_m: float | None = None,
    blind_wedge_left_min_rad: float | None = None,
    blind_wedge_left_max_rad: float | None = None,
    blind_wedge_right_min_rad: float | None = None,
    blind_wedge_right_max_rad: float | None = None,
    apply_blind_wedge_mask: bool = True,
) -> np.ndarray:
    """Valid ranges whose bearing falls within ``center ± half_fov``.

    Bearings come from ``lidar_angles`` (0 rad = forward, +pi/2 = left, -pi/2 =
    right, +/-pi = rear). When angles are unavailable a full 360 deg scan indexed
    from ``angle_min = -pi`` is assumed, so every sector helper agrees on which
    way is forward regardless of the scan's index ordering.

    A staticmethod on purpose: called both as an instance method (which passes its
    own tuning-sourced thresholds explicitly) and directly as
    ``CollisionAvoidanceController.sector_ranges(...)`` by external,
    instance-less callers (e.g. telemetry_bridge_node.py's OLED summary), which
    fall back to these keyword defaults.

    Args:
        lidar_ranges: Array of LIDAR range measurements.
        lidar_angles: Per-ray bearings (radians), or None to synthesise a full
            ``[-pi, pi)`` sweep.
        center_rad: Centre bearing of the sector (radians).
        half_fov_rad: Half-width of the sector (radians).
        filter_self_detection: Also discard rays no farther than
            ``self_detection_threshold_m`` -- mount occlusion or cable clutter
            reflecting the chassis itself, not a real obstacle. Only pass this for
            side/rear sectors: never for the pure-forward bearing, where a genuine
            near-contact inside that radius must still register as a threat.
        self_detection_threshold_m: Threshold used when ``filter_self_detection``
            is set (m).
        min_valid_range_m: LIDAR ranges at or below this are treated as invalid
            (no-return) readings (m), used when ``filter_self_detection`` is not
            set.
        blind_wedge_left_min_rad: Start bearing of the left rear blind wedge
            (radians).
        blind_wedge_left_max_rad: End bearing of the left rear blind wedge
            (radians).
        blind_wedge_right_min_rad: Start bearing of the right rear blind wedge
            (radians).
        blind_wedge_right_max_rad: End bearing of the right rear blind wedge
            (radians). These cover the two rear-corner mount-occlusion wedges
            measured 2026-08-04, where self-collision reads as a real close range
            at every distance -- a distance threshold can't separate that from a
            genuine close obstacle at the same bearing, so this is filtered by
            angle instead. Always applied (not gated behind ``filter_self_detection``):
            the pure-forward bearing never overlaps these rear wedges, so there's
            no case where a real forward contact would be discarded by them.
        apply_blind_wedge_mask: Set False to skip the wedge exclusion -- used by
            ``_sector_to_model`` to tell "this bearing is a known blind spot"
            apart from "genuinely nothing out there" by re-running the same query
            with the mask lifted.
    """
    ranges = np.asarray(lidar_ranges, dtype=float)
    if ranges.size == 0:
        return ranges

    if self_detection_threshold_m is None or min_valid_range_m is None or blind_wedge_left_min_rad is None:
        tuning = get_tuning(None)
        if self_detection_threshold_m is None:
            self_detection_threshold_m = tuning.lidar_sectors.SELF_DETECTION_THRESHOLD_M
        if min_valid_range_m is None:
            min_valid_range_m = tuning.lidar_sectors.MIN_VALID_RANGE_M
        if blind_wedge_left_min_rad is None:
            blind_wedge_left_min_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_LEFT_MIN_DEG)
            blind_wedge_left_max_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_LEFT_MAX_DEG)
            blind_wedge_right_min_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MIN_DEG)
            blind_wedge_right_max_rad = math.radians(tuning.lidar_sectors.BLIND_WEDGE_RIGHT_MAX_DEG)

    if lidar_angles is None:
        angles = np.linspace(-math.pi, math.pi, ranges.size, endpoint=False)
    else:
        angles = np.asarray(lidar_angles, dtype=float)

    # Wrapped angular distance from the sector centre, in [-pi, pi].
    delta = np.arctan2(np.sin(angles - center_rad), np.cos(angles - center_rad))
    min_valid = self_detection_threshold_m if filter_self_detection else min_valid_range_m
    if (
        apply_blind_wedge_mask
        and blind_wedge_left_min_rad is not None
        and blind_wedge_left_max_rad is not None
        and blind_wedge_right_min_rad is not None
        and blind_wedge_right_max_rad is not None
    ):
        in_blind_wedge = ((angles >= blind_wedge_left_min_rad) & (angles <= blind_wedge_left_max_rad)) | (
            (angles >= blind_wedge_right_min_rad) & (angles <= blind_wedge_right_max_rad)
        )
    else:
        in_blind_wedge = np.zeros(angles.shape, dtype=bool)
    # Excludes no-return rays at both ends: literal +inf (never survives past the
    # hardware gateway, but still possible from mask_mapped_obstacles/tests) via
    # np.isfinite, and the gateway's fabricated LIDAR_MAX_RANGE substitute (see
    # _NO_RETURN_MARGIN_M) via the upper bound -- ranges > min_valid alone lets
    # both through (inf and 12.0 both exceed any finite lower threshold), and a
    # sector dominated by either turns its min into "wide open" for every caller,
    # both the OLED's displayed clearance and detect_threat_direction's real
    # collision-avoidance sectors.
    mask = (
        (np.abs(delta) <= half_fov_rad)
        & (ranges > min_valid)
        & (ranges < RobotSpecs.LIDAR_MAX_RANGE - _NO_RETURN_MARGIN_M)
        & np.isfinite(ranges)
        & ~in_blind_wedge
    )
    return np.asarray(ranges[mask])


def _sector_to_model(
    lidar_ranges: np.ndarray | tuple[float, ...],
    lidar_angles: np.ndarray | tuple[float, ...] | None,
    center_rad: float,
    half_fov_rad: float,
    filter_self_detection: bool = False,
    self_detection_threshold_m: float | None = None,
    min_valid_range_m: float | None = None,
    no_data_range_m: float | None = None,
    blind_wedge_left_min_rad: float | None = None,
    blind_wedge_left_max_rad: float | None = None,
    blind_wedge_right_min_rad: float | None = None,
    blind_wedge_right_max_rad: float | None = None,
) -> SectorRanges:
    """Compute aggregate metrics for an angular sector as a SectorRanges."""
    ranges = sector_ranges(
        lidar_ranges,
        lidar_angles,
        center_rad,
        half_fov_rad,
        filter_self_detection,
        self_detection_threshold_m,
        min_valid_range_m,
        blind_wedge_left_min_rad,
        blind_wedge_left_max_rad,
        blind_wedge_right_min_rad,
        blind_wedge_right_max_rad,
    )
    if no_data_range_m is None:
        no_data_range_m = get_tuning(None).lidar_sectors.NO_DATA_RANGE_M
    wedge_masked = False
    if ranges.size == 0:
        # Distinguish "this bearing is a known permanent blind spot" from
        # "nothing is out there right now": re-run the same sector query with the
        # wedge exclusion lifted -- if rays appear, every ray this sector could
        # see was inside a blind wedge, not genuinely absent. Both cases still
        # report no_data_range_m (a fully-masked sector is no more "definitely
        # clear" than a fully-empty one), but callers that care (e.g. telemetry)
        # can check wedge_masked.
        unmasked = sector_ranges(
            lidar_ranges,
            lidar_angles,
            center_rad,
            half_fov_rad,
            filter_self_detection,
            self_detection_threshold_m,
            min_valid_range_m,
            apply_blind_wedge_mask=False,
        )
        wedge_masked = unmasked.size > 0
    return SectorRanges(
        bearing_rad=center_rad,
        half_fov_rad=half_fov_rad,
        mean_range_m=float(np.mean(ranges)) if ranges.size > 0 else no_data_range_m,
        min_range_m=float(np.min(ranges)) if ranges.size > 0 else no_data_range_m,
        max_range_m=float(np.max(ranges)) if ranges.size > 0 else no_data_range_m,
        valid_count=int(ranges.size),
        wedge_masked=wedge_masked,
    )
