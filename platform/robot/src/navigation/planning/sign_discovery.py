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

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.domain.models import Pose, SignColor, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.utils import _dist2d, _nearest_ray

if TYPE_CHECKING:
    from collections.abc import Sequence

    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.models import Detection

logger = logging.getLogger(__name__)

# Camera focal length in pixels — derived from HFOV and image width.
_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)


@dataclass(frozen=True)
class SignSpec:
    """Expected traffic sign location and color from scenario metadata."""

    x: float
    y: float
    color: str  # SignColor.RED or SignColor.GREEN


def detection_to_observation(
    det: Detection,
    robot_pose: Pose,
    tuning: NavigationTuning | None = None,
    lidar_ranges_m: Sequence[float] | None = None,
    lidar_angles_rad: Sequence[float] | None = None,
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
    world = _detection_to_world(
        det, (robot_pose.x, robot_pose.y), robot_pose.yaw, tuning, lidar_ranges_m, lidar_angles_rad,
    )
    if world is None:
        return None
    x1, y1, x2, y2 = det.bbox
    return TrafficSignObservation(
        world_x_m=world[0],
        world_y_m=world[1],
        color=det.class_name,
        confidence=det.confidence,
        detected_at_timestamp=0.0,
        bbox_xmin=int(x1),
        bbox_ymin=int(y1),
        bbox_xmax=int(x2),
        bbox_ymax=int(y2),
    )


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
    x1, y1, x2, y2 = det.bbox
    pixel_height = abs(y2 - y1)
    if pixel_height < tuning.sign_discovery.MIN_RELIABLE_BBOX_HEIGHT_PX:
        return None

    # Estimate distance using pinhole model: d = (f * real_h) / pixel_h
    distance = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT) / pixel_height

    # Horizontal angle from image centre.
    cx = (x1 + x2) / 2.0
    theta_h = (cx / RobotSpecs.CAMERA_WIDTH - 0.5) * RobotSpecs.CAMERA_HFOV

    if lidar_ranges_m and lidar_angles_rad:
        lidar_range = _nearest_ray(lidar_ranges_m, lidar_angles_rad, theta_h)
        if tuning.lidar_sectors.MIN_VALID_RANGE_M < lidar_range < RobotSpecs.CAMERA_FAR_CLIP:
            distance = lidar_range

    bearing = robot_yaw + theta_h
    wx = robot_pos[0] + distance * math.cos(bearing)
    wy = robot_pos[1] + distance * math.sin(bearing)
    return wx, wy



@dataclass(slots=True)
class _SignTrack:
    """One candidate sign, accumulated across frames."""

    x: float
    y: float
    best_range: float
    """Range of the closest observation so far — the one (x, y) is taken from."""

    hits: int = 0
    votes: dict[str, float] = field(default_factory=dict)
    """Confidence-weighted colour votes. A sign's colour decides which side the
    robot must pass it, so a single mislabelled frame must not be able to flip
    it; summing confidence across frames makes that need sustained disagreement."""

    published_index: int | None = None
    """Index in the router's sign list once published, or None while pending."""

    @property
    def color(self) -> str:
        return max(self.votes, key=lambda name: self.votes[name])

    def as_spec(self) -> SignSpec:
        return SignSpec(x=self.x, y=self.y, color=self.color)


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
            tuning: Navigation tuning instance. Defaults to the default tuning profile.
        """
        tuning = get_tuning(tuning)
        sd = tuning.sign_discovery
        self._min_confidence = min_confidence
        self._max_ingest_range_m = max_ingest_range_m if max_ingest_range_m is not None else sd.MAX_INGEST_RANGE_M
        self._association_dist_m = association_dist_m if association_dist_m is not None else sd.ASSOCIATION_DIST_M
        self._min_hits = min_hits if min_hits is not None else sd.MIN_HITS
        self._tracks: list[_SignTrack] = []

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

        for obs in observations:
            if obs.confidence < self._min_confidence:
                continue
            if obs.color not in (SignColor.RED, SignColor.GREEN):
                continue

            world = Waypoint(obs.world_x_m, obs.world_y_m)
            observed_range = _dist2d(world, robot_pos)
            if observed_range > self._max_ingest_range_m:
                continue

            self._fold(world, observed_range, obs)

    def _fold(self, world: Waypoint, observed_range: float, obs: TrafficSignObservation) -> None:
        """Merge one projected observation into the nearest track, or start one."""
        track = self._nearest_track(world)
        if track is None:
            track = _SignTrack(x=world.x, y=world.y, best_range=observed_range)
            self._tracks.append(track)

        track.hits += 1
        track.votes[obs.color.value] = track.votes.get(obs.color.value, 0.0) + obs.confidence

        # Closest observation wins outright: pinhole range error is monotone in
        # range, so a nearer reading is strictly better evidence than the
        # running estimate, and averaging would only let worse readings back in.
        if observed_range <= track.best_range:
            track.best_range = observed_range
            track.x, track.y = world.x, world.y

    def _nearest_track(self, world: Waypoint) -> _SignTrack | None:
        """The closest existing track within ``self._association_dist_m``, if any."""
        best: _SignTrack | None = None
        best_dist = self._association_dist_m
        for track in self._tracks:
            d = _dist2d(Waypoint(track.x, track.y), world)
            if d < best_dist:
                best_dist = d
                best = track
        return best

    def newly_confirmed(self) -> list[_SignTrack]:
        """Tracks that have crossed ``self._min_hits`` and are not yet published.

        The caller is responsible for assigning ``published_index`` once it has
        appended the returned specs to its own sign list.
        """
        return [t for t in self._tracks if t.published_index is None and t.hits >= self._min_hits]

    def published(self) -> list[_SignTrack]:
        """Tracks already handed to the router, for in-place position refinement."""
        return [t for t in self._tracks if t.published_index is not None]
