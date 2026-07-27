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

from shared.config.constants import ColorNames, RobotSpecs, TrafficSignSpecs

if TYPE_CHECKING:
    from shared.domain.models import Detection

logger = logging.getLogger(__name__)

# Camera focal length in pixels — derived from HFOV and image width.
_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)

# Bounding boxes shorter than this (px) are too degenerate for a reliable
# pinhole distance estimate.
_MIN_RELIABLE_BBOX_HEIGHT_PX: int = 5


@dataclass(frozen=True)
class SignSpec:
    """Expected traffic sign location and color from scenario metadata."""

    x: float
    y: float
    color: str  # ColorNames.RED or ColorNames.GREEN


def _dist2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _detection_to_world(
    det: Detection,
    robot_pos: tuple[float, float],
    robot_yaw: float,
) -> tuple[float, float] | None:
    """Project a bbox detection to an approximate world position.

    Uses known sign height (TrafficSignSpecs.HEIGHT) as the reference to
    estimate distance from the pixel-space bounding-box height.

    Args:
        det: Single camera detection with bbox (x1, y1, x2, y2).
        robot_pos: Robot (x, y) position (metres).
        robot_yaw: Robot heading (radians, 0 = east).

    Returns:
        Estimated world (x, y) of the sign, or None if bbox is too small.
    """
    x1, y1, x2, y2 = det.bbox
    pixel_height = abs(y2 - y1)
    if pixel_height < _MIN_RELIABLE_BBOX_HEIGHT_PX:
        return None

    # Estimate distance using pinhole model: d = (f * real_h) / pixel_h
    distance = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT) / pixel_height

    # Horizontal angle from image centre.
    cx = (x1 + x2) / 2.0
    theta_h = (cx / RobotSpecs.CAMERA_WIDTH - 0.5) * RobotSpecs.CAMERA_HFOV

    bearing = robot_yaw + theta_h
    wx = robot_pos[0] + distance * math.cos(bearing)
    wy = robot_pos[1] + distance * math.sin(bearing)
    return wx, wy

_MAX_INGEST_RANGE: float = 2.0
"""Ignore observations further than this (m) — see the module docstring.

Well clear of the router's 0.80 m ``activation_dist``, so a sign is discovered
with over a metre of runway left to steer around it.
"""

_ASSOCIATION_DIST: float = 0.25
"""Two observations within this distance (m) are the same sign.

Bounded above by the WRO sign grid's own spacing: signs sit 0.50 m apart along
a corridor and the two lanes are ~0.20-0.40 m apart, so anything much larger
would merge a red and a green sign into one track and average them into the
gap between the lanes.
"""

_MIN_HITS: int = 3
"""Observations before a track is published as a real sign.

A single frame is enough for a false positive; requiring agreement across
frames costs ~0.15 s at the 20 Hz control loop, which is nothing against the
runway :data:`_MAX_INGEST_RANGE` buys.
"""


@dataclass
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

    def __init__(self, min_confidence: float) -> None:
        """Start an empty map.

        Args:
            min_confidence: Detections below this confidence are ignored, so the
                map inherits the router's own detection threshold rather than
                introducing a second, independently-tuned one.
        """
        self._min_confidence = min_confidence
        self._tracks: list[_SignTrack] = []

    def observe(
        self,
        detections: list[Detection] | None,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> None:
        """Fold one frame of detections into the map."""
        if not detections:
            return

        for det in detections:
            if det.confidence < self._min_confidence:
                continue
            if det.class_name not in (ColorNames.RED, ColorNames.GREEN):
                continue

            world = _detection_to_world(det, robot_pos, robot_yaw)
            if world is None:
                continue

            observed_range = _dist2d(world, robot_pos)
            if observed_range > _MAX_INGEST_RANGE:
                continue

            self._fold(world, observed_range, det)

    def _fold(self, world: tuple[float, float], observed_range: float, det: Detection) -> None:
        """Merge one projected observation into the nearest track, or start one."""
        track = self._nearest_track(world)
        if track is None:
            track = _SignTrack(x=world[0], y=world[1], best_range=observed_range)
            self._tracks.append(track)

        track.hits += 1
        track.votes[det.class_name] = track.votes.get(det.class_name, 0.0) + det.confidence

        # Closest observation wins outright: pinhole range error is monotone in
        # range, so a nearer reading is strictly better evidence than the
        # running estimate, and averaging would only let worse readings back in.
        if observed_range <= track.best_range:
            track.best_range = observed_range
            track.x, track.y = world

    def _nearest_track(self, world: tuple[float, float]) -> _SignTrack | None:
        """The closest existing track within :data:`_ASSOCIATION_DIST`, if any."""
        best: _SignTrack | None = None
        best_dist = _ASSOCIATION_DIST
        for track in self._tracks:
            d = _dist2d((track.x, track.y), world)
            if d < best_dist:
                best_dist = d
                best = track
        return best

    def newly_confirmed(self) -> list[_SignTrack]:
        """Tracks that have crossed :data:`_MIN_HITS` and are not yet published.

        The caller is responsible for assigning ``published_index`` once it has
        appended the returned specs to its own sign list.
        """
        return [t for t in self._tracks if t.published_index is None and t.hits >= _MIN_HITS]

    def published(self) -> list[_SignTrack]:
        """Tracks already handed to the router, for in-place position refinement."""
        return [t for t in self._tracks if t.published_index is not None]
