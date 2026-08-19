"""Unit tests for camera-only traffic-sign discovery.

``ObservedSignMap`` is what lets a blind robot route around signs at all: on
the mat there is no scenario file, so the sign layout has to come from the
camera. These pin the properties the router depends on — above all that
published indices are append-only, since ``SignRouter`` keys its
``_passed``/``_engaged`` bookkeeping by index and a reordered list would
silently retarget it onto a different sign.

Detections are built with the real ``vision_emulator``, which inverts the same
pinhole projection the map decodes, rather than with hand-written bboxes — a
hand-built box that happens not to round-trip would test the arithmetic instead
of the behaviour.
"""

from __future__ import annotations

import math
from dataclasses import replace

from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import SignColor, TrafficSignObservation, Waypoint

from src.navigation.planning.sign_discovery import (
    ObservedSignMap,
    SignSpec,
)
from src.simulation.vision_emulator import emulate_sign_observations

# These were module constants until tuning was threaded through; the values are
# now read per-call from NavigationTuning. Bound once here so the tests below
# keep reading as statements about the shipped configuration.
_SIGN_DISCOVERY = NavigationTuning.load_default().sign_discovery
_ASSOCIATION_DIST = _SIGN_DISCOVERY.ASSOCIATION_DIST_M
_MAX_INGEST_RANGE = _SIGN_DISCOVERY.MAX_INGEST_RANGE_M
_MIN_HITS = _SIGN_DISCOVERY.MIN_HITS

_CONFIDENCE = 0.25
"""Matches ``SignRouterConfig.min_confidence``, the threshold the map inherits."""


def _observe(sign_map: ObservedSignMap, signs: list[SignSpec], robot_pos, robot_yaw, times: int = 1) -> None:
    """Feed ``times`` identical frames of emulated observations into the map."""
    for _ in range(times):
        sign_map.observe(emulate_sign_observations(signs, robot_pos, robot_yaw), Waypoint(*robot_pos))


def _publish(sign_map: ObservedSignMap) -> list[SignSpec]:
    """Confirm pending tracks the way ``SignRouter._ingest_detections`` does."""
    published = []
    for track in sign_map.newly_confirmed():
        track.published_index = len(published)
        published.append(track.as_spec())
    return published


class TestConfirmation:
    """A track becomes a sign only once several frames agree."""

    def test_single_frame_is_not_yet_a_sign(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=1)

        assert sign_map.newly_confirmed() == []

    def test_confirmed_after_min_hits(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)

        assert len(sign_map.newly_confirmed()) == 1

    def test_confirmed_only_reported_once(self) -> None:
        """Publishing is the caller's job; a track must not be offered twice."""
        sign_map = ObservedSignMap(_CONFIDENCE)
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)
        _publish(sign_map)

        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)

        assert sign_map.newly_confirmed() == []


class TestRangeGate:
    """Pinhole range error grows with distance, so far observations are dropped."""

    def test_observation_beyond_max_range_is_ignored(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        far = _MAX_INGEST_RANGE + 0.5
        _observe(sign_map, [SignSpec(1.0, 1.0 - far, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS * 2)

        assert sign_map.newly_confirmed() == []

    def test_observation_inside_max_range_is_kept(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        near = _MAX_INGEST_RANGE - 0.5
        _observe(sign_map, [SignSpec(1.0, 1.0 - near, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)

        assert len(sign_map.newly_confirmed()) == 1


class TestAssociation:
    """Repeat sightings must merge; genuinely distinct signs must not."""

    def test_same_sign_seen_from_two_poses_is_one_track(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign = SignSpec(1.0, 0.4, "red")
        _observe(sign_map, [sign], (1.0, 1.2), -math.pi / 2, times=_MIN_HITS)
        _observe(sign_map, [sign], (1.0, 0.9), -math.pi / 2, times=_MIN_HITS)

        assert len(_publish(sign_map)) == 1

    def test_two_signs_further_apart_than_association_stay_separate(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        signs = [SignSpec(0.8, 0.4, "red"), SignSpec(0.8 + _ASSOCIATION_DIST * 2, 0.4, "green")]
        _observe(sign_map, signs, (1.4, 1.2), -math.pi / 2 - 0.4, times=_MIN_HITS)

        assert len(_publish(sign_map)) == 2

    def test_observations_from_different_robot_corridors_never_merge(self) -> None:
        """A believed pose that is a wrong-but-consistent rigid rotation of the
        truth (the blind-mode rotational-lock failure) can reproject two
        different corridors' signs to nearby, even identical, world XY.
        Distance-only association would wrongly fold them into one track;
        the corridor gate on ``_SignTrack.corridor`` must keep them apart
        whenever the robot itself was in a different corridor for each.
        """
        sign_map = ObservedSignMap(_CONFIDENCE)
        # Same reprojected world XY both times -- well inside association_dist
        # -- but the robot itself was standing in a different corridor
        # (SOUTH, then NORTH) when it made each observation.
        aliased = [TrafficSignObservation(1.5, 0.4, SignColor.RED, 1.0, 0.0)]
        for _ in range(_MIN_HITS):
            sign_map.observe(aliased, Waypoint(1.5, 0.5))
        for _ in range(_MIN_HITS):
            sign_map.observe(aliased, Waypoint(1.5, 2.5))

        assert len(_publish(sign_map)) == 2


class TestPositionEstimate:
    """The closest look wins, because pinhole error is monotone in range."""

    def test_closer_observation_overrides_a_more_distant_one(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign = SignSpec(1.0, 0.4, "red")

        _observe(sign_map, [sign], (1.0, 1.8), -math.pi / 2, times=_MIN_HITS)
        far_estimate = _publish(sign_map)[0]

        _observe(sign_map, [sign], (1.0, 0.7), -math.pi / 2, times=1)
        near_estimate = sign_map.published()[0].as_spec()

        far_error = math.dist((far_estimate.x, far_estimate.y), (sign.x, sign.y))
        near_error = math.dist((near_estimate.x, near_estimate.y), (sign.x, sign.y))
        assert near_error <= far_error

    def test_recovers_ground_truth_position(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign = SignSpec(1.0, 0.4, "red")
        _observe(sign_map, [sign], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)

        found = _publish(sign_map)[0]

        assert math.dist((found.x, found.y), (sign.x, sign.y)) < 0.02


class TestColorVote:
    """Colour decides the pass side, so one bad frame must not flip it."""

    def test_majority_color_wins(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign = SignSpec(1.0, 0.4, "red")
        pose, yaw = (1.0, 1.0), -math.pi / 2

        truthful = emulate_sign_observations([sign], pose, yaw)
        flipped = [replace(o, color=SignColor.GREEN) for o in truthful]

        for _ in range(4):
            sign_map.observe(truthful, Waypoint(*pose))
        sign_map.observe(flipped, Waypoint(*pose))

        assert _publish(sign_map)[0].color == "red"

    def test_below_confidence_detections_are_ignored(self) -> None:
        sign_map = ObservedSignMap(min_confidence=0.99)
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS * 2)

        assert sign_map.newly_confirmed() == []

    def test_below_confidence_observations_are_ignored(self) -> None:
        sign_map = ObservedSignMap(min_confidence=0.99)
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS * 2)

        assert sign_map.newly_confirmed() == []


class TestIndexStability:
    """The invariant SignRouter's index-keyed bookkeeping rests on."""

    def test_published_indices_are_append_only(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        first = SignSpec(1.0, 0.4, "red")
        second = SignSpec(2.0, 0.4, "green")

        _observe(sign_map, [first], (1.0, 1.0), -math.pi / 2, times=_MIN_HITS)
        published = _publish(sign_map)
        assert len(published) == 1

        # A second sign discovered later must not disturb the first one's index.
        _observe(sign_map, [second], (2.0, 1.0), -math.pi / 2, times=_MIN_HITS)
        for track in sign_map.newly_confirmed():
            track.published_index = len(published)
            published.append(track.as_spec())

        indices = sorted(t.published_index for t in sign_map.published())
        assert indices == [0, 1]
        assert math.dist((published[0].x, published[0].y), (first.x, first.y)) < 0.05
