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

import pytest
from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, Pose, SignColor, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.planning.sign_discovery import (
    ObservedSignMap,
    SignSpec,
    _detection_to_world,
    detection_to_observation,
)
from src.ros2.navigation.ros2_hardware_gateway import pose_at_time
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
        sign_map.observe(emulate_sign_observations(signs, Waypoint(*robot_pos), robot_yaw), Waypoint(*robot_pos))


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
        # Both unrelated gates are PINNED, because this test is about the
        # corridor gate alone: `robot_corridor_flip_ticks=1` disables the
        # debounce that settles it, and `max_ingest_range_m=2.0` keeps the
        # range gate out of the way. The range pin is load-bearing -- standing
        # in NORTH (y > 2.0) is necessarily >= 1.6 m from a sign at y = 0.4, so
        # under the shipped 1.5 m gate the second vantage is unreachable and
        # this assertion would pass for the wrong reason (only phase 1's track
        # ever existing), which is the very trap the comment below records.
        sign_map = ObservedSignMap(_CONFIDENCE, robot_corridor_flip_ticks=1, max_ingest_range_m=2.0)
        # Same reprojected world XY both times -- well inside association_dist
        # -- but the robot itself was standing in a different corridor
        # (SOUTH, then NORTH) when it made each observation. NORTH position is
        # 1.7 m from the sign, not 2.1 m (e.g. y=2.5) -- the latter silently
        # drops every phase-2 observation at the pinned 2.0 m ingest gate
        # before association ever runs, which let this assertion pass for the
        # wrong reason (only phase 1's track ever existed) until traced here.
        aliased = [TrafficSignObservation(1.5, 0.4, SignColor.RED, 1.0, 0.0)]
        for _ in range(_MIN_HITS):
            sign_map.observe(aliased, Waypoint(1.5, 0.5))
        for _ in range(_MIN_HITS):
            sign_map.observe(aliased, Waypoint(1.5, 2.1))

        assert len(_publish(sign_map)) == 2

    def test_observations_from_the_same_robot_corridor_still_merge(self) -> None:
        """The corridor gate must not be so strict that revisiting the same
        sign from a slightly different but still-same-corridor vantage (a
        different lap, a wider pass) starts a second track -- only a
        genuinely different robot corridor should.
        """
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign = [TrafficSignObservation(1.5, 0.4, SignColor.RED, 1.0, 0.0)]
        for _ in range(_MIN_HITS):
            sign_map.observe(sign, Waypoint(1.5, 0.5))
        for _ in range(_MIN_HITS):
            sign_map.observe(sign, Waypoint(1.6, 0.6))  # ~0.14 m away, well under the gate

        assert len(_publish(sign_map)) == 1


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

        truthful = emulate_sign_observations([sign], Waypoint(*pose), yaw)
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


class TestPillarAspectGate:
    """Boxes wider than tall are scenery, not pillars.

    The magenta parking-lot barrier reads as RED under motion blur: on
    ``run_20260905_214920`` 199 of 383 red detections were wider than tall,
    against 0.6% of greens, and every accepted box can seed a sign
    (``active_sign_count`` climbed 5 -> 50 on an 8-sign track). Neither colour
    nor confidence separates them -- those boxes carry the RED label at p50
    confidence 0.79 -- so the shape gate is the only filter the measurement
    supports.

    These are hand-built bboxes rather than ``vision_emulator`` ones, unlike
    the rest of this module, because the emulator has no bbox at all: it builds
    ``TrafficSignObservation`` straight from ground truth, which is exactly why
    the simulator cannot exercise this gate and the corpus reports it as a
    no-op.
    """

    @staticmethod
    def _detection(width_px: float, height_px: float, colour: SignColor = SignColor.RED) -> Detection:
        """A detection of the given box shape, centred where a sign would be."""
        centre_x, centre_y = 700.0, 400.0
        return Detection(
            class_name=colour,
            confidence=0.79,
            bbox=(
                centre_x - width_px / 2,
                centre_y - height_px / 2,
                centre_x + width_px / 2,
                centre_y + height_px / 2,
            ),
            x=centre_x,
            y=centre_y,
            width=width_px,
            height=height_px,
            area=width_px * height_px,
        )

    def test_pillar_shaped_box_is_accepted(self) -> None:
        # w/h 0.71 -- the median shape of a real green pillar in that run.
        det = self._detection(100.0, 140.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is not None

    def test_barrier_shaped_box_is_rejected(self) -> None:
        # w/h 1.76 -- the median shape of the magenta parking barrier.
        det = self._detection(176.0, 100.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is None

    def test_marginally_wide_box_is_rejected(self) -> None:
        # w/h 1.01 is the MEDIAN red detection in that run, i.e. the gate has to
        # bite just above square or it keeps half the bad boxes.
        det = self._detection(101.0, 100.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is None

    def test_boundary_is_inclusive(self) -> None:
        det = self._detection(100.0, 100.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is not None

    @staticmethod
    def _clipped_detection(width_px: float, height_px: float) -> Detection:
        """A wide box running off the RIGHT edge of the frame, as a near pillar does."""
        x_max = float(RobotSpecs.CAMERA_WIDTH)
        y_centre = 500.0
        return Detection(
            class_name=SignColor.RED,
            confidence=0.83,
            bbox=(x_max - width_px, y_centre - height_px / 2, x_max, y_centre + height_px / 2),
            x=x_max - width_px / 2,
            y=y_centre,
            width=width_px,
            height=height_px,
            area=width_px * height_px,
        )

    def test_a_clipped_wide_box_is_kept(self) -> None:
        """A clipped box's aspect ratio is not a measurement of its shape.

        run_20260906_145546, 23.4-24.6 s: a red pillar with x_max pinned at the
        frame edge every frame, w/h climbing 0.33 -> 1.27 as the robot closed on
        it, rejected exactly when it was nearest. 61% of the red detections this
        gate rejected across two runs were frame-clipped.
        """
        det = self._clipped_detection(428.0, 337.0)  # w/h 1.27, the measured shape
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is not None

    def test_an_unclipped_wide_box_is_still_rejected(self) -> None:
        """The gate still does its original job on boxes fully inside the frame."""
        det = self._detection(428.0, 337.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0)) is None

    def test_zero_disables_the_gate(self) -> None:
        tuning = NavigationTuning.load_default()
        disabled = replace(
            tuning,
            sign_discovery=tuning.sign_discovery.model_copy(update={"MAX_PILLAR_ASPECT": 0.0}),
        )
        det = self._detection(176.0, 100.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0), tuning=disabled) is not None

    def test_a_wide_box_is_kept_where_no_barrier_can_be(self) -> None:
        """There is one parking lot, in the corridor the robot started in.

        A wide RED box seen from any other corridor cannot be the barrier, so
        rejecting it only throws away a pillar. Measured across
        run_20260906_145546 and _145909: wall-shaped reds carry no corridor
        label 72% of the time -- the start, around the bay, exactly where the
        magenta barrier detections sit -- while pillar-shaped reds spread
        across the driving corridors.
        """
        det = self._detection(176.0, 100.0)  # w/h 1.76, the barrier's shape
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0), barrier_possible=False) is not None

    def test_the_same_wide_box_is_rejected_where_the_barrier_lives(self) -> None:
        """Unchanged where it matters: in the lot's own corridor the gate bites."""
        det = self._detection(176.0, 100.0)
        assert detection_to_observation(det, Pose(1.5, 0.5, 0.0), barrier_possible=True) is None


def _box_at_column(cx: float, pixel_height: float = 40.0) -> Detection:
    """A detection whose box sits at image column ``cx``, nothing else varying."""
    cy = RobotSpecs.CAMERA_HEIGHT / 2
    half = pixel_height / 2
    return Detection(
        class_name="red",
        confidence=0.9,
        bbox=(cx - half, cy - half, cx + half, cy + half),
        x=cx,
        y=cy,
        width=pixel_height,
        height=pixel_height,
        area=pixel_height * pixel_height,
    )


def test_a_box_left_of_centre_is_a_sign_on_the_robots_left() -> None:
    """The camera's bearing sign, stated from GEOMETRY rather than the formula.

    Every other test of this path builds its bounding box by inverting
    ``_detection_to_world``, so it agrees with that function whatever sign it
    uses. ``vision_emulator`` reproduces the true geometry directly and never
    touches a bbox at all. That is how a MIRRORED bearing survived in-tree and
    reached the track: until 2026-09-06 ``theta_h`` was positive for a box on
    the RIGHT of the image, which is the robot's right and therefore a NEGATIVE
    CCW bearing, so every sign was reflected across the robot's heading axis and
    landed on the far wall of a 1 m corridor.

    So this test asserts the thing no other one can: a camera delivers an
    UPRIGHT image (``camera_inverted`` is folded into the driver's flips at
    capture), so an object to the robot's LEFT appears LEFT of centre -- and the
    robot frame is CCW-positive with left positive (``LidarScan``: 0 = forward,
    +pi/2 = left). Confirmed on run_20260906_192424 by predicting the box's
    centre column from the true bearing to a LIDAR-located pillar: 174 px of
    error upright against 528 px mirrored, at three headings spanning 165 deg.
    """
    robot_pos = (0.0, 0.0)
    robot_yaw = 0.0  # facing +x, so +y is the robot's left
    left_of_frame = _detection_to_world(_box_at_column(RobotSpecs.CAMERA_WIDTH * 0.25), robot_pos, robot_yaw)
    right_of_frame = _detection_to_world(_box_at_column(RobotSpecs.CAMERA_WIDTH * 0.75), robot_pos, robot_yaw)
    assert left_of_frame is not None
    assert right_of_frame is not None
    assert left_of_frame[1] > 0.0, "a box left of centre must place the sign to the robot's LEFT"
    assert right_of_frame[1] < 0.0, "a box right of centre must place the sign to the robot's RIGHT"


def test_the_pinhole_range_carries_the_measured_scale() -> None:
    """RANGE_SCALE multiplies the pinhole result, and 1.0 is the raw model.

    The pinhole UNDER-reads by about half on hardware -- the detector's boxes are
    1.75x taller than a 0.10 m pillar subtends -- so the scale is not cosmetic.
    Asserted as a RATIO between two tunings rather than against a fixed distance,
    so re-fitting the constant does not break the test that guards it.
    """
    detection = _box_at_column(RobotSpecs.CAMERA_WIDTH / 2)
    raw = _detection_to_world(
        detection, (0.0, 0.0), 0.0, tuning=tuning_with_overrides({"RANGE_SCALE": 1.0}, group="sign_discovery")
    )
    scaled = _detection_to_world(
        detection, (0.0, 0.0), 0.0, tuning=tuning_with_overrides({"RANGE_SCALE": 2.0}, group="sign_discovery")
    )
    assert raw is not None
    assert scaled is not None
    sensor_x = RobotSpecs.LIDAR_MOUNT_X_OFFSET
    assert scaled[0] - sensor_x == pytest.approx(2.0 * (raw[0] - sensor_x), rel=1e-6)


class TestCameraTimeAlignment:
    """The pose a detection is decoded against must be the pose it was SEEN from.

    `/vision/detections` is a `std_msgs/String` with no header, so until
    2026-09-07 every detection was paired with the pose at RECEIPT. Measured on
    run_20260906_232408/_232748 the camera pipeline runs **0.85 s** behind, and
    at 0.3 m/s through a corner that is most of a sign's lateral offset -- it
    was the entire bearing residual left after the mirror fix (20.2 deg -> 5.4
    deg once corrected). The check that was not fitted to the lag: the recovered
    `cx`-vs-bearing slope reads -309 px/rad at zero lag, which no real lens can
    produce, and -679 at 0.85 s.
    """

    @staticmethod
    def _history() -> list[tuple[float, Pose]]:
        return [(float(i) / 10.0, Pose(x=float(i), y=0.0, yaw=0.0)) for i in range(20)]

    def test_the_pose_is_taken_from_when_the_frame_was_captured(self) -> None:
        history = self._history()
        chosen = pose_at_time(history, target_s=1.0, fallback=Pose(x=99.0, y=0.0, yaw=0.0))
        assert chosen.x == pytest.approx(10.0)

    def test_a_stamp_older_than_the_buffer_falls_back_rather_than_extrapolating(self) -> None:
        """Only true in the opening second of a round, and inventing a pose is worse."""
        fallback = Pose(x=99.0, y=0.0, yaw=0.0)
        assert pose_at_time(self._history(), target_s=-5.0, fallback=fallback) is fallback
        assert pose_at_time([], target_s=1.0, fallback=fallback) is fallback

    def test_the_lag_actually_changes_which_pose_is_used(self) -> None:
        """Guards the whole point: receipt and capture must not resolve alike."""
        history = self._history()
        at_receipt = pose_at_time(history, target_s=1.9, fallback=Pose(x=99.0, y=0.0, yaw=0.0))
        at_capture = pose_at_time(history, target_s=1.9 - 0.85, fallback=Pose(x=99.0, y=0.0, yaw=0.0))
        assert at_receipt.x != at_capture.x
        assert at_receipt.x - at_capture.x == pytest.approx(8.0, abs=1.0)


class TestLidarProposals:
    """A LIDAR proposal is a POSITION with no colour: it refines, it never routes.

    The split these assert is the whole point of the proposer -- the LIDAR sees
    an object ~0.6 m before the camera can classify it, so geometry can be
    settled early while the pass side still waits for a colour it cannot invent.
    """

    def test_proposal_alone_is_never_published(self) -> None:
        """Even far past min_hits: a colourless sign has no pass side."""
        sign_map = ObservedSignMap(_CONFIDENCE)
        for _ in range(_MIN_HITS * 3):
            sign_map.propose([(1.0, 0.4)], Waypoint(1.0, 1.0))

        assert sign_map.newly_confirmed() == []

    def test_proposal_reports_unknown_until_the_camera_votes(self) -> None:
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign_map.propose([(1.0, 0.4)], Waypoint(1.0, 1.0))

        assert sign_map._tracks[0].color is SignColor.UNKNOWN
        assert sign_map._tracks[0].as_spec().color is SignColor.UNKNOWN

    def test_one_camera_frame_publishes_a_proposed_track(self) -> None:
        """POSITION EARLY, COLOUR LATE -- the proposal has already done the hits."""
        sign_map = ObservedSignMap(_CONFIDENCE)
        for _ in range(_MIN_HITS):
            sign_map.propose([(1.0, 0.4)], Waypoint(1.0, 1.0))
        _observe(sign_map, [SignSpec(1.0, 0.4, "red")], (1.0, 1.0), -math.pi / 2, times=1)

        published = _publish(sign_map)
        assert [s.color for s in published] == [SignColor.RED]

    def test_camera_votes_colour_but_cannot_move_a_lidar_fixed_position(self) -> None:
        """The camera's range is a pinhole estimate; the LIDAR measured it.

        Guards the hardware defect where monocular range over-read put believed
        pillars on the walls -- a nearer camera look must not reintroduce it.
        """
        sign_map = ObservedSignMap(_CONFIDENCE)
        sign_map.propose([(1.0, 0.4)], Waypoint(1.0, 1.0))
        # A camera observation offset well inside the association distance, so
        # it folds into the same track rather than starting a new one.
        sign_map.observe(
            [
                TrafficSignObservation(
                    world_x_m=1.0 + _ASSOCIATION_DIST / 2,
                    world_y_m=0.4,
                    color=SignColor.RED,
                    confidence=0.9,
                    detected_at_timestamp=0.0,
                )
            ],
            Waypoint(1.0, 0.6),
        )

        track = sign_map._tracks[0]
        assert track.color is SignColor.RED, "the camera still gets to say WHAT it is"
        assert (track.x, track.y) == (1.0, 0.4), "but not WHERE it is"
