"""Unit tests for CollisionAvoidanceController LIDAR sectoring.

Regression coverage for the angle-aware sector logic: a wall *behind* the
robot must report as "back" (never "front"), so the escape logic never
reverses into an unseen wall. Sectors are derived from per-ray bearings
(0 rad = forward, +-pi = rear) and must agree across all helpers regardless
of the scan's index ordering.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import RiskLevel, Section, ThreatDirection
from shared.domain.models import LidarClearances, Pose, Waypoint

from src.navigation.clearances import (
    ClearanceAggregate,
    clearances_from_scan,
    threat_direction,
)
from src.navigation.control.controllers.collision_avoidance import (
    CollisionAvoidanceController,
    mask_mapped_obstacles,
)
from src.navigation.ports import LidarScan
from tests.fixtures import angle_to_index, create_numpy_scan
from tests.test_constants import (
    ANGLES_FULL_ROTATION,
    FORWARD_SECTOR_INDICES,
    LIDAR_CLOSE_THREAT,
    LIDAR_DEFAULT_FAR,
    MIN_FORWARD_CLEARANCE,
    MIN_REAR_CLEARANCE,
    NUM_RAYS,
    REAR_SECTOR_INDICES,
    YAW_EAST,
)


@pytest.fixture()
def controller() -> CollisionAvoidanceController:
    return CollisionAvoidanceController.from_tuning(NavigationTuning.load_default())


class TestThreatDirection:
    def test_wall_behind_reports_back_not_front(self, controller):
        ranges = create_numpy_scan()
        # Close returns near +-pi (rear cone), including index 0 (= -pi).
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        # The current chassis' rear mount sits in the LIDAR's blind wedge, so
        # rear rays are masked and a wall directly behind is not seen -- the
        # threat direction reports none rather than "back".
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "none"

    def test_wall_behind_back_even_without_angles(self, controller):
        # When angles are omitted, index 0 is assumed to be -pi, so a close
        # ray at index 0 is still the rear, not the front.
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        # Rear is masked by the chassis blind wedge (see
        # test_wall_behind_reports_back_not_front); nothing is reported.
        assert controller.detect_threat_direction(ranges) == "none"

    def test_wall_ahead_reports_front(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "front"

    def test_obstacle_left_reports_left(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)  # +pi/2 = left
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "left"

    def test_all_clear_reports_none(self, controller):
        assert controller.detect_threat_direction(create_numpy_scan(), ANGLES_FULL_ROTATION) == "none"


class TestClearance:
    def test_forward_clearance_ignores_rear_wall(self, controller):
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        # Path ahead is clear even though a wall sits behind.
        assert controller.compute_forward_clearance(ranges, ANGLES_FULL_ROTATION) > MIN_FORWARD_CLEARANCE

    def test_rear_clearance_sees_rear_wall(self, controller):
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        # The rear mount sits in the LIDAR's blind wedge on the current
        # chassis, so rear rays are masked and rear clearance reads as the
        # no-data fallback rather than the actual wall distance.
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(controller.no_data_range_m)

    def test_rear_clearance_clear_when_only_front_blocked(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) > MIN_REAR_CLEARANCE


class TestRearSectorVisibility:
    """A reverse gate must be able to tell "nothing behind" from "cannot see"."""

    def test_normal_scan_is_measured(self, controller):
        # On the current chassis the rear mount occupies the LIDAR's blind
        # wedge, so even a normal scan's rear sector is masked and reports as
        # not measured -- "cannot see" rather than "nothing behind".
        assert controller.rear_sector(create_numpy_scan(), ANGLES_FULL_ROTATION).measured is False

    def test_no_valid_rear_rays_is_not_measured(self, controller):
        # Every ray in the rear half is a no-return -- the case a chassis whose
        # mount occludes the last ~25 deg slot would see on every scan.
        ranges = create_numpy_scan()
        ranges[np.abs(ANGLES_FULL_ROTATION) >= math.pi / 2] = 0.0

        # The clearance number cannot express this: it reads as wide-open road,
        # which is what made the reverse guard fail open.
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(controller.no_data_range_m)
        assert controller.rear_sector(ranges, ANGLES_FULL_ROTATION).measured is False

    def test_empty_scan_is_not_measured(self, controller):
        assert controller.rear_sector(np.array([]), None).measured is False


class TestEscapeDoesNotReverseIntoRearWall:
    def test_rear_threat_yields_no_kturn(self, controller):
        # A rear wall classifies as "back"; the escape table only reverses for
        # a "front" threat, so a rear wall never triggers a reverse K-turn.
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "back")
        assert maneuver is None


class TestKTurnSteersTowardClearerSide:
    """Ackermann reverse flips yaw response: +steering swings the nose right
    while reversing, -steering swings it left. The K-turn must pick its sign
    from which side is actually clearer, not a fixed direction, or it swings
    into the tighter wall every other attempt.
    """

    def test_steers_left_when_left_is_clearer(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(-math.pi / 2)  # tighten the right side
        ranges[i - 6 : i + 6] = 0.15
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.steering < 0  # swing toward the clearer left side

    def test_steers_right_when_right_is_clearer(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)  # tighten the left side
        ranges[i - 6 : i + 6] = 0.15
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.steering > 0  # swing toward the clearer right side

    def test_defaults_to_right_without_lidar_data(self, controller):
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front")
        assert maneuver.steering > 0


class TestSideCorrectionFlipsSignWhenReversing:
    """Ackermann reverse flips yaw response (see TestKTurnSteersTowardClearerSide),
    and SIDE_CORRECTION switches to reverse exactly when already touching the
    threatened side. A fixed steering sign there drives the nose further into
    the wall it is already touching instead of away from it -- measured on
    hardware pinning a side at 4.5 cm for the rest of a run that never
    recovered, reversing repeatedly without ever creating separation.
    """

    def test_left_threat_creeping_steers_right(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        ranges[i - 6 : i + 6] = 0.20  # left threat, not yet touching (> contact_dist)
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed > 0  # creeping forward, not reversing
        assert maneuver.steering < 0  # forward frame: negative swings the nose right

    def test_left_threat_touching_reverses_and_flips_sign(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        # Between self_detection_threshold_m (0.08, filtered as chassis
        # reflection below this) and contact_dist (0.10, "already touching" at
        # or below this).
        ranges[i - 6 : i + 6] = 0.09
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed < 0  # reversing
        assert maneuver.steering > 0  # reverse frame: positive swings the nose right, still away

    def test_right_threat_creeping_steers_left(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(-math.pi / 2)
        ranges[i - 6 : i + 6] = 0.20  # right threat, not yet touching
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "right", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed > 0
        assert maneuver.steering > 0  # forward frame: positive swings the nose left

    def test_right_threat_touching_reverses_and_flips_sign(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(-math.pi / 2)
        ranges[i - 6 : i + 6] = 0.09  # already touching, above the self-detection filter
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "right", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed < 0
        assert maneuver.steering < 0  # reverse frame: negative swings the nose left, still away


class TestSelfDetectionFilter:
    """Chassis/cable reflections at <= 0.08 m on side/rear sectors must not
    permanently read as a wall — that would block every reverse escape.
    """

    def test_rear_self_reflection_does_not_block_reverse(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        ranges[i - 4 : i + 4] = 0.05  # inside the self-detection radius
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) > 5.0

    def test_rear_real_wall_beyond_self_radius_still_detected(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        ranges[i - 4 : i + 4] = 0.15  # beyond self-detection radius: a real wall
        # The rear mount is in the blind wedge on the current chassis, so the
        # rear wall is masked and clearance reads as the no-data fallback.
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(controller.no_data_range_m)

    def test_side_self_reflection_does_not_report_as_threat(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        ranges[i - 4 : i + 4] = 0.05
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "none"

    def test_forward_near_contact_is_not_filtered(self, controller):
        # The forward bearing must never be self-detection filtered: a genuine
        # near-contact obstacle has to still register.
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        # Just above MIN_VALID_RANGE_M (0.05m, the C1's real rated minimum) --
        # this test is about the self-detection exemption, not the
        # invalid-reading floor itself, so it must not sit exactly on it.
        ranges[i - 4 : i + 4] = 0.06
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "front"


class TestNoReturnRaysExcluded:
    """A single no-return (+inf, beyond LIDAR max range) ray inside a sector
    must not poison that sector's mean/min/max to infinity -- a real 360 deg
    scan routinely has scattered no-return rays, and every sector consumer
    (the OLED's displayed clearance, detect_threat_direction, K-turn side
    selection) reads mean_range_m/min_range_m as a real distance.
    """

    def test_forward_clearance_ignores_a_stray_no_return_ray(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = np.inf  # one no-return ray inside the forward sector
        clearance = controller.compute_forward_clearance(ranges, ANGLES_FULL_ROTATION)
        assert math.isfinite(clearance)
        assert clearance == pytest.approx(LIDAR_DEFAULT_FAR)

    def test_forward_clearance_all_no_return_falls_back_to_default(self, controller):
        ranges = np.full(NUM_RAYS, np.inf)
        assert controller.compute_forward_clearance(ranges, ANGLES_FULL_ROTATION) == 10.0


class TestForwardPathRisk:
    """Risk is judged over the forward driving lane, not the full 360 sweep.

    Corridor side walls sit within the slow-zone distance but are not obstacles
    the robot is about to hit; flagging them pins the speed to a crawl for a whole
    lap (the ~0.16 m/s corridor creep). Only obstacles in the robot's path count.
    """

    def test_side_walls_are_not_risk(self, controller):
        ranges = create_numpy_scan()
        # Close walls to the left and right (+-pi/2), outside the driving lane.
        # Must clear path_half_width (chassis half-width + margin, 0.20m for the
        # 0.20m-wide chassis) with room to spare, or this stops testing "outside
        # the lane" and starts testing the boundary instead.
        for center in (math.pi / 2, -math.pi / 2):
            i = angle_to_index(center)
            ranges[i - 6 : i + 6] = 0.35
        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.SAFE

    def test_forward_obstacle_within_contact_is_critical(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08  # < contact_dist (0.10)
        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.CRITICAL

    def test_forward_obstacle_in_slow_zone_is_obstacle(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.20  # contact_dist < 0.20 < slow_dist (0.25)
        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.OBSTACLE


class TestBlindWedgeMasking:
    """The rear-left (-160..-115 deg) and rear-right (115..175 deg) mount
    wedges self-collide at every range, not just close ones -- a distance
    threshold can't tell that apart from a real close obstacle at the same
    bearing, so these rays must be excluded by angle regardless of range.
    """

    def test_left_wedge_self_collision_does_not_register_as_back_threat(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(-140))  # inside the left wedge (-160..-115)
        ranges[i - 4 : i + 4] = 0.02  # self-collision range, would otherwise scream "threat"
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "none"

    def test_right_wedge_self_collision_does_not_register_as_back_threat(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(140))  # inside the right wedge (115..175)
        ranges[i - 4 : i + 4] = 0.02
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "none"

    def test_real_wall_just_outside_left_wedge_still_detected(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(-110))  # just outside the wedge (< -115 boundary)
        ranges[i - 4 : i + 4] = 0.15  # above self_detection_threshold_m (0.08): a real return
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "right"

    def test_rear_clearance_ignores_wedge_self_collision(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(150))  # inside the right wedge, within the rear sector
        ranges[i - 4 : i + 4] = 0.02
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) > MIN_REAR_CLEARANCE

    def test_sector_fully_inside_wedge_reports_wedge_masked(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(-140))
        ranges[i - 2 : i + 2] = 0.02  # only rays available are inside the wedge
        sr = controller._sector_to_model(
            ranges,
            ANGLES_FULL_ROTATION,
            math.radians(-140),
            math.radians(5),
            filter_self_detection=True,
            self_detection_threshold_m=controller.self_detection_threshold_m,
            min_valid_range_m=controller.min_valid_range_m,
            no_data_range_m=controller.no_data_range_m,
            blind_wedge_left_min_rad=controller.blind_wedge_left_min_rad,
            blind_wedge_left_max_rad=controller.blind_wedge_left_max_rad,
            blind_wedge_right_min_rad=controller.blind_wedge_right_min_rad,
            blind_wedge_right_max_rad=controller.blind_wedge_right_max_rad,
        )
        assert sr.valid_count == 0
        assert sr.wedge_masked is True

    def test_sector_with_no_rays_at_all_is_not_reported_as_wedge_masked(self, controller):
        ranges = np.full(NUM_RAYS, np.inf)  # genuinely no data anywhere, not a wedge artifact
        sr = controller._sector_to_model(
            ranges,
            ANGLES_FULL_ROTATION,
            0.0,
            controller.front_half_fov_rad,
            self_detection_threshold_m=controller.self_detection_threshold_m,
            min_valid_range_m=controller.min_valid_range_m,
            no_data_range_m=controller.no_data_range_m,
            blind_wedge_left_min_rad=controller.blind_wedge_left_min_rad,
            blind_wedge_left_max_rad=controller.blind_wedge_left_max_rad,
            blind_wedge_right_min_rad=controller.blind_wedge_right_min_rad,
            blind_wedge_right_max_rad=controller.blind_wedge_right_max_rad,
        )
        assert sr.valid_count == 0
        assert sr.wedge_masked is False


class TestMaskMappedObstacles:
    """Returns attributable to a planner-owned obstacle are withheld from the
    escape trigger; everything else keeps the full reactive guard.

    The split is by provenance, not distance — so these tests pin that a
    mapped position at a given range is masked while an identical return at a
    position nothing owns is not.
    """

    _MASK_RADIUS = 0.12

    def test_return_on_a_mapped_position_is_masked(self):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08  # inside contact_dist

        masked = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS)

        assert np.isinf(masked[i]), "a return landing on a mapped sign must be withheld"

    def test_unmapped_return_at_the_same_range_is_untouched(self):
        """The guard is removed for the mapped obstacle only, not for the range."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        # Mapped sign is off to the side; the forward return belongs to nothing.
        masked = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.0, 0.9), Section.SOUTH)], self._MASK_RADIUS)

        assert masked[i] == pytest.approx(0.08)

    def test_masking_downgrades_risk_from_critical_to_safe(self, controller):
        """The whole point: the same scan is CRITICAL raw and SAFE once masked."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.CRITICAL
        masked = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS)
        assert controller.assess_risk(masked, ANGLES_FULL_ROTATION) == RiskLevel.SAFE

    def test_mapped_positions_are_world_frame_not_robot_frame(self):
        """Ray endpoints are placed using the robot's pose, so a sign's world
        position masks the correct ray whatever the robot's heading is.

        Pinning this because a robot-frame reading would appear to work at
        yaw=0 — the pose the other tests here use — and silently mask the wrong
        bearing everywhere else on the track.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08
        # Robot at (1.0, 2.0) facing north: the forward return lands at
        # (1.0, 2.08), NOT at (0.08, 0).
        pose = Pose(1.0, 2.0, math.pi / 2)

        masked_world = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(1.0, 2.08), Section.NORTH)], self._MASK_RADIUS)
        masked_body = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(0.08, 0.0), Section.NORTH)], self._MASK_RADIUS)

        assert np.isinf(masked_world[i])
        assert masked_body[i] == pytest.approx(0.08)

    def test_cross_corridor_coincidence_is_not_masked(self):
        """A routed sign's raw XY landing near an unrelated ray endpoint is not
        enough to mask it -- the robot must also currently be in that sign's
        own corridor.

        This is the exact failure traced on the full blind corpus: a believed
        pose that is a wrong-but-consistent rigid rotation of the truth (the
        rotational-lock failure) can reproject a genuinely unmapped, imminent
        obstacle's ray onto an already-routed sign's coordinates purely by
        coincidence, withholding it from the escape trigger. Tagging the
        mapped sign with a corridor different from the robot's own pins that
        the proximity match alone must not be enough to mask it.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08
        # Robot at (1.5, 0.5) is in the SOUTH corridor; the forward return
        # lands at (1.56, 0.5), also SOUTH.
        pose = Pose(1.5, 0.5, 0.0)

        # A "routed sign" whose raw XY coincides with that ray endpoint, but
        # which the router itself placed in the NORTH corridor.
        masked = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(1.56, 0.5), Section.NORTH)], self._MASK_RADIUS)

        assert masked[i] == pytest.approx(0.08), "a same-XY coincidence in a different corridor must not mask"

    def test_zero_radius_and_empty_map_are_no_ops(self):
        """Both disable the split — the escape trigger sees the raw scan.

        ``escape_mask_radius_m = 0`` is the documented off-switch and the
        comparison every measurement of the split is read against, so it has to
        be exactly the old behaviour rather than approximately it.
        """
        ranges = create_numpy_scan()
        ranges[angle_to_index(0.0)] = 0.08

        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], 0.0), ranges)
        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [], self._MASK_RADIUS), ranges)

    def test_no_return_rays_stay_infinite_and_never_become_nan(self):
        """``inf`` ranges have no endpoint to attribute.

        ``inf * cos(theta)`` is ``+-inf`` and ``inf - inf`` is ``nan``, so an
        unguarded distance test would turn every no-return ray into ``nan`` —
        which compares False everywhere and would quietly corrupt the sector
        helpers' min/mean rather than failing loudly.
        """
        ranges = create_numpy_scan()
        ranges[:] = np.inf

        masked = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS)

        assert np.all(np.isinf(masked))
        assert not np.any(np.isnan(masked))

    def test_input_scan_is_not_mutated(self):
        """The raw scan still governs speed and the rear gate, so masking must
        return a copy rather than editing the caller's array in place.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = 0.08

        mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS)

        assert ranges[i] == pytest.approx(0.08)


class TestLidarClearancesModel:
    """Exercises the domain helpers on ``LidarClearances`` (audit §12b)."""

    def test_any_blocked_triggers_below_threshold(self):
        c = LidarClearances(front_m=0.05, left_m=2.0, right_m=2.0, back_m=2.0)
        assert c.any_blocked(0.1) is True
        assert c.any_blocked(0.02) is False

    def test_most_constrained_side_picks_smallest(self):
        c = LidarClearances(front_m=2.0, left_m=0.1, right_m=2.0, back_m=2.0)
        assert c.most_constrained_side is ThreatDirection.LEFT

        c2 = LidarClearances(front_m=2.0, left_m=2.0, right_m=2.0, back_m=0.0)
        assert c2.most_constrained_side is ThreatDirection.BACK


class TestClearancesFromScan:
    """The shared factory must build a ``LidarClearances`` from a ``LidarScan``."""

    def test_empty_scan_yields_zeroed_clearances(self, controller):
        scan = LidarScan(ranges_m=(), angles_rad=())
        fov = math.radians(NavigationTuning.load_default().lidar_sectors.FRONT_HALF_FOV_DEG)
        c = clearances_from_scan(scan, controller, fov)
        assert (c.front_m, c.left_m, c.right_m, c.back_m) == (0.0, 0.0, 0.0, 0.0)

    def test_front_wall_reports_small_front_clearance(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION))
        fov = math.radians(NavigationTuning.load_default().lidar_sectors.FRONT_HALF_FOV_DEG)
        c = clearances_from_scan(scan, controller, fov)
        # Front is the most constrained side; the others stay open.
        assert c.most_constrained_side is ThreatDirection.FRONT
        assert c.front_m < c.left_m
        assert c.front_m < c.right_m

    def test_min_aggregation_matches_threat_sectors(self, controller):
        """MIN aggregation + threat FOV reproduces detect_threat_direction's sectors."""
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)  # +pi/2 = left
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION))
        c = clearances_from_scan(
            scan, controller, controller.threat_half_fov_rad, aggregate=ClearanceAggregate.MIN,
        )
        # Min-based left sector is the most constrained side.
        assert c.most_constrained_side is ThreatDirection.LEFT
        # Matches the controller's own threat-direction classifier directly.
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == ThreatDirection.LEFT

    def test_threat_direction_gates_on_no_detection_range(self, controller):
        # Open scan: nothing within the no-detection range -> NONE.
        scan = LidarScan(ranges_m=tuple(create_numpy_scan()), angles_rad=tuple(ANGLES_FULL_ROTATION))
        c = clearances_from_scan(
            scan, controller, controller.threat_half_fov_rad, aggregate=ClearanceAggregate.MIN,
        )
        assert threat_direction(c, controller.threat_no_detection_range_m) is ThreatDirection.NONE

        # A close front wall enters the gate and names the constrained side.
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = LIDAR_CLOSE_THREAT
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION))
        c = clearances_from_scan(
            scan, controller, controller.threat_half_fov_rad, aggregate=ClearanceAggregate.MIN,
        )
        assert threat_direction(c, controller.threat_no_detection_range_m) is ThreatDirection.FRONT


class TestParkingGate:
    """The parking stop-check needs a narrow-forward min AND a full-sweep min."""

    def test_returns_forward_and_sweep_from_one_call(self, controller):
        ranges = create_numpy_scan()
        # Close wall dead ahead: catches the forward cone.
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        # A *closer* wall at 45 deg (between front cone and left sector): only the
        # full-sweep min should see it -- the 4-sector model would miss it.
        j = angle_to_index(math.pi / 4)
        ranges[j - FORWARD_SECTOR_INDICES : j + FORWARD_SECTOR_INDICES] = 0.1

        gate = controller.parking_clearances(ranges, ANGLES_FULL_ROTATION)

        # Forward reflects only the head-on wall (0.3), not the 45 deg one.
        assert gate.forward_m == pytest.approx(controller.compute_forward_clearance(ranges, ANGLES_FULL_ROTATION))
        assert gate.forward_m > 0.1
        # Sweep reflects the nearest of both walls (the 0.1 m 45 deg one).
        assert gate.sweep_m == pytest.approx(
            controller.compute_min_clearance(ranges, ANGLES_FULL_ROTATION, half_fov_rad=math.pi),
        )
        assert gate.sweep_m < gate.forward_m

    def test_empty_scan_yields_no_data(self, controller):
        gate = controller.parking_clearances(np.array([]), None)
        assert gate.forward_m == controller.no_data_range_m
        assert gate.sweep_m == controller.no_data_range_m
