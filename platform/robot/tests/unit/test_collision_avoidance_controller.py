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
from shared.domain.enums import RiskLevel
from shared.domain.models import Pose

from src.navigation.control.controllers.collision_avoidance_controller import (
    CollisionAvoidanceController,
    mask_mapped_obstacles,
)
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

ANGLES = ANGLES_FULL_ROTATION


@pytest.fixture()
def controller() -> CollisionAvoidanceController:
    return CollisionAvoidanceController()


class TestThreatDirection:
    def test_wall_behind_reports_back_not_front(self, controller):
        ranges = create_numpy_scan()
        # Close returns near +-pi (rear cone), including index 0 (= -pi).
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "back"

    def test_wall_behind_back_even_without_angles(self, controller):
        # When angles are omitted, index 0 is assumed to be -pi, so a close
        # ray at index 0 is still the rear, not the front.
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges) == "back"

    def test_wall_ahead_reports_front(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "front"

    def test_obstacle_left_reports_left(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)  # +pi/2 = left
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT

        assert controller.detect_threat_direction(ranges, ANGLES) == "left"

    def test_all_clear_reports_none(self, controller):
        assert controller.detect_threat_direction(create_numpy_scan(), ANGLES) == "none"


class TestClearance:
    def test_forward_clearance_ignores_rear_wall(self, controller):
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        # Path ahead is clear even though a wall sits behind.
        assert controller.compute_forward_clearance(ranges, ANGLES) > MIN_FORWARD_CLEARANCE

    def test_rear_clearance_sees_rear_wall(self, controller):
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES) == pytest.approx(LIDAR_CLOSE_THREAT)

    def test_rear_clearance_clear_when_only_front_blocked(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES) > MIN_REAR_CLEARANCE


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
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES)
        assert maneuver.steering < 0  # swing toward the clearer left side

    def test_steers_right_when_right_is_clearer(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)  # tighten the left side
        ranges[i - 6 : i + 6] = 0.15
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "front", ranges, ANGLES)
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
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES)
        assert maneuver.speed > 0  # creeping forward, not reversing
        assert maneuver.steering < 0  # forward frame: negative swings the nose right

    def test_left_threat_touching_reverses_and_flips_sign(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        # Between self_detection_threshold_m (0.08, filtered as chassis
        # reflection below this) and contact_dist (0.10, "already touching" at
        # or below this).
        ranges[i - 6 : i + 6] = 0.09
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES)
        assert maneuver.speed < 0  # reversing
        assert maneuver.steering > 0  # reverse frame: positive swings the nose right, still away

    def test_right_threat_creeping_steers_left(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(-math.pi / 2)
        ranges[i - 6 : i + 6] = 0.20  # right threat, not yet touching
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "right", ranges, ANGLES)
        assert maneuver.speed > 0
        assert maneuver.steering > 0  # forward frame: positive swings the nose left

    def test_right_threat_touching_reverses_and_flips_sign(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(-math.pi / 2)
        ranges[i - 6 : i + 6] = 0.09  # already touching, above the self-detection filter
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "right", ranges, ANGLES)
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
        assert controller.compute_rear_clearance(ranges, ANGLES) > 5.0

    def test_rear_real_wall_beyond_self_radius_still_detected(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        ranges[i - 4 : i + 4] = 0.15  # beyond self-detection radius: a real wall
        assert controller.compute_rear_clearance(ranges, ANGLES) == pytest.approx(0.15)

    def test_side_self_reflection_does_not_report_as_threat(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        ranges[i - 4 : i + 4] = 0.05
        assert controller.detect_threat_direction(ranges, ANGLES) == "none"

    def test_forward_near_contact_is_not_filtered(self, controller):
        # The forward bearing must never be self-detection filtered: a genuine
        # near-contact obstacle has to still register.
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        # Just above MIN_VALID_RANGE_M (0.05m, the C1's real rated minimum) --
        # this test is about the self-detection exemption, not the
        # invalid-reading floor itself, so it must not sit exactly on it.
        ranges[i - 4 : i + 4] = 0.06
        assert controller.detect_threat_direction(ranges, ANGLES) == "front"


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
        clearance = controller.compute_forward_clearance(ranges, ANGLES)
        assert math.isfinite(clearance)
        assert clearance == pytest.approx(LIDAR_DEFAULT_FAR)

    def test_forward_clearance_all_no_return_falls_back_to_default(self, controller):
        ranges = np.full(NUM_RAYS, np.inf)
        assert controller.compute_forward_clearance(ranges, ANGLES) == 10.0


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
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.SAFE

    def test_forward_obstacle_within_contact_is_critical(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08  # < contact_dist (0.10)
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.CRITICAL

    def test_forward_obstacle_in_slow_zone_is_obstacle(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.20  # contact_dist < 0.20 < slow_dist (0.25)
        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.OBSTACLE


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
        assert controller.detect_threat_direction(ranges, ANGLES) == "none"

    def test_right_wedge_self_collision_does_not_register_as_back_threat(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(140))  # inside the right wedge (115..175)
        ranges[i - 4 : i + 4] = 0.02
        assert controller.detect_threat_direction(ranges, ANGLES) == "none"

    def test_real_wall_just_outside_left_wedge_still_detected(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(-110))  # just outside the wedge (< -115 boundary)
        ranges[i - 4 : i + 4] = 0.15  # above self_detection_threshold_m (0.08): a real return
        assert controller.detect_threat_direction(ranges, ANGLES) == "right"

    def test_rear_clearance_ignores_wedge_self_collision(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(150))  # inside the right wedge, within the rear sector
        ranges[i - 4 : i + 4] = 0.02
        assert controller.compute_rear_clearance(ranges, ANGLES) > MIN_REAR_CLEARANCE

    def test_sector_fully_inside_wedge_reports_wedge_masked(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.radians(-140))
        ranges[i - 2 : i + 2] = 0.02  # only rays available are inside the wedge
        sr = controller._sector_to_model(
            ranges,
            ANGLES,
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
            ANGLES,
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

        masked = mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.isinf(masked[i]), "a return landing on a mapped sign must be withheld"

    def test_unmapped_return_at_the_same_range_is_untouched(self):
        """The guard is removed for the mapped obstacle only, not for the range."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        # Mapped sign is off to the side; the forward return belongs to nothing.
        masked = mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.0, 0.9)], self._MASK_RADIUS)

        assert masked[i] == pytest.approx(0.08)

    def test_masking_downgrades_risk_from_critical_to_safe(self, controller):
        """The whole point: the same scan is CRITICAL raw and SAFE once masked."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        assert controller.assess_risk(ranges, ANGLES) == RiskLevel.CRITICAL
        masked = mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)
        assert controller.assess_risk(masked, ANGLES) == RiskLevel.SAFE

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

        masked_world = mask_mapped_obstacles(ranges, ANGLES, pose, [(1.0, 2.08)], self._MASK_RADIUS)
        masked_body = mask_mapped_obstacles(ranges, ANGLES, pose, [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.isinf(masked_world[i])
        assert masked_body[i] == pytest.approx(0.08)

    def test_zero_radius_and_empty_map_are_no_ops(self):
        """Both disable the split — the escape trigger sees the raw scan.

        ``escape_mask_radius_m = 0`` is the documented off-switch and the
        comparison every measurement of the split is read against, so it has to
        be exactly the old behaviour rather than approximately it.
        """
        ranges = create_numpy_scan()
        ranges[angle_to_index(0.0)] = 0.08

        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.08, 0.0)], 0.0), ranges)
        assert np.array_equal(mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [], self._MASK_RADIUS), ranges)

    def test_no_return_rays_stay_infinite_and_never_become_nan(self):
        """``inf`` ranges have no endpoint to attribute.

        ``inf * cos(theta)`` is ``+-inf`` and ``inf - inf`` is ``nan``, so an
        unguarded distance test would turn every no-return ray into ``nan`` —
        which compares False everywhere and would quietly corrupt the sector
        helpers' min/mean rather than failing loudly.
        """
        ranges = create_numpy_scan()
        ranges[:] = np.inf

        masked = mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert np.all(np.isinf(masked))
        assert not np.any(np.isnan(masked))

    def test_input_scan_is_not_mutated(self):
        """The raw scan still governs speed and the rear gate, so masking must
        return a copy rather than editing the caller's array in place.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = 0.08

        mask_mapped_obstacles(ranges, ANGLES, Pose(0.0, 0.0, 0.0), [(0.08, 0.0)], self._MASK_RADIUS)

        assert ranges[i] == pytest.approx(0.08)
