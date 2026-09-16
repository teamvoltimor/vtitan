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
from shared.config.constants import RobotSpecs
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
    ranges_beyond_chassis,
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

        # Asserts what the test is named for again. The rear was once fully
        # masked by the blind wedges, so a wall behind was not seen at all and
        # this asserted "none". The wedges were re-measured on the current mount,
        # leaving a readable slot at the rear, so the rear is visible and the
        # original behaviour is back. See adr:0056-raw-and-masked-scan.
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == "back"

    def test_wall_behind_back_even_without_angles(self, controller):
        # When angles are omitted, index 0 is assumed to be -pi, so a close
        # ray at index 0 is still the rear, not the front.
        ranges = create_numpy_scan()
        ranges[:REAR_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        ranges[-REAR_SECTOR_INDICES:] = LIDAR_CLOSE_THREAT

        # Rear visible again since the wedge re-measurement (see
        # test_wall_behind_reports_back_not_front and
        # adr:0056-raw-and-masked-scan).
        assert controller.detect_threat_direction(ranges) == "back"

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
        # Asserts the actual wall distance again, as the name says. It once read
        # the no-data fallback while the rear was fully masked; the re-measured
        # wedges restore the original behaviour. See
        # adr:0056-raw-and-masked-scan.
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(LIDAR_CLOSE_THREAT)

    def test_rear_clearance_clear_when_only_front_blocked(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) > MIN_REAR_CLEARANCE


class TestRearSectorVisibility:
    """A reverse gate must be able to tell "nothing behind" from "cannot see"."""

    def test_normal_scan_is_measured(self, controller):
        # Measured again since the wedge re-measurement: the rear sector
        # carries a readable slot at the rear, so a normal scan reports
        # MEASURED. Under the old mount the whole rear was masked and this
        # asserted False -- "cannot see" rather than "nothing behind". See
        # adr:0056-raw-and-masked-scan.
        assert controller.rear_sector(create_numpy_scan(), ANGLES_FULL_ROTATION).measured is True

    def test_no_valid_rear_rays_is_not_measured(self, controller):
        # Every ray in the rear half is a no-return -- the case a chassis whose
        # mount occludes the last ~25 deg slot would see on every scan.
        ranges = create_numpy_scan()
        ranges[np.abs(ANGLES_FULL_ROTATION) >= math.pi / 2] = 0.0

        # The clearance number cannot express this: it reads as wide-open road,
        # which is what made the reverse guard fail open.
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(
            controller.no_data_range_m
        )
        assert controller.rear_sector(ranges, ANGLES_FULL_ROTATION).measured is False

    def test_empty_scan_is_not_measured(self, controller):
        assert controller.rear_sector(np.array([]), None).measured is False


class TestFrontSectorVisibility:
    """The same distinction for the FRONT, which never had it and drove into a wall.

    Measured on hardware: pressed against a wall and physically immobile, every
    ray in the forward cone fell below ``min_valid_range_m`` -- a flat surface
    centimetres away reflects too shallowly to return a signal -- so forward
    clearance reported a wide-open distance for the rest of the run. The
    navigator held speed into the wall and the ``stuck_forward`` escape stood
    down, because by that number the road ahead was clear. See
    adr:0056-raw-and-masked-scan.
    """

    def test_normal_scan_is_measured(self, controller):
        assert controller.front_sector(create_numpy_scan(), ANGLES_FULL_ROTATION).measured is True

    def test_all_invalid_forward_rays_is_not_measured(self, controller):
        """The hardware condition: a wall too close for any ray to return validly."""
        ranges = create_numpy_scan()
        # Below min_valid_range_m across the WHOLE front sector -- physically
        # unmeasurable, not "clear". Masked by angle rather than by index so it
        # covers front_half_fov exactly; an index slice narrower than the sector
        # leaves valid rays at its edges and the sector still reports measured.
        ranges[np.abs(ANGLES_FULL_ROTATION) <= controller.front_half_fov_rad] = 0.0

        # The shorthand cannot express it: reads as wide-open road. This is the
        # number that let the robot drive into a wall it was touching.
        assert controller.compute_forward_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(
            controller.no_data_range_m
        )
        # The sector can.
        assert controller.front_sector(ranges, ANGLES_FULL_ROTATION).measured is False

    def test_empty_scan_is_not_measured(self, controller):
        assert controller.front_sector(np.array([]), None).measured is False

    def test_a_genuine_far_reading_stays_measured(self, controller):
        """ "Far" and "cannot see" must not collapse into each other.

        Guards the fix against over-reach: an open corridor ahead reports large
        ranges and MUST still count as measured, or the degraded-sensor branch
        fires on every straight and the robot crawls the whole race.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = 3.0
        sector = controller.front_sector(ranges, ANGLES_FULL_ROTATION)
        assert sector.measured is True
        assert sector.min_range_m == pytest.approx(3.0)


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
    hardware pinning a side for the rest of a run that never recovered,
    reversing repeatedly without ever creating separation. See
    adr:0050-escape-steering-degrees-and-committed-side.
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


class TestSideCorrectionReversesWhenTouchingForward:
    """A robot pinned at an angle into a corner can be touching in front while
    its side sector still reads clear -- detect_threat_direction's narrower
    angular cone can classify this as a side threat even though assess_risk's
    wider forward-path lane (which triggered CRITICAL in the first place) sees
    the wall. SIDE_CORRECTION's "already touching" check must also look
    forward, or it keeps creeping into a wall it is already touching. Measured
    on real Open Challenge runs that pinned nose-first for many seconds,
    repeatedly issuing SIDE_CORRECTION with a forward-creep speed the whole
    time. See adr:0055-escape-maneuver-selection.
    """

    def test_left_threat_with_forward_contact_reverses(self, controller):
        ranges = create_numpy_scan()
        left_i = angle_to_index(math.pi / 2)
        ranges[left_i - 6 : left_i + 6] = 0.20  # left sector alone reads clear (not touching)
        front_i = angle_to_index(0.0)
        ranges[front_i - 4 : front_i + 4] = 0.08  # but forward is already at contact_dist
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed < 0  # reverses instead of creeping forward into the wall
        assert maneuver.steering > 0  # reverse frame: still swings away, same as a side-touch reversal

    def test_right_threat_with_forward_contact_reverses(self, controller):
        ranges = create_numpy_scan()
        right_i = angle_to_index(-math.pi / 2)
        ranges[right_i - 6 : right_i + 6] = 0.20  # right sector alone reads clear
        front_i = angle_to_index(0.0)
        ranges[front_i - 4 : front_i + 4] = 0.08  # forward already at contact_dist
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "right", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed < 0
        assert maneuver.steering < 0  # reverse frame: still swings away

    def test_left_threat_with_forward_and_side_clear_still_creeps(self, controller):
        # Regression guard: forward clear + side clear must still creep, not
        # reverse on every tick regardless of actual clearance.
        ranges = create_numpy_scan()
        left_i = angle_to_index(math.pi / 2)
        ranges[left_i - 6 : left_i + 6] = 0.20
        maneuver = controller.compute_escape_maneuver(RiskLevel.CRITICAL, "left", ranges, ANGLES_FULL_ROTATION)
        assert maneuver.speed > 0


class TestSelfDetectionFilter:
    """Chassis/cable reflections at <= 0.08 m on side/rear sectors must not
    permanently read as a wall — that would block every reverse escape.
    """

    def test_rear_self_reflection_does_not_block_reverse(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        ranges[i - 4 : i + 4] = 0.05  # inside the self-detection radius
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) > 5.0

    def test_rear_real_wall_beyond_the_chassis_is_still_detected(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        # 0.35 m, i.e. OUTSIDE the body. The old fixture read 0.15 m, which is
        # not a place a wall can be: the chassis rear face is
        # RobotSpecs.LIDAR_TO_REAR_BUMPER = 0.2722 m behind the sensor, so a
        # 0.15 m return straight back is the robot seeing itself. The rear
        # self-detection filter is chassis geometry now rather than a small
        # scalar, so the old fixture asserted that a self-return be reported as a
        # wall, exactly the defect it now guards against. See
        # adr:0056-raw-and-masked-scan.
        ranges[i - 4 : i + 4] = 0.35
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) == pytest.approx(0.35)

    def test_a_rear_return_inside_the_chassis_is_the_robot_not_a_wall(self, controller):
        """The defect that suppressed every escape for a whole hardware round.

        Measured on a hardware run: the rear minimum came from bearings whose
        returns lay INSIDE the chassis on every scan, so `back_m` read a
        self-distance all run and `most_constrained_side` was BACK on most
        driving ticks, and on every contact episode. `compute_escape_maneuver`
        has no BACK branch, so critical-risk ticks produced ZERO manoeuvres. See
        adr:0056-raw-and-masked-scan.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi)
        ranges[i - 4 : i + 4] = 0.125
        # The rest of the sector is open, so it is still MEASURED -- what must
        # not happen is the self-return becoming its minimum and, through
        # `most_constrained_side`, its verdict.
        assert controller.rear_sector(ranges, ANGLES_FULL_ROTATION).min_range_m > RobotSpecs.LIDAR_TO_REAR_BUMPER
        assert controller.compute_rear_clearance(ranges, ANGLES_FULL_ROTATION) != pytest.approx(0.125)

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

    def test_whole_forward_lane_no_return_is_critical_not_safe(self, controller):
        # ros2_hardware_gateway sanitizes NaN/inf no-return rays to
        # LIDAR_MAX_RANGE before this ever sees them -- grazing incidence off
        # something very close reads exactly like this. A forward lane where
        # every ray is this fabricated value must NOT read as "wide open,
        # SAFE": that is precisely what let a real corner go undetected on
        # hardware (a narrow-corridor run with an enlarged centre wall that
        # never turned, the whole forward cone grazing the wall rather than
        # genuinely clear road). See adr:0056-raw-and-masked-scan.
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = RobotSpecs.LIDAR_MAX_RANGE
        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.CRITICAL

    def test_genuinely_empty_scan_is_still_safe(self, controller):
        # Only a lane that HAD rays but every one was a no-return escalates
        # (previous test) -- a scan with nothing in it at all has nothing to
        # judge and must stay SAFE, or every startup tick before the first
        # scan arrives would read as a false emergency.
        assert controller.assess_risk(np.array([]), None) == RiskLevel.SAFE


class TestBlindWedgeMasking:
    """The rear-left and rear-right mount wedges self-collide at every range,
    not just close ones -- a distance threshold can't tell that apart from a
    real close obstacle at the same bearing, so these rays must be excluded by
    angle regardless of range. See adr:0056-raw-and-masked-scan.
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

        masked = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS
        )

        assert np.isinf(masked[i]), "a return landing on a mapped sign must be withheld"

    def test_unmapped_return_at_the_same_range_is_untouched(self):
        """The guard is removed for the mapped obstacle only, not for the range."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        # Mapped sign is off to the side; the forward return belongs to nothing.
        masked = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.0, 0.9), Section.SOUTH)], self._MASK_RADIUS
        )

        assert masked[i] == pytest.approx(0.08)

    def test_masking_downgrades_risk_from_critical_to_safe(self, controller):
        """The whole point: the same scan is CRITICAL raw and SAFE once masked."""
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.08

        assert controller.assess_risk(ranges, ANGLES_FULL_ROTATION) == RiskLevel.CRITICAL
        masked = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS
        )
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

        masked_world = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(1.0, 2.08), Section.NORTH)], self._MASK_RADIUS
        )
        masked_body = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(0.08, 0.0), Section.NORTH)], self._MASK_RADIUS
        )

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
        masked = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(1.56, 0.5), Section.NORTH)], self._MASK_RADIUS
        )

        assert masked[i] == pytest.approx(0.08), "a same-XY coincidence in a different corridor must not mask"

    def test_the_snap_masks_a_return_the_belief_alone_would_miss(self):
        """The whole point: the belief is wrong, the cluster is not.

        MEASURED on the Obstacles rounds that wedged at the same point: over the
        ticks where the router held a commitment, the nearest LIDAR return sat
        far from the BELIEVED sign position against the mask radius, so the
        belief-anchored mask caught almost none of the ticks it exists for.
        Anchored on the cluster instead, it catches most of them. See
        adr:0056-raw-and-masked-scan.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 0.40
        pose = Pose(0.0, 0.0, 0.0)
        # The return lands at (0.40, 0.0). The belief is 0.25 m short of it --
        # well outside the 0.12 m radius, which is the measured hardware case.
        belief = [(Waypoint(0.15, 0.0), Section.SOUTH)]

        unsnapped = mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, pose, belief, self._MASK_RADIUS)
        assert unsnapped[i] == pytest.approx(0.40), "belief-anchored must miss it, as it does on hardware"

        snapped = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, pose, belief, self._MASK_RADIUS,
            cluster_xy=[Waypoint(0.40, 0.0)], assoc_m=0.35,
        )
        assert not np.isfinite(snapped[i]), "cluster-anchored must mask it"

    def test_an_unassociated_belief_does_not_borrow_another_pillars_cluster(self):
        """A cluster further than ``assoc_m`` is a different object.

        Falling back to the belief is the conservative failure: it masks
        nothing, which is the behaviour that shipped. Snapping to whatever is
        nearest would mask a pillar the router is NOT routing around, which is
        strictly worse than the bug this fixes -- the reactive layer would lose
        its guard on an obstacle nobody has a plan for.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - 4 : i + 4] = 1.20
        pose = Pose(0.0, 0.0, 0.0)

        snapped = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, pose, [(Waypoint(0.15, 0.0), Section.SOUTH)],
            self._MASK_RADIUS, cluster_xy=[Waypoint(1.20, 0.0)], assoc_m=0.35,
        )
        assert snapped[i] == pytest.approx(1.20)

    def test_zero_assoc_restores_belief_anchored_masking_exactly(self):
        """The off-switch has to be exact, not approximate.

        Every measurement of the split is read against the unsnapped arm, so a
        zero here must reproduce it byte for byte rather than nearly.
        """
        ranges = create_numpy_scan()
        ranges[angle_to_index(0.0)] = 0.08
        pose = Pose(0.0, 0.0, 0.0)
        belief = [(Waypoint(0.08, 0.0), Section.SOUTH)]

        assert np.array_equal(
            mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, pose, belief, self._MASK_RADIUS),
            mask_mapped_obstacles(
                ranges, ANGLES_FULL_ROTATION, pose, belief, self._MASK_RADIUS,
                cluster_xy=[Waypoint(2.0, 2.0)], assoc_m=0.0,
            ),
        )

    def test_zero_radius_and_empty_map_are_no_ops(self):
        """Both disable the split — the escape trigger sees the raw scan.

        ``escape_mask_radius_m = 0`` is the documented off-switch and the
        comparison every measurement of the split is read against, so it has to
        be exactly the old behaviour rather than approximately it.
        """
        ranges = create_numpy_scan()
        ranges[angle_to_index(0.0)] = 0.08

        assert np.array_equal(
            mask_mapped_obstacles(
                ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], 0.0
            ),
            ranges,
        )
        assert np.array_equal(
            mask_mapped_obstacles(ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [], self._MASK_RADIUS), ranges
        )

    def test_no_return_rays_stay_infinite_and_never_become_nan(self):
        """``inf`` ranges have no endpoint to attribute.

        ``inf * cos(theta)`` is ``+-inf`` and ``inf - inf`` is ``nan``, so an
        unguarded distance test would turn every no-return ray into ``nan`` —
        which compares False everywhere and would quietly corrupt the sector
        helpers' min/mean rather than failing loudly.
        """
        ranges = create_numpy_scan()
        ranges[:] = np.inf

        masked = mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS
        )

        assert np.all(np.isinf(masked))
        assert not np.any(np.isnan(masked))

    def test_input_scan_is_not_mutated(self):
        """The raw scan still governs speed and the rear gate, so masking must
        return a copy rather than editing the caller's array in place.
        """
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = 0.08

        mask_mapped_obstacles(
            ranges, ANGLES_FULL_ROTATION, Pose(0.0, 0.0, 0.0), [(Waypoint(0.08, 0.0), Section.SOUTH)], self._MASK_RADIUS
        )

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
        fov = math.radians(NavigationTuning.load_default().lidar_sectors.front_half_fov_deg)
        c = clearances_from_scan(scan, controller, fov)
        assert (c.front_m, c.left_m, c.right_m, c.back_m) == (0.0, 0.0, 0.0, 0.0)

    def test_front_wall_reports_small_front_clearance(self, controller):
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i - FORWARD_SECTOR_INDICES : i + FORWARD_SECTOR_INDICES] = LIDAR_CLOSE_THREAT
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION))
        fov = math.radians(NavigationTuning.load_default().lidar_sectors.front_half_fov_deg)
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
            scan,
            controller,
            controller.threat_half_fov_rad,
            aggregate=ClearanceAggregate.MIN,
        )
        # Min-based left sector is the most constrained side.
        assert c.most_constrained_side is ThreatDirection.LEFT
        # Matches the controller's own threat-direction classifier directly.
        assert controller.detect_threat_direction(ranges, ANGLES_FULL_ROTATION) == ThreatDirection.LEFT

    def test_threat_direction_gates_on_no_detection_range(self, controller):
        # Open scan: nothing within the no-detection range -> NONE.
        scan = LidarScan(ranges_m=tuple(create_numpy_scan()), angles_rad=tuple(ANGLES_FULL_ROTATION))
        c = clearances_from_scan(
            scan,
            controller,
            controller.threat_half_fov_rad,
            aggregate=ClearanceAggregate.MIN,
        )
        assert threat_direction(c, controller.threat_no_detection_range_m) is ThreatDirection.NONE

        # A close front wall enters the gate and names the constrained side.
        ranges = create_numpy_scan()
        i = angle_to_index(0.0)
        ranges[i] = LIDAR_CLOSE_THREAT
        scan = LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION))
        c = clearances_from_scan(
            scan,
            controller,
            controller.threat_half_fov_rad,
            aggregate=ClearanceAggregate.MIN,
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


class TestRangesBeyondChassis:
    """The per-bearing self-return filter that replaced the escape mask's floor.

    The whole claim is that a SCALAR cannot do this job, so the central test is
    one range at two bearings with opposite verdicts.
    """

    _MARGIN = 0.01

    def test_one_range_two_bearings_opposite_verdicts(self):
        """0.20 m is the robot behind it and a real obstacle beside it."""
        ranges = create_numpy_scan()
        rear, side = angle_to_index(math.pi), angle_to_index(math.pi / 2)
        ranges[rear] = 0.20  # inside the 0.2722 m rear face
        ranges[side] = 0.20  # outside the 0.097 m flank

        kept = ranges_beyond_chassis(ranges, ANGLES_FULL_ROTATION, self._MARGIN)

        assert not np.isfinite(kept[rear])
        assert kept[side] == pytest.approx(0.20)

    def test_return_just_outside_the_boundary_survives(self):
        ranges = create_numpy_scan()
        i = angle_to_index(math.pi / 2)
        ranges[i] = RobotSpecs.WIDTH / 2.0 + self._MARGIN + 0.005

        kept = ranges_beyond_chassis(ranges, ANGLES_FULL_ROTATION, self._MARGIN)

        assert np.isfinite(kept[i])

    def test_distant_returns_are_untouched(self):
        ranges = create_numpy_scan()

        kept = ranges_beyond_chassis(ranges, ANGLES_FULL_ROTATION, self._MARGIN)

        assert np.allclose(kept, ranges)


class TestEscapeSideFollowsCommittedSign:
    """The K-turn's side against the side the ROUTER committed to.

    The two answer different questions -- "which wall is nearer" against "which
    side of the PILLAR must I pass" -- and agree only about half the time, and
    less in a corner. These pin the override and, just as importantly, that
    it is REACHABLE: a flag verified only in the off state is how a previous
    guard shipped inert. See
    adr:0050-escape-steering-degrees-and-committed-side.
    """

    @staticmethod
    def _lopsided_scan() -> tuple[np.ndarray, np.ndarray]:
        """A scan with the RIGHT side unambiguously clearer than the left.

        So the unaided comparison returns +1 (steer right), and an override
        toward the left has something to overrule.
        """
        ranges = create_numpy_scan()
        left = angle_to_index(math.pi / 2)
        right = angle_to_index(-math.pi / 2)
        ranges[left - 20 : left + 20] = 0.30
        ranges[right - 20 : right + 20] = 0.90
        return ranges, ANGLES_FULL_ROTATION

    def _controller(self, *, follows: bool, floor: float = 0.12):
        tuning = NavigationTuning.load_default()
        escape = tuning.escape.model_copy(
            update={
                "escape_side_follows_committed_sign": follows,
                "escape_side_override_min_clearance_m": floor,
            },
        )
        return CollisionAvoidanceController.from_tuning(tuning, escape=escape)

    def test_the_unaided_comparison_picks_the_clearer_side(self):
        """The control: without the flag this is what the scan says."""
        ranges, angles = self._lopsided_scan()
        assert self._controller(follows=False)._k_turn_steer_sign(ranges, angles) == 1.0

    def test_off_ignores_the_routers_side_entirely(self):
        ranges, angles = self._lopsided_scan()
        controller = self._controller(follows=False)

        assert controller._k_turn_steer_sign(ranges, angles, None, -1.0) == 1.0

    def test_on_overrules_the_clearer_side_when_the_wanted_side_has_room(self):
        """REACHABILITY: the same inputs that give +1 above must give -1 here,
        or the flag is inert and no sweep of it means anything."""
        ranges, angles = self._lopsided_scan()
        # Left reads 0.30 m, comfortably over the 0.12 m floor, so the override
        # is allowed even though the right side is three times clearer.
        controller = self._controller(follows=True)

        assert controller._k_turn_steer_sign(ranges, angles, None, -1.0) == -1.0

    def test_a_shut_wanted_side_reverses_straight_instead_of_taking_the_other(self):
        """The guard that makes this safe in a 1.00 m corridor.

        This used to assert 1.0 -- the OTHER side -- on the reasoning that
        refusing to force a shut side is the safe outcome. Half right: forcing
        it would push the pillar over, but taking the other side is a
        WRONG-SIDE PASS, which ends the round outright rather than costing
        points. Both branches lose.

        0.0 is the third option: reverse straight, commit to no side, and let
        the planner choose the side on re-approach. Pinned here because the
        value is load-bearing -- compute_escape_maneuver multiplies it by
        escape_steer_scale, so only exactly 0.0 produces a straight reverse.
        """
        ranges, angles = self._lopsided_scan()
        controller = self._controller(follows=True, floor=0.50)  # 0.30 m left is now below it

        assert controller._k_turn_steer_sign(ranges, angles, None, -1.0) == 0.0

    def test_the_straight_reverse_reaches_the_maneuver_not_just_the_helper(self):
        """A 0.0 from the helper has to survive into the published command."""
        ranges, angles = self._lopsided_scan()
        forward = angle_to_index(0.0)
        ranges[forward - 10 : forward + 10] = 0.05  # a FRONT threat, so a K-turn
        controller = self._controller(follows=True, floor=0.50)

        maneuver = controller.compute_escape_maneuver(
            RiskLevel.CRITICAL, ThreatDirection.FRONT, ranges, angles, None, -1.0,
        )

        assert maneuver is not None
        assert maneuver.steering == 0.0
        assert maneuver.speed < 0  # still a reverse, just an unsteered one

    def test_on_without_a_committed_sign_leaves_the_comparison_alone(self):
        """Every tick of the Open Challenge, and most ticks of Obstacles."""
        ranges, angles = self._lopsided_scan()
        controller = self._controller(follows=True)

        assert controller._k_turn_steer_sign(ranges, angles, None, None) == 1.0

    def test_the_override_reaches_the_maneuver_not_just_the_helper(self):
        ranges, angles = self._lopsided_scan()
        forward = angle_to_index(0.0)
        ranges[forward - 10 : forward + 10] = 0.05  # a FRONT threat, so a K-turn

        held = self._controller(follows=False).compute_escape_maneuver(
            RiskLevel.CRITICAL, ThreatDirection.FRONT, ranges, angles, None, -1.0,
        )
        overridden = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, ThreatDirection.FRONT, ranges, angles, None, -1.0,
        )

        assert held is not None
        assert overridden is not None
        assert held.steering > 0
        assert overridden.steering < 0
        assert held.steering == pytest.approx(-overridden.steering)

    def test_the_floor_is_absolute_not_a_margin_between_sides(self):
        """A relative band would also overrule many of the episodes that
        choose correctly today. This pins that the comparison is against the
        wanted side's own clearance and nothing else. See
        adr:0050-escape-steering-degrees-and-committed-side."""
        controller = self._controller(follows=True, floor=0.25)
        ranges = create_numpy_scan()
        left = angle_to_index(math.pi / 2)
        right = angle_to_index(-math.pi / 2)
        # Both sides identical and both over the floor: the margin between them
        # is zero, which a relative rule would treat as "no information".
        ranges[left - 20 : left + 20] = 0.40
        ranges[right - 20 : right + 20] = 0.40

        assert controller._k_turn_steer_sign(ranges, ANGLES_FULL_ROTATION, None, -1.0) == -1.0


class TestSideCorrectionFollowsTheCommittedPassSide:
    """SIDE_CORRECTION is where the escape actually spends its time.

    MEASURED on a hardware round: most usable episodes were SIDE_CORRECTION,
    and the escape agreed with the side the router needed only rarely. Until
    this flag, ``preferred_sign`` reached only ``_k_turn_steer_sign`` and the
    two SIDE branches chose from the threat side alone -- the operator-reported
    pendulum.

    The rule REFUSES rather than redirects: on a conflict it reverses straight.
    Steering toward the wanted side on clearance alone made things worse (see
    ``_side_correction_steer_sign``). See
    adr:0050-escape-steering-degrees-and-committed-side.
    """

    def _controller(self, *, follows: bool):
        tuning = NavigationTuning.load_default()
        escape = tuning.escape.model_copy(
            update={"side_correction_follows_committed_sign": follows},
        )
        return CollisionAvoidanceController.from_tuning(tuning, escape=escape)

    def _scan_with_room_on_both_sides(self):
        """Both flanks well clear, so only the ROUTER's side can decide."""
        ranges = create_numpy_scan()
        left = angle_to_index(math.pi / 2)
        right = angle_to_index(-math.pi / 2)
        ranges[left - 20 : left + 20] = 0.60
        ranges[right - 20 : right + 20] = 0.60
        return ranges, ANGLES_FULL_ROTATION

    def test_off_keeps_the_old_steer_away_behaviour(self):
        """The control: a LEFT threat creeps forward steering right."""
        ranges, angles = self._scan_with_room_on_both_sides()
        maneuver = self._controller(follows=False).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )

        assert maneuver.steering < 0

    def test_on_refuses_when_the_router_wants_the_threats_side(self):
        """REACHABILITY: same inputs, different answer, or the flag is inert.

        preferred_sign -1.0 is LEFT and the threat is LEFT, so steering away
        would push to the wrong side of the pillar. Reverse straight instead.
        """
        ranges, angles = self._scan_with_room_on_both_sides()
        maneuver = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )

        assert maneuver.steering == 0.0

    def test_it_never_steers_toward_the_threat(self):
        """The restraint that the 12 -> 15 arm lacked, pinned for both flanks."""
        ranges, angles = self._scan_with_room_on_both_sides()
        controller = self._controller(follows=True)

        left = controller.compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )
        right = controller.compute_escape_maneuver(
            RiskLevel.CRITICAL, "right", ranges, angles, preferred_sign=1.0
        )

        assert left.steering == 0.0
        assert right.steering == 0.0

    def test_no_conflict_leaves_the_manoeuvre_untouched(self):
        """Threat left, router wants right: steering away already agrees."""
        ranges, angles = self._scan_with_room_on_both_sides()
        on = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=1.0
        )
        off = self._controller(follows=False).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=1.0
        )

        assert on.steering == off.steering

    def test_the_conflict_test_survives_the_reverse_flip(self):
        """``away_sign`` inverts once touching; the threat side does not.

        Deriving the threat side from ``away_sign`` got this backwards once, so
        it is pinned: touching on the left with the router wanting left is the
        same conflict as the creeping case above, and must still refuse.
        """
        ranges, angles = self._scan_with_room_on_both_sides()
        i = angle_to_index(math.pi / 2)
        ranges[i - 6 : i + 6] = 0.09  # touching left, above self_detection_threshold_m
        maneuver = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )

        assert maneuver.speed < 0, "reversing"
        assert maneuver.steering == 0.0

    def test_a_refusal_reverses_rather_than_creeping_forward(self):
        """The hole the earlier measurement fell through.

        A refusal that only zeroes the steering keeps ``side_correction_speed``
        (+0.1 m/s, FORWARD) whenever the flank is not already touching, so it
        deletes the steer-away reflex and creeps straight at the threat. The
        sibling reverse test forces ``already_touching`` first and therefore
        takes the reverse branch for a different reason, which is why nothing
        caught this. Here the flanks are CLEAR, so only the refusal can pick
        the speed. See adr:0050-escape-steering-degrees-and-committed-side.
        """
        ranges, angles = self._scan_with_room_on_both_sides()
        maneuver = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )

        assert maneuver.steering == 0.0
        assert maneuver.speed < 0, "a refusal must reverse, not creep forward at the threat"

    def test_a_refusal_lasts_long_enough_to_change_the_geometry(self):
        """``side_correction_s`` is short; reversing for that long barely moves.

        The pendulum this rule targets moved the range to the committed pillar
        by next to nothing across a whole episode. A refusal that retreats less
        than the planner needs to re-approach just feeds the limit cycle, so it
        takes the K-turn's duration rather than the side correction's. See
        adr:0050-escape-steering-degrees-and-committed-side.
        """
        ranges, angles = self._scan_with_room_on_both_sides()
        controller = self._controller(follows=True)
        refused = controller.compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=-1.0
        )
        untouched = controller.compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=1.0
        )

        assert refused.duration_frames > untouched.duration_frames

    def test_no_committed_side_leaves_the_manoeuvre_untouched(self):
        """Every tick of the Open Challenge takes this path."""
        ranges, angles = self._scan_with_room_on_both_sides()
        on = self._controller(follows=True).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=None
        )
        off = self._controller(follows=False).compute_escape_maneuver(
            RiskLevel.CRITICAL, "left", ranges, angles, preferred_sign=None
        )

        assert on.steering == off.steering
