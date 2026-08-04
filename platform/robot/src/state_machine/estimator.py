"""State estimator combining IMU heading with a LIDAR-derived world position.

Real hardware has no wheel odometry, so position comes from
:class:`~src.navigation.localization.LidarLocalizer` (an absolute fix against
known track wall geometry, not a relative delta), while heading comes from the
IMU. This replaces an earlier IMU+wheel-odometry complementary filter design;
that filter is not reinstated here because reintroducing an unfused odometry
input would resurrect the same silent-drift risk that motivated the LIDAR fix.
"""

import math

from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import IMUReading, Pose


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.remainder(angle, 2 * math.pi)


YAW_CORRECTION_GAIN = NavigationTuning.load_default().state_estimator.YAW_CORRECTION_GAIN
"""Fraction of the wall-vs-IMU heading discrepancy absorbed per scan.

At the C1's ~10 Hz that is a time constant near two seconds: fast enough to
absorb gyro drift and scale error, which accumulate over a whole round, and
slow enough that the wall estimate's own per-sample noise (measured 0.35 deg
mean, 1.4 deg worst) is averaged away rather than steered on.
"""


class StateEstimator:
    """Combines IMU heading and LIDAR-fixed position into a world-frame Pose."""

    def __init__(self, start_x: float, start_y: float, start_yaw: float) -> None:
        """Initialize the state estimator.

        Args:
            start_x: Starting X coordinate in world frame (also the position
                estimate until the first LIDAR fix arrives).
            start_y: Starting Y coordinate in world frame.
            start_yaw: Starting Yaw (heading) in world frame (also the heading
                estimate until the first IMU reading arrives).
        """
        self._start_yaw = start_yaw
        self._imu_yaw_offset: float | None = None
        self._relative_imu_yaw: float | None = None

        self._x = start_x
        self._y = start_y
        self._yaw_correction = 0.0

    def update_imu(self, reading: IMUReading) -> None:
        """Process incoming IMU orientation data."""
        if self._imu_yaw_offset is None:
            self._imu_yaw_offset = reading.yaw

        self._relative_imu_yaw = wrap_angle(reading.yaw - self._imu_yaw_offset)

    def reset_heading_reference(self) -> None:
        """Re-zero the heading reference against the next IMU reading, and
        clear any accumulated correction.

        The BNO085 in UART-RVC mode reports yaw relative to power-on and has no
        absolute reference, so the offset latched by the first reading is only
        meaningful if the robot was already sitting on the track, aligned, when
        the node started. It usually is not: the robot is powered up, carried to
        the track and set down, which can rotate it arbitrarily -- easily 90 or
        180 degrees, against a heading budget where 5 degrees already costs
        real pass rate.

        Also zeroes ``_yaw_correction`` -- both the slow wall-heading
        complementary-filter drift (``correct_yaw``) and the full
        direction-reassumption jump (``apply_yaw_correction``, up to +/-pi)
        accumulate there, and neither belongs to a race that hasn't started
        yet. Without this, a race that overturns the assumed direction (see
        ``apply_yaw_correction``) leaves that correction sitting in
        ``_yaw_correction`` after the race ends; the *next* race's reset()
        re-zeros the IMU offset and rebuilds the path for its own (possibly
        different) direction, but silently kept the previous race's leftover
        correction, netting out to the wrong race's heading convention.
        Confirmed on real hardware 2026-08-04: a CW race immediately
        following a CCW one started at pose_yaw ~0 deg (CCW's convention)
        instead of ~180 deg (CW's), because the CCW race's -pi correction
        was still sitting in ``_yaw_correction`` (see
        docs/known-issues-backlog.md).

        Call this at the moment the robot is known to be in its starting pose,
        which is the start-button press. Everything before then is transport.
        """
        self._imu_yaw_offset = None
        self._relative_imu_yaw = None
        self._yaw_correction = 0.0

    def update_position(self, x: float, y: float) -> None:
        """Process an absolute world-frame position fix (from LIDAR localization)."""
        self._x = x
        self._y = y

    def reset_position(self, x: float, y: float) -> None:
        """Re-seed the position estimate at the start of a new race.

        The state machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING
        purely from the button, with no process restart (see
        ``reset_heading_reference``'s docstring for the same fact about
        heading) -- so without this, a new race's very first tick starts from
        wherever the *previous* race's ``update_position`` last left ``_x``/
        ``_y``, not from the new race's actual starting pose. If the LIDAR
        localizer drifted during that previous race (its coarse-to-fine grid
        search is only ever a local correction -- see its own docstring --
        so a large true displacement between ticks, e.g. during an escape
        maneuver, can desync the search from truth with no way back), every
        subsequent race inherits and compounds that drift instead of starting
        clean. Confirmed on real hardware 2026-08-04: two consecutive races
        the same day showed pose_x/pose_y in the hundreds of metres,
        continuous across the race boundary, on a track no larger than 3m
        square (see docs/known-issues-backlog.md).

        Args:
            x: The new race's starting X coordinate (world frame).
            y: The new race's starting Y coordinate (world frame).
        """
        self._x = x
        self._y = y

    def correct_yaw(self, measured_yaw: float, gain: float = YAW_CORRECTION_GAIN) -> None:
        """Pull the heading estimate toward an absolute measurement of it.

        A complementary filter, and the only thing that bounds heading. The IMU
        supplies the high-frequency truth -- it is fast, smooth and locally
        excellent -- but a 6-axis fusion with no magnetometer has no absolute
        reference, so its error is a ramp. The walls supply the low-frequency
        truth: noisier per sample, but it does not grow with time.

        Applied as a slowly-moving offset rather than by overwriting yaw. A
        hard assignment would inject the wall estimate's per-scan noise
        straight into steering at 10 Hz, and discard the IMU's short-term
        accuracy, which is the half of the pair worth keeping.

        Args:
            measured_yaw: Absolute yaw from
                :func:`~src.navigation.wall_heading.estimate_yaw_from_walls`.
            gain: Fraction of the discrepancy absorbed per correction.
        """
        error = wrap_angle(measured_yaw - self.estimate_pose().yaw)
        self._yaw_correction = wrap_angle(self._yaw_correction + gain * error)

    def apply_yaw_correction(self, delta_rad: float) -> None:
        """Shift the heading estimate by a known, exact amount, applied in full.

        Unlike ``correct_yaw`` (a gradual complementary-filter pull toward a
        *noisy* measurement, gained down so per-scan wall noise doesn't inject
        straight into steering), this is for a *known* delta that must land
        exactly and immediately -- e.g. when blind direction inference
        overturns the direction assumed at construction. ``assumed_start_conditions``
        pairs a starting yaw with the assumed direction (the two travel-direction
        unit vectors for a section are exact opposites, so CW and CCW always
        differ by exactly pi), so discovering the assumption was wrong leaves
        the heading estimate anchored to a reference that no longer matches
        the just-corrected direction, or the just-rebuilt path -- unless
        the caller corrects it by exactly the same delta.

        Args:
            delta_rad: Signed radians to add to the current estimate.
        """
        self._yaw_correction = wrap_angle(self._yaw_correction + delta_rad)

    def estimate_pose(self) -> Pose:
        """Calculate and return the current fused Pose in the world frame."""
        if self._relative_imu_yaw is not None:
            world_yaw = wrap_angle(self._relative_imu_yaw + self._start_yaw + self._yaw_correction)
        else:
            world_yaw = wrap_angle(self._start_yaw + self._yaw_correction)

        return Pose(x=self._x, y=self._y, yaw=world_yaw)
