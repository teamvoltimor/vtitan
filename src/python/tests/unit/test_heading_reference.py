"""The heading zero must be taken at the start button, not at node startup.

The BNO085 in UART-RVC mode reports yaw relative to power-on with no absolute
reference, so whatever orientation the robot is in when the first IMU message
arrives becomes "forward". In practice the robot is powered up, carried to the
track and set down, which rotates it arbitrarily between those two moments.

Without a re-zero the world heading is wrong by the whole transport rotation --
potentially 90 or 180 degrees, against a budget where 5 degrees already costs 8
of 28 fixtures.
"""

from __future__ import annotations

import math

import pytest
from shared.domain.models import IMUReading

from src.state_machine.estimator import StateEstimator

_START_YAW = math.pi / 2


def _imu(yaw_deg: float) -> IMUReading:
    return IMUReading(yaw=math.radians(yaw_deg), pitch=0.0, roll=0.0)


def _estimator() -> StateEstimator:
    return StateEstimator(start_x=1.5, start_y=0.45, start_yaw=_START_YAW)


class TestHeadingReference:
    def test_first_reading_defines_forward(self) -> None:
        """Baseline: the initial reading is taken as the starting heading."""
        est = _estimator()
        est.update_imu(_imu(37.0))
        assert est.estimate_pose().yaw == pytest.approx(_START_YAW)

    def test_rotation_after_the_reference_is_tracked(self) -> None:
        est = _estimator()
        est.update_imu(_imu(37.0))
        est.update_imu(_imu(57.0))
        assert est.estimate_pose().yaw == pytest.approx(_START_YAW + math.radians(20.0))

    def test_transport_rotation_corrupts_heading_without_a_reset(self) -> None:
        """The bug this exists to prevent.

        The node starts while the robot is held at some arbitrary angle, then it
        is carried to the track and set down 90 degrees away. With no re-zero
        the robot believes it is pointing 90 degrees off its true heading.
        """
        est = _estimator()
        est.update_imu(_imu(0.0))  # node start, robot in hand
        est.update_imu(_imu(90.0))  # set down on the track, rotated in transit
        assert est.estimate_pose().yaw == pytest.approx(_START_YAW + math.radians(90.0))

    def test_reset_at_button_press_restores_the_true_heading(self) -> None:
        est = _estimator()
        est.update_imu(_imu(0.0))  # node start, robot in hand
        est.update_imu(_imu(90.0))  # carried to the track and set down

        est.reset_heading_reference()  # start button
        est.update_imu(_imu(90.0))

        assert est.estimate_pose().yaw == pytest.approx(_START_YAW)

    def test_reset_survives_a_full_reversal(self) -> None:
        """Being carried backwards to the track is the worst realistic case."""
        est = _estimator()
        est.update_imu(_imu(0.0))
        est.update_imu(_imu(180.0))

        est.reset_heading_reference()
        est.update_imu(_imu(180.0))

        assert est.estimate_pose().yaw == pytest.approx(_START_YAW)

    def test_rotation_after_the_reset_is_still_tracked(self) -> None:
        """Re-zeroing must not freeze the heading."""
        est = _estimator()
        est.update_imu(_imu(0.0))
        est.reset_heading_reference()
        est.update_imu(_imu(140.0))  # new reference
        est.update_imu(_imu(155.0))  # turned 15 degrees while racing

        assert est.estimate_pose().yaw == pytest.approx(_START_YAW + math.radians(15.0))

    def test_position_is_untouched_by_a_heading_reset(self) -> None:
        est = _estimator()
        est.update_position(2.0, 1.0)
        est.reset_heading_reference()
        pose = est.estimate_pose()
        assert (pose.x, pose.y) == (2.0, 1.0)

    def test_heading_falls_back_to_start_yaw_between_reset_and_next_reading(self) -> None:
        """No stale relative angle may survive the reset."""
        est = _estimator()
        est.update_imu(_imu(0.0))
        est.update_imu(_imu(90.0))
        est.reset_heading_reference()
        assert est.estimate_pose().yaw == pytest.approx(_START_YAW)

    def test_reset_clears_a_leftover_direction_reassumption_correction(self) -> None:
        """2026-08-04: a CW race right after a CCW one started at ~0 deg
        instead of ~180 deg on real hardware.

        apply_yaw_correction (a full, up-to-pi jump applied when blind
        direction inference overturns the assumed direction -- see
        track_navigator_node._commit_direction) writes to the same
        _yaw_correction reset_heading_reference must clear, or the next
        race silently inherits the previous race's direction-convention
        correction on top of its own (possibly different) start_yaw.
        """
        est = _estimator()
        est.update_imu(_imu(0.0))
        est.apply_yaw_correction(math.pi)  # e.g. the previous race's CW->CCW jump

        est.reset_heading_reference()
        est.update_imu(_imu(0.0))

        assert est.estimate_pose().yaw == pytest.approx(_START_YAW)
