"""
Hardware validation tests for LEGO motors via Build HAT.

Run on: Raspberry Pi Zero (connected to Build HAT)

Run with: python -m pytest tests/hardware/pi_zero/test_motors.py -v

Requirements:
    - Raspberry Pi Zero with Build HAT attached
    - LEGO Large motor on port A (steering)
    - LEGO Large motor on port B (drive)
    - Build HAT library: pip install buildhat
"""

import logging
import os

import pytest

from src.hardware.motors.build_hat import (
    Config as MotorConfig,
    Driver as BuildHatDriver,
)
from src.hardware.motors.config import MotorDriveConfig, MotorSteeringConfig
from src.logger import configure_json_logging

configure_json_logging()

logger = logging.getLogger(__name__)


@pytest.fixture(autouse=True)
def check_hardware(driver):
    try:
        driver.connect()
    except Exception as e:
        pytest.skip(f"Hardware not available: {e}")


@pytest.fixture()
def driver():
    """Create driver instance.

    Limits and center are placeholders — calibrate them on-device with
    ``find_limits_interactive()`` before relying on steering range.
    """
    config = MotorConfig(
        steering=MotorSteeringConfig(
            port="A",
            left_limit_angle=-83.0,
            center_angle=0.0,
            right_limit_angle=22.0,
        ),
        drive=MotorDriveConfig(
            port="B",
            min_speed=0,
            max_speed=100,
            default_speed=15,
        ),
        test_duration=1,
    )
    return BuildHatDriver(config=config)


class TestMotorConnection:
    """Test motor connectivity."""

    def test_connect(self, driver):
        """Connect to motors."""
        driver.connect()
        assert driver._steering is not None
        assert driver._drive is not None
        logger.info("Motor connection test passed")


class TestMotorPosition:
    """Test reading motor positions."""

    def test_get_steering_position(self, driver):
        """Read steering position."""
        driver.connect()
        position = driver.get_steering_position()
        logger.info("Steering position", extra={"details": {"position_deg": position}})
        assert isinstance(position, (int, float))

    def test_get_drive_position(self, driver):
        """Read drive position."""
        driver.connect()
        position = driver.get_drive_position()
        logger.info("Drive position", extra={"details": {"position_deg": position}})
        assert isinstance(position, (int, float))

    def test_get_steering_speed(self, driver):
        """Read steering speed."""
        driver.connect()
        speed = driver.get_steering_speed()
        logger.info("Steering speed", extra={"details": {"speed_deg_s": speed}})
        assert isinstance(speed, (int, float))


class TestDriveMotor:
    """Test drive motor movement."""

    def test_run_drive_forward(self, driver):
        """Run drive motor forward."""
        driver.connect()
        driver.run_drive_forward(speed=15)
        import time

        time.sleep(1.0)
        driver.stop_drive()
        logger.info("Drive forward test passed")

    def test_run_drive_reverse(self, driver):
        """Run drive motor reverse."""
        driver.connect()
        driver.run_drive_reverse(speed=15)
        import time

        time.sleep(1.0)
        driver.stop_drive()
        logger.info("Drive reverse test passed")


class TestSteeringControl:
    """Test steering control."""

    def test_center_steering(self, driver):
        """Center steering."""
        driver.connect()
        driver.center_steering()
        import time

        time.sleep(1.0)
        position = driver.get_steering_position()
        logger.info("Steering centered", extra={"details": {"position_deg": position}})
        assert abs(position) < 10.0


class TestCalibration:
    """Test calibration."""

    def test_load_calibration(self, driver):
        """Load calibration from file."""
        calib = driver.load_calibration()
        logger.info("Calibration loaded", extra={"details": {"calibration": calib.__dict__}})
        assert calib is not None


def find_limits_interactive():
    """
    Interactive script to find steering limits.

    Usage:
        python -c "from tests.hardware.pi_zero.test_motors import find_limits_interactive; find_limits_interactive()"
    """
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    config = MotorConfig(
        steering=MotorSteeringConfig(
            port="A",
            left_limit_angle=-83.0,
            center_angle=0.0,
            right_limit_angle=22.0,
        ),
        drive=MotorDriveConfig(
            port="B",
            min_speed=0,
            max_speed=100,
            default_speed=15,
        ),
        test_duration=1,
    )
    driver = BuildHatDriver(config=config)
    driver.connect()

    log.info("Voldemorbot v2 Steering Limit Finder")

    log.info("Step 1: Center wheels manually, press Enter")
    input()
    driver.center_steering()
    import time

    time.sleep(1.0)

    log.info("Step 2: Testing Right Limit - wheels will turn right")
    input()
    driver._steering.start(15)
    time.sleep(2.0)
    driver._steering.stop()
    right_limit = driver.get_steering_position()
    log.info("Right limit", extra={"details": {"right_limit_deg": right_limit}})

    log.info("Step 3: Testing Left Limit - wheels will turn left")
    input()
    driver._steering.start(-15)
    time.sleep(2.0)
    driver._steering.stop()
    left_limit = driver.get_steering_position()
    log.info("Left limit", extra={"details": {"left_limit_deg": left_limit}})

    log.info(
        "Results",
        extra={
            "details": {
                "left_limit_deg": left_limit,
                "right_limit_deg": right_limit,
                "range_deg": right_limit - left_limit,
            },
        },
    )

    save = input("\nSave to calibration.json? (y/n): ")
    if save.lower() == "y":
        driver.save_calibration(left_limit, right_limit)
        log.info("Calibration saved")
    else:
        log.info("Calibration not saved")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi_zero/test_motors.py -v")
    logger.info("Or run: find_limits_interactive()")
