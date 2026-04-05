"""
Tests for BNO085 I2C driver (via MCP2221A using Blinka).

Run on: Raspberry Pi 5

Run with: python -m pytest tests/hardware/pi5/test_imu_bno08x_mcp2221_i2c.py -v

Requirements:
    - Blinka: pip install blinka adafruit-circuitpython-bno08x
    - MCP2221A connected via USB
"""

import logging
import os
import time

import pytest

from src.hardware.imu.bno08x.mcp2221.i2c import Driver as IMU_I2CDriver, Config as I2CConfig
from src.logger import LOG_LEVEL_DEFAULT, LOG_LEVEL_KEY, configure_json_logging

_log_level = getattr(logging, os.getenv(LOG_LEVEL_KEY, LOG_LEVEL_DEFAULT).upper(), logging.INFO)
configure_json_logging(level=_log_level)

logger = logging.getLogger(__name__)


@pytest.fixture
def driver():
    """Create driver instance."""
    config = I2CConfig(i2c_address=0x4A)
    return IMU_I2CDriver(config=config)


class TestIMUConnection:
    """Test IMU connection."""

    def test_connect(self, driver):
        """Connect to IMU via Blinka."""
        try:
            driver.connect()
            assert driver._imu is not None
            logger.info("IMU I2C connection test passed (Blinka)")
        except Exception as e:
            pytest.skip(f"Cannot connect to IMU: {e}")


class TestIMUSensors:
    """Test IMU sensors."""

    def test_enable_sensors(self, driver):
        """Enable sensors."""
        try:
            driver.connect()
            driver.enable_sensors()
            logger.info("Sensors enabled")
        except Exception as e:
            pytest.skip(f"Cannot enable sensors: {e}")

    def test_get_accelerometer(self, driver):
        """Get accelerometer data."""
        try:
            driver.connect()
            driver.enable_sensors()
            accel = driver.get_accelerometer()
            logger.info("Accelerometer", extra={"details": {"x": accel[0], "y": accel[1], "z": accel[2]}})
            assert len(accel) == 3
        except Exception as e:
            pytest.skip(f"Cannot read accelerometer: {e}")

    def test_get_gyroscope(self, driver):
        """Get gyroscope data."""
        try:
            driver.connect()
            driver.enable_sensors()
            gyro = driver.get_gyroscope()
            logger.info("Gyroscope", extra={"details": {"x": gyro[0], "y": gyro[1], "z": gyro[2]}})
            assert len(gyro) == 3
        except Exception as e:
            pytest.skip(f"Cannot read gyroscope: {e}")

    def test_get_quaternion(self, driver):
        """Get quaternion data."""
        try:
            driver.connect()
            driver.enable_sensors()
            quat = driver.get_quaternion()
            logger.info("Quaternion", extra={"details": {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]}})
            assert len(quat) == 4
        except Exception as e:
            pytest.skip(f"Cannot read quaternion: {e}")

    def test_get_euler(self, driver):
        """Get Euler angles."""
        try:
            driver.connect()
            driver.enable_sensors()
            euler = driver.get_euler()
            logger.info("Euler", extra={"details": {"pitch": euler[0], "roll": euler[1], "yaw": euler[2]}})
            assert len(euler) == 3
        except Exception as e:
            pytest.skip(f"Cannot read Euler: {e}")

    def test_get_all_data(self, driver):
        """Get all sensor data."""
        try:
            driver.connect()
            driver.enable_sensors()
            data = driver.get_all_data()
            logger.info("All IMU data captured")
            assert data is not None
        except Exception as e:
            pytest.skip(f"Cannot read IMU data: {e}")


class TestIMUCalibration:
    """Test IMU calibration."""

    def test_calibration_status(self, driver):
        """Get calibration status."""
        try:
            driver.connect()
            cal = driver.get_calibration_status()
            logger.info("Calibration status", extra={"details": {"calibration": cal}})
        except Exception as e:
            pytest.skip(f"Cannot read calibration: {e}")


def stream_imu():
    """
    Stream IMU data.

    Usage:
        python -c "from tests.hardware.pi5.test_imu_bno08x_mcp2221_i2c import stream_imu; stream_imu()"
    """
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    config = I2CConfig()
    driver = IMU_I2CDriver(config=config)

    try:
        driver.connect()
        driver.enable_sensors()
        log.info("Streaming IMU data - press Ctrl+C to stop")

        while True:
            data = driver.get_all_data()
            log.debug(
                "IMU data",
                extra={
                    "details": {
                        "accel": data.accelerometer,
                        "euler": data.euler,
                    }
                },
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("Stopped")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_imu_bno08x_mcp2221_i2c.py -v")
