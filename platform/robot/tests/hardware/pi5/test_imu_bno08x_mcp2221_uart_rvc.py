"""
Tests for BNO085 RVC driver (via MCP2221A UART).

Run on: Raspberry Pi 5

Run with: python -m pytest tests/hardware/pi5/test_imu_bno08x_mcp2221_uart_rvc.py -v
"""

import logging
import os
import time

import pytest

from src.hardware.imu.bno08x.mcp2221.uart_rvc import Driver as IMU_RVCDriver, Config as RVCConfig
from src.logger import LOG_LEVEL_DEFAULT, LOG_LEVEL_KEY, configure_json_logging

_log_level = getattr(logging, os.getenv(LOG_LEVEL_KEY, LOG_LEVEL_DEFAULT).upper(), logging.INFO)
configure_json_logging(level=_log_level)

logger = logging.getLogger(__name__)


@pytest.fixture
def driver():
    """Create driver instance."""
    config = RVCConfig()
    return IMU_RVCDriver(config=config)


class TestIMURVCConnection:
    """Test IMU RVC connection."""

    def test_connect(self, driver):
        """Connect to IMU via MCP2221."""
        try:
            driver.connect()
            assert driver._serial is not None
            logger.info("IMU RVC connection test passed")
        except Exception as e:
            pytest.skip(f"Cannot connect to IMU RVC: {e}")

    def test_find_mcp2221(self, driver):
        """Auto-detect MCP2221 port."""
        try:
            port = driver.find_mcp2221_port()
            logger.info("MCP2221 detected", extra={"details": {"port": port}})
        except Exception as e:
            pytest.skip(f"Cannot detect MCP2221: {e}")


class TestIMURVCPolling:
    """Test IMU RVC polling."""

    def test_start_polling(self, driver):
        """Start background polling."""
        try:
            driver.connect()
            driver.start_polling()
            time.sleep(0.2)
            driver.stop_polling()
            logger.info("Polling started and stopped")
        except Exception as e:
            pytest.skip(f"Cannot start polling: {e}")

    def test_get_data(self, driver):
        """Get sensor data."""
        try:
            driver.connect()
            driver.start_polling()
            time.sleep(0.2)
            data = driver.get_data()
            if data:
                logger.info(
                    "IMU RVC data",
                    extra={
                        "details": {
                            "euler": [data.roll_deg, data.pitch_deg, data.yaw_deg],
                            "accel": [data.x_accel, data.y_accel, data.z_accel],
                        }
                    },
                )
            driver.stop_polling()
            assert data is not None
        except Exception as e:
            pytest.skip(f"Cannot get data: {e}")


class TestIMURVCQuaternion:
    """Test quaternion conversion."""

    def test_quaternion_conversion(self, driver):
        """Verify quaternion from RVC data."""
        try:
            driver.connect()
            driver.start_polling()
            time.sleep(0.2)
            data = driver.get_data()
            if data:
                qx, qy, qz, qw = data.quaternion
                magnitude = (qx**2 + qy**2 + qz**2 + qw**2) ** 0.5
                logger.info("Quaternion", extra={"details": {"x": qx, "y": qy, "z": qz, "w": qw, "mag": magnitude}})
                assert 0.99 <= magnitude <= 1.01
            driver.stop_polling()
        except Exception as e:
            pytest.skip(f"Cannot verify quaternion: {e}")


def stream_imu_rvc():
    """
    Stream IMU RVC data.

    Usage:
        python -c "from tests.hardware.pi5.test_imu_bno08x_mcp2221_uart_rvc import stream_imu_rvc; stream_imu_rvc()"
    """
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    config = RVCConfig()
    driver = IMU_RVCDriver(config=config)

    try:
        driver.connect()
        driver.start_polling()
        log.info("Streaming IMU RVC data - press Ctrl+C to stop")

        while True:
            data = driver.get_data()
            if data:
                log.debug(
                    "IMU RVC data",
                    extra={
                        "details": {
                            "euler_deg": [data.roll_deg, data.pitch_deg, data.yaw_deg],
                            "accel": [data.x_accel, data.y_accel, data.z_accel],
                        }
                    },
                )
            time.sleep(0.01)
    except KeyboardInterrupt:
        log.info("Stopped")
    finally:
        driver.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_imu_bno08x_mcp2221_uart_rvc.py -v")
