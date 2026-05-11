"""
Tests for CameraDriver (camera hardware driver).

Run on: Raspberry Pi 5

Run with: python -m pytest tests/hardware/pi5/test_camera.py -v
"""

import logging
import time

import pytest

from src.hardware.camera.rpi.camera_module_3 import (
    Config as CameraConfig,
    Driver as CameraDriver,
)
from src.logger import LOG_LEVEL, configure_json_logging

_log_level = getattr(logging, LOG_LEVEL.value.upper(), logging.INFO)
configure_json_logging(level=_log_level)

logger = logging.getLogger(__name__)


@pytest.fixture()
def driver():
    """Create driver instance."""
    config = CameraConfig(device="/dev/video0", width=1536, height=864, fps=30)
    return CameraDriver(config=config)


class TestCameraConnection:
    """Test camera connection."""

    def test_open(self, driver):
        """Open camera."""
        try:
            driver.open()
            assert driver._capture is not None
            logger.info("Camera open test passed")
        except (RuntimeError, OSError, ValueError) as e:
            pytest.skip(f"Cannot open camera: {e}")
        finally:
            driver.close()

    def test_capture_frame(self, driver):
        """Capture a frame."""
        try:
            driver.open()
            frame = driver.capture_frame()
            logger.info("Frame captured", extra={"details": {"width": frame.width, "height": frame.height}})
            assert frame.frame is not None
        except (RuntimeError, OSError, ValueError) as e:
            pytest.skip(f"Cannot capture frame: {e}")
        finally:
            driver.close()


class TestCameraPerformance:
    """Test camera performance."""

    def test_get_resolution(self, driver):
        """Get resolution."""
        try:
            driver.open()
            width, height = driver.get_resolution()
            logger.info("Resolution", extra={"details": {"width": width, "height": height}})
        except (RuntimeError, OSError, ValueError) as e:
            pytest.skip(f"Cannot get resolution: {e}")
        finally:
            driver.close()

    def test_measure_fps(self, driver):
        """Measure FPS."""
        try:
            driver.open()
            fps = driver.measure_fps(num_frames=30)
            logger.info("FPS measured", extra={"details": {"fps": fps}})
            assert fps > 10
        except (RuntimeError, OSError, ValueError) as e:
            pytest.skip(f"Cannot measure FPS: {e}")
        finally:
            driver.close()

    def test_measure_latency(self, driver):
        """Measure latency."""
        try:
            driver.open()
            latency = driver.measure_latency(num_frames=10)
            logger.info("Latency measured", extra={"details": {"latency_ms": latency * 1000}})
            assert latency < 0.1
        except (RuntimeError, OSError, ValueError) as e:
            pytest.skip(f"Cannot measure latency: {e}")
        finally:
            driver.close()


def preview_camera():
    """
    Open camera preview.

    Usage:
        python -c "from tests.hardware.pi5.test_camera import preview_camera; preview_camera()"
    """
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    import cv2

    config = CameraConfig(device="/dev/video0", fps=30)

    try:
        driver.open()
        log.info("Preview started - press q to exit")

        start_time = time.time()
        while time.time() - start_time < 10:
            frame = driver.capture_frame()
            cv2.imshow("Preview", frame.frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except (RuntimeError, OSError, ValueError) as e:
        log.error("Preview error: %s", e)
    finally:
        driver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_camera.py -v")
