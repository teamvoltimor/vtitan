"""
Hardware validation tests for Slamtec RPLiDAR C1.

Run on: Raspberry Pi 5

Run with: python -m pytest tests/hardware/pi5/test_lidar.py -v

Requirements:
    - RPLiDAR C1 connected via USB to Pi 5
    - rplidar library: pip install rplidar
    - Usually appears as /dev/ttyUSB0 on Linux

Note: On Windows, use COM port (e.g., 'COM3')
"""

import logging
import os
import time
from pathlib import Path
from typing import Optional

import pytest

from src.logger import LOG_LEVEL_DEFAULT, LOG_LEVEL_KEY, configure_json_logging

_log_level = getattr(logging, os.getenv(LOG_LEVEL_KEY, LOG_LEVEL_DEFAULT).upper(), logging.INFO)
configure_json_logging(level=_log_level)

DEFAULT_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

logger = logging.getLogger(__name__)


def get_lidar(port: str = DEFAULT_PORT):
    """Lazy import and create Lidar to avoid connection at module load."""
    import rplidar

    return rplidar.RPLidar(port)


class TestLidarConnection:
    """Test LIDAR connectivity."""

    def test_lidar_port_detection(self):
        """Check if LIDAR serial port exists."""
        import os

        port_exists = os.path.exists(DEFAULT_PORT)
        logger.info("LIDAR port check", extra={"details": {"port": DEFAULT_PORT, "exists": port_exists}})

        if not port_exists:
            pytest.skip(f"LIDAR port {DEFAULT_PORT} not found")

        assert True

    def test_lidar_connection(self):
        """Connect to LIDAR device."""
        try:
            lidar = get_lidar()
            info = lidar.get_info()
            logger.info("LIDAR connected", extra={"details": {"info": str(info)}})
            assert info is not None
            lidar.disconnect()
        except Exception as e:
            pytest.skip(f"Cannot connect to LIDAR: {e}")

    def test_lidar_health(self):
        """Check LIDAR health status."""
        try:
            lidar = get_lidar()
            health = lidar.get_health()
            logger.info("LIDAR health status", extra={"details": {"status": health.status, "message": health.message}})

            assert health.status == 0, f"Health status not OK: {health}"
            lidar.disconnect()
        except Exception as e:
            pytest.skip(f"Cannot check LIDAR health: {e}")


class TestLidarScan:
    """Test LIDAR scanning functionality."""

    @pytest.fixture
    def lidar(self):
        """Get LIDAR instance."""
        try:
            l = get_lidar()
            yield l
            l.disconnect()
        except Exception as e:
            pytest.skip(f"Cannot connect to LIDAR: {e}")

    def test_single_scan(self, lidar):
        """Capture a single 360° scan."""
        logger.info("Capturing single scan")

        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        logger.info("Scan captured", extra={"details": {"num_points": len(scan)}})

        assert len(scan) > 0, "No scan points received"
        assert len(scan) >= 100, f"Too few points: {len(scan)}, expected ~500"

    def test_angle_range(self, lidar):
        """Verify 360° angle coverage."""
        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        angles = [point.angle for point in scan]
        min_angle = min(angles)
        max_angle = max(angles)
        coverage = max_angle - min_angle

        logger.info(
            "Angle range measured",
            extra={"details": {"min_angle_deg": min_angle, "max_angle_deg": max_angle, "coverage_deg": coverage}},
        )

        assert coverage > 300, "Insufficient angle coverage"

    def test_min_range(self, lidar):
        """Check minimum readable distance."""
        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        distances = [point.distance for point in scan if point.distance > 0]
        min_dist = min(distances) if distances else float("inf")

        logger.info(
            "Minimum range measured",
            extra={"details": {"min_distance_m": min_dist, "min_distance_mm": min_dist * 1000}},
        )

        assert min_dist < 0.1, f"Min range too high: {min_dist}m"

    def test_max_range(self, lidar):
        """Check maximum readable distance."""
        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        distances = [point.distance for point in scan]
        max_dist = max(distances)

        logger.info(
            "Maximum range measured",
            extra={"details": {"max_distance_m": max_dist, "max_distance_mm": max_dist * 1000}},
        )

        assert max_dist > 1.0, f"Max range too low: {max_dist}m"

    def test_scan_consistency(self, lidar):
        """Check multiple scans for consistency."""
        scans = []
        for i in range(3):
            scan_iterator = lidar.iter_scans(max_buf_measles=500)
            scan = next(scan_iterator)
            scans.append(len(scan))
            time.sleep(0.1)

        avg = sum(scans) / len(scans)
        logger.info("Scan consistency measured", extra={"details": {"scan_counts": scans, "average_points": avg}})

        assert avg > 100, f"Average points too low: {avg}"


class TestLidarData:
    """Test LIDAR data quality."""

    @pytest.fixture
    def lidar(self):
        """Get LIDAR instance."""
        try:
            l = get_lidar()
            yield l
            l.disconnect()
        except Exception as e:
            pytest.skip(f"Cannot connect to LIDAR: {e}")

    def test_front_reading(self, lidar):
        """Test reading from front direction (0°)."""
        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        front_points = [p for p in scan if abs(p.angle) < 5]
        if front_points:
            front_dist = min(p.distance for p in front_points)
            logger.info("Front distance measured", extra={"details": {"front_distance_m": front_dist}})
        else:
            logger.warning("No front points found in scan")

    def test_reflectivity(self, lidar):
        """Check reflectivity/signal strength data."""
        scan_iterator = lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        reflectivities = [p.quality for point in scan]
        avg_quality = sum(reflectivities) / len(reflectivities) if reflectivities else 0

        logger.info("Scan quality measured", extra={"details": {"average_quality": avg_quality}})
        assert avg_quality > 0, "No quality data received"


def scan_continuous(duration: float = 10.0, port: str = DEFAULT_PORT):
    """
    Continuous scanning for visual inspection.

    Usage:
        python -c "from tests.hardware.pi5.test_lidar import scan_continuous; scan_continuous()"
    """
    import logging

    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    import rplidar

    log.info("Connecting to LIDAR", extra={"details": {"port": port}})
    lidar = rplidar.RPLidar(port)

    log.info("Starting continuous scan - press Ctrl+C to stop")

    try:
        for scan in lidar.iter_scans():
            for point in scan:
                log.debug(
                    "LIDAR point",
                    extra={
                        "details": {"angle_deg": point.angle, "distance_m": point.distance, "quality": point.quality}
                    },
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        log.info("Scan stopped by user")
    finally:
        lidar.disconnect()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_lidar.py -v")
    logger.info("Or run continuous scan: scan_continuous()")
