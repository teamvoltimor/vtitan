"""
Hardware driver for Slamtec RPLiDAR C1.

Used by: Raspberry Pi 5
"""

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

from src.env import EnvVar
from src.logger import configure_json_logging

configure_json_logging()


LIDAR_PORT = EnvVar[str](key="LIDAR_PORT", default="/dev/ttyUSB0")
LIDAR_BAUDRATE = EnvVar[int](key="LIDAR_BAUDRATE", default=115200, cast=int)
LIDAR_TIMEOUT = EnvVar[float](key="LIDAR_TIMEOUT", default=1.0, cast=float)


@dataclass
class LidarConfig:
    """LIDAR configuration."""

    port: str = LIDAR_PORT.value
    baudrate: int = LIDAR_BAUDRATE.value
    timeout: float = LIDAR_TIMEOUT.value


@dataclass
class LidarPoint:
    """Single LIDAR scan point."""

    angle: float
    distance: float
    quality: int


class RPLidarDriver:
    """Driver for RPLiDAR C1."""

    def __init__(self, config: Optional[LidarConfig] = None):
        self.config = config or LidarConfig()
        self._lidar = None
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to LIDAR."""
        import rplidar

        self.logger.info("Connecting to RPLiDAR", extra={"details": {"port": self.config.port}})
        self._lidar = rplidar.RPLidar(self.config.port)
        self.logger.info("Connected to RPLiDAR")

    @property
    def lidar(self):
        """Get LIDAR instance."""
        if self._lidar is None:
            self.connect()
        return self._lidar

    def get_info(self) -> dict:
        """Get LIDAR device info."""
        return self.lidar.get_info()

    def get_health(self) -> Tuple[int, str]:
        """Get LIDAR health status."""
        health = self.lidar.get_health()
        return health.status, health.message

    def capture_scan(self) -> List[LidarPoint]:
        """Capture a single 360° scan."""
        self.logger.debug("Capturing scan")
        scan_iterator = self.lidar.iter_scans(max_buf_measles=500)
        scan = next(scan_iterator)

        points = [LidarPoint(angle=p.angle, distance=p.distance, quality=p.quality) for p in scan]
        self.logger.info("Scan captured", extra={"details": {"num_points": len(points)}})
        return points

    def get_angles(self) -> Tuple[float, float]:
        """Get min and max angles in scan."""
        points = self.capture_scan()
        angles = [p.angle for p in points]
        return min(angles), max(angles)

    def get_min_distance(self) -> float:
        """Get minimum readable distance."""
        points = self.capture_scan()
        distances = [p.distance for p in points if p.distance > 0]
        return min(distances) if distances else float("inf")

    def get_max_distance(self) -> float:
        """Get maximum readable distance."""
        points = self.capture_scan()
        return max(p.distance for p in points)

    def get_front_distance(self) -> Optional[float]:
        """Get distance to front (0°)."""
        points = self.capture_scan()
        front_points = [p for p in points if abs(p.angle) < 5]
        if front_points:
            return min(p.distance for p in front_points)
        return None

    def disconnect(self) -> None:
        """Disconnect from LIDAR."""
        if self._lidar:
            self._lidar.disconnect()
            self.logger.info("Disconnected from RPLiDAR")
