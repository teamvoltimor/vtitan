"""Reusable test fixture builders for robot tests.

Combines shared domain models with test-specific builders to reduce duplication
and make test intent clearer. Builders are fluent for easy test setup.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from shared.config.constants import ParkingLotSpecs, RobotSpecs, TrackDimensions
from shared.domain.models import BlockPosition, Detection, IMUReading, ParkingLot, Pose, Waypoint

from src.simulation.kinematics import AckermannState
from tests.test_constants import ANGLES_FULL_ROTATION, NUM_RAYS

if TYPE_CHECKING:
    from collections.abc import Callable

    from shared.domain.enums import Section

    from src.navigation.ports import DriveCommand


@dataclass(frozen=True)
class LidarScan:
    """A complete LIDAR scan with ranges and angles."""

    ranges: list[float]
    angles: list[float]
    num_rays: int = 360

    def __post_init__(self):
        if len(self.ranges) != len(self.angles):
            msg = f"ranges ({len(self.ranges)}) and angles ({len(self.angles)}) must match"
            raise ValueError(msg)


class LidarScanBuilder:
    """Fluent builder for LIDAR scans in various corridor geometries.

    Example:
        scan = LidarScanBuilder().corridor(left_m=0.5, right_m=0.5, ahead_m=1.0).build()
    """

    def __init__(self, num_rays: int = 360):
        self.num_rays = num_rays
        self._bearing_to_range: Callable[[float], float] = lambda _: 10.0

    def _angles(self) -> list[float]:
        """Generate bearing angles for all rays."""
        return [-math.pi + i * 2 * math.pi / self.num_rays for i in range(self.num_rays)]

    def clear_path(self, far_distance_m: float = 10.0) -> LidarScanBuilder:
        """Clear path in all directions."""
        self._bearing_to_range = lambda _: far_distance_m
        return self

    def corridor(
        self, left_m: float, right_m: float, ahead_m: float
    ) -> LidarScanBuilder:
        """Square corridor: walls left/right, open ahead.

        Args:
            left_m: Distance to left wall
            right_m: Distance to right wall
            ahead_m: Distance forward before wall
        """

        def bearing_to_range(bearing: float) -> float:
            if abs(bearing) < math.pi / 4:
                return ahead_m / max(math.cos(bearing), 1e-3)
            if bearing > 0:
                return left_m / max(math.sin(bearing), 1e-3)
            return right_m / max(math.sin(-bearing), 1e-3)

        self._bearing_to_range = bearing_to_range
        return self

    def obstacle_ahead(self, distance_m: float, angle_width_deg: float = 20) -> LidarScanBuilder:
        """Obstacle directly ahead with fallback to far.

        Args:
            distance_m: Distance to obstacle
            angle_width_deg: Half-width of obstacle arc
        """
        width_rad = math.radians(angle_width_deg)

        def bearing_to_range(bearing: float) -> float:
            if abs(bearing) < width_rad:
                return distance_m
            return 10.0

        self._bearing_to_range = bearing_to_range
        return self

    def wall_close(
        self,
        near_m: float = 0.05,
        angle_width_deg: float = 10,
        far_m: float = 1.5,
    ) -> LidarScanBuilder:
        """Wall very close in a sector, moderate distance elsewhere.

        Args:
            near_m: Distance to close wall
            angle_width_deg: Half-width of close wall sector
            far_m: Distance elsewhere
        """
        width_rad = math.radians(angle_width_deg)

        def bearing_to_range(bearing: float) -> float:
            if abs(bearing) < width_rad:
                return near_m
            return far_m

        self._bearing_to_range = bearing_to_range
        return self

    def oblique_corridor(
        self,
        corridor_bearing_rad: float,
        corridor_distance_m: float,
        offset_from_wall_m: float,
        wall_width_deg: float = 60,
    ) -> LidarScanBuilder:
        """Corridor at an angle to the robot's heading.

        Simulates the oblique chassis scenario: robot is offset from a wall
        at an angle, corridor axis is at corridor_bearing_rad.

        Args:
            corridor_bearing_rad: Where the corridor actually runs
            corridor_distance_m: How far ahead the corridor is open
            offset_from_wall_m: How far the robot is from the nearest wall
            wall_width_deg: Half-width of wall bearing arc
        """
        wall_width_rad = math.radians(wall_width_deg)

        def bearing_to_range(bearing: float) -> float:
            if abs(bearing - corridor_bearing_rad) < math.radians(20):
                return corridor_distance_m
            if abs(bearing) < math.radians(wall_width_deg / 2):
                return offset_from_wall_m / max(math.sin(corridor_bearing_rad), 1e-3)
            return 0.9

        self._bearing_to_range = bearing_to_range
        return self

    def build(self) -> LidarScan:
        """Build the scan."""
        angles = self._angles()
        ranges = [self._bearing_to_range(a) for a in angles]
        return LidarScan(ranges=ranges, angles=angles, num_rays=self.num_rays)


@dataclass(frozen=True)
class MotionState:
    """Complete motion state combining pose and velocity in world frame.

    Wraps Pose and Ackermann state for navigation tests.
    """

    pose: Pose
    ackermann: AckermannState

    @property
    def position(self) -> tuple[float, float]:
        return (self.pose.x, self.pose.y)

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def yaw(self) -> float:
        return self.pose.yaw


class MotionStateBuilder:
    """Fluent builder for robot motion states."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.steer = 0.0
        self.speed = 0.0

    def at(self, x: float, y: float, yaw: float = 0.0) -> MotionStateBuilder:
        """Set position and heading."""
        self.x = x
        self.y = y
        self.yaw = yaw
        return self

    def moving(self, speed_mps: float, steer_norm: float = 0.0) -> MotionStateBuilder:
        """Set motion (speed and steering angle)."""
        self.speed = speed_mps
        self.steer = steer_norm * RobotSpecs.MAX_STEERING_ANGLE
        return self

    def build(self) -> MotionState:
        """Build the motion state."""
        pose = Pose(x=self.x, y=self.y, yaw=self.yaw)
        ackermann = AckermannState(
            x=self.x, y=self.y, yaw=self.yaw, steer=self.steer, speed=self.speed
        )
        return MotionState(pose=pose, ackermann=ackermann)


@dataclass(frozen=True)
class WaypointPath:
    """A sequence of waypoints forming a path."""

    waypoints: list[Waypoint]

    def __len__(self) -> int:
        return len(self.waypoints)

    def __getitem__(self, idx: int) -> Waypoint:
        return self.waypoints[idx]


class WaypointPathBuilder:
    """Fluent builder for waypoint paths."""

    def __init__(self):
        self.waypoints: list[Waypoint] = []

    def add(self, x: float, y: float) -> WaypointPathBuilder:
        """Add a waypoint."""
        self.waypoints.append(Waypoint(x=x, y=y))
        return self

    def line(self, start_x: float, start_y: float, end_x: float, end_y: float, num_points: int = 5) -> WaypointPathBuilder:
        """Add a line of waypoints from start to end."""
        xs = np.linspace(start_x, end_x, num_points)
        ys = np.linspace(start_y, end_y, num_points)
        for x, y in zip(xs, ys):
            self.waypoints.append(Waypoint(x=float(x), y=float(y)))
        return self

    def arc(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        start_angle_rad: float,
        end_angle_rad: float,
        num_points: int = 8,
    ) -> WaypointPathBuilder:
        """Add waypoints along a circular arc."""
        angles = np.linspace(start_angle_rad, end_angle_rad, num_points)
        for angle in angles:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.waypoints.append(Waypoint(x=float(x), y=float(y)))
        return self

    def build(self) -> WaypointPath:
        """Build the path."""
        if not self.waypoints:
            msg = "Cannot build an empty path"
            raise ValueError(msg)
        return WaypointPath(waypoints=self.waypoints)


class TrackPositionFixtures:
    """Predefined track positions for common test scenarios."""

    # Track corners (one division line in from each wall)
    CORNER_NORTH = Waypoint(x=TrackDimensions.CENTER_COORD, y=TrackDimensions.MAX_COORD - 0.2)
    CORNER_SOUTH = Waypoint(x=TrackDimensions.CENTER_COORD, y=0.2)
    CORNER_EAST = Waypoint(x=TrackDimensions.MAX_COORD - 0.2, y=TrackDimensions.CENTER_COORD)
    CORNER_WEST = Waypoint(x=0.2, y=TrackDimensions.CENTER_COORD)

    # Track center
    CENTER = Waypoint(x=TrackDimensions.CENTER_COORD, y=TrackDimensions.CENTER_COORD)

    # Starting positions (outer band)
    START_SOUTH = Waypoint(x=TrackDimensions.CENTER_COORD, y=0.15)
    START_NORTH = Waypoint(x=TrackDimensions.CENTER_COORD, y=TrackDimensions.MAX_COORD - 0.15)
    START_EAST = Waypoint(x=TrackDimensions.MAX_COORD - 0.15, y=TrackDimensions.CENTER_COORD)
    START_WEST = Waypoint(x=0.15, y=TrackDimensions.CENTER_COORD)

    # Inner block corners
    INNER_MIN = Waypoint(x=TrackDimensions.CORNER_MIN, y=TrackDimensions.CORNER_MIN)
    INNER_MAX = Waypoint(
        x=TrackDimensions.CORNER_MAX, y=TrackDimensions.CORNER_MAX
    )


class ParkingLotFixtures:
    """Predefined parking lot configurations for test scenarios."""

    _PARK_A = 1.00  # First block position along corridor
    _PARK_B = _PARK_A + ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.LENGTH
    _PARK_FAR = TrackDimensions.MAX_COORD - ParkingLotSpecs.WALL_OFFSET

    @staticmethod
    def create(section: Section, block1_x: float, block1_y: float, block2_x: float, block2_y: float) -> ParkingLot:  # noqa: ARG004 - section names the corridor but the lot layout is section-independent
        """Create a parking lot with given block positions."""
        return ParkingLot(
            block1_position=BlockPosition(x=block1_x, y=block1_y),
            block2_position=BlockPosition(x=block2_x, y=block2_y),
        )

    @staticmethod
    def south() -> ParkingLot:
        """Standard parking lot in the South section."""
        return ParkingLot(
            block1_position=BlockPosition(x=ParkingLotFixtures._PARK_A, y=ParkingLotSpecs.WALL_OFFSET),
            block2_position=BlockPosition(x=ParkingLotFixtures._PARK_B, y=ParkingLotSpecs.WALL_OFFSET),
        )

    @staticmethod
    def north() -> ParkingLot:
        """Standard parking lot in the North section."""
        return ParkingLot(
            block1_position=BlockPosition(x=ParkingLotFixtures._PARK_A, y=ParkingLotFixtures._PARK_FAR),
            block2_position=BlockPosition(x=ParkingLotFixtures._PARK_B, y=ParkingLotFixtures._PARK_FAR),
        )

    @staticmethod
    def east() -> ParkingLot:
        """Standard parking lot in the East section."""
        return ParkingLot(
            block1_position=BlockPosition(x=ParkingLotFixtures._PARK_FAR, y=ParkingLotFixtures._PARK_A),
            block2_position=BlockPosition(x=ParkingLotFixtures._PARK_FAR, y=ParkingLotFixtures._PARK_B),
        )

    @staticmethod
    def west() -> ParkingLot:
        """Standard parking lot in the West section."""
        return ParkingLot(
            block1_position=BlockPosition(x=ParkingLotSpecs.WALL_OFFSET, y=ParkingLotFixtures._PARK_A),
            block2_position=BlockPosition(x=ParkingLotSpecs.WALL_OFFSET, y=ParkingLotFixtures._PARK_B),
        )


# Helpers for numpy-based LIDAR scans (e.g., collision avoidance controller tests)


def create_numpy_scan(default_distance_m: float = 10.0) -> np.ndarray:
    """Create a numpy array LIDAR scan with uniform distance.

    Use for tests that work with numpy arrays directly (e.g., CollisionAvoidanceController).
    """
    return np.full(NUM_RAYS, default_distance_m)


def angle_to_index(bearing_rad: float, angles: np.ndarray = ANGLES_FULL_ROTATION) -> int:
    """Find the ray index closest to a given bearing angle.

    Args:
        bearing_rad: Bearing in radians (0 = forward, ±π = rear)
        angles: Array of ray angles (default: ANGLES_FULL_ROTATION)

    Returns:
        Index of the ray closest to the bearing
    """
    return int(np.argmin(np.abs(angles - bearing_rad)))


def create_scan_with_sectors(
    default_distance_m: float = 10.0,
    angles: np.ndarray = ANGLES_FULL_ROTATION,
    sector_width_indices: int = 4,
    **closed_sectors: float,
) -> list[float]:
    """Create a LIDAR scan with specific sectors closed and others at default distance.

    Args:
        default_distance_m: Distance for all rays (default: far/clear)
        angles: Array of ray angles
        sector_width_indices: Half-width (in indices) of sectors to close
        **closed_sectors: Named sectors (front/back/left/right) and their distances

    Returns:
        List of range measurements (one per ray)

    Example:
        scan = create_scan_with_sectors(front=0.06, back=0.09)
    """
    ranges = np.full(NUM_RAYS, default_distance_m)
    sector_centers = {"front": 0.0, "left": math.pi / 2, "right": -math.pi / 2, "back": math.pi}

    for name, dist in closed_sectors.items():
        if name not in sector_centers:
            msg = f"Unknown sector '{name}'. Use: front, back, left, right"
            raise ValueError(msg)
        center_rad = sector_centers[name]
        idx = angle_to_index(center_rad, angles)
        ranges[idx - sector_width_indices : idx + sector_width_indices] = dist

    return ranges.tolist()


class FakeGateway:
    """Minimal HardwareGateway mock for testing CoreNavigator.

    Implements the HardwareGateway protocol structurally for driving CoreNavigator
    directly without ROS2 or the simulator.

    Attributes:
        pose: The robot's current pose (mutable for test scenarios)
        commands: List of published drive commands
    """

    def __init__(self, pose: Pose, lidar: LidarScan | None = None) -> None:
        self.pose = pose
        self.lidar = lidar
        self.commands: list[DriveCommand] = []

    def publish_drive(self, command: DriveCommand) -> None:
        """Record a drive command."""
        self.commands.append(command)

    def get_current_pose(self) -> Pose | None:
        """Return the robot's current pose."""
        return self.pose

    def get_lidar_scan(self) -> LidarScan | None:
        """Return the latest LIDAR scan."""
        return self.lidar

    def get_imu_reading(self) -> IMUReading | None:
        """Return the latest IMU reading."""
        return IMUReading(yaw=self.pose.yaw, pitch=0.0, roll=0.0)

    def get_vision_detections(self) -> list[Detection]:
        """Return vision detections."""
        return []
