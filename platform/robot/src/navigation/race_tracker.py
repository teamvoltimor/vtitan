"""Race metrics tracking and state management.

Bookkeeping for lap counting, elapsed time, and performance metrics.
No ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import time
from typing import Any

from pydantic import BaseModel, Field
from shared.domain.enums import Direction, Section
from shared.domain.models import LoopProgress, Waypoint

from src.logger.constants import DETAILS_KEY

logger = logging.getLogger(__name__)


# Travel direction unit vectors for each (section, direction) combination.
# Used by LapDetector as the finish-line normal, and by
# ``corridor_estimator.section_from_heading`` — for a fixed travel direction all
# four vectors are distinct, so a heading identifies the corridor outright.
TRAVEL_DIRS: dict[tuple[Section, Direction], tuple[float, float]] = {
    (Section.SOUTH, Direction.CLOCKWISE): (-1.0, 0.0),
    (Section.NORTH, Direction.CLOCKWISE): (1.0, 0.0),
    (Section.EAST, Direction.CLOCKWISE): (0.0, -1.0),
    (Section.WEST, Direction.CLOCKWISE): (0.0, 1.0),
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): (1.0, 0.0),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): (-1.0, 0.0),
    (Section.EAST, Direction.COUNTERCLOCKWISE): (0.0, 1.0),
    (Section.WEST, Direction.COUNTERCLOCKWISE): (0.0, -1.0),
}


class LapDetector:
    """Geometric start/finish-line detector with waypoint-index corroboration.

    A lap is counted only when BOTH conditions hold:
    1. **Geometric**: robot crosses the start/finish line in the forward direction
       (dot product changes from negative to non-negative).
    2. **Waypoint**: the waypoint sequence has wrapped at least once since the
       last confirmed lap (i.e., the robot has made meaningful progress).

    This prevents double-counting from overshoot, stuck-loops, or waypoint
    skips near the finish line.

    Args:
        start_pos: World position of the starting zone centre.
        start_section: Which corridor the starting zone is in.
        direction: CW or CCW traversal direction.
    """

    def __init__(
        self,
        start_pos: Waypoint,
        start_section: Section,
        direction: Direction,
    ) -> None:
        self._origin: Waypoint = start_pos
        self._normal: tuple[float, float] = TRAVEL_DIRS[(start_section, direction)]
        self._start_section = start_section
        self._prev_dot: float | None = None
        self._waypoint_pending: bool = False

    def notify_waypoint_wrapped(self) -> None:
        """Call this when the waypoint sequence index wraps to 0."""
        self._waypoint_pending = True

    def update(
        self,
        robot_pos: Waypoint,
        current_section: Section,
    ) -> bool:
        """Check whether a valid lap crossing occurred at this position.

        Args:
            robot_pos: Current robot position in world frame.
            current_section: Corridor section determined from robot position.

        Returns:
            True if a confirmed lap was just completed; False otherwise.
        """
        nx, ny = self._normal
        dot = (robot_pos.x - self._origin.x) * nx + (robot_pos.y - self._origin.y) * ny

        geometric_cross = (
            self._prev_dot is not None
            and self._prev_dot < 0.0
            and dot >= 0.0
            and current_section is self._start_section
        )

        # After a crossing reset prev_dot so the next lap must first retreat
        # to negative-dot territory before another crossing counts.
        if geometric_cross:
            self._prev_dot = dot  # keep current (positive) value
        else:
            self._prev_dot = dot

        if geometric_cross and self._waypoint_pending:
            self._waypoint_pending = False
            return True

        return False


class RaceMetrics(BaseModel):
    """Performance metrics for a race run."""

    elapsed_time: float = 0.0  # seconds
    total_distance: float = 0.0  # metres
    current_lap: int = 1
    completed_laps: int = 0
    waypoint_index: int = 0
    max_speed: float = 0.0  # m/s
    avg_speed: float = 0.0  # m/s
    escape_maneuvers: int = 0
    stuck_detections: int = 0
    collision_warnings: int = 0
    lap_splits: list[float] = Field(default_factory=list)  # elapsed time at each lap completion
    extra_data: dict[str, Any] = Field(default_factory=dict, alias="extra")

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return self.model_dump(by_alias=True)


class RaceTracker:
    """Track race metrics and state throughout execution."""

    def __init__(self, num_laps: int):
        """Initialize race tracker.

        Args:
            num_laps: Total number of laps to complete.
        """
        self.num_laps = num_laps
        self.metrics = RaceMetrics()
        self._start_time = time.time()
        self._last_pos: Waypoint | None = None
        self._speed_samples: list[float] = []
        self._lap_start_distance: float = 0.0

    def update_position(
        self,
        current_pos: Waypoint,
        current_speed: float,
        waypoint_index: int,
    ) -> None:
        """Update tracker with current robot state.

        Args:
            current_pos: Current position.
            current_speed: Current linear speed (m/s).
            waypoint_index: Current waypoint index.
        """
        # Update elapsed time
        self.metrics.elapsed_time = time.time() - self._start_time

        # Update waypoint index
        self.metrics.waypoint_index = waypoint_index

        # Track distance traveled
        if self._last_pos is not None:
            dx = current_pos.x - self._last_pos.x
            dy = current_pos.y - self._last_pos.y
            distance_increment = (dx**2 + dy**2) ** 0.5
            self.metrics.total_distance += distance_increment

        self._last_pos = current_pos

        # Track speed
        self.metrics.max_speed = max(self.metrics.max_speed, current_speed)
        self._speed_samples.append(current_speed)

        # Update average speed
        if self._speed_samples:
            self.metrics.avg_speed = sum(self._speed_samples) / len(self._speed_samples)

    def increment_lap(self) -> None:
        """Record completion of current lap.

        Called when robot reaches end of waypoint sequence.
        """
        self.metrics.completed_laps += 1
        self.metrics.current_lap = self.metrics.completed_laps + 1
        self._lap_start_distance = self.metrics.total_distance
        split_time = time.time() - self._start_time
        self.metrics.lap_splits.append(round(split_time, 3))
        logger.info(
            "Lap %d/%d completed",
            self.metrics.completed_laps,
            self.num_laps,
            extra={
                "details": {
                    "elapsed": f"{self.metrics.elapsed_time:.1f}s",
                    "lap_split": f"{split_time:.2f}s",
                    "distance": f"{self.metrics.total_distance:.2f}m",
                },
            },
        )
        if self.metrics.completed_laps >= self.num_laps:
            logger.info(
                "Race FINISHED",
                extra={DETAILS_KEY: {"lap_splits": self.metrics.lap_splits}},
            )

    def record_escape_maneuver(self) -> None:
        """Record execution of escape maneuver (K-turn, etc.)."""
        self.metrics.escape_maneuvers += 1

    def record_stuck_detection(self) -> None:
        """Record detection of stuck robot condition."""
        self.metrics.stuck_detections += 1

    def record_collision_warning(self) -> None:
        """Record high-risk collision scenario."""
        self.metrics.collision_warnings += 1

    def is_race_complete(self) -> bool:
        """Check if all laps are completed.

        Returns:
            True if completed_laps >= num_laps.
        """
        return self.metrics.completed_laps >= self.num_laps

    def get_race_metrics(self) -> RaceMetrics:
        """Get current race metrics.

        Returns:
            RaceMetrics dataclass with current state.
        """
        return self.metrics

    def progress(self) -> LoopProgress:
        """Snapshot of lap, waypoint, and distance progress."""
        return LoopProgress(
            lap_number=self.metrics.current_lap,
            waypoint_index=self.metrics.waypoint_index,
            distance_m=self.metrics.total_distance - (self._lap_start_distance if self._last_pos else self.metrics.total_distance),
            total_distance_m=self.metrics.total_distance,
        )

    def get_race_summary(self) -> dict[str, Any]:
        """Get summary statistics for the race.

        Returns:
            Dictionary with key metrics and stats.
        """
        summary = self.metrics.to_dict()
        summary["laps_remaining"] = max(0, self.num_laps - self.metrics.completed_laps)
        summary["est_finish_time"] = (
            self.metrics.elapsed_time * self.num_laps / max(1, self.metrics.completed_laps)
            if self.metrics.completed_laps > 0
            else 0.0
        )
        return summary

    def log_summary(self) -> None:
        """Log race summary to logger."""
        summary = self.get_race_summary()
        logger.info(
            "Race Summary",
            extra={DETAILS_KEY: summary},
        )
