"""Race metrics tracking and state management.

Bookkeeping for lap counting, elapsed time, and performance metrics.
No ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


@dataclass
class RaceMetrics:
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
    extra_data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "elapsed_time": self.elapsed_time,
            "total_distance": self.total_distance,
            "current_lap": self.current_lap,
            "completed_laps": self.completed_laps,
            "waypoint_index": self.waypoint_index,
            "max_speed": self.max_speed,
            "avg_speed": self.avg_speed,
            "escape_maneuvers": self.escape_maneuvers,
            "stuck_detections": self.stuck_detections,
            "collision_warnings": self.collision_warnings,
            "extra": self.extra_data,
        }


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
        self._lap_start_pos: tuple[float, float] | None = None
        self._last_pos: tuple[float, float] | None = None
        self._speed_samples: list[float] = []

    def update_position(
        self,
        current_pos: tuple[float, float],
        current_speed: float,
        waypoint_index: int,
    ) -> None:
        """Update tracker with current robot state.

        Args:
            current_pos: Current (x, y) position.
            current_speed: Current linear speed (m/s).
            waypoint_index: Current waypoint index.
        """
        # Update elapsed time
        self.metrics.elapsed_time = time.time() - self._start_time

        # Update waypoint index
        self.metrics.waypoint_index = waypoint_index

        # Track distance traveled
        if self._last_pos is not None:
            dx = current_pos[0] - self._last_pos[0]
            dy = current_pos[1] - self._last_pos[1]
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
        logger.info(
            f"Lap {self.metrics.completed_laps}/{self.num_laps} completed",
            extra={
                "details": {
                    "elapsed": f"{self.metrics.elapsed_time:.1f}s",
                    "distance": f"{self.metrics.total_distance:.2f}m",
                }
            },
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
            extra={"details": summary},
        )
