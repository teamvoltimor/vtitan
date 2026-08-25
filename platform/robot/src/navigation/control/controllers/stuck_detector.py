"""Stuck detection and recovery controller.

Detects when the robot is stuck (not making forward progress) and
triggers recovery maneuvers.
"""

from __future__ import annotations

import logging
from collections import deque
from typing import TYPE_CHECKING

import numpy as np

from src.config.tuning_helpers import get_tuning

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning
    from shared.domain.models import Waypoint

logger = logging.getLogger(__name__)


class StuckDetector:
    """Detects and responds to stuck robot conditions.

    Maintains a position history and checks if the robot has moved
    a minimum distance in recent frames. If not, triggers recovery.

    Attributes:
        move_threshold: Minimum movement distance to consider not stuck (m)
        timeout_frames: Frames without movement before declaring stuck
        history_size: Number of positions to maintain in history
        confirmation_checks: Consecutive below-threshold checks required
            before declaring the robot stuck
    """

    def __init__(
        self,
        move_threshold: float,
        timeout_frames: int,
        history_size: int,
        confirmation_checks: int,
        min_history_for_distance: int | None = None,
        tuning: NavigationTuning | None = None,
    ):
        """Initialize stuck detector.

        Args:
            move_threshold: Min distance to move to avoid stuck (m)
            timeout_frames: Frames before timeout (at 20Hz, 40≈2s)
            history_size: Max position history to maintain
            confirmation_checks: Consecutive below-threshold checks required
                before declaring the robot stuck
            min_history_for_distance: Minimum tracked positions to compute movement distance.
                Defaults to tuning value.
            tuning: Navigation tuning instance. Defaults to loaded defaults.

        Uses tuning: escape.MIN_HISTORY_FOR_DISTANCE
        """
        tuning = get_tuning(tuning)
        if history_size < timeout_frames:
            msg = (
                f"history_size ({history_size}) must be >= timeout_frames ({timeout_frames}), "
                "otherwise the position history evicts entries before the timeout window "
                "is reached and the stuck check silently becomes less sensitive."
            )
            raise ValueError(msg)

        self.move_threshold = move_threshold
        self.timeout_frames = timeout_frames
        self.history_size = history_size
        self.confirmation_checks = confirmation_checks
        self._min_history_for_distance = (
            min_history_for_distance if min_history_for_distance is not None else tuning.escape.MIN_HISTORY_FOR_DISTANCE
        )

        # Position history
        self.position_history: deque[Waypoint] = deque(maxlen=history_size)
        self.frame_count = 0
        self.stuck_count = 0
        self.is_stuck = False

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> StuckDetector:
        """Build detector from NavigationTuning parameters.

        Args:
            tuning: NavigationTuning instance (usually from load_default).

        Returns:
            StuckDetector with values from tuning.
        """
        return cls(
            move_threshold=tuning.escape.STUCK_MOVE_THRESHOLD,
            timeout_frames=tuning.escape.STUCK_TIMEOUT_FRAMES,
            history_size=max(tuning.escape.STUCK_TIMEOUT_FRAMES * 2, tuning.escape.STUCK_HISTORY_FLOOR),
            confirmation_checks=tuning.escape.STUCK_CONFIRMATION_CHECKS,
            tuning=tuning,
        )

    def update(self, current_pos: Waypoint) -> bool:
        """Update detector with current position.

        Args:
            current_pos: Current robot position

        Returns:
            True if robot is stuck, False otherwise
        """
        self.position_history.append(current_pos)
        self.frame_count += 1

        # Need enough history to check
        if len(self.position_history) < self.timeout_frames:
            self.is_stuck = False
            return False

        # Check if moved in recent frames
        old_pos = next(iter(self.position_history))
        curr_pos = self.position_history[-1]

        distance_moved = np.sqrt((curr_pos.x - old_pos.x) ** 2 + (curr_pos.y - old_pos.y) ** 2)

        if distance_moved < self.move_threshold:
            self.stuck_count += 1
            if self.stuck_count > self.confirmation_checks:
                self.is_stuck = True
                logger.warning("Robot stuck: moved only %.4f m in %d frames", distance_moved, self.timeout_frames)
                return True
        else:
            self.stuck_count = 0
            self.is_stuck = False

        return self.is_stuck

    def reset(self) -> None:
        """Reset stuck detector (e.g., after escape maneuver)."""
        self.position_history.clear()
        self.stuck_count = 0
        self.is_stuck = False

    def get_diagnostics(self) -> dict[str, float | int]:
        """Get current diagnostics.

        Returns:
            Dict with stuck status and metrics
        """
        if len(self.position_history) >= self._min_history_for_distance:
            distance = np.sqrt(
                (self.position_history[-1].x - self.position_history[0].x) ** 2
                + (self.position_history[-1].y - self.position_history[0].y) ** 2,
            )
        else:
            distance = 0.0

        return {
            "is_stuck": self.is_stuck,
            "stuck_count": self.stuck_count,
            "recent_movement": distance,
            "history_size": len(self.position_history),
            "frame_count": self.frame_count,
        }
