"""Stuck detection and recovery controller.

Detects when the robot is stuck (not making forward progress) and
triggers recovery maneuvers.
"""

from __future__ import annotations

import logging
from collections import deque

import numpy as np

logger = logging.getLogger(__name__)

_MIN_HISTORY_FOR_DISTANCE: int = 2
"""Minimum number of tracked positions needed to compute a movement distance."""


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
        move_threshold: float = 0.03,
        timeout_frames: int = 40,
        history_size: int = 60,
        confirmation_checks: int = 3,
    ):
        """Initialize stuck detector.

        Args:
            move_threshold: Min distance to move to avoid stuck (m)
            timeout_frames: Frames before timeout (at 20Hz, 40≈2s)
            history_size: Max position history to maintain
            confirmation_checks: Consecutive below-threshold checks required
                before declaring the robot stuck
        """
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

        # Position history
        self.position_history: deque[tuple[float, float]] = deque(maxlen=history_size)
        self.frame_count = 0
        self.stuck_count = 0
        self.is_stuck = False

    def update(self, current_pos: tuple[float, float]) -> bool:
        """Update detector with current position.

        Args:
            current_pos: Current robot position (x, y)

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

        distance_moved = np.sqrt((curr_pos[0] - old_pos[0]) ** 2 + (curr_pos[1] - old_pos[1]) ** 2)

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
        if len(self.position_history) >= _MIN_HISTORY_FOR_DISTANCE:
            distance = np.sqrt(
                (self.position_history[-1][0] - self.position_history[0][0]) ** 2
                + (self.position_history[-1][1] - self.position_history[0][1]) ** 2,
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
