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
        progress_window_frames: int = 10**9,  # TEMP control arm
        progress_min_path_m: float = 1.0,
        progress_radius_m: float = 0.40,
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
            progress_window_frames: Length of the no-progress window (at 20Hz, 200≈10s).
                Must be longer than one oscillation period, or a pendulum's two
                window ends land on opposite swings and read as displacement.
            progress_min_path_m: Path length the window must carry before the
                no-progress test applies at all, so a robot legitimately holding
                still is left to the endpoint test.
            progress_radius_m: Max distance from the window's centroid for the
                window to count as going nowhere.

        Uses tuning: escape.min_history_for_distance

        The three ``progress_*`` values are a first cut read off
        ``run_20260913_191930`` (see :meth:`_has_no_progress`), not a swept
        optimum; they want a sweep before they are trusted as tuned.
        """
        tuning = get_tuning(tuning)
        if history_size < timeout_frames:
            msg = (
                f"history_size ({history_size}) must be >= timeout_frames ({timeout_frames}), "
                "otherwise the position history evicts entries before the timeout window "
                "is reached and the stuck check silently becomes less sensitive."
            )
            raise ValueError(msg)
        if progress_window_frames < 2:
            # maxlen would be 0, and the running-sum eviction below indexes [0].
            msg = f"progress_window_frames ({progress_window_frames}) must be >= 2 to hold a step"
            raise ValueError(msg)

        self.move_threshold = move_threshold
        self.timeout_frames = timeout_frames
        self.history_size = history_size
        self.confirmation_checks = confirmation_checks
        self._min_history_for_distance = (
            min_history_for_distance if min_history_for_distance is not None else tuning.escape.min_history_for_distance
        )

        # Position history
        self.position_history: deque[Waypoint] = deque(maxlen=history_size)
        self.frame_count = 0
        self.stuck_count = 0
        self.is_stuck = False

        # Long-window no-progress history; see _has_no_progress.
        self._progress_window_frames = progress_window_frames
        self._progress_min_path_m = progress_min_path_m
        self._progress_radius_m = progress_radius_m
        self._progress_positions: deque[Waypoint] = deque(maxlen=progress_window_frames)
        self._progress_steps: deque[float] = deque(maxlen=progress_window_frames - 1)
        self._progress_path_m = 0.0

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> StuckDetector:
        """Build detector from NavigationTuning parameters.

        Args:
            tuning: NavigationTuning instance (usually from load_default).

        Returns:
            StuckDetector with values from tuning.
        """
        # Seconds -> ticks at the loop rate; see EscapeManeuverParams.frames.
        hz = tuning.control.control_hz
        return cls(
            move_threshold=tuning.escape.stuck_move_threshold,
            timeout_frames=tuning.escape.stuck_timeout_frames(hz),
            history_size=max(tuning.escape.stuck_timeout_frames(hz) * 2, tuning.escape.stuck_history_floor_frames(hz)),
            confirmation_checks=tuning.escape.stuck_confirmation_checks,
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
        self._track_progress(current_pos)
        self.frame_count += 1

        # A pendulum passes the endpoint test below on nearly every tick, so it
        # has to be caught on its own terms before that test clears the latch.
        if self._has_no_progress():
            if not self.is_stuck:
                logger.warning(
                    "Robot making no progress: %.2f m travelled inside a %.2f m radius over %d frames",
                    self._progress_path_m,
                    self._progress_radius_m,
                    self._progress_window_frames,
                )
            self.is_stuck = True
            return True

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

    def _track_progress(self, current_pos: Waypoint) -> None:
        """Fold one position into the long no-progress window, O(1) per tick."""
        if self._progress_positions:
            prev = self._progress_positions[-1]
            step = float(np.hypot(current_pos.x - prev.x, current_pos.y - prev.y))
            # The deque evicts on append, so drop the outgoing step from the
            # running sum BEFORE appending or the total drifts up forever.
            if len(self._progress_steps) == self._progress_steps.maxlen:
                self._progress_path_m -= self._progress_steps[0]
            self._progress_steps.append(step)
            self._progress_path_m += step
        self._progress_positions.append(current_pos)

    def _has_no_progress(self) -> bool:
        """Whether the robot has been travelling without getting anywhere.

        The endpoint test in :meth:`update` compares the two ENDS of a short
        window, so it measures net displacement and cannot see a robot that
        swings back and forth: each swing puts the ends far apart and resets
        the latch. Measured on ``run_20260913_191930``, 77.2 s inside a
        0.58 x 0.59 m box cost 11.079 m of absolute wheel travel for 0.480 m of
        net pose displacement, and ``is_stuck`` never once armed.

        This asks the question the other way round: over a window longer than
        one oscillation, did a lot of PATH buy any RADIUS? Distance is taken
        from the window's centroid rather than its first sample, because a
        sinusoid sampled over a non-integer number of periods puts its two ends
        on opposite swings and reads as displacement it never made.
        """
        if len(self._progress_positions) < self._progress_window_frames:
            return False
        if self._progress_path_m < self._progress_min_path_m:
            return False
        n = len(self._progress_positions)
        cx = sum(p.x for p in self._progress_positions) / n
        cy = sum(p.y for p in self._progress_positions) / n
        radius = max(float(np.hypot(p.x - cx, p.y - cy)) for p in self._progress_positions)
        return radius < self._progress_radius_m

    def reset(self) -> None:
        """Reset stuck detector (e.g., after escape maneuver)."""
        self.position_history.clear()
        self.stuck_count = 0
        self.is_stuck = False
        self._progress_positions.clear()
        self._progress_steps.clear()
        self._progress_path_m = 0.0

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
