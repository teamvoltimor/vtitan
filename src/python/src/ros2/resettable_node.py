"""Mixin for ROS2 nodes that carry state which must not survive into the next race.

The state machine's FINISHED -> BOOT_CHECK -> READY -> RACING cycle (two long
presses and a short press) is meant to let the robot be re-armed and re-run
entirely from the physical button, with no SSH session or power cycle. That
only works if every node holding race-scoped state (lap counts, waypoint
index, parking phase, ...) actually clears it when a new race starts --
otherwise the second run inherits the first one's outcome. Nodes with no such
state don't need to override anything.
"""


class ResettableNode:
    """Mixin adding a reset() hook for nodes with race-scoped internal state."""

    def reset(self) -> None:
        """Clear state ahead of a new race. No-op unless overridden."""
