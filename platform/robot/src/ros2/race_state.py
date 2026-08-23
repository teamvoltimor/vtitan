"""Race-state subscription and RACING detection shared across ROS2 nodes.

Two things used to be hand-inlined in every node that gates on the state
machine: the latched ``/robot_state`` subscription (with its non-obvious
``QOS_LATCHED_STATE`` requirement -- a RELIABLE reader against this BEST_EFFORT
writer delivers nothing, silently) and the ``msg.data.strip().lower() ==
RobotState.RACING.value`` parse. ``track_navigator_node`` and ``vision/node``
each repeated both. This module is the one place to get them right.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from shared.domain.enums import RobotState
from std_msgs.msg import String

from src.ros2.qos import QOS_LATCHED_STATE

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.node import Node
    from shared.config.ros_topics import RosTopicConfig


def parse_racing(msg: String) -> bool:
    """Return whether *msg* (a ``/robot_state`` String) reports RACING.

    Centralises the ``msg.data.strip().lower() == RobotState.RACING.value``
    parse that used to be duplicated at every ``_on_robot_state`` handler.
    """
    return msg.data.strip().lower() == RobotState.RACING.value


def subscribe_to_race_state(
    node: Node,
    topics: RosTopicConfig,
    callback: Callable[[String], None],
) -> None:
    """Subscribe *callback* to the latched ``/robot_state`` topic.

    Uses :data:`src.ros2.qos.QOS_LATCHED_STATE` on both ends of the pair, which
    is required for delivery (a RELIABLE reader against this BEST_EFFORT writer
    gets nothing). The state machine republishes every tick, so a dropped sample
    is corrected within one tick -- nothing may block on this.
    """
    node.create_subscription(String, topics.state_machine.state, callback, QOS_LATCHED_STATE)


class RacingState:
    """Tracks the latest RACING flag derived from ``/robot_state`` messages.

    Mixes into a node: keep one instance, feed it from a ``subscribe_to_race_state``
    callback via :meth:`update`, and read :attr:`is_racing`. The transition
    hooks let each node run its own start/stop side effects without re-detecting
    the edge itself.
    """

    def __init__(self) -> None:
        self._racing = False

    @property
    def is_racing(self) -> bool:
        """Whether the most recent ``/robot_state`` reported RACING."""
        return self._racing

    def update(
        self, msg: String, *, on_start: Callable[[], None] | None = None, on_stop: Callable[[], None] | None = None
    ) -> bool:
        """Update the racing flag from *msg*, firing transition hooks once.

        Args:
            msg: The ``/robot_state`` String message.
            on_start: Called once on the RACING-entered edge (was not racing, now racing).
            on_stop: Called once on the RACING-left edge (was racing, now not).

        Returns:
            The new racing flag.
        """
        was_racing = self._racing
        self._racing = parse_racing(msg)
        if self._racing and not was_racing and on_start is not None:
            on_start()
        elif was_racing and not self._racing and on_stop is not None:
            on_stop()
        return self._racing
