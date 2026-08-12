"""Shared QoS profiles reused across ROS2 nodes.

Every latched-state publisher/subscriber pair in this codebase (robot state,
challenge mode, bag-recorder run path, system status, ...) needs the exact
same profile on both ends: a mismatch is an incompatible QoS pair that DDS
resolves by delivering nothing at all, silently. ``vision.node`` and
``track_navigator_node`` each hand-built this profile 2x with the same
explanatory comment; a single shared constant means there is only one place
to get it right.
"""

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# TRANSIENT_LOCAL so a late subscriber (e.g. the OLED, which restarts
# independently on the Pi Zero) gets the writer's last publish instead of
# waiting for a periodic re-publish that never comes. BEST_EFFORT because a
# RELIABLE writer blocks on a slow reader -- measured on the OLED's own board
# stalling for 30+ seconds under contention. Both ends of a pair using this
# profile must use it: a RELIABLE reader against a BEST_EFFORT writer (or vice
# versa) is an incompatible QoS pair that DDS resolves by delivering nothing.
QOS_LATCHED_STATE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)
