"""Shared QoS profiles reused across ROS2 nodes.

Every latched-state publisher/subscriber pair in this codebase (robot state,
challenge mode, bag-recorder run path, system status, ...) needs the exact
same profile on both ends: a mismatch is an incompatible QoS pair that DDS
resolves by delivering nothing at all, silently. ``vision.node`` and
``track_navigator_node`` each hand-built this profile 2x with the same
explanatory comment; a single shared constant means there is only one place
to get it right.
"""

from rclpy.duration import Duration
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

# RELIABLE variant of the above, for the one pair (challenge_mode_node's
# jumper_inserted publisher <-> state_machine_node's subscriber) that must not
# silently drop a reading: unlike the BEST_EFFORT profiles above, nothing else
# republishes the jumper state on a tick loop to paper over a dropped sample.
QOS_LATCHED_STATE_RELIABLE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

# Depth=1, BEST_EFFORT, default (VOLATILE) durability -- a late subscriber
# gets nothing until the next publish, unlike QOS_LATCHED_STATE. For live
# per-tick readouts (lap count, UI summary, button hold-progress) where a
# late subscriber catching the next tick is fine and latching the stale value
# from before it existed would be wrong.
QOS_LIVE_READOUT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

# Depth=10, default (VOLATILE) durability and reliability -- the generic
# profile for ordinary high-rate string/metric streams (race metrics, nav
# debug, detections) that used to be spelled "..., 10" inline at every call
# site. Deliberately distinct from QOS_LATCHED_STATE/QOS_LIVE_READOUT (both
# depth=1): those are latched/low-rate, this is for a plain stream where a
# dropped sample under backpressure is acceptable.
QOS_STREAM = QoSProfile(depth=10)

# Reliable + 200ms deadline, for every publisher/subscriber pair on
# /ackermann_cmd. DEADLINE is a two-sided QoS policy: a subscriber that
# requests one (state_machine_node's echo-back to /race_metrics) rejects any
# publisher that doesn't offer a deadline at least as tight -- DDS calls that
# an incompatible match and delivers nothing between that pair, silently.
# ros2_hardware_gateway.py's real drive-command publisher used to offer
# QOS_STREAM's Infinite deadline against that subscription's 200ms request,
# so state_machine_node never actually received live driving commands (see
# git log for the incident). Every node that publishes OR subscribes to
# /ackermann_cmd must use this profile, not construct its own.
QOS_ACKERMANN_CMD = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    deadline=Duration(nanoseconds=200_000_000),
)
