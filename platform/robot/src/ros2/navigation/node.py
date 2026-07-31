"""Re-exports for the ROS2 track navigator entry point.

Split into :mod:`~src.ros2.navigation.ros2_hardware_gateway` and
:mod:`~src.ros2.navigation.track_navigator_node`; kept as a thin shim here
since ``ros2_ws/src/vtitan_navigation/vtitan_navigation/node.py`` (and other
callers) import ``ROS2HardwareGateway``/``TrackNavigator``/``main`` from this
module path.
"""

from src.ros2.navigation.ros2_hardware_gateway import _DRIVE_JOINT, ROS2HardwareGateway  # noqa: F401 -- re-exported for test_navigation_node.py's cross-file topic-contract check (accesses this module's ``_DRIVE_JOINT`` by attribute, not by name in this file)
from src.ros2.navigation.track_navigator_node import TrackNavigator, main

__all__ = ["ROS2HardwareGateway", "TrackNavigator", "main"]
