"""
ROS2 test fixtures and configuration.
"""

import sys
import threading
from pathlib import Path
from unittest import mock

# Mock buildhat to avoid import errors when testing ROS2 nodes
sys.modules["buildhat"] = mock.MagicMock()

# src/hardware/button/gpio/driver.py does `from buildhat.serinterface import
# threading` — a real (if accidental) source dependency on an unrelated
# buildhat internal module, present only because it happens to import the
# stdlib threading module at its own top level. A bare MagicMock for
# "buildhat" doesn't behave like a package (no __path__), so importing the
# submodule "buildhat.serinterface" fails outright unless it's mocked too.
_mock_serinterface = mock.MagicMock()
_mock_serinterface.threading = threading
sys.modules["buildhat.serinterface"] = _mock_serinterface

# The real per-board node implementations (ackermann_motor_node, button_node,
# state_machine_node, ...) live across several ament_python packages under
# ros2_ws/src/voldemorbot_* (drivers/navigation/vision/state_machine/bringup),  # noqa: ERA001 -- prose, not commented-out code; ruff misreads this glob-like package path
# none of which are on PYTHONPATH — they're normally only importable after a
# colcon build. They're pure Python (ament_python, no compiled extensions), so
# adding each source dir directly lets tests import the exact deployed node
# code without needing a full ROS2 workspace build.
_ROS2_WS_SRC = Path(__file__).resolve().parents[2] / "ros2_ws" / "src"
for _pkg_dir in sorted(_ROS2_WS_SRC.glob("voldemorbot_*")):
    if str(_pkg_dir) not in sys.path:
        sys.path.insert(0, str(_pkg_dir))
