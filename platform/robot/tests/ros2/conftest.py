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
# state_machine_node, ...) live in the ament_python package under ros2_ws/,
# which isn't on PYTHONPATH — it's normally only importable after a colcon
# build. It's pure Python (ament_python, no compiled extensions), so adding
# its source dir directly lets tests import the exact deployed node code
# without needing a full ROS2 workspace build.
_ROS2_WS_PKG = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "voldemorbot_robot"
if str(_ROS2_WS_PKG) not in sys.path:
    sys.path.insert(0, str(_ROS2_WS_PKG))
