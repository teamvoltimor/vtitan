"""Pure-Python navigation utilities — no ROS2 or Gazebo required."""

import sys
from pathlib import Path

# Ensure shared package is available
# From gazebo/runtime/src/navigation/__init__.py → go up 5 levels to platform/ → then to shared/src
_this_file = Path(__file__).resolve()
_shared_src = _this_file.parents[4] / "shared" / "src"
if str(_shared_src) not in sys.path:
    sys.path.insert(0, str(_shared_src))

from src.navigation.open_lap_planner import (
    LapResult,
    MockLapRunner,
    OpenLapPlanner,
    TrackCorners,
)

__all__ = [
    "LapResult",
    "MockLapRunner",
    "OpenLapPlanner",
    "TrackCorners",
]
