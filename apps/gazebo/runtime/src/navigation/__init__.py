"""Pure-Python navigation utilities — no ROS2 or Gazebo required."""

import sys
from pathlib import Path

# Ensure shared package is available
# From apps/gazebo/runtime/src/navigation/__init__.py → go up 6 levels to the repo root → then to src/python/shared/src
_this_file = Path(__file__).resolve()
_shared_src = _this_file.parents[5] / "src" / "python" / "shared" / "src"
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
