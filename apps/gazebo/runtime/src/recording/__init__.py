"""Public API for the recording package."""

import sys
from pathlib import Path

# Ensure shared package is available
# From apps/gazebo/runtime/src/recording/__init__.py → go up 6 levels to the repo root → then to src/python/shared/src
_this_file = Path(__file__).resolve()
_shared_src = _this_file.parents[5] / "src" / "python" / "shared" / "src"
if str(_shared_src) not in sys.path:
    sys.path.insert(0, str(_shared_src))

from src.recording.bag_converter import BagToVideoConverter
from src.recording.recorder import PipelineOrchestrator, VideoRecorderNode

__all__ = [
    "BagToVideoConverter",
    "PipelineOrchestrator",
    "VideoRecorderNode",
]
