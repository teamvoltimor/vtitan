"""Public API for the recording package."""

import sys
from pathlib import Path

# Ensure shared package is available
_this_file = Path(__file__).resolve()
_shared_src = _this_file.parents[4] / "shared" / "src"
if str(_shared_src) not in sys.path:
    sys.path.insert(0, str(_shared_src))

from src.recording.bag_converter import BagToVideoConverter
from src.recording.recorder import PipelineOrchestrator, VideoRecorderNode

__all__ = [
    "BagToVideoConverter",
    "PipelineOrchestrator",
    "VideoRecorderNode",
]
