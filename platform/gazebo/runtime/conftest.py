"""Pytest configuration for the simulation runtime."""

import sys
from pathlib import Path

# Add the shared package to the Python path
runtime_dir = Path(__file__).parent
shared_src = (runtime_dir / "../../shared/src").resolve()
sys.path.insert(0, str(shared_src))
