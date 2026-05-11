"""Pytest configuration for the platform package."""

import sys
from pathlib import Path

# Add the platform directory to the Python path so imports work correctly
platform_root = Path(__file__).parent
sys.path.insert(0, str(platform_root))
