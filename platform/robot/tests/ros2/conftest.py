"""
ROS2 test fixtures and configuration.
"""

import sys
from unittest import mock

# Mock buildhat to avoid import errors when testing ROS2 nodes
sys.modules["buildhat"] = mock.MagicMock()
