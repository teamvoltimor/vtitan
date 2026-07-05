"""Deprecated location for the hardware port.

The port is now owned by the consumer — see :mod:`src.navigation.ports`.
This module re-exports the same names so existing imports keep resolving.
"""

from __future__ import annotations

from src.navigation.ports import DriveCommand, HardwareGateway, LidarScan

__all__ = ["DriveCommand", "HardwareGateway", "LidarScan"]
