"""Headless pure-Python simulation of the WRO 2026 robot.

Drives the real ``CoreNavigator`` stack through a simulated ``HardwareGateway``
(Ackermann kinematics + raycast LIDAR + collision model) so navigation can be
validated end-to-end without Gazebo or ROS2.
"""

from src.simulation.gateway import (
    ScenarioSimulator,
    SimResult,
    SimulatedHardwareGateway,
)
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import TrackModel

__all__ = [
    "AckermannKinematics",
    "AckermannState",
    "ScenarioSimulator",
    "SimResult",
    "SimulatedHardwareGateway",
    "TrackModel",
]
