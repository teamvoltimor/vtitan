"""Headless pure-Python simulation of the WRO 2026 robot.

Drives the real ``CoreNavigator`` stack through a simulated ``HardwareGateway``
(Ackermann kinematics + raycast LIDAR + collision model) so navigation can be
validated end-to-end without Gazebo or ROS2.
"""

from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.scenario_result import PoseDisturbance, SimResult
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.simulated_hardware_gateway import SimulatedHardwareGateway
from src.simulation.track_model import TrackModel

__all__ = [
    "AckermannKinematics",
    "AckermannState",
    "PoseDisturbance",
    "ScenarioSimulator",
    "SimResult",
    "SimulatedHardwareGateway",
    "TrackModel",
]
