"""
WRO 2026 Future Engineers - Simulation Constants.

This package contains all official WRO specifications and simulation parameters.
All measurements are in meters unless otherwise specified.

Official Source: WRO Future Engineers Competition Rules 2026
Last Updated: 2026-02-08

Split by theme (track/mat, robot, simulation-only, string identifiers) so no
single file grows past a few hundred lines; every class is re-exported here
so existing imports (``from shared.config.constants import X``) are
unaffected by the split.
"""

from __future__ import annotations

from shared.config.constants.identifiers import (
    DictKeys,
    FileExtensions,
    FilePaths,
    FolderNames,
    ModelNames,
    TfFrames,
)
from shared.config.constants.robot import RobotSpecs
from shared.config.constants.simulation import CompetitionSpecs
from shared.config.constants.track import (
    CorridorDimensions,
    GridSections,
    ParkingLotSpecs,
    StartingZoneSpecs,
    TrackDimensions,
    TrackMarkings,
    TrafficSignSpecs,
    WallSpecs,
)

__all__ = [
    "CompetitionSpecs",
    "CorridorDimensions",
    "DictKeys",
    "FileExtensions",
    "FilePaths",
    "FolderNames",
    "GridSections",
    "ModelNames",
    "ParkingLotSpecs",
    "RobotSpecs",
    "StartingZoneSpecs",
    "TfFrames",
    "TrackDimensions",
    "TrackMarkings",
    "TrafficSignSpecs",
    "WallSpecs",
]
