"""Loaded-once TOML-backed singletons shared by track.py and robot.py."""

from __future__ import annotations

from shared.config.robot_constants import RobotConstants
from shared.config.track_constants import TrackConstants

_robot = RobotConstants.load_default()
_track = TrackConstants.load_default()
