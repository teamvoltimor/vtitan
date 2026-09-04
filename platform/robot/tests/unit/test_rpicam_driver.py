"""Tests for the rpicam-vid driver's command-line construction.

``_command()`` is pure string-building with no hardware access, so it belongs
in the default unit run rather than under ``tests/hardware/`` (which needs a
real Pi and is excluded by default).
"""

from __future__ import annotations

from src.hardware.camera.rpicam.driver import Config, Driver, ExposureMode


def test_auto_exposure_command_has_no_shutter_or_gain_flags() -> None:
    driver = Driver(config=Config(camera_exposure_mode=ExposureMode.SHORT))
    cmd = driver._command()

    assert "--shutter" not in cmd
    assert "--gain" not in cmd


def test_exposure_mode_short_maps_to_sport() -> None:
    driver = Driver(config=Config(camera_exposure_mode=ExposureMode.SHORT))
    cmd = driver._command()

    assert cmd[cmd.index("--exposure") + 1] == "sport"


def test_exposure_mode_normal_maps_to_normal() -> None:
    driver = Driver(config=Config(camera_exposure_mode=ExposureMode.NORMAL))
    cmd = driver._command()

    assert cmd[cmd.index("--exposure") + 1] == "normal"


def test_manual_exposure_time_sets_shutter_and_gain() -> None:
    driver = Driver(config=Config(camera_exposure_time_us=8000, camera_analogue_gain=2.0))
    cmd = driver._command()

    assert cmd[cmd.index("--shutter") + 1] == "8000"
    assert cmd[cmd.index("--gain") + 1] == "2.0"


def test_hflip_and_vflip_flags_follow_resolved_flips() -> None:
    # inverted=False isolates hflip/vflip from resolved_flips()'s mount-inversion XOR.
    driver = Driver(config=Config(camera_inverted=False, camera_hflip=True, camera_vflip=False))
    cmd = driver._command()

    assert "--hflip" in cmd
    assert "--vflip" not in cmd
