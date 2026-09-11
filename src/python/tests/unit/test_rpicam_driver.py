"""Tests for the rpicam-vid driver's command-line construction.

``_command()`` is pure string-building with no hardware access, so it belongs
in the default unit run rather than under ``tests/hardware/`` (which needs a
real Pi and is excluded by default).
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from src.hardware.camera.rpicam.driver import (
    AfMode,
    AfSpeed,
    AwbMode,
    Config,
    Driver,
    ExposureMode,
    MeteringMode,
    NoiseReductionMode,
)


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


def test_manual_focus_pins_the_lens_and_omits_autofocus_speed() -> None:
    # The whole point of MANUAL: the voice coil stops hunting, so there is no
    # search speed to set.
    driver = Driver(config=Config(camera_af_mode=AfMode.MANUAL, camera_lens_position=0.8))
    cmd = driver._command()

    assert cmd[cmd.index("--autofocus-mode") + 1] == "manual"
    assert cmd[cmd.index("--lens-position") + 1] == "0.8"
    assert "--autofocus-speed" not in cmd


def test_continuous_focus_sets_speed_and_omits_lens_position() -> None:
    driver = Driver(config=Config(camera_af_mode=AfMode.CONTINUOUS, camera_af_speed=AfSpeed.FAST))
    cmd = driver._command()

    assert cmd[cmd.index("--autofocus-mode") + 1] == "continuous"
    assert cmd[cmd.index("--autofocus-speed") + 1] == "fast"
    assert "--lens-position" not in cmd


def test_manual_focus_without_a_lens_position_is_rejected() -> None:
    # rpicam-vid would accept this and leave the lens wherever the previous run
    # parked it, giving a focus that varies silently between runs.
    with pytest.raises(ValidationError):
        Config(camera_af_mode=AfMode.MANUAL, camera_lens_position=None)


def test_colour_and_noise_settings_reach_the_command() -> None:
    driver = Driver(
        config=Config(
            camera_awb_mode=AwbMode.FLUORESCENT,
            camera_noise_reduction_mode=NoiseReductionMode.FAST,
            camera_sharpness=1.2,
        )
    )
    cmd = driver._command()

    assert cmd[cmd.index("--awb") + 1] == "fluorescent"
    # rpicam-vid spells the denoise modes as colour-denoise variants, so FAST
    # is not passed through under its own name.
    assert cmd[cmd.index("--denoise") + 1] == "cdn_fast"
    assert cmd[cmd.index("--sharpness") + 1] == "1.2"


def test_noise_reduction_minimal_maps_to_colour_denoise_off() -> None:
    # MINIMAL has no exact rpicam-vid counterpart; cdn_off is the closest,
    # keeping spatial denoise while dropping the colour pass.
    driver = Driver(config=Config(camera_noise_reduction_mode=NoiseReductionMode.MINIMAL))
    cmd = driver._command()

    assert cmd[cmd.index("--denoise") + 1] == "cdn_off"


def test_unknown_enum_values_are_rejected_at_load_time() -> None:
    # A typo'd TOML value must fail on startup, not reach rpicam-vid as a
    # bad argument that kills the capture process mid-race.
    with pytest.raises(ValidationError):
        Config(camera_awb_mode="sunset")
    with pytest.raises(ValidationError):
        Config(camera_noise_reduction_mode="turbo")


def test_fixed_awb_gains_replace_the_named_preset() -> None:
    # rpicam-vid ignores --awb when --awbgains is present, so emitting both
    # would advertise a preset that has no effect.
    driver = Driver(config=Config(camera_awb_gains=(1.8, 1.6)))
    cmd = driver._command()

    assert cmd[cmd.index("--awbgains") + 1] == "1.8,1.6"
    assert "--awb" not in cmd


def test_awb_gains_combined_with_a_preset_is_rejected() -> None:
    with pytest.raises(ValidationError):
        Config(camera_awb_gains=(1.8, 1.6), camera_awb_mode=AwbMode.FLUORESCENT)


def test_metering_and_ev_are_emitted_only_while_auto_exposure_is_on() -> None:
    auto = Driver(config=Config(camera_metering_mode=MeteringMode.SPOT, camera_exposure_value=0.5))._command()

    assert auto[auto.index("--metering") + 1] == "spot"
    assert auto[auto.index("--ev") + 1] == "0.5"

    # A fixed shutter turns AE off, so there is nothing for either to steer.
    manual = Driver(config=Config(camera_exposure_time_us=5000, camera_metering_mode=MeteringMode.SPOT))._command()

    assert "--metering" not in manual
    assert "--ev" not in manual


def test_flicker_period_is_passed_with_a_unit_suffix() -> None:
    driver = Driver(config=Config(camera_flicker_period_us=10000))
    cmd = driver._command()

    assert cmd[cmd.index("--flicker-period") + 1] == "10000us"


def test_flicker_period_is_omitted_when_unset() -> None:
    assert "--flicker-period" not in Driver(config=Config(camera_flicker_period_us=None))._command()
