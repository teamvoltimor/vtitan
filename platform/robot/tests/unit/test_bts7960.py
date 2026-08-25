"""Unit tests for the BTS7960/IBT-2 drive backend (pure, no hardware).

``Driver`` only imports ``gpiozero`` lazily inside ``connect()``, so these
tests exercise everything reachable without it: config defaults and the
not-connected guard on ``run_drive_forward``/``run_drive_reverse``. Assumes
the external demux described in docs/bts7960-ibt2-wiring.md.
"""

from __future__ import annotations

import pytest

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import DriveDriver
from src.hardware.motors.bts7960.config import Bts7960PwmConfig
from src.hardware.motors.bts7960.driver import Driver


class TestBts7960PwmConfig:
    def test_defaults(self):
        config = Bts7960PwmConfig()
        assert config.pwmchip == 0
        assert config.pwm_channel == 1
        assert config.frequency_hz == 1000


class TestDriver:
    def test_is_drive_driver(self):
        assert isinstance(
            Driver(pwm_pin=13, dir_select_pin=5, r_en_pin=6, l_en_pin=26),
            DriveDriver,
        )

    def test_run_drive_forward_before_connect_raises(self):
        driver = Driver(pwm_pin=13, dir_select_pin=5, r_en_pin=6, l_en_pin=26)
        with pytest.raises(MotorConnectionError):
            driver.run_drive_forward(50)

    def test_run_drive_reverse_before_connect_raises(self):
        driver = Driver(pwm_pin=13, dir_select_pin=5, r_en_pin=6, l_en_pin=26)
        with pytest.raises(MotorConnectionError):
            driver.run_drive_reverse(50)

    def test_no_feedback_of_its_own(self):
        driver = Driver(pwm_pin=13, dir_select_pin=5, r_en_pin=6, l_en_pin=26)
        assert driver.get_drive_position() == 0.0
        assert driver.get_drive_speed() == 0.0
