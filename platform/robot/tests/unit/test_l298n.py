"""Unit tests for the L298N drive backend (pure, no hardware).

``Driver`` only imports ``gpiozero`` lazily inside ``connect()``, so these
tests exercise everything reachable without it: config defaults and the
not-connected guard on ``run_drive_forward``/``run_drive_reverse``.
"""

from __future__ import annotations

import pytest

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import DriveDriver
from src.hardware.motors.l298n.config import L298nPwmConfig
from src.hardware.motors.l298n.driver import Driver


class TestL298nPwmConfig:
    def test_defaults(self):
        config = L298nPwmConfig()
        assert config.pwmchip == 0
        assert config.pwm_channel == 1
        assert config.frequency_hz == 1000


class TestDriver:
    def test_is_drive_driver(self):
        assert isinstance(Driver(pwm_pin=13, dir_a_pin=5, dir_b_pin=6), DriveDriver)

    def test_run_drive_forward_before_connect_raises(self):
        driver = Driver(pwm_pin=13, dir_a_pin=5, dir_b_pin=6)
        with pytest.raises(MotorConnectionError):
            driver.run_drive_forward(50)

    def test_run_drive_reverse_before_connect_raises(self):
        driver = Driver(pwm_pin=13, dir_a_pin=5, dir_b_pin=6)
        with pytest.raises(MotorConnectionError):
            driver.run_drive_reverse(50)

    def test_no_feedback_of_its_own(self):
        driver = Driver(pwm_pin=13, dir_a_pin=5, dir_b_pin=6)
        assert driver.get_drive_position() == 0.0
        assert driver.get_drive_speed() == 0.0
