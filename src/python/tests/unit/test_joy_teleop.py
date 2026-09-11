"""Unit tests for the joystick -> Ackermann command mapping."""

from __future__ import annotations

from src.teleop.config import Config
from src.teleop.mapping import compute_command


def _make_config(**overrides) -> Config:
    defaults = {
        "steering_axis_index": 0,
        "throttle_axis_index": 4,
        "deadman_button_index": 6,
        "max_steering_deg": 30.0,
        "max_speed_mps": 0.3,
    }
    defaults.update(overrides)
    return Config(**defaults)


def _axes(steering: float = 0.0, throttle: float = 0.0) -> list[float]:
    axes = [0.0] * 8
    axes[0] = steering
    axes[4] = throttle
    return axes


def _buttons(deadman: int = 0) -> list[int]:
    buttons = [0] * 8
    buttons[6] = deadman
    return buttons


def test_steering_moves_without_deadman_held():
    config = _make_config()
    speed, steering = compute_command(_axes(steering=0.5), _buttons(deadman=0), config, joy_is_fresh=True)
    assert speed == 0.0
    assert steering == 15.0


def test_speed_zero_when_deadman_not_held():
    config = _make_config()
    speed, _ = compute_command(_axes(throttle=1.0), _buttons(deadman=0), config, joy_is_fresh=True)
    assert speed == 0.0


def test_speed_follows_throttle_when_deadman_held():
    config = _make_config()
    speed, _ = compute_command(_axes(throttle=0.5), _buttons(deadman=1), config, joy_is_fresh=True)
    assert speed == 0.15


def test_speed_zero_when_joy_is_stale_even_with_deadman_held():
    config = _make_config()
    speed, _ = compute_command(_axes(throttle=1.0), _buttons(deadman=1), config, joy_is_fresh=False)
    assert speed == 0.0


def test_speed_clamped_to_max():
    config = _make_config()
    speed, _ = compute_command(_axes(throttle=2.0), _buttons(deadman=1), config, joy_is_fresh=True)
    assert speed == 0.3


def test_steering_clamped_to_max():
    config = _make_config()
    _, steering = compute_command(_axes(steering=-2.0), _buttons(), config, joy_is_fresh=True)
    assert steering == -30.0


def test_steering_invert_flips_sign():
    config = _make_config(steering_invert=True)
    _, steering = compute_command(_axes(steering=0.5), _buttons(), config, joy_is_fresh=True)
    assert steering == -15.0


def test_throttle_invert_flips_sign():
    config = _make_config(throttle_invert=True)
    speed, _ = compute_command(_axes(throttle=0.5), _buttons(deadman=1), config, joy_is_fresh=True)
    assert speed == -0.15
