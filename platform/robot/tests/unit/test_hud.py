"""Tests for the navigation HUD overlay.

Mirrors test_vision_overlay.py's style: the HUD is only ever seen by a human
(or, now, a recorded video), so nothing downstream catches it being wrong.
These pin the parts that would silently produce a blank/misleading/crashing
frame -- including missing telemetry, which happens on every real race before
the first /nav_debug or /scan message arrives.
"""

from __future__ import annotations

import math

import numpy as np

from src.vision.hud import HudConfig, _fmt_heading_deg, draw_radar, draw_stats


def _blank(width: int = 640, height: int = 360) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


_NAV_DEBUG = {
    "phase": "normal_drive",
    "direction": "clockwise",
    "current_corridor": "south",
    "laps_completed": 1,
    "num_laps": 3,
    "pose_yaw": math.radians(182),
    "commanded_speed_mps": 0.133,
    "commanded_steering_norm": -0.22,
    "crosstrack_error_m": 0.031,
    "risk": "low",
    "active_maneuver_type": None,
    "active_sign_count": 2,
}


class TestHeadingFromPoseYaw:
    """The HEADING line -- deliberately read from pose_yaw, not raw IMU
    angular_velocity, which reads zero on this hardware (see hud.py's module
    docstring)."""

    def test_converts_radians_to_degrees(self) -> None:
        assert _fmt_heading_deg(math.pi) == "180deg"

    def test_none_before_the_first_nav_debug_message(self) -> None:
        assert _fmt_heading_deg(None) == "--"


class TestDrawStats:
    def test_draws_something_for_populated_telemetry(self) -> None:
        out = draw_stats(_blank(), _NAV_DEBUG, "obstacles")
        assert out.any()

    def test_leaves_the_input_untouched(self) -> None:
        frame = _blank()
        draw_stats(frame, _NAV_DEBUG, "obstacles")
        assert not frame.any()

    def test_none_nav_debug_still_draws_a_panel_not_a_crash(self) -> None:
        """Before the first /nav_debug message arrives -- every real race's first tick."""
        out = draw_stats(_blank(), None, None)
        assert out.any()

    def test_a_field_missing_from_this_phase_does_not_crash(self) -> None:
        """crosstrack_error_m is only set on the normal_drive phase -- see
        NavigatorDebugSnapshot's own docstring. Must render, not raise."""
        out = draw_stats(_blank(), {"phase": "escape", "crosstrack_error_m": None}, "obstacles")
        assert out.any()

    def test_status_panel_is_top_left_and_control_panel_is_top_right(self) -> None:
        frame = _blank(width=640, height=360)
        out = draw_stats(frame, _NAV_DEBUG, "obstacles")
        top_left = out[:60, :150]
        top_right = out[:60, -150:]
        bottom_half = out[200:, :]
        assert top_left.any(), "expected the status panel top-left"
        assert top_right.any(), "expected the control panel top-right"
        assert not bottom_half.any(), "stats must not land in the bottom half (that's the radar's territory)"

    def test_config_colours_are_actually_used(self) -> None:
        config = HudConfig(panel_rgb=(1, 2, 3), text_rgb=(4, 5, 6))
        out = draw_stats(_blank(), _NAV_DEBUG, "obstacles", config=config)
        assert (out == np.array([1, 2, 3])).all(axis=-1).any(), "expected the configured panel colour somewhere"


class TestDrawRadar:
    def test_no_scan_draws_the_ring_but_no_points(self) -> None:
        out = draw_radar(_blank(), None, None)
        assert out.any(), "the background/ring must still draw"

    def test_empty_scan_does_not_raise(self) -> None:
        out = draw_radar(_blank(), [], [])
        assert out.any()

    def test_leaves_the_input_untouched(self) -> None:
        frame = _blank()
        draw_radar(frame, [1.0], [0.0])
        assert not frame.any()

    def test_nan_and_inf_ranges_are_handled_not_raised(self) -> None:
        """A real LaserScan can carry both for a no-return ray."""
        out = draw_radar(_blank(), [float("nan"), float("inf"), 1.0], [0.0, math.pi / 2, math.pi], max_range_m=3.0)
        assert out.any()

    def test_radar_lands_in_the_bottom_right(self) -> None:
        frame = _blank(width=640, height=360)
        out = draw_radar(frame, [1.0] * 8, [i * math.pi / 4 for i in range(8)], max_range_m=3.0)
        top_half = out[:180, :]
        bottom_left = out[180:, :400]
        bottom_right = out[180:, 400:]
        assert not top_half.any()
        assert not bottom_left.any()
        assert bottom_right.any()

    def test_too_small_a_frame_skips_the_radar_instead_of_raising(self) -> None:
        out = draw_radar(_blank(width=20, height=20), [1.0], [0.0])
        assert out is not None  # must not raise; drawing nothing is the correct behaviour here
