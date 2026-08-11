"""Draw a navigation stats HUD and a mini LIDAR radar onto a frame.

Composited onto the per-run recorded video (see
docs/internal/plans/2026-08-11-navigation-hud-overlay-and-open-challenge-recording.md)
so the video shows not just what the camera saw but what the robot decided --
a "visual mcap." Kept as pure numpy/cv2 functions with no ROS2 dependency, so
the design can be iterated on and previewed (scripts/vision/preview_hud.py)
without a running node, hardware, or even a real video.

Both functions render onto whatever resolution they're handed and never raise
on missing data -- a field not yet present in the latest NavigatorDebugSnapshot
(e.g. before the first /nav_debug message, or a field that's None on the
current phase -- see NavigatorDebugSnapshot's own docstring) renders as "--",
never a stale prior value and never a crash.

Layout: a status panel top-left (challenge/phase/direction/corridor/lap/
heading -- "where things stand"), a control panel top-right (speed/steer/
crosstrack/risk/maneuver/signs -- "what the robot is doing about it right
now"), and the LIDAR radar bottom-right. All pixel sizes, colours and the
radar's display range are tuning constants, not literals -- see HudConfig /
config/hardware/vision/hud.toml, same pattern every other hardware/vision
config in this repo follows (VisionNode's own Config, NavigationTuning, etc.).

The HEADING line is the "gyroscope" stat -- deliberately read from
NavigatorDebugSnapshot's pose_yaw (already IMU-quaternion-fused by the pose
estimator), not from a raw /imu/data angular_velocity subscription, which
reads zero on this hardware (a known quirk from earlier work; see memory).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

if TYPE_CHECKING:
    from collections.abc import Sequence

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_RGB = tuple[int, int, int]


class HudConfig(HardwareBaseSettings):
    """Tuning constants for the navigation HUD overlay.

    Sourced from config/hardware/vision/hud.toml (env prefix VISION_HUD_,
    same override precedence as every other HardwareBaseSettings config in
    this repo -- init/env win, then the TOML file, then these field
    defaults). Colours are RGB (the recorder's own working order until the
    final RGB->BGR conversion in VideoRecorder._run).
    """

    model_config = SettingsConfigDict(env_prefix="vision_hud_", toml_file=CONFIG_DIR / "vision" / "hud.toml")

    font_scale: float = 0.45
    line_height_px: int = 18
    text_rgb: _RGB = (255, 255, 255)
    panel_rgb: _RGB = (0, 0, 0)
    panel_alpha: float = 0.55
    margin_px: int = 8
    column_gap_px: int = 10
    """Gap between a panel's label column and its value column. cv2's font
    isn't monospace, so this is real pixel spacing measured per-panel from
    each label's actual rendered width (see _draw_panel), not padding baked
    into the label strings -- string-padding assumes a fixed character
    width, which produces a ragged, unaligned value column in a proportional
    font."""

    radar_radius_px: int = 70
    radar_margin_px: int = 12
    radar_bg_rgb: _RGB = (0, 0, 0)
    radar_bg_alpha: float = 0.55
    radar_ring_rgb: _RGB = (90, 90, 90)
    radar_point_rgb: _RGB = (80, 255, 80)
    radar_robot_rgb: _RGB = (255, 220, 80)
    max_radar_range_m: float = 3.0


_DEFAULT_HUD_CONFIG = HudConfig()


def _fmt(value: Any, unit: str = "") -> str:
    """Render a NavigatorDebugSnapshot field, or "--" for None/missing."""
    if value is None:
        return "--"
    if isinstance(value, float):
        return f"{value:.2f}{unit}"
    return f"{value}{unit}"


def _fmt_heading_deg(pose_yaw_rad: float | None) -> str:
    """Heading in degrees, from the estimator's own IMU-fused pose_yaw.

    Not raw IMU angular_velocity, which reads zero on this hardware -- see
    the module docstring's "gyroscope" note. pose_yaw is already the correct
    source: same quaternion-derived heading telemetry_bridge_node computes
    for the OLED, just read from NavigatorDebugSnapshot instead of a second
    /imu/data subscription.
    """
    if pose_yaw_rad is None:
        return "--"
    return f"{math.degrees(pose_yaw_rad):.0f}deg"


def _status_lines(nav_debug: dict | None, active_challenge: str | None) -> list[tuple[str, str]]:
    """Top-left panel: where things stand."""
    d = nav_debug or {}
    laps_completed = d.get("laps_completed")
    num_laps = d.get("num_laps")
    lap = "--" if laps_completed is None or num_laps is None else f"{laps_completed}/{num_laps}"
    return [
        ("CHALLENGE", _fmt(active_challenge)),
        ("PHASE", _fmt(d.get("phase"))),
        ("DIR", _fmt(d.get("direction"))),
        ("CORRIDOR", _fmt(d.get("current_corridor"))),
        ("LAP", lap),
        ("HEADING", _fmt_heading_deg(d.get("pose_yaw"))),
    ]


def _control_lines(nav_debug: dict | None) -> list[tuple[str, str]]:
    """Top-right panel: what the robot is doing about it right now."""
    d = nav_debug or {}
    return [
        ("SPEED", _fmt(d.get("commanded_speed_mps"), " m/s")),
        ("STEER", _fmt(d.get("commanded_steering_norm"))),
        ("XTRACK", _fmt(d.get("crosstrack_error_m"), " m")),
        ("RISK", _fmt(d.get("risk"))),
        ("MANEUVER", _fmt(d.get("active_maneuver_type"))),
        ("SIGNS", _fmt(d.get("active_sign_count"))),
    ]


def _text_width(text: str, config: HudConfig) -> int:
    (w, _), _ = cv2.getTextSize(text, _FONT, config.font_scale, 1)
    return w


def _panel_size(rows: list[tuple[str, str]], config: HudConfig) -> tuple[int, int, int]:
    """Return (panel_width, panel_height, value_column_x_offset).

    The value column sits at a fixed x for every row, measured from the
    widest LABEL actually in this panel (cv2's font isn't monospace, so this
    -- not padding the label strings with spaces -- is what makes the value
    column actually line up).
    """
    label_w = max(_text_width(label, config) for label, _ in rows)
    value_w = max(_text_width(value, config) for _, value in rows)
    value_col_x = config.margin_px + label_w + config.column_gap_px
    panel_w = value_col_x + value_w + config.margin_px
    panel_h = config.margin_px * 2 + config.line_height_px * len(rows)
    return panel_w, panel_h, value_col_x


def _draw_panel(canvas: np.ndarray, rows: list[tuple[str, str]], *, top: bool, left: bool, config: HudConfig) -> None:
    """Draw one shaded, column-aligned text panel anchored to a corner of *canvas*, in place."""
    height, width = canvas.shape[:2]
    panel_w, panel_h, value_col_x = _panel_size(rows, config)
    panel_w = min(width, panel_w)
    panel_h = min(height, panel_h)
    x0 = 0 if left else max(0, width - panel_w)
    y0 = 0 if top else max(0, height - panel_h)

    region = canvas[y0 : y0 + panel_h, x0 : x0 + panel_w]
    shaded = np.full_like(region, config.panel_rgb)
    canvas[y0 : y0 + panel_h, x0 : x0 + panel_w] = cv2.addWeighted(
        shaded, config.panel_alpha, region, 1 - config.panel_alpha, 0,
    )

    for i, (label, value) in enumerate(rows):
        y = y0 + config.margin_px + config.line_height_px * (i + 1) - 4
        if y >= y0 + panel_h:
            break  # panel ran out of room (tiny frame) -- draw what fits, never raise
        cv2.putText(canvas, label, (x0 + config.margin_px, y), _FONT, config.font_scale, config.text_rgb, 1, cv2.LINE_AA)
        cv2.putText(canvas, value, (x0 + value_col_x, y), _FONT, config.font_scale, config.text_rgb, 1, cv2.LINE_AA)


def draw_stats(
    canvas: np.ndarray,
    nav_debug: dict | None,
    active_challenge: str | None = None,
    config: HudConfig | None = None,
) -> np.ndarray:
    """Return a copy of *canvas* with status (top-left) and control (top-right) panels.

    Args:
        canvas: Frame in RGB order.
        nav_debug: The latest parsed NavigatorDebugSnapshot JSON, or None before
            the first /nav_debug message arrives.
        active_challenge: "open"/"obstacles"/None, shown on the status panel.
        config: Tuning constants; defaults to the checked-in config/hardware/vision/hud.toml.

    Returns:
        A new array; the input is left untouched, same contract as overlay.annotate.
    """
    config = config or _DEFAULT_HUD_CONFIG
    out = np.ascontiguousarray(canvas).copy()
    _draw_panel(out, _status_lines(nav_debug, active_challenge), top=True, left=True, config=config)
    _draw_panel(out, _control_lines(nav_debug), top=True, left=False, config=config)
    return out


def draw_radar(
    canvas: np.ndarray,
    ranges_m: Sequence[float] | None,
    angles_rad: Sequence[float] | None,
    max_range_m: float | None = None,
    config: HudConfig | None = None,
) -> np.ndarray:
    """Return a copy of *canvas* with a small LIDAR radar plot in the bottom-right corner.

    0 rad is straight ahead (up on the plot); angle increases counter-clockwise,
    matching the robot's own body frame -- not screen/compass convention.

    Args:
        canvas: Frame in RGB order.
        ranges_m: Latest LaserScan ranges, or None/empty before the first /scan message.
        angles_rad: Matching per-ray angles (same length as ranges_m).
        max_range_m: Range that maps to the radar circle's outer edge; anything
            farther (including inf/nan, both of which a real LaserScan can carry
            for a no-return ray) is clipped to the edge rather than dropped, so
            a wide-open corridor still shows a ring instead of a hole. Defaults
            to config.max_radar_range_m.
        config: Tuning constants; defaults to the checked-in config/hardware/vision/hud.toml.

    Returns:
        A new array; the input is left untouched.
    """
    config = config or _DEFAULT_HUD_CONFIG
    max_range_m = max_range_m if max_range_m is not None else config.max_radar_range_m
    out = np.ascontiguousarray(canvas).copy()
    height, width = out.shape[:2]
    radius = config.radar_radius_px
    # The background box sits flush in the bottom-right corner -- (width, height)
    # is its actual bottom-right pixel, same as the stats panels sit flush in
    # their corners -- with radar_margin_px as padding *inside* that box
    # between its edge and the circle, the same role margin_px plays for text
    # inside a stats panel. box_extent is the box's half-size along each axis.
    box_extent = radius + config.radar_margin_px
    cx = width - box_extent
    cy = height - box_extent
    if cx - box_extent < 0 or cy - box_extent < 0:
        return out  # frame too small for the radar to fit -- skip rather than draw garbage

    bg_box = out[cy - box_extent : height, cx - box_extent : width]
    shaded = np.full_like(bg_box, config.radar_bg_rgb)
    out[cy - box_extent : height, cx - box_extent : width] = cv2.addWeighted(
        shaded, config.radar_bg_alpha, bg_box, 1 - config.radar_bg_alpha, 0,
    )
    cv2.circle(out, (cx, cy), radius, config.radar_ring_rgb, 1, cv2.LINE_AA)
    cv2.circle(out, (cx, cy), radius // 2, config.radar_ring_rgb, 1, cv2.LINE_AA)

    if ranges_m and angles_rad:
        for r, theta in zip(ranges_m, angles_rad, strict=False):
            if r is None or math.isnan(r):
                continue
            clipped = min(r, max_range_m)  # inf (a real "no return" LaserScan value) clips fine too
            px = cx + int((clipped / max_range_m) * radius * np.sin(theta))
            py = cy - int((clipped / max_range_m) * radius * np.cos(theta))
            if 0 <= px < width and 0 <= py < height:
                cv2.circle(out, (px, py), 1, config.radar_point_rgb, -1, cv2.LINE_AA)

    cv2.drawMarker(out, (cx, cy), config.radar_robot_rgb, cv2.MARKER_TRIANGLE_UP, 8, 2, cv2.LINE_AA)

    return out
