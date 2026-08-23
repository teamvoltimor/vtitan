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
now"), the LIDAR radar bottom-right, and the team mark watermarked bottom-left
(assets/vision/voltimor-mark.png -- see scripts/vision/_make_hud_logo.py
for how it was derived from the brand asset) -- purely cosmetic, unlike
the other three. All pixel sizes, colours and the radar's display
range are tuning constants, not literals -- see HudConfig /
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

from src.hardware.settings_base import CONFIG_DIR, ROBOT_ROOT, HardwareBaseSettings

if TYPE_CHECKING:
    from collections.abc import Sequence

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

    font_face: int = cv2.FONT_HERSHEY_DUPLEX
    """Duplex, not Simplex -- a cleaner double-stroke face that reads as a
    modern geometric sans at this size, instead of Simplex's single-stroke
    "typewriter" look. cv2 can't load real TTF fonts without extra system
    libs, so this is the built-in ceiling for "professional" rather than a
    placeholder choice."""
    font_scale: float = 0.55
    text_thickness: int = 1
    line_height_px: int = 22
    text_rgb: _RGB = (248, 250, 252)
    """Value-column colour -- near-white for maximum contrast against the
    dim slate panel."""
    label_rgb: _RGB = (168, 178, 190)
    """Label-column colour -- still dimmer than text_rgb so the eye lands on
    values (what the robot is doing) before labels (what they mean), the
    same label/value convention a flight HUD or telemetry dashboard uses,
    but light enough to stay legible on its own against the dark panel."""
    accent_rgb: _RGB = (0, 194, 255)
    """Single accent colour reused everywhere something should read as
    "live"/"foreground": the panel's inner-edge bar and the radar points --
    one accent, not a colour per element, is what keeps the overlay reading
    as minimalist rather than a rainbow of debug colours."""
    border_rgb: _RGB = (58, 64, 72)
    panel_rgb: _RGB = (12, 14, 18)
    panel_alpha: float = 0.55
    margin_px: int = 10
    column_gap_px: int = 14
    """Gap between a panel's label column and its value column. cv2's font
    isn't monospace, so this is real pixel spacing measured per-panel from
    each label's actual rendered width (see _draw_panel), not padding baked
    into the label strings -- string-padding assumes a fixed character
    width, which produces a ragged, unaligned value column in a proportional
    font."""

    radar_radius_px: int = 90
    radar_margin_px: int = 12
    radar_bg_rgb: _RGB = (12, 14, 18)
    radar_bg_alpha: float = 0.55
    radar_ring_rgb: _RGB = (58, 64, 72)
    radar_crosshair_rgb: _RGB = (36, 40, 46)
    radar_point_rgb: _RGB = (30, 136, 229)
    """Sampled from the team mark's body/circuit blue (assets/vision/
    voltimor-mark.png), not the panel's cyan accent_rgb -- distinct from the
    accent bars so the radar points read as "the logo's blue," not just
    another use of the same UI accent colour."""
    radar_robot_rgb: _RGB = (255, 255, 255)
    max_radar_range_m: float = 3.0

    logo_size_px: int = 128
    logo_margin_px: int = 8
    logo_alpha: float = 0.85
    """Multiplies the mark PNG's own per-pixel alpha -- a light watermark, not
    a solid sticker, so it doesn't compete with the radar for attention."""


_DEFAULT_HUD_CONFIG = HudConfig()
_LOGO_PATH = ROBOT_ROOT / "assets" / "vision" / "voltimor-mark.png"


def _load_logo_rgba(path: object) -> np.ndarray | None:
    """Load the team mark as RGBA, or None if the asset is missing/unreadable.

    Never raises -- draw_logo falls back to drawing nothing, the same
    never-crash-the-recording contract the rest of this module holds for
    missing telemetry. Loaded once at import time since it's a small, static
    asset re-read on every frame otherwise.
    """
    bgra = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if bgra is None or bgra.ndim != 3 or bgra.shape[2] != 4:
        return None
    b, g, r, a = cv2.split(bgra)
    return cv2.merge([r, g, b, a])


_LOGO_RGBA = _load_logo_rgba(_LOGO_PATH)


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
    (w, _), _ = cv2.getTextSize(text, config.font_face, config.font_scale, config.text_thickness)
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
    """Draw one shaded, column-aligned text panel anchored to a corner of *canvas*, in place.

    Styled as a minimalist glass panel: a dim slate fill, a hairline border,
    and a single accent-coloured bar on the panel's inner edge (the edge
    facing the centre of the frame) instead of a full accent outline -- a
    quieter "this is live telemetry" cue than a bright box around every panel.
    """
    height, width = canvas.shape[:2]
    panel_w, panel_h, value_col_x = _panel_size(rows, config)
    panel_w = min(width, panel_w)
    panel_h = min(height, panel_h)
    x0 = 0 if left else max(0, width - panel_w)
    y0 = 0 if top else max(0, height - panel_h)
    x1, y1 = x0 + panel_w, y0 + panel_h

    region = canvas[y0:y1, x0:x1]
    shaded = np.full_like(region, config.panel_rgb)
    canvas[y0:y1, x0:x1] = cv2.addWeighted(shaded, config.panel_alpha, region, 1 - config.panel_alpha, 0)
    cv2.rectangle(canvas, (x0, y0), (x1 - 1, y1 - 1), config.border_rgb, 1, cv2.LINE_AA)
    accent_x = (x1 - 1) if left else x0
    cv2.line(canvas, (accent_x, y0), (accent_x, y1 - 1), config.accent_rgb, 2, cv2.LINE_AA)

    for i, (label, value) in enumerate(rows):
        y = y0 + config.margin_px + config.line_height_px * (i + 1) - 4
        if y >= y1:
            break  # panel ran out of room (tiny frame) -- draw what fits, never raise
        cv2.putText(
            canvas,
            label,
            (x0 + config.margin_px, y),
            config.font_face,
            config.font_scale,
            config.label_rgb,
            config.text_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            value,
            (x0 + value_col_x, y),
            config.font_face,
            config.font_scale,
            config.text_rgb,
            config.text_thickness,
            cv2.LINE_AA,
        )


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

    box_x0, box_y0 = cx - box_extent, cy - box_extent
    bg_box = out[box_y0:height, box_x0:width]
    shaded = np.full_like(bg_box, config.radar_bg_rgb)
    out[box_y0:height, box_x0:width] = cv2.addWeighted(
        shaded, config.radar_bg_alpha, bg_box, 1 - config.radar_bg_alpha, 0
    )
    cv2.rectangle(out, (box_x0, box_y0), (width - 1, height - 1), config.border_rgb, 1, cv2.LINE_AA)

    # Faint crosshair through the centre, clipped to the ring -- a quiet
    # "this is a plot, not a decoration" cue, dimmer than the ring itself so
    # it reads as structure rather than another line competing for attention.
    cv2.line(out, (cx - radius, cy), (cx + radius, cy), config.radar_crosshair_rgb, 1, cv2.LINE_AA)
    cv2.line(out, (cx, cy - radius), (cx, cy + radius), config.radar_crosshair_rgb, 1, cv2.LINE_AA)
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


def draw_logo(canvas: np.ndarray, config: HudConfig | None = None) -> np.ndarray:
    """Return a copy of *canvas* with the team mark watermarked into the bottom-left corner.

    Purely cosmetic branding, unlike draw_stats/draw_radar's telemetry -- so a
    missing or unreadable assets/vision/voltimor-mark.png (e.g. a fresh
    checkout that hasn't pulled LFS/binary assets yet) skips the logo
    entirely rather than raising, same never-crash-the-recording contract as
    a missing NavigatorDebugSnapshot field.

    Args:
        canvas: Frame in RGB order.
        config: Tuning constants; defaults to the checked-in config/hardware/vision/hud.toml.

    Returns:
        A new array; the input is left untouched.
    """
    config = config or _DEFAULT_HUD_CONFIG
    out = np.ascontiguousarray(canvas).copy()
    if _LOGO_RGBA is None:
        return out
    height, width = out.shape[:2]
    size = config.logo_size_px
    x0, y0 = config.logo_margin_px, height - config.logo_margin_px - size
    if size <= 0 or x0 < 0 or y0 < 0 or x0 + size > width:
        return out  # frame too small for the mark to fit -- skip rather than draw garbage

    mark = cv2.resize(_LOGO_RGBA, (size, size), interpolation=cv2.INTER_AREA)
    region = out[y0 : y0 + size, x0 : x0 + size].astype(np.float32)
    mark_rgb = mark[:, :, :3].astype(np.float32)
    alpha = (mark[:, :, 3:4].astype(np.float32) / 255.0) * config.logo_alpha
    out[y0 : y0 + size, x0 : x0 + size] = (mark_rgb * alpha + region * (1 - alpha)).astype(np.uint8)

    return out
