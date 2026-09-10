"""Render the navigation HUD onto a synthetic frame.

Lets its layout/legibility be judged and iterated on without a running node,
real hardware, or even a real video -- see
docs/internal/plans/2026-08-11-navigation-hud-overlay-and-open-challenge-recording.md.

The synthetic frame is sized to match what VideoRecorder actually writes
(video_width from config/hardware/vision/node.toml, height derived from
config/hardware/camera/rpi_camera_module_3.toml's native aspect ratio), not
an arbitrary size -- so this preview reflects real recorded footage.

Usage:
    pixi run -e dev python scripts/vision/preview_hud.py [--out DIR]

Writes a handful of PNGs (typical/edge-case telemetry, each drawn over a flat
background colour -- no camera involved) to --out (default: this script's own
output/ subfolder).
"""

from __future__ import annotations

import argparse
import math
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from src.vision.hud import HudConfig, draw_logo, draw_radar, draw_stats

_CONFIG_DIR = Path(__file__).resolve().parents[3] / "config" / "hardware"


def _output_frame_size() -> tuple[int, int]:
    """(width, height) of what VideoRecorder actually writes, from the checked-in configs.

    Mirrors VideoRecorder._run's own resize math (video_recorder.py) so this
    preview matches real recorded footage instead of an arbitrary synthetic
    size. video_width comes from vision/node.toml; the source frame's aspect
    ratio comes from rpi_camera_module_3.toml -- the Picamera2 driver VisionNode
    actually instantiates for direct capture (node.py's _start_direct_capture).
    camera/config.toml's width/height (640x640) is a separate, unrelated
    StreamingDriver config that the direct-capture vision pipeline never
    reads -- using it here would silently mismatch real recorded footage,
    which is the camera's native 1536x864 (16:9), not square.
    """
    node_toml = tomllib.loads((_CONFIG_DIR / "vision" / "node.toml").read_text())
    camera_toml = tomllib.loads((_CONFIG_DIR / "camera" / "rpi_camera_module_3.toml").read_text())
    video_width = node_toml.get("video_width", 640)
    cam_width = camera_toml.get("camera_width", 1536)
    cam_height = camera_toml.get("camera_height", 864)
    return video_width, round(video_width * cam_height / cam_width)


_WIDTH, _HEIGHT = _output_frame_size()
_BACKGROUND_RGB = (40, 180, 40)  # a visible green, not the camera -- see the docstring
_HUD_CONFIG = HudConfig()  # config/hardware/vision/hud.toml -- same config draw_stats/draw_radar default to
_DEAD_AHEAD_COS_THRESHOLD = 0.99  # "basically dead ahead/astern" for this synthetic preview only
_NEAR_ZERO_SIN = 1e-3  # avoids a division blow-up exactly dead-ahead/astern, where sin(theta) -> 0


def _blank_frame() -> np.ndarray:
    frame = np.empty((_HEIGHT, _WIDTH, 3), dtype=np.uint8)
    frame[:] = _BACKGROUND_RGB
    return frame


def _synthetic_corridor_scan(width_m: float = 1.0, num_rays: int = 180) -> tuple[list[float], list[float]]:
    """A LIDAR scan as if standing centred in a straight corridor of the given width.

    Not physically exact (no wall-intersection raycast) -- close enough to check
    the radar plot actually reads as "corridor-shaped" for a preview.
    """
    angles = [-math.pi + 2 * math.pi * i / num_rays for i in range(num_rays)]
    half = width_m / 2
    ranges = []
    for theta in angles:
        side = abs(math.sin(theta))
        forward = abs(math.cos(theta))
        # Distance to the nearer of the two walls along this ray, clipped to a
        # long "open ahead/behind" range near dead-ahead/dead-astern.
        ranges.append(half / side if side > _NEAR_ZERO_SIN else _HUD_CONFIG.max_radar_range_m * 2)
        if forward > _DEAD_AHEAD_COS_THRESHOLD:
            ranges[-1] = _HUD_CONFIG.max_radar_range_m * 2
    return ranges, angles


_TYPICAL_NAV_DEBUG = {
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

_ESCAPE_NAV_DEBUG = {
    "phase": "escape",
    "direction": "counterclockwise",
    "current_corridor": "east",
    "laps_completed": 2,
    "num_laps": 3,
    "pose_yaw": math.radians(-64),
    "commanded_speed_mps": -0.101,
    "commanded_steering_norm": 0.85,
    "crosstrack_error_m": None,  # not set on the escape phase -- see NavigatorDebugSnapshot's docstring
    "risk": "high",
    "active_maneuver_type": "reverse_turn",
    "active_sign_count": 0,
}


def _render(name: str, nav_debug: dict | None, active_challenge: str | None, out_dir: Path) -> None:
    frame = _blank_frame()
    ranges, angles = _synthetic_corridor_scan()
    frame = draw_stats(frame, nav_debug, active_challenge)
    frame = draw_radar(frame, ranges, angles)
    frame = draw_logo(frame)
    path = out_dir / f"{name}.png"
    cv2.imwrite(str(path), frame[:, :, ::-1])  # RGB -> BGR for cv2.imwrite
    print(f"wrote {path}")


def main() -> None:
    """Render each sample telemetry scenario to a PNG under --out."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path(__file__).parent / "output")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    _render("typical_obstacles", _TYPICAL_NAV_DEBUG, "obstacles", args.out)
    _render("escape_maneuver", _ESCAPE_NAV_DEBUG, "obstacles", args.out)
    _render("open_challenge", {**_TYPICAL_NAV_DEBUG, "active_sign_count": None}, "open", args.out)
    _render("before_first_nav_debug", None, None, args.out)


if __name__ == "__main__":
    main()
