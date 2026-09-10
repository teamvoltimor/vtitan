"""Measure image sharpness across lens positions to pick `camera_lens_position`.

The Module 3 Wide's autofocus hunts for the whole of a run and only settles
once the robot stops, so the camera ships in MANUAL focus at a fixed dioptre.
That dioptre was chosen from the lens's theoretical hyperfocal distance, which
assumes this module's dioptre scale is calibrated -- this script replaces the
assumption with a measurement.

Sharpness is the variance of the Laplacian over a centre crop: high on crisp
edges, low on smooth ones. It is only comparable between frames of the SAME
scene, so keep the robot and the target still for the whole sweep and do not
compare numbers across runs.

Run it once per working distance and take a position that scores well at both
ends -- a peak at 0.6 m that collapses at 2.5 m is worse than a flatter curve
covering the range, because signs must be detected long before they are close.

Usage (from ``src``, robot stationary, target in frame)::

    pixi run python scripts/hardware/diag_focus_sweep.py --label "sign at 0.6m"
    pixi run python scripts/hardware/diag_focus_sweep.py --label "sign at 2.5m"
"""

from __future__ import annotations

import argparse
import logging
import time

import cv2
import numpy as np

from src.hardware.camera.rpicam.driver import AfMode, Config, Driver
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()
log = logging.getLogger(__name__)

# rpicam-vid needs a moment after start before the lens has settled and
# auto-exposure has converged; frames before that are not comparable.
_SETTLE_SEC = 1.5
_FRAMES_PER_POSITION = 5
# Sharpness is measured on a centre crop so that a busy background (or the
# floor at the frame edge) cannot outvote the target.
_CROP_FRACTION = 0.5
_COLOUR_FRAME_DIMS = 3
# A peak this close to its runner-up is noise, not a focus optimum.
_FLAT_CURVE_RATIO = 1.1


def _sharpness(frame: np.ndarray) -> float:
    """Variance of the Laplacian over the centre crop."""
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == _COLOUR_FRAME_DIMS else frame
    height, width = grey.shape[:2]
    half_h = int(height * _CROP_FRACTION / 2)
    half_w = int(width * _CROP_FRACTION / 2)
    crop = grey[height // 2 - half_h : height // 2 + half_h, width // 2 - half_w : width // 2 + half_w]
    return float(cv2.Laplacian(crop, cv2.CV_64F).var())


def _measure(position: float) -> float:
    """Median sharpness of a few frames captured at one lens position.

    Median, not mean: a single dropped or torn MJPEG frame would otherwise
    drag the score for an otherwise sharp position.
    """
    driver = Driver(Config(camera_af_mode=AfMode.MANUAL, camera_lens_position=position))
    driver.connect()
    try:
        time.sleep(_SETTLE_SEC)
        scores = [_sharpness(driver.capture_frame().frame) for _ in range(_FRAMES_PER_POSITION)]
    finally:
        driver.close()
    return float(np.median(scores))


def main() -> int:
    """Sweep the lens across a dioptre range and report the sharpest position."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, default=0.0, help="first lens position, dioptres (0 = infinity)")
    parser.add_argument("--stop", type=float, default=3.0, help="last lens position, dioptres")
    parser.add_argument("--step", type=float, default=0.2, help="dioptre increment")
    parser.add_argument("--label", default="", help="what the camera is pointed at, echoed in the report")
    args = parser.parse_args()

    log.info("Starting focus sweep", extra={DETAILS_KEY: {"target": args.label or "(unlabelled)"}})

    results: list[tuple[float, float]] = []
    for raw in np.arange(args.start, args.stop + args.step / 2, args.step):
        position = float(raw)
        score = _measure(position)
        results.append((position, score))
        log.info(
            "Measured lens position",
            extra={
                DETAILS_KEY: {
                    "dioptres": round(position, 2),
                    "focus_distance_m": round(1 / position, 2) if position > 0 else None,
                    "sharpness": round(score, 1),
                }
            },
        )

    best_position, best_score = max(results, key=lambda r: r[1])
    others = [score for position, score in results if position != best_position]
    flat = bool(others) and best_score < max(others) * _FLAT_CURVE_RATIO

    log.info(
        "Sweep complete",
        extra={
            DETAILS_KEY: {
                "target": args.label or "(unlabelled)",
                "best_dioptres": round(best_position, 2),
                "best_focus_distance_m": round(1 / best_position, 2) if best_position > 0 else None,
                "best_sharpness": round(best_score, 1),
                "curve_is_flat": flat,
            }
        },
    )
    if flat:
        log.warning(
            "Peak is within the flat-curve margin of another position, so this run does not "
            "discriminate between them -- aim at a target with more fine detail and repeat.",
            extra={DETAILS_KEY: {"flat_curve_ratio": _FLAT_CURVE_RATIO}},
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
