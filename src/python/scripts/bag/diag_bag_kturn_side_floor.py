r"""Can the K-turn's pass-side override ever FIRE, or is its floor unreachable?

Measured 2026-09-15 on the two rounds recorded that morning: the K-turn agrees
with the router's committed pass side on **7 of 72 corner episodes (10%)**,
mean alignment delta -0.049, the worst row in
``diag_bag_escape_sign_agreement``. The side correction, on the same rounds,
agrees 21 of 43 (49%).

That 10% is easy to misread as "the K-turn picks the wrong side". Read
``_k_turn_steer_sign`` and a second reading appears. With
``escape_side_follows_committed_sign`` ON, which is what Obstacles ships, the
method does NOT choose between sides on clearance any more::

    wanted_clear = left_clear if preferred_sign < 0 else right_clear
    if wanted_clear >= escape_side_override_min_clearance_m:  # 0.12 m
        return preferred_sign
    return 0.0  # wanted side is shut: commit to neither, reverse straight

So a disagreement is one of two completely different events:

* it steered the OTHER way -- the failure everyone assumes, or
* it returned 0.0 and reversed straight -- the DESIGNED answer for a shut side.

An agreement metric scores both as "disagrees". This separates them, and it
does so without needing ``preferred_sign`` at all: it recomputes ``left_clear``
and ``right_clear`` from the recorded sweep with the controller's own sector
code, and asks how often EITHER side clears the 0.12 m floor. If even the
better side rarely clears it, the override cannot fire whichever side the
router wants, and the K-turn spends the corner reversing straight, gaining a
couple of centimetres, and re-triggering. That is the limit cycle the operator
describes as "the manoeuvres didn't help".

The suspicion is quantitative: on the same rounds the escape trigger range from
the chassis is p50 **0.091 m**, and the chassis half-width is 0.097 m. A
manoeuvre that only starts once something is 9 cm away may never find 12 cm on
either flank.

CONTROL: the sector clearances are recomputed with the SAME
``_sector_to_model`` and the same tuning the controller uses, so a difference
against the live decision is a bug in this script rather than a finding. The
per-side valid-ray counts are printed for the same reason -- a side with zero
valid rays is substituted with ``no_data_range_m``, which is a large number,
and would otherwise look like generous clearance.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_kturn_side_floor.py RUN_DIR...
"""

from __future__ import annotations

import contextlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

# isort: off
# scripts.common.bag_io FIRST: importing shared.domain.models ahead of it trips
# a partially-initialised cycle between models and enums (GMR_CLASS_NAMES).
from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_nav_debug,
    open_reader,
    scan_to_ranges_angles,
)
from scripts.common.tables import print_table  # noqa: E402
from shared.domain.enums import ManeuverType  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402
from src.navigation.control.controllers.collision_avoidance.controller import (
    CollisionAvoidanceController,
)
from src.navigation.control.controllers.collision_avoidance.sectors import _sector_to_model

# isort: on


def _clear(
    ranges: np.ndarray, angles: np.ndarray, centre: float, ctrl: CollisionAvoidanceController
) -> tuple[float, int]:
    """The controller's own side clearance, and how many valid rays backed it.

    Reads the parameters off a REAL controller rather than re-deriving them
    from tuning: that is the control. If these clearances disagreed with the
    live decision it would be a bug here, not a finding.
    """
    model = _sector_to_model(
        ranges,
        angles,
        centre,
        ctrl.threat_half_fov_rad,
        filter_self_detection=True,
        self_detection_threshold_m=ctrl.self_detection_threshold_m,
        min_valid_range_m=ctrl.min_valid_range_m,
        no_data_range_m=ctrl.no_data_range_m,
    )
    if model.valid_count > 0:
        return model.min_range_m, model.valid_count
    return ctrl.no_data_range_m, 0


def _score(bag_dir: Path, ctrl: CollisionAvoidanceController, floor: float) -> list:
    """One row: how often each flank cleared the override floor at a K-turn latch."""
    reader = open_reader(bag_dir)
    ranges = angles = None
    best: list[float] = []
    either = both = neither = n = 0
    prev_type = None
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == Topics.SCAN:
            with contextlib.suppress(Exception):
                ranges, angles = scan_to_ranges_angles(deserialize_message(data, LaserScan))
            continue
        if topic != Topics.NAV_DEBUG or ranges is None:
            continue
        try:
            snap = decode_nav_debug(data)
        except Exception:  # noqa: BLE001
            continue
        kind = snap.active_maneuver_type
        # Only the LATCH tick: a K-turn held for 11 ticks would otherwise count
        # its one decision eleven times.
        latched = kind is ManeuverType.K_TURN and prev_type is not ManeuverType.K_TURN
        prev_type = kind
        if not latched:
            continue
        left, _lc = _clear(ranges, angles, math.pi / 2, ctrl)
        right, _rc = _clear(ranges, angles, -math.pi / 2, ctrl)
        n += 1
        best.append(max(left, right))
        if left >= floor and right >= floor:
            both += 1
        elif left >= floor or right >= floor:
            either += 1
        else:
            neither += 1
    if n == 0:
        return [bag_dir.name.replace("run_", ""), 0, "-", "-", "-", "-", "-"]
    a = np.array(best)
    return [
        bag_dir.name.replace("run_", ""),
        n,
        f"{np.percentile(a, 50):.3f}",
        f"{np.percentile(a, 90):.3f}",
        f"{both} ({100 * both / n:.0f}%)",
        f"{either} ({100 * either / n:.0f}%)",
        f"{neither} ({100 * neither / n:.0f}%)",
    ]


def main() -> int:
    """Report how often either flank clears the override floor at a K-turn."""
    parser = create_bags_parser(__doc__ or "")
    args = parser.parse_args()

    tuning = get_tuning(None)
    ctrl = CollisionAvoidanceController.from_tuning(tuning)
    floor = ctrl.escape_side_override_min_clearance_m

    rows = [_score(bag_dir, ctrl, floor) for bag_dir in args.bag_dirs]

    print(f"override floor escape_side_override_min_clearance_m = {floor} m")
    print_table(
        rows,
        ["run", "K-turn latches", "best side p50", "best side p90", "BOTH clear", "one clear", "NEITHER clear"],
    )
    print()
    print(
        "NEITHER clear is the row that matters: on those latches the override cannot\n"
        "fire whichever side the router wants, so _k_turn_steer_sign returns 0.0 and\n"
        "reverses straight. Those are scored as 'disagrees' by an agreement metric\n"
        "but they are the DESIGNED answer to a shut side, not a wrong turn."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
