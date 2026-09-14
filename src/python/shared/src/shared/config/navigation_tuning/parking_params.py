"""Parallel-parking maneuver tuning group.

Fields are inherited from the generated DTO
(:mod:`shared.config.generated.navigation.parking.parking_schema`). This module
adds only the tuning layer's shipped fallbacks, so a bare ``ParkingParams()``
still matches the checked-in ``parking.toml`` without re-declaring a field.
"""

from __future__ import annotations

from typing import Any, ClassVar

from shared.config.generated.navigation.parking.parking_schema import (
    NavigationParkingParking,
)
from shared.config.navigation_tuning._shared import TuningModel


class ParkingParams(TuningModel, NavigationParkingParking):
    """Parallel-parking maneuver parameters.

    Attributes (inherited from the generated DTO):
        parallel_tolerance_m: WRO rule max allowed wheel-to-wall distance
            difference (m) for "parallel" parking.
        pos_reach_dist_m: Distance (m) threshold for "reached staging
            position."
        default_max_frames: Max control ticks before the parking maneuver
            gives up (20s @ 20Hz by default).
        saturated_steer_threshold: Normalized steering magnitude counted as
            "at physical lock."
        saturation_stuck_ticks: Consecutive ticks of saturated steering
            before triggering a reverse-reorient.
        speed: Constant driving speed (m/s) during the parking maneuver.
        min_lookahead_dist_m: Floor distance (m) to avoid near-zero-distance
            curvature blow-up in pure pursuit.
        wall_standoff_m: Closest the chassis footprint may approach the
            field wall backing the parking lot.
        marker_standoff_m: Closest the chassis footprint may approach a
            parking-bay marker fin.
        attempt_after_final_lap: Whether to pursue the parking bay once the
            final lap is counted. ``False`` stops in the finish section
            instead.
        derive_lot_from_in_bay_start: Build the parking lot from the START
            POSE when metadata carries none.
    """

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "parallel_tolerance_m": 0.02,
        "pos_reach_dist_m": 0.04,
        "default_max_frames": 400,
        "saturated_steer_threshold": 0.999,
        "saturation_stuck_ticks": 20,
        "speed": 0.12,
        "min_lookahead_dist_m": 0.02,
        "wall_standoff_m": 0.05,
        "marker_standoff_m": 0.01,
        "attempt_after_final_lap": False,
        "derive_lot_from_in_bay_start": True,
    }

    # derive_lot_from_in_bay_start:
    # Build the parking lot from the START POSE when metadata carries none.
    #
    # Without this, parking is unreachable code on hardware. Blind runs give the
    # navigator only ``starting_conditions``, so ``park_controller_from_metadata``
    # finds no ``parking_lot`` and returns None -- and measured over 255 bags a
    # ParkController has NEVER been constructed on this robot: 0 of 227 readable,
    # with 67 of them reaching three laps, so the lap precondition was met 67 times
    # and parking still never engaged. Every parking figure this project holds
    # comes from the simulator.
    #
    # The lot needs no sensing to be located: in the Obstacles Challenge the robot
    # STARTS INSIDE IT, so the start pose IS the lot, with the two fins
    # ``BLOCK_SPACING_FACTOR`` chassis lengths apart (0.45 m) along the wall.
    #
    # ON by default because "it never even tried" is a worse failure than "it tried
    # and could not", and the operator asked to SEE it attempt the manoeuvre.
    #
    # IT WILL VERY PROBABLY FAIL, and that is not this flag's doing. Depth is
    # 0.194 m of chassis against a 0.20 m pocket -- 6 mm of total slack, +/-3 mm on
    # the centre -- and the maximum heading error is 1.16 deg against the 6.0 deg
    # the rule allows, while measured cross-track at sign passes is 46.3 mm.
    #
    # attempt_after_final_lap:
    # ``False`` (shipped) ends the round the way rule 1.3 pays for: the lap
    # counter increments at the along-track CENTRE of the 1 m start straight, so
    # the robot is already inside the finish section at that instant, and
    # ``WaypointParams.FINISH_APPROACH_M`` has capped the approach to
    # ``slow_mps`` so the ~0.09 m coast stays inside the 0.50 m of section left
    # ahead of it.
    #
    # Shipped False because the pursuit is a large net LOSS while the bay is
    # geometrically unreachable (0.194 m chassis into a 0.20 m bay). Measured
    # blind on the 256 corpus, parking OFF against ON, one invocation:
    # ``in-time`` 158 vs 62, collisions 4 vs 51, timeouts 61 vs 110 -- while
    # ``laps>=3`` is IDENTICAL at 159.
