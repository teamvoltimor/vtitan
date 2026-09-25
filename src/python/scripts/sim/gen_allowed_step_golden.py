"""Write golden cases of ``allowed_step`` for the Go port's parity test.

The Go simulator's ``collision.AllowedStep`` is a port of
``collision_stepping.allowed_step``; this records what the Python oracle
returns on a fixed, seeded set of steps near every kind of surface, so
``allowed_step_parity_test.go`` can hold the port to it (platform plan item
2.11). Regenerate after changing either side:

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        PYTHONPATH=. pixi run -e sim python scripts/sim/gen_allowed_step_golden.py
"""

from __future__ import annotations

import json
import math
import random
from pathlib import Path

from shared.config.constants.robot import RobotSpecs
from shared.domain.enums import Section

from src.navigation.track_geometry import corridor_geometry_from_widths
from src.simulation.collision_stepping import allowed_step
from src.simulation.kinematics import AckermannState
from src.simulation.track_model import ContactSurface, ObstacleBox, TrackModel

OUT = Path(__file__).resolve().parents[3] / "go/internal/sim/collision/testdata/allowed_step_python.json"
SEED = 2026
CASES_PER_SET = 400
MAX_STEP_M = 0.08
MAX_TURN_RAD = 0.15
# Share of clear (untouched) steps kept: they only test the trivial branch.
KEEP_CLEAR = 0.02

WIDTHS = {Section.NORTH: 1.0, Section.SOUTH: 0.6, Section.EAST: 1.0, Section.WEST: 0.6}
# A sign in the north corridor and a parking fin against the south outer wall.
OBSTACLES = [
    {"cx": 1.5, "cy": 2.5, "length": 0.05, "width": 0.05, "yaw": 0.0, "is_parking_lot": False},
    {"cx": 1.2, "cy": 0.1, "length": 0.2, "width": 0.02, "yaw": math.pi / 2, "is_parking_lot": True},
]
SOLID_SETS = {
    "open": [ContactSurface.INNER_WALL, ContactSurface.OBSTACLE, ContactSurface.PARKING_LOT],
    "obstacles": [ContactSurface.OUTER_WALL, ContactSurface.PARKING_LOT],
}


# 1 um off the face: closer than the smallest fraction of a step the
# bisection tries (1e-3 of 5 mm), so the step is refused outright.
_NOSE = RobotSpecs.LENGTH / 2 + 1e-6
HEAD_ON = {
    # Into the east face of the inner block (x = 0.6 with a 0.6 m west
    # corridor), and into the north face of the parking fin.
    "open": [
        (AckermannState(x=0.6 - _NOSE, y=1.5, yaw=0.0, v=0.3), 0.0),
        (AckermannState(x=1.2, y=0.2 + _NOSE, yaw=-math.pi / 2, v=0.3), -math.pi / 2),
    ],
    # Into the south and west outer walls.
    "obstacles": [
        (AckermannState(x=2.0, y=_NOSE, yaw=-math.pi / 2, v=0.3), -math.pi / 2),
        (AckermannState(x=_NOSE, y=1.5, yaw=math.pi, v=0.3), math.pi),
    ],
}


def both_modes(
    track: TrackModel, surfaces: list[ContactSurface], state: AckermannState, candidate: AckermannState
) -> list[dict]:
    """The case once scaled and once sliding."""
    out = []
    for slide in (False, True):
        got = allowed_step(track, frozenset(surfaces), state, candidate, slide=slide)
        out.append(
            {
                "solid": [s.value for s in surfaces],
                "slide": slide,
                "state": [state.x, state.y, state.yaw, state.v],
                "candidate": [candidate.x, candidate.y, candidate.yaw, candidate.v],
                "want": None if got is None else [got.x, got.y, got.yaw, got.v],
            }
        )
    return out


def main() -> None:
    """Sample the cases, run the oracle on each, and write the JSON."""
    track = TrackModel(
        corridor_geometry_from_widths(WIDTHS),
        obstacles=[ObstacleBox.from_pose(**o) for o in OBSTACLES],
    )
    rng = random.Random(SEED)  # noqa: S311 -- a repeatable fixture, not a secret
    cases = []
    for name, surfaces in SOLID_SETS.items():
        solid = frozenset(surfaces)
        kept = 0
        while kept < CASES_PER_SET:
            state = AckermannState(
                x=rng.uniform(0.0, 3.0), y=rng.uniform(0.0, 3.0), yaw=rng.uniform(-math.pi, math.pi), v=0.3
            )
            # Only starts clear of every solid surface: the gateway never
            # steps from inside one.
            if track.contact_surface(state.x, state.y, state.yaw) in solid:
                continue
            heading = state.yaw + rng.choice((0.0, math.pi)) + rng.uniform(-0.6, 0.6)
            step = rng.uniform(0.0, MAX_STEP_M)
            candidate = AckermannState(
                x=state.x + step * math.cos(heading),
                y=state.y + step * math.sin(heading),
                yaw=state.yaw + rng.uniform(-MAX_TURN_RAD, MAX_TURN_RAD),
                v=0.3,
            )
            # Keep mostly steps that touch something; the clear ones are trivial.
            blocked = track.contact_surface(candidate.x, candidate.y, candidate.yaw) in solid
            if not blocked and rng.random() > KEEP_CLEAR:
                continue
            kept += 1
            cases.extend(both_modes(track, surfaces, state, candidate))
        # Head-on: the nose a hair off each solid face, driving square into
        # it, which random sampling almost never produces.
        for state, heading in HEAD_ON[name]:
            for step in (0.005, 0.02):
                candidate = AckermannState(
                    x=state.x + step * math.cos(heading), y=state.y + step * math.sin(heading), yaw=state.yaw, v=0.3
                )
                cases.extend(both_modes(track, surfaces, state, candidate))
    OUT.write_text(
        json.dumps(
            {
                "seed": SEED,
                "chassis": [RobotSpecs.LENGTH, RobotSpecs.WIDTH],
                "widths": {s.value: w for s, w in WIDTHS.items()},
                "obstacles": OBSTACLES,
                "cases": cases,
            },
            indent=1,
        )
        + "\n"
    )
    print(f"wrote {len(cases)} cases to {OUT}")  # noqa: T201 -- CLI output


if __name__ == "__main__":
    main()
