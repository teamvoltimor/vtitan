"""Sweep harness for the Obstacles Challenge sign-avoidance investigation.

Runs all 16 Go-generated obstacles fixtures under a parameterised
``NavigationTuning`` / ``SignRouterConfig`` and reports the four metrics that
must always be read together (see
``platform/robot/docs/sign-avoidance-investigation.md``): collisions, laps>=1,
laps>=3 and timeouts. Tracking collisions alone has already produced one wrong
conclusion in that log.

``success`` is deliberately not the headline: it also requires ``parked``, and
parking is independently blocked by chassis-vs-pocket geometry, so it stays
0/16 regardless of any driving change. ``laps>=3`` is the driving-success
metric.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_sign_sweep.py diagnose          # run this one first
    python scripts/diag_sign_sweep.py lookahead 0.12 0.20 0.30 0.40
    python scripts/diag_sign_sweep.py arc 0.25 0.30 0.35 0.40 0.45
    python scripts/diag_sign_sweep.py crosstrack 0.12 0.20

Swept modes (``lookahead`` ``arc`` ``speed`` ``offset`` ``buffer``
``crosstrack``) take the values to sweep as positional arguments. Fixed
comparison modes (``baseline`` ``profile`` ``diagnose`` ``ghost`` ``lidar``)
ignore them.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.constants import DictKeys
from shared.config.navigation_tuning import NavigationTuning

import src.navigation.planning.sign_router as sign_router_module
import src.simulation.gateway as gateway_module
from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.gateway import ScenarioSimulator
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.track_model import TrackModel, obstacles_from_metadata

if TYPE_CHECKING:
    from collections.abc import Callable

MAX_STEPS = 6000
"""Matches ``tests/unit/test_obstacles_challenge_sim.py``."""

_TARGET_LAPS = 3
"""Laps a scenario must finish to count as a driving success."""

_DEPTH_BUFFER_ATTR = "_DEFORM_DEPTH_BUFFER"
"""Module-level knob in ``sign_router`` with no public seam, swept via setattr."""


@dataclass(frozen=True, slots=True)
class SweepConfig:
    """One point in a parameter sweep."""

    label: str
    lookahead_short: float | None = None
    lookahead_long: float | None = None
    fast_speed: float | None = None
    arc_radius: float | None = None
    steer_kp: float | None = None
    max_steering_rate: float | None = None
    strip_obstacles: bool = False
    """Delete signs and the parking lot from the metadata entirely.

    Isolates the path tracker from sign avoidance: the corridor layout, start
    pose and lap count stay identical, but there is nothing to route around and
    nothing to hit. If a scenario still fails here, the failure is not a sign
    problem.
    """

    strip_signs: bool = False
    """Delete only the traffic signs, keeping the parking lot physical."""

    strip_parking: bool = False
    """Delete only the parking lot, keeping the traffic signs physical."""

    ghost_signs: bool = False
    """Keep signs in the metadata — so ``SignRouter`` still routes around them —
    but remove them from the track's collision/raycast set.

    Separates "the router aims badly" from "the sign stopped the run". With
    physical signs a run dies at its first bad pass, hiding every later one.
    """

    lateral_offset: float | None = None
    """Override ``SignRouterConfig.lateral_offset`` (default 0.20 m)."""

    lidar_blind: bool = False
    """Keep signs physically collidable but invisible to the LIDAR.

    Splits the two things ``fd33fd5`` made true at once. If results move, the
    failure runs through perception and the collision controller's escape
    maneuver rather than through the waypoint path the sign router deforms.
    """

    deform_depth_buffer: float | None = None
    """Override ``sign_router._DEFORM_DEPTH_BUFFER`` (default 0.30 m).

    How far past the inner square's own [CORNER_MIN, CORNER_MAX] span the
    lookahead target may sit and still be deformed. Since the target leads the
    robot by up to ``LOOKAHEAD_LONG``, too small a buffer switches avoidance off
    during the final approach to any sign at grid depth 1.0 or 2.0.
    """

    def tuning(self) -> NavigationTuning:
        """Materialise the ``NavigationTuning`` this config asks for.

        Starts from plain defaults, not ``for_obstacles()`` — the point of the
        sweep is to re-derive that profile, so it must not be baked into the
        baseline.
        """
        base = NavigationTuning()
        pursuit = base.pursuit
        if self.lookahead_short is not None:
            pursuit = replace(pursuit, LOOKAHEAD_SHORT=self.lookahead_short)
        if self.lookahead_long is not None:
            pursuit = replace(pursuit, LOOKAHEAD_LONG=self.lookahead_long)
        if self.steer_kp is not None:
            pursuit = replace(pursuit, STEER_KP=self.steer_kp)
        if self.max_steering_rate is not None:
            pursuit = replace(pursuit, MAX_STEERING_RATE=self.max_steering_rate)
        speed = base.speed
        if self.fast_speed is not None:
            speed = replace(speed, FAST_SPEED=self.fast_speed)
        waypoints = base.waypoints
        if self.arc_radius is not None:
            waypoints = replace(waypoints, ARC_RADIUS=self.arc_radius)
        return replace(base, pursuit=pursuit, speed=speed, waypoints=waypoints)


@dataclass(frozen=True, slots=True)
class ScenarioOutcome:
    """Per-scenario result, reduced to what the sweep reports on."""

    label: str
    collided: bool
    laps: int
    timed_out: bool
    collision_xy: tuple[float, float] | None
    collision_kind: str
    collision_step: int
    steps: int


def _classify_collision(metadata: dict[str, Any], pose: tuple[float, float, float]) -> str:
    """Name what the chassis was overlapping when the run ended.

    "Collisions" is a single counter covering three unrelated failures — outer
    wall, inner keep-out block, and an actual sign or parking block. Sweeping a
    sign-avoidance parameter against a number dominated by wall contacts
    measures the wrong thing, so every sweep reports the split.
    """
    x, y, yaw = pose
    widths = corridor_widths_from_metadata(metadata)
    if TrackModel(widths).footprint_collides(x, y, yaw):
        return "wall"
    signs_only = {k: v for k, v in metadata.items() if k != DictKeys.PARKING_LOT}
    if TrackModel(widths, obstacles=obstacles_from_metadata(signs_only)).footprint_collides(x, y, yaw):
        return "sign"
    parking_only = {k: v for k, v in metadata.items() if k != DictKeys.SIGN_POSITIONS}
    if TrackModel(widths, obstacles=obstacles_from_metadata(parking_only)).footprint_collides(x, y, yaw):
        return "parking"
    return "none"


def _run_one(args: tuple[int, SweepConfig]) -> ScenarioOutcome:
    index, config = args
    scenario = all_obstacles_demo_scenarios()[index]
    metadata = scenario.metadata
    dropped = set()
    if config.strip_obstacles or config.strip_signs:
        dropped.add(DictKeys.SIGN_POSITIONS)
    if config.strip_obstacles or config.strip_parking:
        dropped.add(DictKeys.PARKING_LOT)
    if dropped:
        metadata = {k: v for k, v in metadata.items() if k not in dropped}

    restore: list[tuple[Any, str, Any]] = []
    if config.ghost_signs:
        original = gateway_module.obstacles_from_metadata
        restore.append((gateway_module, "obstacles_from_metadata", original))
        gateway_module.obstacles_from_metadata = lambda md: original(
            {k: v for k, v in md.items() if k != DictKeys.SIGN_POSITIONS},
        )
    if config.lateral_offset is not None:
        original_cfg = sign_router_module.SignRouterConfig
        restore.append((sign_router_module, "SignRouterConfig", original_cfg))
        offset = config.lateral_offset
        # SignRouter calls ``SignRouterConfig()`` with no arguments, so swapping
        # in a zero-arg factory is enough to retune it without a public seam.
        sign_router_module.SignRouterConfig = lambda: replace(original_cfg(), lateral_offset=offset)
    if config.lidar_blind:
        original_track = gateway_module.TrackModel
        restore.append((gateway_module, "TrackModel", original_track))
        gateway_module.TrackModel = lambda widths, obstacles=None: original_track(
            widths,
            obstacles=obstacles,
            lidar_sees_obstacles=False,
        )
    if config.deform_depth_buffer is not None:
        restore.append((sign_router_module, _DEPTH_BUFFER_ATTR, getattr(sign_router_module, _DEPTH_BUFFER_ATTR)))
        setattr(sign_router_module, _DEPTH_BUFFER_ATTR, config.deform_depth_buffer)

    try:
        sim = ScenarioSimulator(
            metadata,
            num_laps=scenario.laps,
            seed=scenario.seed,
            tuning=config.tuning(),
        )
        result = sim.run(max_steps=MAX_STEPS)
    finally:
        for module, name, value in restore:
            setattr(module, name, value)

    classify_meta = metadata
    if config.ghost_signs:
        classify_meta = {k: v for k, v in metadata.items() if k != DictKeys.SIGN_POSITIONS}
    kind = _classify_collision(classify_meta, result.final_pose) if result.collided else "none"
    return ScenarioOutcome(
        label=scenario.label,
        collided=result.collided,
        laps=result.laps_completed,
        timed_out=result.timed_out,
        collision_xy=result.collision_xy,
        collision_kind=kind,
        collision_step=result.steps,
        steps=result.steps,
    )


@dataclass(frozen=True, slots=True)
class SweepResult:
    """Aggregate of one config across all 16 fixtures."""

    config: SweepConfig
    outcomes: list[ScenarioOutcome]

    @property
    def collisions(self) -> int:
        """Scenarios that ended in a terminal collision."""
        return sum(1 for o in self.outcomes if o.collided)

    @property
    def laps_ge_1(self) -> int:
        """Scenarios that completed at least one lap."""
        return sum(1 for o in self.outcomes if o.laps >= 1)

    @property
    def laps_ge_3(self) -> int:
        """Scenarios that completed the full three laps — the driving-success metric."""
        return sum(1 for o in self.outcomes if o.laps >= _TARGET_LAPS)

    @property
    def timeouts(self) -> int:
        """Scenarios that ran out of step budget."""
        return sum(1 for o in self.outcomes if o.timed_out)

    def kind(self, name: str) -> int:
        """Scenarios whose collision was of the given kind (wall/sign/parking)."""
        return sum(1 for o in self.outcomes if o.collision_kind == name)

    def row(self) -> str:
        """The one-line summary: all four metrics plus the collision-kind split."""
        n = len(self.outcomes)
        return (
            f"RESULT {self.config.label:<32} "
            f"collisions {self.collisions:>2}/{n} "
            f"(wall {self.kind('wall'):>2} sign {self.kind('sign'):>2} park {self.kind('parking'):>2})  "
            f"laps>=1 {self.laps_ge_1:>2}/{n}  "
            f"laps>=3 {self.laps_ge_3:>2}/{n}  "
            f"timeouts {self.timeouts:>2}/{n}"
        )

    def detail(self) -> str:
        """Per-scenario rows, for when an aggregate needs breaking down."""
        return "\n".join(
            f"DETAIL   {o.label:<34} {o.collision_kind:<9} laps={o.laps} steps={o.steps} "
            f"at={None if o.collision_xy is None else (round(o.collision_xy[0], 2), round(o.collision_xy[1], 2))}"
            for o in self.outcomes
        )


def run_sweep(configs: list[SweepConfig], workers: int, verbose: bool = False) -> list[SweepResult]:
    """Run every config over every fixture, fixtures fanned out across processes."""
    scenario_count = len(all_obstacles_demo_scenarios())
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for config in configs:
            outcomes = list(pool.map(_run_one, [(i, config) for i in range(scenario_count)]))
            result = SweepResult(config, outcomes)
            if verbose:
                print(result.detail(), flush=True)
            print(result.row(), flush=True)
            results.append(result)
    return results


# Cross-track error measurement


def _cross_track_errors(args: tuple[int, float | None]) -> list[float]:
    """Per-tick distance from the robot to its own planned path.

    Run with signs and parking stripped, deliberately. With signs present the
    router deforms the lookahead target on purpose, so distance-to-nominal-path
    measures intended avoidance rather than tracking error — and the run ends
    in a collision after a few hundred ticks anyway. Stripping them gives the
    honest "how far off its own line does the chassis sit" number that the
    +-6.7 cm sign-pass slack budget has to be compared against.

    ``lookahead`` dominates this measurement — a longer lookahead cuts corners —
    so it is an explicit argument rather than whatever the default happens to
    be. ``None`` means the shipped default.
    """
    index, lookahead = args
    scenario = all_obstacles_demo_scenarios()[index]
    metadata = {k: v for k, v in scenario.metadata.items() if k not in (DictKeys.SIGN_POSITIONS, DictKeys.PARKING_LOT)}
    config = SweepConfig(
        "crosstrack", lookahead_short=lookahead, lookahead_long=None if lookahead is None else lookahead * 2
    )
    sim = ScenarioSimulator(metadata, num_laps=scenario.laps, seed=scenario.seed, tuning=config.tuning())
    path = sim.waypoints
    segments = list(itertools.pairwise(path))
    errors: list[float] = []

    def record(state: Any, _scan: Any) -> None:
        errors.append(min(_point_segment_dist(state.x, state.y, a, b) for a, b in segments))

    sim.run(max_steps=MAX_STEPS, on_step=record)
    return errors


def _point_segment_dist(px: float, py: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    """Perpendicular distance from a point to the segment ``a``-``b``.

    Distance to the nearest *waypoint* would overstate cross-track error by up
    to half the waypoint spacing — enough to matter for a number being compared
    against a +-6.7 cm budget.
    """
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    if span == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / span))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _percentile(values: list[float], q: float) -> float:
    """Linear-interpolated percentile ``q`` (0-1) of ``values``."""
    if not values:
        return math.nan
    ordered = sorted(values)
    pos = q * (len(ordered) - 1)
    lo = math.floor(pos)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def report_cross_track(workers: int, lookaheads: list[float]) -> None:
    """Print pooled cross-track error, one row per lookahead setting.

    With no values given, measures the shipped default only.
    """
    scenario_count = len(all_obstacles_demo_scenarios())
    settings: list[float | None] = list(lookaheads) if lookaheads else [None]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for lookahead in settings:
            per_scenario = pool.map(_cross_track_errors, [(i, lookahead) for i in range(scenario_count)])
            pooled = [e for errors in per_scenario for e in errors]
            label = "default (0.20/0.40)" if lookahead is None else f"lookahead {lookahead:.2f}/{lookahead * 2:.2f}"
            print(
                f"CROSSTRACK {label:<24} "
                f"median {_percentile(pooled, 0.5) * 100:5.1f}cm  "
                f"p90 {_percentile(pooled, 0.9) * 100:5.1f}cm  "
                f"max {max(pooled) * 100:5.1f}cm",
                flush=True,
            )


_SWEPT_MODES: dict[str, Callable[[float], SweepConfig]] = {
    "lookahead": lambda v: SweepConfig(
        f"lookahead {v:.2f}/{v * 2:.2f}",
        lookahead_short=v,
        lookahead_long=v * 2,
    ),
    "arc": lambda v: SweepConfig(f"arc_radius {v:.2f}", arc_radius=v),
    "speed": lambda v: SweepConfig(f"fast_speed {v:.2f}", fast_speed=v),
    "offset": lambda v: SweepConfig(f"lateral_offset {v:.3f}", lateral_offset=v),
    # The offset sweep CROSSED with lidar_blind, which is the only way to see
    # that the two interact. Swept alone, `offset` is byte-identical from 0.20
    # to 0.32 — the reactive escape layer fires at every sign pass and decides
    # the run before the router's aim can matter, so the knob reads inert.
    # Take the signs away from the collision controller (they stay physical,
    # so collisions are still real) and the same knob becomes monotone:
    # 16/16 -> 15/16 -> 14/16 over 0.20/0.24/0.28. Neither single-knob mode
    # shows this; `offset` says the router does nothing and `lidar` says
    # perception is a wash.
    "masked-offset": lambda v: SweepConfig(
        f"lateral_offset {v:.3f}, lidar blind to signs",
        lateral_offset=v,
        lidar_blind=True,
    ),
    "buffer": lambda v: SweepConfig(f"depth_buffer {v:.2f}", deform_depth_buffer=v),
}
"""Modes that sweep one numeric knob across the values given on the CLI."""

_FIXED_MODES: dict[str, list[SweepConfig]] = {
    "baseline": [SweepConfig("defaults")],
    "profile": [
        SweepConfig("defaults"),
        SweepConfig("for_obstacles (removed)", lookahead_short=0.12, lookahead_long=0.24, fast_speed=0.30),
    ],
    # Separates "the tracker broke" from "sign avoidance failed" — run this
    # first on any change; no aggregate collision count can tell them apart.
    "diagnose": [
        SweepConfig("everything physical"),
        SweepConfig("no signs, parking physical", strip_signs=True),
        SweepConfig("signs physical, no parking", strip_parking=True),
        SweepConfig("nothing physical", strip_obstacles=True),
    ],
    "ghost": [
        SweepConfig("ghost signs, router on", ghost_signs=True),
        SweepConfig("ghost signs, router off", ghost_signs=True, lateral_offset=0.0),
        SweepConfig("physical signs, router off", lateral_offset=0.0),
    ],
    "lidar": [
        SweepConfig("lidar sees signs (default)"),
        SweepConfig("lidar blind to signs", lidar_blind=True),
        SweepConfig("lidar blind, router off", lidar_blind=True, lateral_offset=0.0),
    ],
}
"""Modes with a fixed comparison set, ignoring any CLI values."""

MODES = ("crosstrack", *_FIXED_MODES, *_SWEPT_MODES)


def _build_configs(mode: str, values: list[float]) -> list[SweepConfig]:
    """Map a CLI mode plus its numeric arguments to the configs to run."""
    if mode in _FIXED_MODES:
        return _FIXED_MODES[mode]
    if mode in _SWEPT_MODES:
        return [_SWEPT_MODES[mode](v) for v in values]
    msg = f"unhandled mode {mode!r}"
    raise ValueError(msg)


def main() -> None:
    """Parse arguments and run the requested sweep."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        choices=MODES,
    )
    parser.add_argument("values", nargs="*", type=float)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--verbose", action="store_true", help="print per-scenario detail rows")
    args = parser.parse_args()

    if args.mode == "crosstrack":
        report_cross_track(args.workers, args.values)
        return

    run_sweep(_build_configs(args.mode, args.values), args.workers, verbose=args.verbose)


if __name__ == "__main__":
    main()
