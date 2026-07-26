"""Is a compliant park geometrically achievable at all with this chassis?

The bay is a POCKET, not a slot: each magenta block is a fin standing perpendicular to the
outer wall spanning the full 0.20 m depth (verified: block at (1.50, 2.90) yaw pi/2 ->
x in [1.49, 1.51], y in [2.80, 3.00]). So the only opening is the corridor side, and the
entry has to be lateral -- i.e. a genuine parallel park, not a drive-through.

Two reports:

  analytic — containment budget as a function of chassis width and residual heading error.
  search   — brute-force search over two-arc reverse parallel-park trajectories under real
             Ackermann physics, scoring collision-free runs by how far the footprint still
             protrudes from the bay.
"""

from __future__ import annotations

import math
import sys

from shared.config.constants import CorridorDimensions, ParkingLotSpecs, RobotSpecs
from shared.config.enums import Section

from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.track_model import ObstacleBox, TrackModel, _convex_overlap, _rect_corners

_DT = 0.05
_BAY_DEPTH = ParkingLotSpecs.LENGTH
_TRACK_WIDTHS = dict.fromkeys(Section, CorridorDimensions.OBSTACLES_WIDTH)

# Fixture 0000 (NORTH): fins at x = 1.50 and 1.95, y = 2.90, outer wall at y = 3.0.
_FIN_A, _FIN_B, _FIN_LAT = 1.50, 1.95, 2.90
_WALL = 3.0
_BAY = (
    _FIN_A + ParkingLotSpecs.WIDTH / 2,
    _WALL - _BAY_DEPTH,
    _FIN_B - ParkingLotSpecs.WIDTH / 2,
    _WALL,
)


def _fins() -> list[ObstacleBox]:
    return [
        ObstacleBox.from_pose(
            cx=cx,
            cy=_FIN_LAT,
            length=ParkingLotSpecs.LENGTH,
            width=ParkingLotSpecs.WIDTH,
            yaw=math.pi / 2,
        )
        for cx in (_FIN_A, _FIN_B)
    ]


def _protrusion(x: float, y: float, yaw: float) -> float:
    """Max distance any chassis corner sticks out of the bay rectangle (0 = contained)."""
    x_min, y_min, x_max, y_max = _BAY
    worst = 0.0
    for cx, cy in _rect_corners(x, y, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH):
        worst = max(worst, x_min - cx, cx - x_max, y_min - cy, cy - y_max)
    return max(0.0, worst)


def report_analytic() -> None:
    """Containment budget: swept width vs bay depth, and the depth each chassis width needs."""
    print("=== Containment budget ===\n")
    print(f"bay: {_BAY_DEPTH:.2f} m deep x {_BAY[2] - _BAY[0]:.2f} m long")
    print(f"chassis: {RobotSpecs.LENGTH:.2f} x {RobotSpecs.WIDTH:.2f} m")
    print(f"longitudinal slack: {_BAY[2] - _BAY[0] - RobotSpecs.LENGTH:.3f} m total\n")

    print("Swept half-width of the chassis at heading error theta (must be <= bay_depth/2 = 0.100):")
    print(f"{'theta':>7} {'half-sweep':>11} {'total':>8} {'fits?':>6}")
    for deg in (0, 1, 2, 3, 5, 8):
        th = math.radians(deg)
        half = RobotSpecs.WIDTH / 2 * math.cos(th) + RobotSpecs.LENGTH / 2 * math.sin(th)
        print(f"{deg:>6}d {half:>11.4f} {2 * half:>8.4f} {'yes' if 2 * half <= _BAY_DEPTH else 'NO':>6}")

    print("\nBay depth needed for a given chassis width, with per-side margin (theta = 0):")
    print(f"{'width':>7} {'m=0':>7} {'m=1cm':>7} {'m=2cm':>7}")
    for w in (0.20, 0.18, 0.16, 0.14):
        print(f"{w:>7.2f} {w:>7.2f} {w + 0.02:>7.2f} {w + 0.04:>7.2f}")

    print("\nSimulator's outer wall collision margin: 0.04 m (mesh 0.18 vs visual 0.10 thickness).")
    print("-> to avoid a modelled wall contact the centre must sit >= 0.14 m from the wall,")
    print(f"   but to be contained in a {_BAY_DEPTH:.2f} m bay it must sit <= 0.10 m from it. Contradiction.")


def _simulate_two_arc(
    lat0: float,
    ahead: float,
    n1: int,
    n2: int,
    n3: int,
    steer_sign: float,
    speed: float = 0.12,
) -> tuple[bool, float, tuple[float, float, float]]:
    """Reverse two-arc parallel park. Returns (collided, final protrusion, final pose).

    Starts parallel to the wall at ``lat0`` metres from it, having already driven ``ahead``
    metres past the bay (travel is -x at yaw=pi, so "past" is -x and the bay is behind the
    robot, reachable in reverse), then: reverse at full lock toward the wall (n1 ticks),
    reverse at full opposite lock to straighten (n2 ticks), forward creep to centre (n3).
    """
    track = TrackModel(_TRACK_WIDTHS, obstacles=_fins())
    kin = AckermannKinematics()
    bay_mid_x = (_BAY[0] + _BAY[2]) / 2
    state = AckermannState(x=bay_mid_x - ahead, y=_WALL - lat0, yaw=math.pi)

    plan = [(-speed, steer_sign, n1), (-speed, -steer_sign, n2), (speed, 0.0, n3)]
    for tgt_speed, steer, n in plan:
        for _ in range(n):
            state = kin.step(state, target_speed=tgt_speed, target_steer_norm=steer, dt=_DT)
            if track.footprint_collides(state.x, state.y, state.yaw):
                return True, math.inf, (state.x, state.y, state.yaw)
    return False, _protrusion(state.x, state.y, state.yaw), (state.x, state.y, state.yaw)


def report_search() -> None:
    """Brute-force two-arc reverse parallel-park search; report the least-protruding result."""
    print("\n=== Two-arc reverse parallel park: brute-force search ===\n")
    # Counter-phase steering pivots about the chassis centre, so the turn reference length is
    # WHEELBASE/2, not WHEELBASE -- the bicycle-model formula printed here previously reported
    # twice the radius the trajectories below are actually flown at.
    r_min = RobotSpecs.WHEELBASE / 2.0 / math.tan(RobotSpecs.MAX_STEERING_ANGLE)
    print(f"R_min = {r_min:.3f} m (counter-phase 4WS, wheel lock {math.degrees(RobotSpecs.MAX_STEERING_ANGLE):.1f} deg)")
    print("searching lat0 x ahead x arc durations, both steer senses, under real Ackermann physics\n")

    best = None
    tried = collision_free = 0
    for lat0 in (0.28, 0.32, 0.36, 0.40, 0.45, 0.50):
        for ahead in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
            for n1 in range(4, 30, 2):
                for n2 in range(4, 30, 2):
                    for n3 in (0, 2, 4, 6):
                        for sign in (1.0, -1.0):
                            tried += 1
                            collided, prot, pose = _simulate_two_arc(lat0, ahead, n1, n2, n3, sign)
                            if collided:
                                continue
                            collision_free += 1
                            yaw_err = abs(math.atan2(math.sin(pose[2] - math.pi), math.cos(pose[2] - math.pi)))
                            score = (prot, yaw_err)
                            if best is None or score < best[0]:
                                best = (score, lat0, ahead, n1, n2, n3, sign, pose)

    print(f"trajectories tried={tried}  collision-free={collision_free}")
    if best is None:
        print("no collision-free trajectory found")
        return
    (prot, yaw_err), lat0, ahead, n1, n2, n3, sign, pose = best
    print(f"\nbest: lat0={lat0:.2f} ahead={ahead:.2f} arcs={n1}/{n2}/{n3} steer_sign={sign:+.0f}")
    print(f"  final pose = ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):+.1f} deg)")
    print(f"  heading error vs wall-parallel = {math.degrees(yaw_err):.1f} deg")
    print(f"  protrusion from bay = {prot * 100:.1f} cm  ({'CONTAINED' if prot == 0 else 'not contained'})")


def _hits_fins_or_wall(x: float, y: float, yaw: float, wall_margin: float) -> bool:
    """Collision against the fins and the outer wall, with a configurable wall margin.

    ``wall_margin=0.04`` reproduces the simulator's fat collision mesh; ``0.0`` is the
    physical mat, which is what the real robot actually has to clear.
    """
    corners = _rect_corners(x, y, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
    if any(cy > _WALL - wall_margin for _, cy in corners):
        return True
    return any(_convex_overlap(corners, fin.to_box().corners(), yaw) for fin in _fins())


def report_shuffle() -> None:
    """Incremental shuffle park: alternating reverse/forward arcs that translate sideways.

    A two-arc park cannot enter a pocket this shallow, but a shuffle can walk the chassis in
    a few millimetres at a time while returning to parallel each cycle. This measures how far
    in it gets, how many cycles it costs, and whether the simulator's fat wall mesh is what
    blocks it or the real geometry does.
    """
    print("\n=== Incremental shuffle park ===\n")
    for wall_margin, label in ((0.04, "simulator collision mesh"), (0.0, "physical mat")):
        kin = AckermannKinematics()
        bay_mid_x = (_BAY[0] + _BAY[2]) / 2
        state = AckermannState(x=bay_mid_x, y=_WALL - 0.26, yaw=math.pi)
        best_lat = _WALL - state.y
        cycles_used = 0
        blocked = False

        for cycle in range(40):
            snapshot = state
            # Reverse arc toward the wall, then forward arc to straighten back to parallel.
            ok = True
            for tgt_speed, steer, n in ((-0.08, -1.0, 6), (0.08, 1.0, 6)):
                for _ in range(n):
                    nxt = kin.step(state, target_speed=tgt_speed, target_steer_norm=steer, dt=_DT)
                    if _hits_fins_or_wall(nxt.x, nxt.y, nxt.yaw, wall_margin):
                        ok = False
                        break
                    state = nxt
                if not ok:
                    break
            if not ok:
                state = snapshot
                blocked = True
                break
            cycles_used = cycle + 1
            best_lat = min(best_lat, _WALL - state.y)

        yaw_err = abs(math.atan2(math.sin(state.yaw - math.pi), math.cos(state.yaw - math.pi)))
        prot = _protrusion(state.x, state.y, state.yaw)
        print(f"{label:<26} wall_margin={wall_margin:.2f}")
        print(f"  cycles completed = {cycles_used}{' (blocked)' if blocked else ''}")
        print(f"  closest centre-to-wall reached = {best_lat:.3f} m (need 0.100 for containment)")
        print(f"  final heading error = {math.degrees(yaw_err):.1f} deg")
        print(f"  protrusion from bay = {prot * 100:.1f} cm\n")


def main() -> None:
    """Run the report named on the command line (default: all)."""
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which in ("analytic", "all"):
        report_analytic()
    if which in ("search", "all"):
        report_search()
    if which in ("shuffle", "all"):
        report_shuffle()


if __name__ == "__main__":
    main()
