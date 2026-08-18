"""Parking diagnostics against the real Go-generated bay geometry.

Three reports, all driving real Ackermann physics with the parking blocks present as
physical obstacles (which ``tests/unit/test_parking.py`` still does not do):

  contacts    — sweep approach error through ParkController; classify what the chassis hits.
  containment — of the runs ParkController calls "parked", how many put the whole chassis
                footprint inside the bay rectangle? (the `done` flag is a centre-in-box test)
  straightin  — feasibility probe for the recommended wall-parallel straight-in entry:
                what lateral/heading precision does entering the slot actually require?

Usage: python scripts/sim/diag_park_sweep.py [contacts|containment|straightin|all]
"""

from __future__ import annotations

import math
import sys

from shared.config.constants import CorridorDimensions, ParkingLotSpecs, RobotSpecs, TrackDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Section
from shared.domain.models import Pose

from scripts.common.tables import print_table
from src.navigation.maneuvers.parking import park_controller_from_metadata
from src.simulation.kinematics import AckermannKinematics, AckermannState
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.track_model import ObstacleBox, TrackModel, _convex_overlap, _rect_corners

_TRACK_WIDTHS = dict.fromkeys(Section, CorridorDimensions.OBSTACLES_WIDTH)
_DT = 1.0 / NavigationTuning.load_default().control.CONTROL_HZ
_MAX_STEPS = 800
_LATERAL_ERRORS = (-0.15, -0.05, 0.0, 0.05, 0.15)
_YAW_ERRORS = (math.radians(-15), 0.0, math.radians(15))

# Bay depth: the fins stand perpendicular to the outer wall and are LENGTH long.
_BAY_DEPTH = ParkingLotSpecs.LENGTH
_FIN_THICKNESS = ParkingLotSpecs.WIDTH
_STOP_TOLERANCE = 0.01  # metres: how close to the bay midpoint counts as "stopped there"

# Straight-in approach probe parameters
_STRAIGHTIN_YAW_ERRORS = (-0.05, -0.02, 0.0, 0.02, 0.05)
_STRAIGHTIN_LATERAL_OFFSETS = (0.09, 0.10, 0.11, 0.12, 0.14)
_STRAIGHTIN_UPSTREAM_DIST = 0.35  # metres: start distance before bay along travel direction
_STRAIGHTIN_MAX_STEPS = 200
_STRAIGHTIN_SPEED = 0.10  # m/s
_STAGING_APPROACH_OFFSET = 0.5  # metres: offset before bay for ParkController test


def _section_of(metadata: dict) -> Section:
    return Section(metadata["starting_conditions"]["section"].lower())


def _low_side(section: Section) -> bool:
    """Whether this corridor's outer wall is at coordinate 0 (vs 3)."""
    return section in (Section.SOUTH, Section.WEST)


def _is_ns(section: Section) -> bool:
    return section in (Section.NORTH, Section.SOUTH)


def _blocks(metadata: dict) -> list[tuple[str, ObstacleBox]]:
    p = metadata["parking_lot"]
    return [
        (
            name,
            ObstacleBox.from_pose(
                cx=float(p[pos]["x"]),
                cy=float(p[pos]["y"]),
                length=ParkingLotSpecs.LENGTH,
                width=ParkingLotSpecs.WIDTH,
                yaw=float(p[yawk]),
            ),
        )
        for name, pos, yawk in (
            ("block1", "block1_position", "block1_yaw"),
            ("block2", "block2_position", "block2_yaw"),
        )
    ]


def _bay_rect(metadata: dict, section: Section) -> tuple[float, float, float, float]:
    """The parking lot as (x_min, y_min, x_max, y_max): between the fins, wall to fin tip."""
    p = metadata["parking_lot"]
    b1, b2 = p["block1_position"], p["block2_position"]
    along = sorted([b1["x"], b2["x"]]) if _is_ns(section) else sorted([b1["y"], b2["y"]])
    # Inner faces of the two fins.
    a_lo = along[0] + _FIN_THICKNESS / 2
    a_hi = along[1] - _FIN_THICKNESS / 2
    if _low_side(section):
        d_lo, d_hi = 0.0, _BAY_DEPTH
    else:
        d_lo, d_hi = TrackDimensions.MAX_COORD - _BAY_DEPTH, TrackDimensions.MAX_COORD
    if _is_ns(section):
        return a_lo, d_lo, a_hi, d_hi
    return d_lo, a_lo, d_hi, a_hi


def _footprint_contained(x: float, y: float, yaw: float, rect: tuple[float, float, float, float]) -> bool:
    x_min, y_min, x_max, y_max = rect
    return all(
        x_min <= cx <= x_max and y_min <= cy <= y_max
        for cx, cy in _rect_corners(x, y, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
    )


def _travel_yaw(section: Section, direction: str) -> float:
    cw = direction == "clockwise"
    if section is Section.SOUTH:
        return math.pi if cw else 0.0
    if section is Section.NORTH:
        return 0.0 if cw else math.pi
    if section is Section.EAST:
        return math.pi / 2 if cw else -math.pi / 2
    return -math.pi / 2 if cw else math.pi / 2


def _classify(
    track: TrackModel,
    blocks: list[tuple[str, ObstacleBox]],
    x: float,
    y: float,
    yaw: float,
) -> set[str]:
    """Which objects the chassis footprint overlaps at this pose."""
    corners = _rect_corners(x, y, yaw, RobotSpecs.LENGTH, RobotSpecs.WIDTH)
    hits = set()
    ob = track._outer_collision  # noqa: SLF001
    if any(cx < ob.x_min or cx > ob.x_max or cy < ob.y_min or cy > ob.y_max for cx, cy in corners):
        hits.add("outer-wall")
    if _convex_overlap(corners, track._inner_collision.corners(), yaw):  # noqa: SLF001
        hits.add("inner-block")
    for name, box in blocks:
        if _convex_overlap(corners, box.to_box().corners(), yaw):
            hits.add(name)
    return hits


def _run_one(
    metadata: dict,
    section: Section,
    direction: str,
    lat_err: float,
    yaw_err: float,
) -> dict:
    """Drive ParkController from one perturbed approach pose. Returns a result dict."""
    blocks = _blocks(metadata)
    ctrl = park_controller_from_metadata(metadata, section)
    if ctrl is None:
        msg = f"scenario has no parking lot: {metadata.get('scenario_id')}"
        raise ValueError(msg)
    sx, sy = ctrl.staging.x, ctrl.staging.y
    yaw = _travel_yaw(section, direction) + yaw_err
    centre = CorridorDimensions.OBSTACLES_WIDTH / 2
    lat_centre = centre if _low_side(section) else TrackDimensions.MAX_COORD - centre
    if _is_ns(section):
        start = (sx - _STAGING_APPROACH_OFFSET * math.cos(yaw), lat_centre + lat_err)
    else:
        start = (lat_centre + lat_err, sy - _STAGING_APPROACH_OFFSET * math.sin(yaw))

    track = TrackModel(_TRACK_WIDTHS, obstacles=[b for _, b in blocks])
    kin = AckermannKinematics()
    state = AckermannState(x=start[0], y=start[1], yaw=yaw)
    hits: set[str] = set()
    first_contact_phase = None
    done = False
    phase = "stage"
    for _ in range(_MAX_STEPS):
        cmd = ctrl.update(Pose(state.x, state.y, state.yaw))
        phase = cmd.phase
        if cmd.done:
            done = not ctrl.is_timed_out
            break
        state = kin.step(state, target_speed=cmd.linear, target_steer_norm=cmd.steering, dt=_DT)
        new = _classify(track, blocks, state.x, state.y, state.yaw)
        if new and first_contact_phase is None:
            first_contact_phase = phase
        hits |= new

    rect = _bay_rect(metadata, section)
    return {
        "done": done,
        "timed_out": ctrl.is_timed_out,
        "hits": hits,
        "first_contact_phase": first_contact_phase,
        "contained": _footprint_contained(state.x, state.y, state.yaw, rect),
        "final": (state.x, state.y, state.yaw),
        "rect": rect,
    }


def report_contacts_and_containment() -> None:
    """Sweep approach error; report contacts and how many runs truly end inside the bay."""
    print("=== ParkController: contacts + footprint containment (16 fixtures x 15 approaches) ===\n")
    hit_counts: dict[str, int] = {}
    phase_counts: dict[str, int] = {}
    runs = done_n = contained_n = done_but_not_contained = 0

    for scenario in all_obstacles_demo_scenarios():
        meta = scenario.metadata
        section = _section_of(meta)
        direction = meta["starting_conditions"]["direction"]
        s_done = s_contained = 0
        s_hits: dict[str, int] = {}
        for lat_err in _LATERAL_ERRORS:
            for yaw_err in _YAW_ERRORS:
                r = _run_one(meta, section, direction, lat_err, yaw_err)
                runs += 1
                done_n += r["done"]
                s_done += r["done"]
                contained_n += r["contained"]
                s_contained += r["contained"]
                done_but_not_contained += r["done"] and not r["contained"]
                for h in r["hits"]:
                    hit_counts[h] = hit_counts.get(h, 0) + 1
                    s_hits[h] = s_hits.get(h, 0) + 1
                if r["first_contact_phase"]:
                    key = r["first_contact_phase"]
                    phase_counts[key] = phase_counts.get(key, 0) + 1

        n = len(_LATERAL_ERRORS) * len(_YAW_ERRORS)
        print(
            f"{scenario.label:<42} done={s_done:>2}/{n} contained={s_contained:>2}/{n} contacts={s_hits or '{}'}",
        )

    print(f"\nruns={runs}  done={done_n}  footprint_contained_in_bay={contained_n}")
    print(f"reported done but NOT contained = {done_but_not_contained}")
    print(f"contact breakdown={hit_counts}")
    print(f"phase of first contact={phase_counts}")


def report_straight_in() -> None:
    """Feasibility of the recommended wall-parallel straight-in entry.

    Places the chassis on a wall-hugging line just upstream of the near fin and drives it
    straight along the wall into the slot, sweeping lateral offset and heading error. This
    is pure geometry -- no controller -- so it measures the tolerance budget any future
    controller has to hit, independent of how well it steers.
    """
    print("\n=== Straight-in entry: tolerance budget (fixture 0000, NORTH) ===\n")
    scenario = all_obstacles_demo_scenarios()[0]
    meta = scenario.metadata
    section = _section_of(meta)
    direction = meta["starting_conditions"]["direction"]
    blocks = _blocks(meta)
    rect = _bay_rect(meta, section)
    track = TrackModel(_TRACK_WIDTHS, obstacles=[b for _, b in blocks])
    wall = 0.0 if _low_side(section) else TrackDimensions.MAX_COORD
    bay_mid = (rect[0] + rect[2]) / 2 if _is_ns(section) else (rect[1] + rect[3]) / 2

    print(f"bay rect={tuple(round(v, 3) for v in rect)}  depth={_BAY_DEPTH} m  chassis width={RobotSpecs.WIDTH} m")
    print("lateral = chassis centre distance from the outer wall\n")

    kin = AckermannKinematics()
    table_rows = []
    for lateral in _STRAIGHTIN_LATERAL_OFFSETS:
        row_data = [lateral]
        for yaw_err in _STRAIGHTIN_YAW_ERRORS:
            yaw = _travel_yaw(section, direction) + yaw_err
            lat_coord = wall + lateral if _low_side(section) else wall - lateral
            along_start = bay_mid - _STRAIGHTIN_UPSTREAM_DIST * math.cos(yaw) if _is_ns(section) else bay_mid - _STRAIGHTIN_UPSTREAM_DIST * math.sin(yaw)
            state = (
                AckermannState(x=along_start, y=lat_coord, yaw=yaw)
                if _is_ns(section)
                else AckermannState(x=lat_coord, y=along_start, yaw=yaw)
            )
            hit = False
            contained_at_stop = False
            for _ in range(_STRAIGHTIN_MAX_STEPS):
                state = kin.step(state, target_speed=_STRAIGHTIN_SPEED, target_steer_norm=0.0, dt=_DT)
                if _classify(track, blocks, state.x, state.y, state.yaw):
                    hit = True
                    break
                along = state.x if _is_ns(section) else state.y
                if abs(along - bay_mid) < _STOP_TOLERANCE:
                    contained_at_stop = _footprint_contained(state.x, state.y, state.yaw, rect)
                    break
            row_data.append("HIT " if hit else ("ok  " if contained_at_stop else "out "))
        table_rows.append(row_data)
    headers = ["lat(m)"] + [f"{math.degrees(y):+.0f}d" for y in _STRAIGHTIN_YAW_ERRORS]
    print_table(table_rows, headers)

    print("\nok = whole footprint inside the bay at the stop point; out = clear but protruding; HIT = contact")


def main() -> None:
    """Run the report named on the command line (default: all)."""
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which in ("contacts", "containment", "all"):
        report_contacts_and_containment()
    if which in ("straightin", "all"):
        report_straight_in()


if __name__ == "__main__":
    main()
