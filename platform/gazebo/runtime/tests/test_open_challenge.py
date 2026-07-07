"""Open challenge scenario generation tests.

These tests validate every invariant of a WRO 2026 Open Challenge scenario
without launching Gazebo or ROS2.  The simgen binary is called once per
session to generate 10 deterministic scenarios (seed=42).

Run with:
    pixi run test -v                       # structured output, captured logs
    pixi run test -v -s                    # + print() output for summaries
    pixi run test -v --log-cli-level=INFO  # stream log records to terminal
    pixi run test -k open -v               # only open challenge tests
"""

import json
import logging
import math
import subprocess
from collections import Counter
from pathlib import Path
from typing import Any

import pytest

from shared.domain.enums import Direction, Section
from src.scenario.models import ScenarioMetadata

logger = logging.getLogger(__name__)

# WRO 2026 Open Challenge constraints
_VALID_WIDTHS_MM = frozenset({600, 1000})
_VALID_SECTIONS = frozenset({"South", "North", "East", "West"})
_VALID_DIRECTIONS = frozenset({"clockwise", "counterclockwise"})
_TRACK_MIN = 0.0
_TRACK_MAX = 3.0
_VALID_YAWS = (0.0, math.pi, math.pi / 2, -(math.pi / 2))
_YAW_TOL = 1e-9
_EXPECTED_COUNT = 10
_FIXED_SEED = 42


def _yaw_valid(yaw: float) -> bool:
    return any(abs(yaw - v) < _YAW_TOL for v in _VALID_YAWS)


def _corridor_summary(meta: dict[str, Any]) -> str:
    w = meta["corridor_widths"]
    return f"S={w['south']['width_mm']} N={w['north']['width_mm']} E={w['east']['width_mm']} W={w['west']['width_mm']}"


class TestOpenChallengeInvariants:
    """Every scenario must satisfy these — they encode the WRO Open Challenge rules."""

    def test_correct_scenario_count(self, open_metadata_list: list[dict[str, Any]]) -> None:
        assert len(open_metadata_list) == _EXPECTED_COUNT

    def test_challenge_type_is_open(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            assert m["challenge_type"] == "open", (
                f"scenario {m['scenario_id']}: challenge_type={m['challenge_type']!r}"
            )

    def test_no_traffic_signs(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sid = m["scenario_id"]
            assert m["num_signs"] == 0, f"scenario {sid}: {m['num_signs']} signs (Open must have 0)"
            assert m["sign_positions"] == [], f"scenario {sid}: sign_positions not empty"

    def test_no_parking_lot(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sid = m["scenario_id"]
            assert not m["has_parking_lot"], f"scenario {sid}: has_parking_lot=true"
            assert m.get("parking_lot") is None, f"scenario {sid}: parking_lot field is not null"

    def test_corridor_widths_all_four_sections(self, open_metadata_list: list[dict[str, Any]]) -> None:
        expected = {"south", "north", "east", "west"}
        for m in open_metadata_list:
            assert set(m["corridor_widths"].keys()) == expected, (
                f"scenario {m['scenario_id']}: sections {set(m['corridor_widths'])} != {expected}"
            )

    def test_corridor_width_values_valid(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sid = m["scenario_id"]
            for section, w in m["corridor_widths"].items():
                assert w["width_mm"] in _VALID_WIDTHS_MM, (
                    f"scenario {sid} [{section}]: width_mm={w['width_mm']} not in {{600, 1000}}"
                )
                assert w["type"] in ("narrow", "wide"), (
                    f"scenario {sid} [{section}]: type={w['type']!r}"
                )

    def test_corridor_width_type_consistent_with_mm(self, open_metadata_list: list[dict[str, Any]]) -> None:
        expected_mm = {"narrow": 600, "wide": 1000}
        for m in open_metadata_list:
            sid = m["scenario_id"]
            for section, w in m["corridor_widths"].items():
                assert w["width_mm"] == expected_mm[w["type"]], (
                    f"scenario {sid} [{section}]: type={w['type']} but width_mm={w['width_mm']}"
                )

    def test_starting_section_valid(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sc = m["starting_conditions"]
            assert sc["section"] in _VALID_SECTIONS, (
                f"scenario {m['scenario_id']}: section={sc['section']!r}"
            )

    def test_starting_direction_valid(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sc = m["starting_conditions"]
            assert sc["direction"] in _VALID_DIRECTIONS, (
                f"scenario {m['scenario_id']}: direction={sc['direction']!r}"
            )

    def test_starting_position_within_track(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sc = m["starting_conditions"]
            pos = sc["position"]
            sid = m["scenario_id"]
            assert _TRACK_MIN <= pos["x"] <= _TRACK_MAX, (
                f"scenario {sid}: x={pos['x']:.3f} outside [{_TRACK_MIN}, {_TRACK_MAX}]"
            )
            assert _TRACK_MIN <= pos["y"] <= _TRACK_MAX, (
                f"scenario {sid}: y={pos['y']:.3f} outside [{_TRACK_MIN}, {_TRACK_MAX}]"
            )

    def test_starting_yaw_is_cardinal(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            sc = m["starting_conditions"]
            assert _yaw_valid(sc["yaw"]), (
                f"scenario {m['scenario_id']}: yaw={sc['yaw']:.4f} not in {{0, ±π/2, π}}"
            )

    def test_seed_recorded_in_metadata(self, open_metadata_list: list[dict[str, Any]]) -> None:
        for m in open_metadata_list:
            assert m.get("seed") == _FIXED_SEED, (
                f"scenario {m['scenario_id']}: seed={m.get('seed')!r}, expected {_FIXED_SEED}"
            )

    def test_sdf_files_exist_and_nonempty(self, open_scenarios_dir: Path) -> None:
        sdf_files = sorted(open_scenarios_dir.glob("scenario_*.sdf"))
        assert len(sdf_files) == _EXPECTED_COUNT, (
            f"Expected {_EXPECTED_COUNT} SDF files, found {len(sdf_files)}"
        )
        for f in sdf_files:
            assert f.stat().st_size > 0, f"{f.name} is empty"

    def test_sdf_and_metadata_filenames_consistent(self, open_scenarios_dir: Path) -> None:
        sdf_stems = {f.stem for f in open_scenarios_dir.glob("scenario_*.sdf")}
        meta_stems = {
            f.stem.replace("_metadata", "")
            for f in open_scenarios_dir.glob("*_metadata.json")
        }
        assert sdf_stems == meta_stems, (
            f"SDF/metadata mismatch — SDF-only: {sdf_stems - meta_stems}, "
            f"meta-only: {meta_stems - sdf_stems}"
        )


def _run_simgen_inline(
    simgen_bin: Path,
    out_dir: Path,
    challenge: str,
    num: int,
    seed: int | None,
) -> list[dict[str, Any]]:
    cmd = [
        str(simgen_bin), "generate",
        "--challenge", challenge,
        "--num-scenarios", str(num),
        "--output-dir", str(out_dir),
    ]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, f"simgen failed:\n{result.stderr}"
    return [
        json.loads(f.read_text())
        for f in sorted((out_dir / "scenarios").glob("*_metadata.json"))
    ]


class TestOpenChallengeReproducibility:
    """The same seed must produce identical output across runs."""

    def test_same_seed_same_output(
        self,
        simgen_bin: Path,
        tmp_path_factory: pytest.TempPathFactory,
    ) -> None:
        seed = 99
        meta_a = _run_simgen_inline(simgen_bin, tmp_path_factory.mktemp("repro_a"), "open", 5, seed)
        meta_b = _run_simgen_inline(simgen_bin, tmp_path_factory.mktemp("repro_b"), "open", 5, seed)

        for i, (ma, mb) in enumerate(zip(meta_a, meta_b)):
            assert ma["starting_conditions"] == mb["starting_conditions"], (
                f"scenario {i}: starting_conditions differ with seed={seed}"
            )
            assert ma["corridor_widths"] == mb["corridor_widths"], (
                f"scenario {i}: corridor_widths differ with seed={seed}"
            )

    def test_different_seeds_produce_variety(
        self,
        simgen_bin: Path,
        tmp_path_factory: pytest.TempPathFactory,
    ) -> None:
        sections = set()
        for seed in range(1, 6):
            metas = _run_simgen_inline(simgen_bin, tmp_path_factory.mktemp(f"var_s{seed}"), "open", 1, seed)
            for m in metas:
                sections.add(m["starting_conditions"]["section"])
        assert len(sections) > 1, f"Only section {sections} seen across 5 different seeds"


class TestOpenChallengeDistribution:
    """Statistical checks on the 10-scenario batch (seed=42)."""

    def test_both_width_types_represented(self, open_metadata_list: list[dict[str, Any]]) -> None:
        all_types = {
            w["type"]
            for m in open_metadata_list
            for w in m["corridor_widths"].values()
        }
        assert "narrow" in all_types, "No narrow corridor in 10 scenarios — possible RNG issue"
        assert "wide" in all_types, "No wide corridor in 10 scenarios — possible RNG issue"

    def test_multiple_sections_used(self, open_metadata_list: list[dict[str, Any]]) -> None:
        sections = {m["starting_conditions"]["section"] for m in open_metadata_list}
        assert len(sections) > 1, f"All 10 scenarios used same section: {sections}"

    def test_multiple_directions_used(self, open_metadata_list: list[dict[str, Any]]) -> None:
        directions = {m["starting_conditions"]["direction"] for m in open_metadata_list}
        assert len(directions) > 1, f"All 10 scenarios used same direction: {directions}"

    def test_log_distribution_summary(self, open_metadata_list: list[dict[str, Any]]) -> None:
        sections = Counter(m["starting_conditions"]["section"] for m in open_metadata_list)
        directions = Counter(m["starting_conditions"]["direction"] for m in open_metadata_list)
        widths = Counter(
            w["type"]
            for m in open_metadata_list
            for w in m["corridor_widths"].values()
        )
        logger.info("section distribution:   %s", dict(sections))
        logger.info("direction distribution: %s", dict(directions))
        logger.info("corridor width types:   %s (out of %d total)", dict(widths), sum(widths.values()))


class TestOpenChallengeRandomVariety:
    """Generate a fresh random batch (no fixed seed) and assert the same invariants."""

    def test_random_scenarios_satisfy_invariants(
        self,
        random_open_scenarios: list[dict[str, Any]],
    ) -> None:
        for m in random_open_scenarios:
            sid = m["scenario_id"]
            assert m["challenge_type"] == "open"
            assert m["num_signs"] == 0, f"scenario {sid}: random open has signs"
            assert not m["has_parking_lot"], f"scenario {sid}: random open has parking lot"
            for section, w in m["corridor_widths"].items():
                assert w["width_mm"] in _VALID_WIDTHS_MM, (
                    f"scenario {sid} [{section}]: random width_mm={w['width_mm']}"
                )
            sc = m["starting_conditions"]
            assert sc["section"] in _VALID_SECTIONS
            assert sc["direction"] in _VALID_DIRECTIONS
            assert _TRACK_MIN <= sc["position"]["x"] <= _TRACK_MAX
            assert _TRACK_MIN <= sc["position"]["y"] <= _TRACK_MAX
            assert _yaw_valid(sc["yaw"])


def test_open_challenge_seeded_summary(open_metadata_list: list[dict[str, Any]]) -> None:
    """Human-readable log of all 10 seeded scenarios for review.

    Run with -v --log-cli-level=INFO or -s to see this output.
    """
    header = f"OPEN CHALLENGE — {len(open_metadata_list)} scenarios (seed={_FIXED_SEED})"
    sep = "─" * 70
    logger.info(sep)
    logger.info(header)
    logger.info(sep)
    for m in open_metadata_list:
        sc = m["starting_conditions"]
        direction_abbr = "CW " if sc["direction"] == "clockwise" else "CCW"
        logger.info(
            "#%02d | %s %-6s | pos=(%5.2f, %5.2f) yaw=%+.2f | corridors: %s",
            m["scenario_id"],
            direction_abbr,
            sc["section"],
            sc["position"]["x"],
            sc["position"]["y"],
            sc["yaw"],
            _corridor_summary(m),
        )
    logger.info(sep)

    # Also print for -s visibility
    print(f"\n\n{sep}")
    print(header)
    print(sep)
    for m in open_metadata_list:
        sc = m["starting_conditions"]
        direction_abbr = "CW " if sc["direction"] == "clockwise" else "CCW"
        print(
            f"  #{m['scenario_id']:02d} | {direction_abbr} {sc['section']:<6} | "
            f"pos=({sc['position']['x']:5.2f}, {sc['position']['y']:5.2f}) "
            f"yaw={sc['yaw']:+.2f} | {_corridor_summary(m)}"
        )
    print(sep)


def test_open_challenge_random_summary(random_open_scenarios: list[dict[str, Any]]) -> None:
    """Human-readable log of 5 randomly generated scenarios (no seed)."""
    sep = "─" * 70
    logger.info(sep)
    logger.info("OPEN CHALLENGE — 5 random scenarios (no fixed seed)")
    logger.info(sep)
    for m in random_open_scenarios:
        sc = m["starting_conditions"]
        direction_abbr = "CW " if sc["direction"] == "clockwise" else "CCW"
        logger.info(
            "#%02d | %s %-6s | pos=(%5.2f, %5.2f) yaw=%+.2f | corridors: %s",
            m["scenario_id"],
            direction_abbr,
            sc["section"],
            sc["position"]["x"],
            sc["position"]["y"],
            sc["yaw"],
            _corridor_summary(m),
        )
    logger.info(sep)

    print(f"\n\n{sep}")
    print("OPEN CHALLENGE — 5 random scenarios (no fixed seed)")
    print(sep)
    for m in random_open_scenarios:
        sc = m["starting_conditions"]
        direction_abbr = "CW " if sc["direction"] == "clockwise" else "CCW"
        print(
            f"  #{m['scenario_id']:02d} | {direction_abbr} {sc['section']:<6} | "
            f"pos=({sc['position']['x']:5.2f}, {sc['position']['y']:5.2f}) "
            f"yaw={sc['yaw']:+.2f} | {_corridor_summary(m)}"
        )
    print(sep)
