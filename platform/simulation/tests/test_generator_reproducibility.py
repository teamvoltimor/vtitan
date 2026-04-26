"""Reproducibility test: same seed → identical SDF + metadata."""

import json
from pathlib import Path

import pytest
from shared.config.enums import ScenarioType

from src.generation.generator import ScenarioGenerator


@pytest.fixture
def base_world(tmp_path):
    """Minimal valid SDF world file for testing."""
    sdf = tmp_path / "world.sdf"
    sdf.write_text(
        '<?xml version="1.0"?>'
        '<sdf version="1.9">'
        "<world name=\"wro_track\">"
        "</world>"
        "</sdf>",
        encoding="utf-8",
    )
    return str(sdf)


def _run_generator(base_world: str, output_dir: Path, seed: int, challenge: ScenarioType) -> tuple[str, dict]:
    gen = ScenarioGenerator(
        base_world_path=base_world,
        output_dir=str(output_dir),
        challenge_type=challenge,
        seed=seed,
    )
    world_file, metadata = gen.create_scenario_world(scenario_index=0, randomize_all=True)
    sdf_bytes = Path(world_file).read_bytes()
    return sdf_bytes, metadata


@pytest.mark.parametrize("challenge", [ScenarioType.OPEN, ScenarioType.OBSTACLES])
def test_same_seed_produces_identical_sdf(tmp_path, base_world, challenge):
    seed = 42
    out_a = tmp_path / "run_a"
    out_b = tmp_path / "run_b"

    sdf_a, _ = _run_generator(base_world, out_a, seed, challenge)
    sdf_b, _ = _run_generator(base_world, out_b, seed, challenge)

    assert sdf_a == sdf_b, "SDF bytes differ between two runs with the same seed"


@pytest.mark.parametrize("challenge", [ScenarioType.OPEN, ScenarioType.OBSTACLES])
def test_same_seed_produces_identical_metadata(tmp_path, base_world, challenge):
    seed = 42
    out_a = tmp_path / "run_a"
    out_b = tmp_path / "run_b"

    _, meta_a = _run_generator(base_world, out_a, seed, challenge)
    _, meta_b = _run_generator(base_world, out_b, seed, challenge)

    # Exclude scenario_id (index-based, not seed-based) from comparison
    for key in meta_a:
        if key == "scenario_id":
            continue
        assert meta_a[key] == meta_b[key], f"Metadata key '{key}' differs between runs"


@pytest.mark.parametrize("challenge", [ScenarioType.OPEN, ScenarioType.OBSTACLES])
def test_different_seeds_produce_different_sdf(tmp_path, base_world, challenge):
    out_a = tmp_path / "run_a"
    out_b = tmp_path / "run_b"

    sdf_42, _ = _run_generator(base_world, out_a, 42, challenge)
    sdf_99, _ = _run_generator(base_world, out_b, 99, challenge)

    assert sdf_42 != sdf_99, "Different seeds should produce different SDFs"
