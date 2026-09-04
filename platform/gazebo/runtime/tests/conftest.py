"""Shared pytest fixtures for the vtitan-platform runtime tests.

Session-scoped fixtures build the simgen Go binary once and generate scenario
batches once per test session.  Individual tests receive the parsed metadata
list and can assert invariants without re-running generation.
"""

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

# Ensure src/ and shared/src/ are importable in the pixi conda env,
# where the pip packages from pyproject.toml are not installed.
_RUNTIME_ROOT = Path(__file__).parent.parent
_SHARED_SRC = _RUNTIME_ROOT.parent.parent / "shared" / "src"
for _p in (_RUNTIME_ROOT, _SHARED_SRC):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

import pytest

from src.scenario.models import ScenarioMetadata

_GENERATOR_DIR = Path(__file__).parent.parent.parent / "generator"
_SCENARIOS_SUBFOLDER = "scenarios"
_META_GLOB = "*_metadata.json"
_OPEN_NUM_SCENARIOS = 10
_OPEN_SEED = 42


def _find_or_build_simgen() -> Path | None:
    bin_name = "simgen.exe" if sys.platform == "win32" else "simgen"
    bin_path = _GENERATOR_DIR / "bin" / bin_name

    if bin_path.exists():
        return bin_path

    bin_path.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        ["go", "build", "-o", str(bin_path), "./cmd/simgen"],
        cwd=_GENERATOR_DIR,
        capture_output=True,
        text=True,
    )
    return bin_path if result.returncode == 0 else None


@pytest.fixture(scope="session")
def simgen_bin() -> Path:
    """Return the path to the simgen binary, building it if necessary."""
    bin_path = _find_or_build_simgen()
    if bin_path is None:
        pytest.skip(
            "simgen binary not found and go build failed — "
            "install Go or pre-build with: cd platform/robot-go && go build ./cmd/simgen"
        )
    return bin_path


def run_simgen(
    simgen_bin: Path,
    out_dir: Path,
    challenge: str,
    num: int,
    seed: int | None,
) -> Path:
    """Run simgen generate and return the scenarios sub-directory."""
    cmd = [
        str(simgen_bin),
        "generate",
        "--challenge",
        challenge,
        "--num-scenarios",
        str(num),
        "--output-dir",
        str(out_dir),
    ]
    if seed is not None:
        cmd += ["--seed", str(seed)]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        pytest.fail(
            f"simgen failed (exit={result.returncode}):\n"
            f"stderr:\n{result.stderr}\n"
            f"stdout:\n{result.stdout}"
        )
    return out_dir / _SCENARIOS_SUBFOLDER


def load_metadata(scenarios_dir: Path) -> list[dict[str, Any]]:
    """Parse all metadata JSON files in a scenarios directory."""
    return [
        json.loads(f.read_text())
        for f in sorted(scenarios_dir.glob(_META_GLOB))
    ]


@pytest.fixture(scope="session")
def open_scenarios_dir(
    simgen_bin: Path,
    tmp_path_factory: pytest.TempPathFactory,
) -> Path:
    """Generate 10 Open challenge scenarios with seed=42 (runs once per session)."""
    out = tmp_path_factory.mktemp("open_seed42")
    return run_simgen(simgen_bin, out, "open", _OPEN_NUM_SCENARIOS, _OPEN_SEED)


@pytest.fixture(scope="session")
def open_metadata_list(open_scenarios_dir: Path) -> list[dict[str, Any]]:
    """Raw metadata dicts for all 10 seeded Open challenge scenarios."""
    return load_metadata(open_scenarios_dir)


@pytest.fixture(scope="session")
def open_scenarios(open_scenarios_dir: Path) -> list[ScenarioMetadata]:
    """Typed ScenarioMetadata objects for all 10 seeded Open challenge scenarios."""
    return [ScenarioMetadata.from_dict(d) for d in load_metadata(open_scenarios_dir)]


@pytest.fixture
def random_open_scenarios(
    simgen_bin: Path,
    tmp_path: Path,
) -> list[dict[str, Any]]:
    """Generate 5 Open challenge scenarios with a random seed (fresh per test)."""
    scenarios_dir = run_simgen(simgen_bin, tmp_path, "open", 5, seed=None)
    return load_metadata(scenarios_dir)
