"""Shared building blocks for the ``scripts/sim/diag_open_*.py`` scenario sweeps.

``diag_open_exhaustive.py`` (serial baseline), ``diag_open_parallel.py`` (pooled
across cores) and ``diag_open_ab.py`` (two pooled arms compared) all draw from
the same 640-case Open Challenge population and must draw it identically to
stay comparable -- same seed, same sample. That draw, the ``--laps/--sample/
--seed/--all[/--jobs]`` flags that control it, and the ``ProcessPoolExecutor``
dispatch two of the three scripts ran with near-identical boilerplate, are
collected here instead of staying independently copy-pasted.

Nothing here is scenario-specific: no ``Section``/``Direction``/corridor-width
knowledge lives in this file, only the sampling and dispatch mechanics around
whatever population and worker function a caller supplies.
"""

from __future__ import annotations

import dataclasses
import os
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import TYPE_CHECKING

from shared.config.navigation_tuning import NavigationTuning

if TYPE_CHECKING:
    import argparse
    from collections.abc import Callable, Sequence

_SPARE_CORES = 2
"""Cores left for the rest of the machine, so a sweep does not make it unusable."""


def add_sweep_args(
    parser: argparse.ArgumentParser,
    *,
    default_laps: int,
    default_sample: int,
    default_seed: int,
    jobs: bool = False,
) -> None:
    """Add the ``--laps/--sample/--seed/--all[/--jobs]`` flags every sweep script shares."""
    parser.add_argument("--laps", type=int, default=default_laps)
    parser.add_argument("--sample", type=int, default=default_sample, help="How many scenarios to draw.")
    parser.add_argument("--seed", type=int, default=default_seed, help="Draw seed; same seed, same sample.")
    parser.add_argument("--all", action="store_true", help="Run the whole population instead of a sample.")
    parser.add_argument(
        "--case",
        type=int,
        action="append",
        help="Population index to run, repeatable. Reproduces exactly the case --all reports under that number.",
    )
    if jobs:
        parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")


def add_tuning_arg(parser: argparse.ArgumentParser) -> None:
    """Add ``--tuning``, the one flag the original diag scripts never wired up.

    Every sweep ran the checked-in default tuning regardless of this flag's
    presence elsewhere in the codebase (``track_navigator_node.py`` has its
    own copy) -- a sweep script has no way to validate a candidate tuning
    profile before it reaches hardware. Kept optional and defaulted to the
    checked-in profile so existing invocations are unaffected.
    """
    parser.add_argument("--tuning", help="Optional navigation tuning YAML override; defaults to the checked-in profile.")


def add_yaw_gain_compensation_arg(parser: argparse.ArgumentParser) -> None:
    """Add ``--yaw-gain-compensation``, the one-field override ``--tuning`` cannot express.

    ``--tuning`` replaces the whole tree with a YAML file's contents over
    Pydantic defaults, so a file naming one key silently drops the shipped
    value of every other key. Measuring a single knob needs the opposite: the
    shipped tree with exactly one field changed.
    """
    parser.add_argument(
        "--yaw-gain-compensation",
        type=float,
        default=None,
        help="Override pursuit.YAW_GAIN_COMPENSATION (1.0 = shipped/off, 0.55 = full understeer compensation).",
    )


def load_tuning(path: str | None, yaw_gain_compensation: float | None = None) -> NavigationTuning:
    """Resolve ``--tuning``'s value the same way ``track_navigator_node.py`` does.

    ``yaw_gain_compensation`` overrides that one pursuit field on top of the
    resolved tree. ``NavigationTuning`` is a dataclass wrapping frozen Pydantic
    models, so the override needs ``replace`` outside and ``model_copy`` inside
    -- assignment raises, and rebuilding the model from scratch would reset
    every sibling field to its default.
    """
    tuning = NavigationTuning.load_from_yaml(path) if path else NavigationTuning.load_default()
    if yaw_gain_compensation is None:
        return tuning
    return dataclasses.replace(
        tuning,
        pursuit=tuning.pursuit.model_copy(update={"YAW_GAIN_COMPENSATION": yaw_gain_compensation}),
    )


def draw_sample[T](population: Sequence[T], *, sample: int, seed: int, all_: bool) -> list[T]:
    """The one sampling rule all three sweep scripts must share to stay comparable.

    Cases are drawn uniformly at random rather than enumerated systematically,
    because stepping the population's axes together (e.g. cell index alongside
    section) correlates them -- see ``diag_open_exhaustive``'s module docstring.
    """
    if all_ or sample >= len(population):
        return list(population)
    # Suppression is justified here: the draw must be reproducible from a
    # seed, which is the one thing a cryptographic generator will not do.
    return random.Random(seed).sample(population, sample)  # noqa: S311


def select_cases[T](
    population: Sequence[T],
    *,
    sample: int,
    seed: int,
    all_: bool,
    case: Sequence[int] | None = None,
) -> list[tuple[int, T]]:
    """Pair each selected case with the index the simulator must be seeded by.

    Every sweep seeds the simulator by the case's position in the run, so that
    position is not a label -- it decides what the case does. Filtering to a
    single case by re-enumerating from zero would therefore run a *different*
    scenario than the one the sweep reported under that number, which is the
    one thing a reproduce flag must not do.

    ``--case N`` selects ``population[N]`` and keeps N as the seed, matching
    ``--all`` exactly. Sampled runs keep their existing position-based seeding,
    so numbers recorded against a ``--sample``/``--seed`` pair stay comparable.
    """
    if case:
        return [(n, population[n]) for n in case]
    return list(enumerate(draw_sample(population, sample=sample, seed=seed, all_=all_)))


def resolve_jobs(jobs: int) -> int:
    """Turn ``--jobs 0`` (the default) into a real worker count."""
    return jobs or max(1, (os.cpu_count() or 4) - _SPARE_CORES)


def run_pool[T, R](
    fn: Callable[[T], R],
    payloads: Sequence[T],
    jobs: int,
    *,
    on_result: Callable[[R, int, int], None] | None = None,
) -> list[R]:
    """Run ``fn`` over every payload in a process pool, in completion order.

    ``fn`` must be a module-level function taking one picklable argument --
    ``ProcessPoolExecutor`` requires it, which is also why every sweep
    script's per-case worker re-derives its enums from primitive values
    instead of receiving them directly. Results come back in completion
    order, not submission order; callers that need a stable order sort by
    whatever index field they put in their own result.
    """
    results: list[R] = []
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(fn, p) for p in payloads]
        for done in as_completed(futures):
            row = done.result()
            results.append(row)
            if on_result:
                on_result(row, len(results), len(payloads))
    return results


def print_pool_progress(name: str) -> Callable[[object, int, int], None]:
    """The plain ``name: done/total`` progress line ``diag_open_ab.py``'s arms print."""

    def _on_result(_row: object, done: int, total: int) -> None:
        print(f"  {name}: {done}/{total}", end="\r", flush=True)

    return _on_result
