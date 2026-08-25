"""Analyze CSVs produced by ``diag_path_track.py``.

Prints overall, post-convergence and per-lap statistics for the cross-track
error against both the displayed (believed) path and the true path.

Usage (from ``platform/robot``, with PYTHONPATH=".")::

    python scripts/sim/diag_path_track_analyze.py scripts/sim/output/diag_path_track_450.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.stats import percentile
from scripts.common.tables import print_table


def _float(rows: list[dict], key: str) -> list[float]:
    return [float(r[key]) for r in rows if r.get(key) not in (None, "", "None")]


def _summarize(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": float("nan"), "p50": float("nan"), "p95": float("nan"), "max": float("nan")}
    return {
        "mean": sum(values) / len(values),
        "p50": percentile(values, 0.5),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def _format_summary(stats: dict[str, float]) -> list[str]:
    return [f"{stats['mean']:.3f}", f"{stats['p50']:.3f}", f"{stats['p95']:.3f}", f"{stats['max']:.3f}"]


def analyze(csv_path: Path) -> None:
    """Print statistics for a single diag_path_track CSV."""
    rows = list(csv.DictReader(csv_path.open()))
    print(f"\n{csv_path.name}: {len(rows)} ticks")

    displayed = _float(rows, "cte_displayed_m")
    true_path = _float(rows, "cte_true_m")

    if not displayed:
        print("  No valid CTE rows.")
        return

    table = [
        ["vs displayed path", *_format_summary(_summarize(displayed))],
        ["vs true path", *_format_summary(_summarize(true_path))],
    ]
    print("Overall:")
    print_table(table, ["", "mean", "p50", "p95", "max"])

    for skip in (50, 100, 200):
        if len(rows) <= skip:
            continue
        disp = _float(rows[skip:], "cte_displayed_m")
        true = _float(rows[skip:], "cte_true_m")
        print(f"\nAfter {skip} ticks:")
        print_table(
            [
                ["vs displayed path", *_format_summary(_summarize(disp))],
                ["vs true path", *_format_summary(_summarize(true))],
            ],
            ["", "mean", "p50", "p95", "max"],
        )

    n_laps = 3
    lap_size = len(rows) // n_laps
    if lap_size > 0:
        print("\nPer lap (rough split):")
        table = []
        for i in range(n_laps):
            start = i * lap_size
            end = (i + 1) * lap_size if i < n_laps - 1 else len(rows)
            disp = _float(rows[start:end], "cte_displayed_m")
            true = _float(rows[start:end], "cte_true_m")
            table.append(
                [f"lap {i + 1}", *_format_summary(_summarize(disp)), *_format_summary(_summarize(true))]
            )
        print_table(table, ["", "disp mean", "disp p50", "disp p95", "disp max", "true mean", "true p50", "true p95", "true max"])

    print("\nWidth evolution:")
    prev: tuple[str, ...] | None = None
    for r in rows[::50]:
        widths = (r["south_m"], r["north_m"], r["east_m"], r["west_m"])
        if widths != prev:
            print(
                f"  step={r['step']} pose=({float(r['x']):.2f},{float(r['y']):.2f}) "
                f"widths={widths}",
            )
            prev = widths

    print("\nFirst 10 ticks:")
    print_table(
        [
            [
                r["step"],
                f"({float(r['x']):.2f},{float(r['y']):.2f})",
                f"{float(r['cte_displayed_m']):.3f}",
                f"{float(r['cte_true_m']):.3f}",
            ]
            for r in rows[:10]
        ],
        ["step", "pose", "displayed cte", "true cte"],
    )


def main() -> None:
    """Parse arguments and analyse each supplied CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path, help="CSV file(s) from diag_path_track.py.")
    args = parser.parse_args()

    for path in args.csv:
        analyze(path)


if __name__ == "__main__":
    main()
