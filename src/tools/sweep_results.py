"""Record corpus sweep arms with the coordinates that make them comparable.

``adr:0087-test-methodology`` says a measurement without its commit, corpus,
seed and profile is meaningless, and that results are not comparable across
time. The sweeps under ``src/python/scripts/sim/sweeps/`` honoured that by
discipline only: each wrote a failure list to ``/tmp`` and a count to the
terminal, and the coordinates lived in whoever ran it. This store makes them
part of the row.

One row per arm, holding:

* ``commit`` and ``tree_dirty``: HEAD, and a hash of ``git diff HEAD`` taken
  with the arm's own override still applied, so two arms on one commit differ
  here exactly by what the sweep edited;
* ``corpus``: the pytest target the arm ran. The corpus is fixed-seed, so the
  target plus the commit pins the seed too;
* ``profile``: ``VTITAN_HARDWARE_PROFILE`` as the arm saw it (the suite wants it
  unset, and a row that says otherwise explains itself);
* the failure SET, not just the count, because arms are compared by diffing
  sets.

The database lives outside the repo (``$VTITAN_RESULTS_DB``, default
``~/.local/share/vtitan/sweeps.sqlite``) so it survives checkouts and worktrees
and cannot be committed by accident.

Commands::

    sweep_results.py record --sweep S --arm A --overrides k=v,... --corpus T --raw pytest.out
    sweep_results.py list [--sweep S]
    sweep_results.py diff RUN_A RUN_B
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import os
import re
import sqlite3
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DB = Path.home() / ".local" / "share" / "vtitan" / "sweeps.sqlite"

_FAILED_LINE = re.compile(r"^FAILED (\S+)")
_COUNT = re.compile(r"(\d+) (failed|passed)")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    id          INTEGER PRIMARY KEY,
    recorded_at TEXT NOT NULL,
    sweep       TEXT NOT NULL,
    arm         TEXT NOT NULL,
    overrides   TEXT NOT NULL,
    commit_sha  TEXT NOT NULL,
    tree_dirty  TEXT NOT NULL,
    corpus      TEXT NOT NULL,
    profile     TEXT NOT NULL,
    failed      INTEGER NOT NULL,
    passed      INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS failures (
    run_id  INTEGER NOT NULL REFERENCES runs(id),
    test_id TEXT NOT NULL,
    PRIMARY KEY (run_id, test_id)
);
"""


def _git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True).stdout


def _connect() -> sqlite3.Connection:
    path = Path(os.environ.get("VTITAN_RESULTS_DB", DEFAULT_DB))
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(path)
    conn.executescript(_SCHEMA)
    return conn


def parse_pytest(raw: str) -> tuple[list[str], int, int]:
    """Return (failed test ids, failed count, passed count) from ``pytest -rf -q`` output.

    Raises:
        ValueError: the output has no summary line, i.e. pytest did not finish.
    """
    failures = sorted({m.group(1) for line in raw.splitlines() if (m := _FAILED_LINE.match(line))})
    counts = {"failed": 0, "passed": 0}
    summary = [line for line in raw.splitlines() if _COUNT.search(line)]
    if not summary:
        msg = "no pytest summary line: the run did not finish, refusing to record it"
        raise ValueError(msg)
    for number, kind in _COUNT.findall(summary[-1]):
        counts[kind] = int(number)
    return failures, counts["failed"], counts["passed"]


def record(args: argparse.Namespace) -> int:
    """Store one arm."""
    failures, failed, passed = parse_pytest(Path(args.raw).read_text(encoding="utf-8"))
    dirty = hashlib.sha256(_git("diff", "HEAD").encode()).hexdigest()[:12]
    with _connect() as conn:
        cur = conn.execute(
            "INSERT INTO runs (recorded_at, sweep, arm, overrides, commit_sha, tree_dirty, corpus, profile, failed,"
            " passed) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (
                dt.datetime.now(dt.UTC).isoformat(timespec="seconds"),
                args.sweep,
                args.arm,
                args.overrides,
                _git("rev-parse", "HEAD").strip(),
                dirty,
                args.corpus,
                os.environ.get("VTITAN_HARDWARE_PROFILE", ""),
                failed,
                passed,
            ),
        )
        conn.executemany("INSERT INTO failures (run_id, test_id) VALUES (?, ?)", [(cur.lastrowid, f) for f in failures])
    print(f"recorded run {cur.lastrowid}: {args.sweep}/{args.arm} failed={failed} passed={passed}")
    return 0


def list_runs(args: argparse.Namespace) -> int:
    """Print recorded runs, newest last."""
    query = "SELECT id, recorded_at, sweep, arm, commit_sha, tree_dirty, profile, failed, passed, overrides FROM runs"
    params: tuple[str, ...] = ()
    if args.sweep:
        query += " WHERE sweep = ?"
        params = (args.sweep,)
    with _connect() as conn:
        for row in conn.execute(query + " ORDER BY id", params):
            run_id, at, sweep, arm, sha, dirty, profile, failed, passed, overrides = row
            print(
                f"{run_id:>5} {at} {sweep}/{arm} {sha[:8]}+{dirty} profile={profile or '-'}"
                f" failed={failed} passed={passed} {overrides or '(baseline)'}"
            )
    return 0


def diff(args: argparse.Namespace) -> int:
    """Print which failures run B fixed and broke relative to run A.

    Refuses runs whose coordinates differ in commit, corpus or profile unless
    ``--force``: a set diff across those is the cross-time comparison ADR 0087
    rules out, and it would print just as confidently.
    """
    with _connect() as conn:
        rows = {
            r[0]: r[1:]
            for r in conn.execute(
                "SELECT id, commit_sha, corpus, profile FROM runs WHERE id IN (?, ?)", (args.a, args.b)
            )
        }
        if len(rows) != 2:  # noqa: PLR2004 -- exactly the two runs asked for
            print(f"unknown run id(s): {sorted({args.a, args.b} - rows.keys())}")
            return 1
        if rows[args.a] != rows[args.b] and not args.force:
            print(f"runs differ in (commit, corpus, profile): {rows[args.a]} vs {rows[args.b]}; --force to diff anyway")
            return 1
        sets = {
            rid: {r[0] for r in conn.execute("SELECT test_id FROM failures WHERE run_id = ?", (rid,))}
            for rid in (args.a, args.b)
        }
    fixed, broke = sorted(sets[args.a] - sets[args.b]), sorted(sets[args.b] - sets[args.a])
    print(f"run {args.b} vs {args.a}: fixed={len(fixed)} broke={len(broke)}")
    for label, tests in (("FIXED", fixed), ("BROKE", broke)):
        for test in tests:
            print(f"  {label} {test}")
    return 0


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="Store one sweep arm")
    rec.add_argument("--sweep", required=True)
    rec.add_argument("--arm", required=True)
    rec.add_argument("--overrides", default="", help="the arm's key=value edits, empty for the baseline")
    rec.add_argument("--corpus", required=True, help="the pytest target the arm ran")
    rec.add_argument("--raw", required=True, help="the arm's pytest -rf output")
    rec.set_defaults(func=record)

    lst = sub.add_parser("list", help="List recorded runs")
    lst.add_argument("--sweep")
    lst.set_defaults(func=list_runs)

    dif = sub.add_parser("diff", help="Failures fixed and broken between two runs")
    dif.add_argument("a", type=int)
    dif.add_argument("b", type=int)
    dif.add_argument("--force", action="store_true", help="diff across commit/corpus/profile anyway")
    dif.set_defaults(func=diff)

    args = parser.parse_args()
    try:
        return args.func(args)
    except ValueError as err:
        print(f"sweep_results: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
