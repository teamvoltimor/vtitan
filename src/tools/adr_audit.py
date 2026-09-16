"""Audit that measurement evidence in code carries an ADR reference.

A code file that names a recorded run (``run_YYYYMMDD_HHMMSS``) or marks a result
REFUTED is recording a measurement. Per the ADR conventions the measurement lives
in an ADR and the code points at it with ``adr:NNNN``, so this check fails when
such a file has no reference at all. That keeps the rationale from quietly
disappearing into a comment that has no reading beyond the code.

Commands:

* ``check``: scan the code roots and fail on any file with a measurement marker
  and no ``adr:`` reference.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

DEFAULT_ROOTS = (
    "src/python/src",
    "src/python/tests",
    "src/python/scripts",
    "src/go",
)

_MEASUREMENT = re.compile(r"run_20\d{6}(?:_\d{6})?|\b(?:REFUTED|refuted)\b")
_ADR_REF = re.compile(r"adr:\d{4}")
_SUFFIXES = (".py", ".go")
_SKIP_PARTS = ("generated", ".claude", "worktrees")
_SKIP_NAMES = (".pb.go", "_pb2.py")


def violations(root: Path) -> list[Path]:
    """Return the code files under ``root`` that record a measurement with no ADR ref.

    Args:
        root: Directory to scan recursively.

    Returns:
        Sorted list of offending files.
    """
    found: list[Path] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.suffix not in _SUFFIXES:
            continue
        text_path = str(path)
        if any(part in text_path for part in _SKIP_PARTS):
            continue
        if path.name.endswith(_SKIP_NAMES):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if _MEASUREMENT.search(text) and not _ADR_REF.search(text):
            found.append(path)
    return found


def cmd_check(roots: list[str]) -> int:
    """Run the audit and print the offenders.

    Args:
        roots: Directories to scan.

    Returns:
        Process exit code (0 clean, 1 violations).
    """
    offenders: list[Path] = []
    for name in roots:
        root = Path(name)
        if not root.is_dir():
            print(f"adr-audit: skipping missing root {name}")
            continue
        offenders.extend(violations(root))

    if not offenders:
        print(f"ADR refs OK ({len(roots)} roots)")
        return 0

    print("Code records a measurement but has no adr:NNNN reference:")
    for path in offenders:
        print(f"  {path}")
    return 1


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("check", help="Fail on measurement markers with no ADR ref")
    check.add_argument("roots", nargs="*", default=list(DEFAULT_ROOTS))
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point."""
    args = build_parser().parse_args(argv)
    if args.command == "check":
        return cmd_check(args.roots)
    return 2


if __name__ == "__main__":
    sys.exit(main())
