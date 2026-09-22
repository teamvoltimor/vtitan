"""Audit that tracked prose still points at things that exist.

The repo already has a drift check for config (``configgen.py check``): every
TOML key is described and every ``x-journal`` ref resolves. Prose had nothing,
and it drifted on a schedule: an ADR's Decision named a directory that had
moved the day before, lint exclusions were disarmed by package moves three
times in one session without any tool noticing, and tracked files cited a plan
that had just been made gitignored. See ``adr:0097-doc-ownership-and-drift-checks``.

Each check covers only statements that claim to describe the present:

* ``adr-paths``: in an *accepted* ADR's Decision and Consequences sections,
  every backticked path under a live root resolves. Context and History
  describe the past and are exempt, as are superseded and deprecated ADRs.
* ``lint-exclusions``: every path-scoped exclusion in ``src/go/.golangci.yml``
  matches at least one tracked file. A dead exclusion silently makes lint
  stricter somewhere else, or more permissive after a later move.
* ``doc-refs``: a tracked file citing a ``docs/**.md`` path must point at a
  file that exists and is tracked. Pre-existing offenders live in
  ``doc_audit_baseline.txt``, which may only shrink: a new offender fails, and
  so does a baseline entry that has since been fixed, so the list cannot rot.

Commands:

* ``check``: run every check, print the offenders, exit 1 if any.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ADR_DIR = REPO_ROOT / "other" / "docs" / "adr"
GOLANGCI = REPO_ROOT / "src" / "go" / ".golangci.yml"
BASELINE = Path(__file__).resolve().parent / "doc_audit_baseline.txt"

# Roots a present-tense path can start with, and the directories it may be
# relative to. `platform/` is deliberately absent: it was dissolved, and an
# ADR describing the move from it is not describing the present tree.
_LIVE_ROOTS = ("src", "other", ".github", "internal", "pkg", "cmd", "scripts", "tests", "test", "ros2_ws", "shared")
_PATH_BASES = (".", "src/python", "src/go", "src", "other")
_NORMATIVE_SECTIONS = ("Decision", "Consequences")

_BACKTICK_PATH = re.compile(r"`([A-Za-z0-9_.\-]+(?:/[A-Za-z0-9_.\-]+)+/?)`")
_SECTION = re.compile(r"^## (.+)$", re.MULTILINE)
_STATUS = re.compile(r"^- Status: (\S+)", re.MULTILINE)
_EXCLUSION_PATH = re.compile(r"^\s*-?\s*path:\s*'?([^'\n]+?)'?\s*$", re.MULTILINE)
_DOC_REF = re.compile(r"(?<![\w./-])((?:[\w.-]+/)*docs/[\w./-]+?\.md)\b")
_DOC_REF_SUFFIXES = (".go", ".py", ".yml", ".yaml", ".toml", ".proto", ".md", ".sh", ".json")


def _tracked_files() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return [line for line in out.splitlines() if line]


def _resolves(ref: str, extra_base: str = "") -> list[Path]:
    bases = (*_PATH_BASES, extra_base) if extra_base else _PATH_BASES
    found = []
    for base in bases:
        candidate = (REPO_ROOT / base / ref).resolve()
        if candidate.exists():
            found.append(candidate)
    return found


def check_adr_paths() -> list[str]:
    """Return dead paths in accepted ADRs' normative sections."""
    problems = []
    for adr in sorted(ADR_DIR.glob("0*.md")):
        text = adr.read_text(encoding="utf-8")
        status = _STATUS.search(text)
        if status is None or status.group(1) != "accepted":
            continue
        parts = _SECTION.split(text)
        for name, body in zip(parts[1::2], parts[2::2], strict=True):
            if name.strip() not in _NORMATIVE_SECTIONS:
                continue
            for match in _BACKTICK_PATH.finditer(body):
                ref = match.group(1).rstrip("/")
                if ref.split("/")[0] not in _LIVE_ROOTS:
                    continue
                if not _resolves(ref):
                    problems.append(f"{adr.relative_to(REPO_ROOT)} ({name.strip()}): `{ref}` does not exist")
    return problems


def check_lint_exclusions(tracked: list[str]) -> list[str]:
    """Return path-scoped golangci exclusions that match no tracked file."""
    go_files = [f.removeprefix("src/go/") for f in tracked if f.startswith("src/go/")]
    problems = []
    for match in _EXCLUSION_PATH.finditer(GOLANGCI.read_text(encoding="utf-8")):
        pattern = match.group(1)
        # Only location-scoped patterns can go dead by a move; `_test\.go` and
        # `mock_.+\.go$` are about file kinds, not places.
        if "/" not in pattern:
            continue
        regex = re.compile(pattern)
        if not any(regex.search(f) for f in go_files):
            problems.append(f"src/go/.golangci.yml: exclusion path '{pattern}' matches no file")
    return problems


def _is_ignored(path: Path) -> bool:
    return (
        subprocess.run(
            ["git", "check-ignore", "-q", str(path)],
            cwd=REPO_ROOT,
            check=False,
        ).returncode
        == 0
    )


def find_doc_refs(tracked: list[str]) -> set[str]:
    """Return ``file: ref (why)`` for every tracked citation of a missing or ignored doc."""
    found = set()
    for rel in tracked:
        if not rel.endswith(_DOC_REF_SUFFIXES) or "/generated/" in rel or rel.endswith((".pb.go", "_pb2.py")):
            continue
        try:
            text = (REPO_ROOT / rel).read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for match in _DOC_REF.finditer(text):
            ref = match.group(1)
            hits = _resolves(ref, extra_base=str(Path(rel).parent))
            if not hits:
                found.add(f"{rel}: {ref} (missing)")
            elif all(_is_ignored(h) for h in hits):
                found.add(f"{rel}: {ref} (gitignored)")
    return found


def _read_baseline() -> set[str]:
    if not BASELINE.exists():
        return set()
    lines = BASELINE.read_text(encoding="utf-8").splitlines()
    return {line for line in lines if line and not line.startswith("#")}


def check_doc_refs(tracked: list[str]) -> list[str]:
    """Return doc-ref offenders not in the baseline, and baseline entries now fixed."""
    found = find_doc_refs(tracked)
    baseline = _read_baseline()
    problems = [f"{entry}" for entry in sorted(found - baseline)]
    problems += [
        f"{BASELINE.name}: '{entry}' is fixed, delete it from the baseline" for entry in sorted(baseline - found)
    ]
    return problems


def check() -> int:
    """Run every check and report."""
    tracked = _tracked_files()
    results = {
        "adr-paths": check_adr_paths(),
        "lint-exclusions": check_lint_exclusions(tracked),
        "doc-refs": check_doc_refs(tracked),
    }
    failed = False
    for name, problems in results.items():
        if problems:
            failed = True
            print(f"{name}: {len(problems)} problem(s)")
            for problem in problems:
                print(f"  {problem}")
    if failed:
        return 1
    print(f"docs OK ({', '.join(results)})")
    return 0


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="Run every docs drift check")
    parser.parse_args()
    return check()


if __name__ == "__main__":
    sys.exit(main())
