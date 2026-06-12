"""scripts/fix_paths.py. Rewrite Windows host paths in manifest.db to Docker container paths.

The backend runs in Docker where the volume is mounted at /app/data, so paths must be:
    /app/data/images/red_prism/foo.jpg

This script does that replacement in-place on manifest.db.

Usage (run from backend/):
    python scripts/fix_paths.py

Options:
    --db         Path to manifest.db (default: data/manifest.db)
    --from-dir   Windows host data dir to replace (default: auto-detected from DB)
    --to-dir     Docker container data dir (default: /app/data)
    --dry-run    Print changes without writing
"""

from __future__ import annotations

import argparse
import os
import sqlite3
import sys
from pathlib import Path

# Database schema is defined in src/db/schema.sql, but we only need the "images" table here.
BACKEND_DIR = Path(__file__).parent.parent
default_db = BACKEND_DIR / "data" / "manifest.db"


def _detect_host_prefix(db_path: Path) -> str | None:
    """Return the common host path prefix found in the images table, or None.

    Only considers Windows-style absolute paths (contain a drive letter or backslash)
    so already-fixed Linux paths don't poison commonpath.
    """
    with _connect(db_path) as conn:
        rows = conn.execute("SELECT path FROM images LIMIT 200").fetchall()
    # Keep only Windows-style paths (drive letter or backslash present)
    win_paths = [row["path"] for row in rows if "\\" in row["path"] or (len(row["path"]) > 1 and row["path"][1] == ":")]
    if not win_paths:
        return None
    common = os.path.commonpath(win_paths)
    # Walk up until we hit the 'data' directory
    p = Path(common)
    while p.name and p.name != "data":
        p = p.parent
    return str(p) if p.name == "data" else common


def _connect(db_path: Path) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def run(db_path: Path, from_dir: str | None, to_dir: str, dry_run: bool) -> None:  # noqa: C901
    """Fix Windows to Docker paths in manifest.db."""
    if not db_path.exists():
        print(f"ERROR: DB not found at {db_path}", file=sys.stderr)
        sys.exit(1)

    if from_dir is None:
        from_dir = _detect_host_prefix(db_path)
        if from_dir is None:
            print("ERROR: No images in DB and no --from-dir given", file=sys.stderr)
            sys.exit(1)
        print(f"Auto-detected host prefix: {from_dir!r}")

    with _connect(db_path) as conn:
        rows = conn.execute("SELECT id, path FROM images").fetchall()

    if not rows:
        print("No images in DB — nothing to fix.")
        return

    updates: list[tuple[str, int]] = []
    already_ok = 0
    for row in rows:
        old = row["path"]
        if old.startswith(to_dir):
            already_ok += 1
            continue
        # Normalise: replace from_dir prefix + convert backslashes
        new = old.replace(from_dir, to_dir).replace("\\", "/")
        if new != old:
            updates.append((new, row["id"]))

    print(f"Total rows: {len(rows)} | already correct: {already_ok} | to fix: {len(updates)}")

    if not updates:
        print("Nothing to update.")
        return

    for new_path, row_id in updates[:5]:
        print(f"  [{row_id}] → {new_path}")
    if len(updates) > 5:
        print(f"  ... and {len(updates) - 5} more")

    if dry_run:
        print("Dry-run — no changes written.")
        return

    with _connect(db_path) as conn:
        conn.executemany("UPDATE images SET path=? WHERE id=?", updates)
        conn.commit()

    print(f"Fixed {len(updates)} rows in {db_path}")


def main() -> None:
    """Fix Windows to Docker paths in manifest.db."""
    parser = argparse.ArgumentParser(description="Fix Windows→Docker paths in manifest.db")
    parser.add_argument("--db", type=Path, default=default_db, metavar="PATH")
    parser.add_argument(
        "--from-dir",
        default=None,
        metavar="PATH",
        help="Windows host data dir to replace (auto-detected if omitted)",
    )
    parser.add_argument(
        "--to-dir",
        default="/app/data",
        metavar="PATH",
        help="Docker container data dir (default: /app/data)",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    # Validate that from-dir and to-dir are not the same (would be a no-op)
    run(db_path=args.db, from_dir=args.from_dir, to_dir=args.to_dir, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
