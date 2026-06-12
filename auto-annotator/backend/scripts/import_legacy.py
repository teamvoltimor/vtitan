"""scripts/import_legacy.py – Import legacy gmr/labeled.zip into the auto-annotator backend.

Implements Path A from docs/LEGACY_DATASET_IMPORT_REVIEW.md.
Self-contained: stdlib only, no src imports, no torch/numpy dependency.

Usage (run from backend/):
    python scripts/import_legacy.py --zip <path/to/gmr/labeled.zip>

Optional flags:
    --dry-run      Print what would happen without writing anything.
    --data-dir     Override backend/data/ directory (default: <script>/../data).
    --db           Override DB path (default: <data-dir>/manifest.db).
    --path-prefix  Prefix stored in DB for image paths (default: same as --data-dir).
                   Set to /app/data when the backend runs in Docker so serve_image
                   can resolve paths inside the container.
                   Example: --path-prefix /app/data
"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import sqlite3
import sys
import tempfile
import zipfile
from datetime import UTC, datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Logging (JSON to stdout, mirrors src.utils pattern)
# ---------------------------------------------------------------------------


class _JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload = {
            "timestamp": datetime.now(UTC).isoformat(),
            "level": record.levelname,
            "message": record.getMessage(),
        }
        extra = getattr(record, "_extra", None)
        if extra:
            payload.update(extra)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload)


def _make_logger(name: str) -> logging.Logger:
    log = logging.getLogger(name)
    if not log.handlers:
        h = logging.StreamHandler(sys.stdout)
        h.setFormatter(_JsonFormatter())
        log.addHandler(h)
        log.setLevel(logging.DEBUG)
        log.propagate = False
    return log


def _log(logger: logging.Logger, level: str, msg: str, **kw: object) -> None:
    record = logger.makeRecord(logger.name, getattr(logging, level), "", 0, msg, (), None)
    record.__dict__.update(kw)
    logger.handle(record)


logger = _make_logger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_NAME_REMAP: dict[str, str] = {
    "green rectangular prism": "green_prism",
    "magenta rectangular prism": "magenta_prism",
    "red rectangular prism": "red_prism",
}

_VALID_EXTS: frozenset[str] = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})

_DEFAULT_CLASSES: list[tuple[str, str]] = [
    ("red_prism", "#ee2737"),
    ("green_prism", "#44d62c"),
    ("magenta_prism", "#ff00ff"),
]

# ---------------------------------------------------------------------------
# DB helpers (pure sqlite3, no src.db dependency)
# ---------------------------------------------------------------------------

_DDL = """
CREATE TABLE IF NOT EXISTS classes (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT UNIQUE NOT NULL,
    color      TEXT DEFAULT '#dc322f',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE TABLE IF NOT EXISTS images (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    path        TEXT UNIQUE NOT NULL,
    status      INTEGER DEFAULT 0,
    format_used TEXT,
    created_at  TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at  TIMESTAMP
);
"""


def _connect(db_path: Path) -> sqlite3.Connection:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def _init_db(db_path: Path) -> None:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    with _connect(db_path) as conn:
        conn.executescript(_DDL)
        conn.commit()
        count = conn.execute("SELECT COUNT(*) FROM classes").fetchone()[0]
        if count == 0:
            conn.executemany("INSERT OR IGNORE INTO classes (name, color) VALUES (?, ?)", _DEFAULT_CLASSES)
            conn.commit()


def _get_classes(db_path: Path) -> list[tuple[int, str]]:
    with _connect(db_path) as conn:
        rows = conn.execute("SELECT id, name FROM classes ORDER BY id ASC").fetchall()
    return [(row["id"], row["name"]) for row in rows]


def _add_images(paths: list[str], db_path: Path) -> int:
    with _connect(db_path) as conn:
        existing = {row["path"] for row in conn.execute("SELECT path FROM images").fetchall()}
        new_rows = [(p,) for p in paths if p not in existing]
        if new_rows:
            conn.executemany("INSERT OR IGNORE INTO images (path) VALUES (?)", new_rows)
            conn.commit()
    return len(new_rows)


def _mark_done(path: str, fmt: str, db_path: Path) -> None:
    now = datetime.now(UTC).isoformat()
    with _connect(db_path) as conn:
        conn.execute(
            "UPDATE images SET status=1, format_used=?, updated_at=? WHERE path=?",
            (fmt, now, path),
        )
        conn.commit()


def _write_data_yaml(data_dir: Path, db_path: Path) -> None:
    classes = _get_classes(db_path)
    if not classes:
        return
    images_dir = data_dir / "images"
    existing = [(cid, name) for cid, name in classes if (images_dir / name).is_dir()]
    if not existing:
        return
    train_lines = "\n".join(f"  - images/{name}" for _, name in existing)
    names_lines = "\n".join(f"  - {name}" for _, name in classes)
    yaml_content = (
        f"path: {data_dir.resolve()}\n"
        f"train:\n{train_lines}\n\n"
        f"val:\n{train_lines}\n\n"
        f"nc: {len(classes)}\n"
        f"names:\n{names_lines}\n"
    )
    out = data_dir / "data.yaml"
    out.write_text(yaml_content, encoding="utf-8")
    _log(logger, "INFO", "data_yaml_written", path=str(out))


# ---------------------------------------------------------------------------
# Core logic
# ---------------------------------------------------------------------------


def _parse_legacy_classes(classes_txt: Path) -> dict[int, str]:
    lines = [ln.strip() for ln in classes_txt.read_text(encoding="utf-8").splitlines() if ln.strip()]
    return dict(enumerate(lines))


def _build_idx_remap(legacy_classes: dict[int, str], new_name_to_idx: dict[str, int]) -> dict[int, int]:
    remap: dict[int, int] = {}
    for legacy_idx, legacy_name in legacy_classes.items():
        new_name = _NAME_REMAP.get(legacy_name)
        if new_name is None:
            _log(logger, "WARNING", "unknown_legacy_class", legacy_idx=legacy_idx, legacy_name=legacy_name)
            continue
        new_idx = new_name_to_idx.get(new_name)
        if new_idx is None:
            _log(logger, "WARNING", "class_not_in_db", new_name=new_name)
            continue
        remap[legacy_idx] = new_idx
    return remap


def _rewrite_label(lines: list[str], idx_remap: dict[int, int]) -> list[str] | None:
    out: list[str] = []
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        new_idx = idx_remap.get(int(parts[0]))
        if new_idx is None:
            return None
        out.append(" ".join([str(new_idx), *parts[1:]]))
    return out


def run(zip_path: Path, dry_run: bool, data_dir: Path, db_path: Path, path_prefix: str | None = None) -> None:  # noqa: C901, PLR0912, PLR0915
    """Import legacy dataset from zip into auto-annotator."""
    if not zip_path.exists():
        _log(logger, "ERROR", "zip_not_found", path=str(zip_path))
        sys.exit(1)
    if not zipfile.is_zipfile(zip_path):
        _log(logger, "ERROR", "not_a_zip", path=str(zip_path))
        sys.exit(1)

    if not dry_run:
        _init_db(db_path)

    with tempfile.TemporaryDirectory(prefix="voldemorbot_import_") as staging_root:
        _log(logger, "INFO", "extracting_zip", zip=str(zip_path))
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(staging_root)

        staging = Path(staging_root) / "labeled"
        classes_txt = staging / "classes.txt"
        if not classes_txt.exists():
            _log(logger, "ERROR", "classes_txt_missing", expected=str(classes_txt))
            sys.exit(1)

        legacy_classes = _parse_legacy_classes(classes_txt)
        _log(logger, "INFO", "legacy_classes", mapping=legacy_classes)

        if dry_run:
            db_classes: list[tuple[int, str]] = [(i + 1, name) for i, (name, _) in enumerate(_DEFAULT_CLASSES)]
        else:
            db_classes = _get_classes(db_path)

        new_name_to_idx: dict[str, int] = {name: idx for idx, (_, name) in enumerate(db_classes)}
        _log(logger, "INFO", "db_classes", mapping=new_name_to_idx)

        idx_remap = _build_idx_remap(legacy_classes, new_name_to_idx)
        _log(logger, "INFO", "idx_remap", mapping=idx_remap)

        if not idx_remap:
            _log(logger, "ERROR", "empty_remap_abort")
            sys.exit(1)

        images_src = staging / "processed" / "images"
        labels_src = staging / "processed" / "labels"

        if not images_src.is_dir():
            _log(logger, "ERROR", "images_dir_missing", expected=str(images_src))
            sys.exit(1)

        image_files = sorted(f for f in images_src.iterdir() if f.suffix.lower() in _VALID_EXTS)
        _log(logger, "INFO", "images_found", count=len(image_files))

        images_dir = data_dir / "images"
        labels_dir = data_dir / "labels"

        imported = 0
        skipped = 0
        dest_paths: list[str] = []

        for img_path in image_files:
            stem = img_path.stem
            label_path = labels_src / f"{stem}.txt"

            if not label_path.exists():
                _log(logger, "WARNING", "label_missing_skip", image=img_path.name)
                skipped += 1
                continue

            raw_lines = [ln.strip() for ln in label_path.read_text(encoding="utf-8").splitlines() if ln.strip()]
            if not raw_lines:
                _log(logger, "WARNING", "empty_label_skip", image=img_path.name)
                skipped += 1
                continue

            rewritten = _rewrite_label(raw_lines, idx_remap)
            if rewritten is None:
                _log(logger, "WARNING", "unmapped_class_skip", image=img_path.name)
                skipped += 1
                continue

            primary_idx = int(rewritten[0].split()[0])
            primary_name = db_classes[primary_idx][1]
            dest_img = images_dir / primary_name / img_path.name
            dest_lbl = labels_dir / primary_name / f"{stem}.txt"

            if dry_run:
                _log(logger, "INFO", "dry_run_would_write", image=str(dest_img), label=str(dest_lbl))
                imported += 1
                continue

            dest_img.parent.mkdir(parents=True, exist_ok=True)
            dest_lbl.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(img_path, dest_img)
            dest_lbl.write_text("\n".join(rewritten) + "\n", encoding="utf-8")

            if path_prefix is not None:
                # Store path as <path_prefix>/images/<class>/<file> with forward slashes
                db_path_str = f"{path_prefix.rstrip('/')}/images/{primary_name}/{img_path.name}"
            else:
                db_path_str = str(dest_img)
            dest_paths.append(db_path_str)
            imported += 1

        if dry_run:
            _log(logger, "INFO", "dry_run_complete", would_import=imported, skipped=skipped)
            return

        if dest_paths:
            inserted = _add_images(dest_paths, db_path)
            _log(logger, "INFO", "images_registered", inserted=inserted, total=len(dest_paths))
            for p in dest_paths:
                _mark_done(p, "det", db_path)
            _log(logger, "INFO", "images_marked_done", count=len(dest_paths))

        _write_data_yaml(data_dir, db_path)
        _log(logger, "INFO", "import_complete", imported=imported, skipped=skipped)


def main() -> None:
    """CLI entry point for legacy import."""
    backend_dir = Path(__file__).parent.parent
    default_data = backend_dir / "data"
    default_db = default_data / "manifest.db"

    parser = argparse.ArgumentParser(description="Import legacy gmr/labeled.zip into auto-annotator.")
    parser.add_argument("--zip", required=True, type=Path, metavar="PATH", help="Path to gmr/labeled.zip")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without writing files or DB")
    parser.add_argument(
        "--data-dir", type=Path, default=default_data, metavar="PATH", help="Override backend/data/ dir",
    )
    parser.add_argument("--db", type=Path, default=default_db, metavar="PATH", help="Override DB path")
    parser.add_argument(
        "--path-prefix",
        default=None,
        metavar="PATH",
        help="Path prefix stored in DB (use /app/data when backend runs in Docker)",
    )
    args = parser.parse_args()

    run(zip_path=args.zip, dry_run=args.dry_run, data_dir=args.data_dir, db_path=args.db, path_prefix=args.path_prefix)


if __name__ == "__main__":
    main()
