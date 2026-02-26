"""src.db – SQLite persistence layer (package).

Public API is re-exported from :mod:`src.db.core` so callers can use either::

    from src import db
    db.get_next()            # works because __init__ re-exports get_next

or::

    from src.db import get_next, init_db

Sub-modules:
    * :mod:`src.db.schema`  – DDL strings and default class seed data.
    * :mod:`src.db.queries` – All SQL query strings as named constants.
    * :mod:`src.db.core`    – Implementation that uses schema and queries.
"""

from src.db.core import (
    add_images_from_paths,
    classes_to_yolo_map,
    get_all_images,
    get_by_id,
    get_classes,
    get_next,
    get_prev,
    get_stats,
    init_db,
    mark_done,
    mark_skipped,
    upsert_class,
)

__all__ = [
    "add_images_from_paths",
    "classes_to_yolo_map",
    "get_all_images",
    "get_by_id",
    "get_classes",
    "get_next",
    "get_prev",
    "get_stats",
    "init_db",
    "mark_done",
    "mark_skipped",
    "upsert_class",
]
