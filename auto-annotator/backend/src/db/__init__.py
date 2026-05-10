"""src.db. SQLite persistence layer (package).

Public API is re-exported from `src.db.core` so callers can use either:

    from src import db
    db.get_next()            # works because __init__ re-exports get_next

or

    from src.db import get_next, init_db

Sub-modules:
    * :mod:`src.db.schema`  – DDL strings and default class seed data.
    * :mod:`src.db.queries` – All SQL query strings as named constants.
    * :mod:`src.db.core`    – Implementation that uses schema and queries.
"""

from src.db.core import (
    add_images_from_paths,
    classes_to_yolo_map,
    delete_image,
    get_all_images,
    get_aug_count,
    get_by_id,
    get_children,
    get_classes,
    get_done_originals,
    get_grouped_images,
    get_next,
    get_prev,
    get_stats,
    init_db,
    mark_done,
    mark_skipped,
    register_augmented_image,
    upsert_class,
)

__all__ = [
    "add_images_from_paths",
    "classes_to_yolo_map",
    "delete_image",
    "get_all_images",
    "get_aug_count",
    "get_by_id",
    "get_children",
    "get_classes",
    "get_done_originals",
    "get_grouped_images",
    "get_next",
    "get_prev",
    "get_stats",
    "init_db",
    "mark_done",
    "mark_skipped",
    "register_augmented_image",
    "upsert_class",
]
