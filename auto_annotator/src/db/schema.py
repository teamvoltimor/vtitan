"""src.db.schema – SQLite DDL and seed data for the manifest database.

Centralises table definitions and default class records so that the DB layer
never embeds raw SQL DDL strings inline.
"""

DDL: str = """
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
"""Complete DDL script that creates both tables if they do not yet exist."""

DEFAULT_CLASSES: list[tuple[str, str]] = [
    ("red_prism", "#ee2737"),
    ("green_prism", "#44d62c"),
    ("magenta_prism", "#ff00ff"),
]
"""Annotation classes inserted on first run when the classes table is empty.

Each tuple is ``(name, hex_colour)`` matching the WRO 2026 traffic-sign palette.
"""
