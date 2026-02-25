"""src.schema – SQLite DDL for the manifest database."""

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

DEFAULT_CLASSES: list[tuple[str, str]] = [
    ("red_prism", "#ee2737"),
    ("green_prism", "#44d62c"),
    ("magenta_prism", "#ff00ff"),
]
