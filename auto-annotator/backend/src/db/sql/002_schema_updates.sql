-- Add soft delete column and constraints to images
CREATE TABLE images_new (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    path        TEXT UNIQUE NOT NULL,
    status      INTEGER DEFAULT 0 CHECK (status IN (0, 1, 2)),
    format_used TEXT,
    parent_id   INTEGER REFERENCES images(id) ON DELETE CASCADE,
    created_at  TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at  TIMESTAMP,
    deleted_at  TIMESTAMP
);

INSERT INTO images_new (id, path, status, format_used, parent_id, created_at, updated_at)
SELECT id, path, status, format_used, parent_id, created_at, updated_at FROM images;

DROP TABLE images;
ALTER TABLE images_new RENAME TO images;

CREATE INDEX ix_images_status    ON images(status);
CREATE INDEX ix_images_parent_id ON images(parent_id);

-- Add constraints to classes
CREATE TABLE classes_new (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT UNIQUE NOT NULL,
    color      TEXT DEFAULT '#dc322f' CHECK (color LIKE '#%' AND length(color) IN (4, 7, 9)),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

INSERT INTO classes_new (id, name, color, created_at)
SELECT id, name, color, created_at FROM classes;

DROP TABLE classes;
ALTER TABLE classes_new RENAME TO classes;
