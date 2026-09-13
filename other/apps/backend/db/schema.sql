CREATE TABLE IF NOT EXISTS sessions (
    session_id  TEXT    NOT NULL PRIMARY KEY,
    created_at  INTEGER NOT NULL,          -- Unix milliseconds
    entry_count INTEGER NOT NULL DEFAULT 0
);
