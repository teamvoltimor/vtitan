package store

const (
	// SQLiteJournalMode is the SQLite PRAGMA for WAL journal mode.
	SQLiteJournalMode = "journal_mode(WAL)"
	// SQLiteForeignKeys is the SQLite PRAGMA to enable foreign key constraints.
	SQLiteForeignKeys = "foreign_keys(ON)"
	// SQLiteBusyTimeout is the SQLite PRAGMA for busy timeout in milliseconds.
	SQLiteBusyTimeout = "busy_timeout(5000)"
	// DBBusyTimeoutMs is the database busy timeout in milliseconds.
	DBBusyTimeoutMs = 5000
)
