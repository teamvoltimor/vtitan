package recorder

import (
	"bufio"
	"context"
	"database/sql"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"

	"go.uber.org/zap"
	"google.golang.org/protobuf/encoding/protojson"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/sqlcdb"
	_ "modernc.org/sqlite" // SQLite driver (pure Go, no cgo)
)

const (
	sessionIDFormat    = "session_%d"
	sqliteDSNOptions   = "?_journal_mode=WAL&_busy_timeout=5000"
	sqliteMaxOpenConns = 1
	writerBufSize      = 64 * 1024
	scannerBufSize     = 1 << 20
	dirPerm            = 0o755
	filePerm           = 0o644
)

// ErrSessionNotFound is returned when a session ID does not exist in the index.
var ErrSessionNotFound = errors.New("session not found")

type (
	// SessionInfo is the public summary type for a recorded session.
	// time.Time comes first so its embedded pointer (loc) is within the first
	// 24 bytes, reducing the GC-scanned span.
	SessionInfo struct {
		CreatedAt  time.Time
		SessionID  string
		EntryCount int
	}

	// Recorder persists RobotSnapshot frames to JSONL files and maintains a SQLite
	// session index. One Recorder instance = one active session; each server restart
	// opens a fresh session.
	// Pointer fields are grouped first to minimize the GC-scanned span.
	Recorder struct {
		queries     *sqlcdb.Queries
		sqlDB       *sql.DB
		file        *os.File
		writer      *bufio.Writer
		log         *zap.Logger
		marshaler   protojson.MarshalOptions
		unmarshaler protojson.UnmarshalOptions
		sessionID   string
		framesDir   string
		mu          sync.Mutex
		maxSessions int
	}
)

const initSchema = `
CREATE TABLE IF NOT EXISTS sessions (
    session_id  TEXT    NOT NULL PRIMARY KEY,
    created_at  INTEGER NOT NULL,
    entry_count INTEGER NOT NULL DEFAULT 0
);`

// New opens (or creates) the SQLite session index at dbPath, creates the frames
// directory, and registers a new session for this process lifetime.
func New(ctx context.Context, dbPath, framesDir string, maxSessions int, log *zap.Logger) (*Recorder, error) {
	if err := os.MkdirAll(framesDir, dirPerm); err != nil {
		return nil, fmt.Errorf("create frames dir: %w", err)
	}
	if err := os.MkdirAll(filepath.Dir(dbPath), dirPerm); err != nil {
		return nil, fmt.Errorf("create db dir: %w", err)
	}

	db, err := sql.Open("sqlite", dbPath+sqliteDSNOptions)
	if err != nil {
		return nil, fmt.Errorf("open sqlite: %w", err)
	}
	db.SetMaxOpenConns(sqliteMaxOpenConns) // SQLite write serialization

	if _, err := db.ExecContext(ctx, initSchema); err != nil {
		db.Close()
		return nil, fmt.Errorf("apply schema: %w", err)
	}

	sessionID := fmt.Sprintf(sessionIDFormat, time.Now().UnixMilli())
	queries := sqlcdb.New(db)

	params := sqlcdb.InsertSessionParams{SessionID: sessionID, CreatedAt: time.Now().UnixMilli()}
	if err := queries.InsertSession(ctx, params); err != nil {
		db.Close()
		return nil, fmt.Errorf("insert session: %w", err)
	}

	r := &Recorder{
		queries:     queries,
		sqlDB:       db,
		sessionID:   sessionID,
		framesDir:   framesDir,
		maxSessions: maxSessions,
		log:         log,
		marshaler:   protojson.MarshalOptions{EmitUnpopulated: false, UseProtoNames: true},
		unmarshaler: protojson.UnmarshalOptions{DiscardUnknown: true},
	}

	if err := r.pruneOldSessions(ctx); err != nil {
		log.Warn("prune old sessions", zap.Error(err))
	}

	log.Info("recorder started", zap.String("session_id", sessionID), zap.String("frames_dir", framesDir))
	return r, nil
}

// Record persists a snapshot frame to the active session's JSONL file and
// increments the session's entry count in SQLite.
// Persistence errors are logged but do not propagate — callers (the ingest
// stream) must not be interrupted by disk errors.
func (r *Recorder) Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error {
	b, err := r.marshaler.Marshal(snap)
	if err != nil {
		return fmt.Errorf("marshal snapshot: %w", err)
	}

	r.mu.Lock()
	defer r.mu.Unlock()

	if err := r.ensureFile(); err != nil {
		return err
	}
	if _, err := r.writer.Write(b); err != nil {
		return fmt.Errorf("write frame: %w", err)
	}
	if err := r.writer.WriteByte('\n'); err != nil {
		return fmt.Errorf("write newline: %w", err)
	}
	if err := r.writer.Flush(); err != nil {
		return fmt.Errorf("flush: %w", err)
	}

	if err := r.queries.IncrementEntryCount(ctx, r.sessionID); err != nil {
		r.log.Warn("increment entry count", zap.Error(err))
	}
	return nil
}

// ListSessions returns all sessions ordered newest-first from the SQLite index.
func (r *Recorder) ListSessions(ctx context.Context) ([]SessionInfo, error) {
	rows, err := r.queries.ListSessions(ctx)
	if err != nil {
		return nil, fmt.Errorf("list sessions: %w", err)
	}
	out := make([]SessionInfo, len(rows))
	for i, row := range rows {
		out[i] = SessionInfo{
			CreatedAt:  time.UnixMilli(row.CreatedAt),
			SessionID:  row.SessionID,
			EntryCount: int(row.EntryCount),
		}
	}
	return out, nil
}

// LoadSession reads all frames from the JSONL file for the given session.
// Returns ErrSessionNotFound if the session ID is not in the index.
func (r *Recorder) LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error) {
	if _, err := r.queries.GetSession(ctx, sessionID); errors.Is(err, sql.ErrNoRows) {
		return nil, ErrSessionNotFound
	} else if err != nil {
		return nil, fmt.Errorf("get session: %w", err)
	}

	path := filepath.Join(r.framesDir, sessionID+".jsonl")
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("open session file: %w", err)
	}
	defer f.Close()

	var snaps []*telemetryv1.RobotSnapshot
	scanner := bufio.NewScanner(f)
	scanner.Buffer(make([]byte, scannerBufSize), scannerBufSize)

	for scanner.Scan() {
		line := scanner.Bytes()
		if len(line) == 0 {
			continue
		}
		snap := &telemetryv1.RobotSnapshot{}
		if err := r.unmarshaler.Unmarshal(line, snap); err != nil {
			r.log.Warn("skip malformed frame", zap.Error(err))
			continue
		}
		snaps = append(snaps, snap)
	}
	if err := scanner.Err(); err != nil {
		return nil, fmt.Errorf("scan session file: %w", err)
	}
	return snaps, nil
}

// Close flushes any pending writes and closes both the JSONL file and the DB.
func (r *Recorder) Close() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.writer != nil {
		_ = r.writer.Flush()
	}
	if r.file != nil {
		_ = r.file.Close()
	}
	return r.sqlDB.Close()
}

func (r *Recorder) ensureFile() error {
	if r.file != nil {
		return nil
	}
	path := filepath.Join(r.framesDir, r.sessionID+".jsonl")
	f, err := os.OpenFile(path, os.O_CREATE|os.O_APPEND|os.O_WRONLY, filePerm)
	if err != nil {
		return fmt.Errorf("open frames file: %w", err)
	}
	r.file = f
	r.writer = bufio.NewWriterSize(f, writerBufSize)
	return nil
}

// pruneOldSessions deletes the oldest sessions (file + DB row) until the count
// is within maxSessions. Called once at startup after the new session is inserted.
func (r *Recorder) pruneOldSessions(ctx context.Context) error {
	count, err := r.queries.CountSessions(ctx)
	if err != nil {
		return err
	}
	for count > int64(r.maxSessions) {
		rows, err := r.queries.ListSessions(ctx)
		if err != nil || len(rows) == 0 {
			return err
		}
		oldest := rows[len(rows)-1] // ListSessions is DESC → oldest is last
		_ = os.Remove(filepath.Join(r.framesDir, oldest.SessionID+".jsonl"))
		if err := r.queries.DeleteSession(ctx, oldest.SessionID); err != nil {
			return err
		}
		r.log.Info("pruned old session", zap.String("session_id", oldest.SessionID))
		count--
	}
	return nil
}
