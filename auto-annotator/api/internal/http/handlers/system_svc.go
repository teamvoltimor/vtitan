package handlers

import (
	"context"
	"database/sql"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
)

type systemService struct {
	db  *sql.DB
	cfg config.Config
}

func newSystemService(sqlDB *sql.DB, cfg config.Config) SystemService {
	return &systemService{db: sqlDB, cfg: cfg}
}

func (s *systemService) Ping(ctx context.Context) error {
	return s.db.PingContext(ctx)
}

func (s *systemService) OpenAPISpec() ([]byte, error) {
	specPath := s.cfg.OpenAPIPath
	if specPath == "" {
		specPath = DefaultOpenAPIPath
	}
	abs, err := filepath.Abs(specPath)
	if err != nil {
		return nil, fmt.Errorf("resolve spec path: %w", err)
	}
	return os.ReadFile(abs)
}

// nowISO8601 returns the current UTC time in the ISO 8601 format used
// throughout the API, matching Python's datetime.isoformat() output.
func nowISO8601() string {
	return time.Now().UTC().Format(TimeFormatISO8601)
}
