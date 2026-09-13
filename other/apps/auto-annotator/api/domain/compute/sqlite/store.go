// Package sqlite implements the compute.Store interface using SQLite via sqlc.
package sqlite

import (
	"context"
	"database/sql"
	"time"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/db"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

const timeFormat = domain.TimeFormat

type computeStore struct{ q *db.Queries }

// New returns a compute.Store backed by the provided sqlc query set.
func New(q *db.Queries) compute.Store { return &computeStore{q: q} }

func (s *computeStore) GetImage(ctx context.Context, id int64) (compute.ImageRef, error) {
	row, err := s.q.GetImageByID(ctx, id)
	if err != nil {
		return compute.ImageRef{}, err
	}
	return compute.ImageRef{
		ID:         row.ID,
		Path:       row.Path,
		FormatUsed: nullStr(row.FormatUsed),
	}, nil
}

func (s *computeStore) ListClassNames(ctx context.Context) ([]string, error) {
	rows, err := s.q.ListClasses(ctx)
	if err != nil {
		return nil, err
	}
	names := make([]string, len(rows))
	for i, r := range rows {
		names[i] = r.Name
	}
	return names, nil
}

func (s *computeStore) InsertAugmented(ctx context.Context, img compute.AugmentedImage) error {
	return s.q.InsertAugmentedImage(ctx, db.InsertAugmentedImageParams{
		Path:       img.Path,
		Status:     domain.StatusDone,
		FormatUsed: sql.NullString{String: img.FormatUsed, Valid: true},
		ParentID:   sql.NullInt64{Int64: img.ParentID, Valid: true},
		UpdatedAt:  sql.NullString{String: time.Now().UTC().Format(timeFormat), Valid: true},
	})
}

func nullStr(ns sql.NullString) string {
	if ns.Valid {
		return ns.String
	}
	return ""
}
