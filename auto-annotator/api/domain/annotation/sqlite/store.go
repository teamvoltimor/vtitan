// Package sqlite implements annotation.Store using SQLite via sqlc.
package sqlite

import (
	"context"
	"database/sql"
	"time"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/db"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/domain"
)

const timeFormat = "2006-01-02T15:04:05.000000-07:00"

type annotationStore struct{ q *db.Queries }

// New returns an annotation.Store backed by the provided sqlc query set.
func New(q *db.Queries) annotation.Store { return &annotationStore{q: q} }

func (s *annotationStore) GetImage(ctx context.Context, id int64) (annotation.Image, error) {
	row, err := s.q.GetImageByID(ctx, id)
	if err != nil {
		return annotation.Image{}, err
	}
	return annotation.Image{
		ID:         row.ID,
		Path:       row.Path,
		Status:     row.Status,
		FormatUsed: nullStr(row.FormatUsed),
	}, nil
}

func (s *annotationStore) ListClasses(ctx context.Context) ([]annotation.Class, error) {
	rows, err := s.q.ListClasses(ctx)
	if err != nil {
		return nil, err
	}
	cls := make([]annotation.Class, len(rows))
	for i, r := range rows {
		cls[i] = annotation.Class{ID: r.ID, Name: r.Name, Color: r.Color}
	}
	return cls, nil
}

func (s *annotationStore) MarkImageDone(ctx context.Context, id int64, format string) error {
	return s.q.MarkImageDone(ctx, db.MarkImageDoneParams{
		Status:     domain.StatusDone,
		FormatUsed: sql.NullString{String: format, Valid: true},
		UpdatedAt:  sql.NullString{String: time.Now().UTC().Format(timeFormat), Valid: true},
		ID:         id,
	})
}

func (s *annotationStore) MarkImageSkipped(ctx context.Context, id int64) error {
	return s.q.MarkImageSkipped(ctx, db.MarkImageSkippedParams{
		Status:    domain.StatusSkipped,
		UpdatedAt: sql.NullString{String: time.Now().UTC().Format(timeFormat), Valid: true},
		ID:        id,
	})
}

func (s *annotationStore) UpsertClass(ctx context.Context, name, color string) error {
	return s.q.UpsertClass(ctx, db.UpsertClassParams{Name: name, Color: color})
}

func nullStr(ns sql.NullString) string {
	if ns.Valid {
		return ns.String
	}
	return ""
}
