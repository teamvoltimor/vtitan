// Package sqlite implements gallery.Store using SQLite via sqlc.
package sqlite

import (
	"context"
	"database/sql"
	"time"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/db"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/gallery"
)

const timeFormat = "2006-01-02T15:04:05.000000-07:00"

type galleryStore struct{ q *db.Queries }

// New returns a gallery.Store backed by the provided sqlc query set.
func New(q *db.Queries) gallery.Store { return &galleryStore{q: q} }

func (s *galleryStore) ListImagesForBrowse(ctx context.Context) ([]gallery.BrowseImage, error) {
	rows, err := s.q.ListImagesForBrowse(ctx)
	if err != nil {
		return nil, err
	}
	imgs := make([]gallery.BrowseImage, len(rows))
	for i, r := range rows {
		imgs[i] = gallery.BrowseImage{
			ID:         r.ID,
			Path:       r.Path,
			Status:     r.Status,
			FormatUsed: nullStr(r.FormatUsed),
			UpdatedAt:  nullStr(r.UpdatedAt),
		}
	}
	return imgs, nil
}

func (s *galleryStore) ListGroupedImages(ctx context.Context) ([]gallery.GroupedImage, error) {
	rows, err := s.q.ListGroupedImages(ctx)
	if err != nil {
		return nil, err
	}
	imgs := make([]gallery.GroupedImage, len(rows))
	for i, r := range rows {
		imgs[i] = gallery.GroupedImage{
			ID:         r.ID,
			Path:       r.Path,
			Status:     r.Status,
			FormatUsed: nullStr(r.FormatUsed),
			UpdatedAt:  nullStr(r.UpdatedAt),
			AugCount:   r.AugCount,
		}
	}
	return imgs, nil
}

func (s *galleryStore) GetStatusCounts(ctx context.Context) ([]gallery.StatusCount, error) {
	rows, err := s.q.GetStatusCounts(ctx)
	if err != nil {
		return nil, err
	}
	counts := make([]gallery.StatusCount, len(rows))
	for i, r := range rows {
		counts[i] = gallery.StatusCount{Status: r.Status, Count: r.Cnt}
	}
	return counts, nil
}

func (s *galleryStore) ListClassNames(ctx context.Context) ([]string, error) {
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

func (s *galleryStore) GetImage(ctx context.Context, id int64) (gallery.BrowseImage, error) {
	row, err := s.q.GetImageByID(ctx, id)
	if err != nil {
		return gallery.BrowseImage{}, err
	}
	return gallery.BrowseImage{
		ID:         row.ID,
		Path:       row.Path,
		Status:     row.Status,
		FormatUsed: nullStr(row.FormatUsed),
		UpdatedAt:  nullStr(row.UpdatedAt),
	}, nil
}

func (s *galleryStore) InsertImageOrIgnore(ctx context.Context, path string) error {
	return s.q.InsertImageOrIgnore(ctx, path)
}

func (s *galleryStore) SoftDeleteImage(ctx context.Context, id int64) error {
	return s.q.SoftDeleteImage(ctx, db.SoftDeleteImageParams{
		DeletedAt: sql.NullString{String: time.Now().UTC().Format(timeFormat), Valid: true},
		ID:        id,
	})
}

func nullStr(ns sql.NullString) string {
	if ns.Valid {
		return ns.String
	}
	return ""
}
