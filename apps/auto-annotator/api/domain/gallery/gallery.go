// Package gallery is the domain for image gallery management.
package gallery

import (
	"context"
	"mime/multipart"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
)

type (
	// BrowseImage is the minimal image record for gallery listing.
	BrowseImage struct {
		Path       string
		FormatUsed string
		UpdatedAt  string
		ID         int64
		Status     int64
	}

	// GroupedImage is a parent image with augmentation count, returned by GetGroupedGallery.
	GroupedImage struct {
		Path       string
		ImageURL   string
		Label      string
		FormatUsed string
		UpdatedAt  string
		ID         int64
		Status     int64
		AugCount   int64
	}

	// StatusCount holds per-status image counts.
	StatusCount struct {
		Status int64
		Count  int64
	}

	// Stats holds aggregate gallery statistics.
	Stats struct {
		Pending int
		Done    int
		Skipped int
		Total   int
		Pct     float64
	}

	// GalleryItem is a single image entry in the gallery, with its annotations.
	GalleryItem struct {
		Path        string
		ImageURL    string
		ThumbURL    string
		Format      string
		Status      string
		UpdatedAt   string
		Annotations []annotation.Shape
		ID          int64
	}

	// Gallery is the gallery response with items and statistics.
	Gallery struct {
		Items []GalleryItem
		Stats Stats
	}
)

// Store is the data boundary for the gallery domain.
type Store interface {
	ListImagesForBrowse(ctx context.Context) ([]BrowseImage, error)
	ListGroupedImages(ctx context.Context) ([]GroupedImage, error)
	GetStatusCounts(ctx context.Context) ([]StatusCount, error)
	ListClassNames(ctx context.Context) ([]string, error)
	GetImage(ctx context.Context, id int64) (BrowseImage, error)
	InsertImageOrIgnore(ctx context.Context, path string) error
	SoftDeleteImage(ctx context.Context, id int64) error
}

// Service is the gallery domain service interface.
type Service interface {
	GetGallery(ctx context.Context) (Gallery, error)
	GetGroupedGallery(ctx context.Context) ([]GroupedImage, error)
	Import(ctx context.Context, files []*multipart.FileHeader) (Gallery, error)
	Delete(ctx context.Context, ids []int64) (Gallery, error)
	ImagePath(ctx context.Context, id int64) (string, error)
	ThumbPath(ctx context.Context, id int64) (string, error)
}
