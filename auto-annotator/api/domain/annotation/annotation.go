// Package annotation is the domain for image annotation management.
package annotation

import (
	"context"
	"fmt"
	"path/filepath"
	"strings"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/dataset"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

type (
	// Point is a normalized 2-D coordinate with float64 precision.
	Point struct {
		X float64
		Y float64
	}

	// Shape is a single polygon or bounding-box annotation.
	Shape struct {
		ID        string
		ClassName string
		Points    []Point
	}

	// Class is an annotation class definition.
	Class struct {
		Name  string
		Color string
		ID    int64
	}

	// Image is the minimal image representation needed by the annotation domain.
	Image struct {
		Path       string
		FormatUsed string
		ID         int64
		Status     int64
	}

	// SaveRequest is the annotation save payload.
	SaveRequest struct {
		ExportFormat string
		Shapes       []Shape
		ImageID      int64
	}

	// UpsertClassReq is the class upsert payload.
	UpsertClassReq struct {
		Name  string
		Color string
	}

	// StatusCount holds the per-status image count returned by the store.
	StatusCount struct {
		Status int64
		Count  int64
	}
)

// Store is the data boundary for the annotation domain.
type Store interface {
	GetImage(ctx context.Context, id int64) (Image, error)
	ListClasses(ctx context.Context) ([]Class, error)
	MarkImageDone(ctx context.Context, id int64, format string) error
	MarkImageSkipped(ctx context.Context, id int64) error
	UpsertClass(ctx context.Context, name, color string) error
}

// Service is the annotation domain service interface.
type Service interface {
	GetAnnotations(ctx context.Context, imageID int64) ([]Shape, error)
	Save(ctx context.Context, req SaveRequest) error
	Skip(ctx context.Context, imageID int64) error
	ListClasses(ctx context.Context) ([]Class, error)
	UpsertClass(ctx context.Context, req UpsertClassReq) error
}

// LoadAnnotations reads saved YOLO labels for a done image and returns its shapes.
// Returns an empty slice when the image is not done or has no format.
func LoadAnnotations(imageID int64, imagePath, status, format string, classNames []string, labelsDir string) []Shape {
	if status != domain.StatusName(domain.StatusDone) || format == "" {
		return []Shape{}
	}
	classDir := filepath.Base(filepath.Dir(imagePath))
	stem := strings.TrimSuffix(filepath.Base(imagePath), filepath.Ext(imagePath))

	records := dataset.Load(labelsDir, classDir, stem)
	shapes := make([]Shape, 0, len(records))
	for i, rec := range records {
		name := fmt.Sprintf(domain.ClassFallbackNameFmt, rec.ClassID)
		if rec.ClassID >= 0 && rec.ClassID < len(classNames) {
			name = classNames[rec.ClassID]
		}

		var pts []dataset.Point
		switch {
		case format == domain.FormatDetectionAbbr && len(rec.Coords) == domain.NumBBoxCoords:
			pts = dataset.CornersFromBBox(rec.Coords[0], rec.Coords[1], rec.Coords[2], rec.Coords[3])
		case len(rec.Coords) >= domain.MinPolygonCoords && len(rec.Coords)%domain.CoordPairSize == 0:
			pts = dataset.PolygonFromCoords(rec.Coords)
		default:
			continue
		}

		points := make([]Point, len(pts))
		for j, p := range pts {
			points[j] = Point{X: p.X, Y: p.Y}
		}
		shapes = append(shapes, Shape{
			ID:        fmt.Sprintf(domain.ShapeIDLoadedFmt, imageID, i),
			ClassName: name,
			Points:    points,
		})
	}
	return shapes
}
