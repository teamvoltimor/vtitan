// Package handlers contains the Gin HTTP handlers and the router wiring.
// Each handler struct owns a domain service interface; the App struct is the composition root.
package handlers

import (
	"context"
	"database/sql"

	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	computedomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
	gallerydomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/gallery"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/config"
)

// SystemService covers health probes and spec serving.
type SystemService interface {
	Ping(ctx context.Context) error
	OpenAPISpec() ([]byte, error)
}

// GalleryHandler handles gallery and image-file routes.
type GalleryHandler struct{ svc gallerydomain.Service }

// AnnotationHandler handles annotation and class routes. It holds a gallery
// reference to rebuild the gallery response after save/skip mutations.
type AnnotationHandler struct {
	gallery    gallerydomain.Service
	annotation annotationdomain.Service
}

// ComputeHandler handles ML compute routes (segment, augment, train, SSE).
type ComputeHandler struct{ svc computedomain.Service }

// SystemHandler handles health and spec routes.
type SystemHandler struct{ svc SystemService }

// App is the composition root that wires handler structs from domain services.
type App struct {
	gallery    *GalleryHandler
	annotation *AnnotationHandler
	compute    *ComputeHandler
	system     *SystemHandler
	cors       []string
}

// New builds the App from domain services and the raw SQL connection (for health pings).
func New(cfg config.Config, annSvc annotationdomain.Service, galSvc gallerydomain.Service, compSvc computedomain.Service, sqlDB *sql.DB) *App {
	return &App{
		gallery:    &GalleryHandler{svc: galSvc},
		annotation: &AnnotationHandler{gallery: galSvc, annotation: annSvc},
		compute:    &ComputeHandler{svc: compSvc},
		system:     &SystemHandler{svc: newSystemService(sqlDB, cfg)},
		cors:       cfg.CORSOrigins,
	}
}
