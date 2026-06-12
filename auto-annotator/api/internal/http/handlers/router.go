package handlers

import (
	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/http/middleware"
)

// Router builds the Gin engine with middleware and all routes. Domain routers
// are mounted under the versioned /api/v1 prefix, matching the Python contract.
func (a *App) Router() *gin.Engine {
	r := gin.New()
	r.Use(
		middleware.RequestID(),
		middleware.Logger(),
		middleware.Recover(),
		middleware.CORS(a.Cfg.CORSOrigins),
	)

	// System probes (unversioned, mirroring FastAPI).
	r.GET("/healthz", a.Liveness)
	r.GET("/readyz", a.Readiness)
	r.GET("/health", a.Readiness)

	v1 := r.Group("/api/v1")
	{
		v1.GET("/gallery", a.GetGallery)
		v1.GET("/gallery/grouped", a.GetGroupedGallery)
		v1.POST("/gallery/import", a.ImportGallery)

		v1.GET("/images/:id", a.ServeImage)
		v1.POST("/images/delete", a.DeleteImages)

		v1.GET("/annotations/:id", a.GetAnnotations)

		v1.GET("/classes", a.ListClasses)
		v1.POST("/classes", a.UpsertClass)

		v1.GET("/models", a.ListModels)
	}

	return r
}
