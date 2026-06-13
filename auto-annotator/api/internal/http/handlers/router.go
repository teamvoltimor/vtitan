package handlers

import (
	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/middleware"
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
	r.GET(RouteLiveness, a.Liveness)
	r.GET(RouteReadiness, a.Readiness)
	r.GET(RouteHealth, a.Readiness)

	v1 := r.Group(RouteAPIv1)
	v1.GET(RouteGallery, a.GetGallery)
	v1.GET(RouteGalleryGrouped, a.GetGroupedGallery)
	v1.POST(RouteGalleryImport, a.ImportGallery)

	v1.GET(RouteImages, a.ServeImage)
	v1.POST(RouteImagesDelete, a.DeleteImages)

	v1.GET(RouteAnnotations, a.GetAnnotations)
	v1.POST(RouteSaveAnnotations, a.SaveAnnotations)
	v1.POST(RouteSkipAnnotation, a.SkipImage)

	v1.GET(RouteListClasses, a.ListClasses)
	v1.POST(RouteUpsertClass, a.UpsertClass)

	v1.GET(RouteListModels, a.ListModels)

	v1.POST(RouteSegment, a.Segment)

	v1.POST(RouteStartAugment, a.StartAugment)
	v1.GET(RouteStatusAugment, a.JobStatus)
	v1.GET(RouteStreamAugment, a.StreamAugment)

	v1.POST(RouteStartTrain, a.StartTrain)
	v1.GET(RouteStatusTrain, a.JobStatus)
	v1.GET(RouteStreamTrain, a.StreamTrain)

	return r
}
