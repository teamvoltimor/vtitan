package handlers

import (
	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/middleware"
)

// Router builds the Gin engine with middleware and all routes wired to their
// respective handler structs.
func (a *App) Router() *gin.Engine {
	r := gin.New()
	r.Use(
		middleware.RequestID(),
		middleware.Logger(),
		middleware.Recover(),
		middleware.CORS(a.cors),
	)

	r.GET(RouteLiveness, a.system.Liveness)
	r.GET(RouteReadiness, a.system.Readiness)
	r.GET(RouteHealth, a.system.Readiness)

	v1 := r.Group(RouteAPIv1)
	v1.GET(RouteGallery, a.gallery.GetGallery)
	v1.GET(RouteGalleryGrouped, a.gallery.GetGroupedGallery)
	v1.POST(RouteGalleryImport, a.gallery.ImportGallery)

	v1.GET(RouteImages, a.gallery.ServeImage)
	v1.GET(RouteImageThumb, a.gallery.ServeThumbnail)
	v1.POST(RouteImagesDelete, a.gallery.DeleteImages)

	v1.GET(RouteAnnotations, a.annotation.GetAnnotations)
	v1.POST(RouteSaveAnnotations, a.annotation.SaveAnnotations)
	v1.POST(RouteSkipAnnotation, a.annotation.SkipImage)

	v1.GET(RouteListClasses, a.annotation.ListClasses)
	v1.POST(RouteUpsertClass, a.annotation.UpsertClass)

	v1.GET(RouteListModels, a.compute.ListModels)

	v1.GET(RouteOpenAPISpec, a.system.ServeOpenAPISpec)

	v1.POST(RouteSegment, a.compute.Segment)

	v1.POST(RouteStartAugment, a.compute.StartAugment)
	v1.GET(RouteStatusAugment, a.compute.JobStatus)
	v1.GET(RouteStreamAugment, a.compute.StreamAugment)

	v1.POST(RouteStartTrain, a.compute.StartTrain)
	v1.GET(RouteStatusTrain, a.compute.JobStatus)
	v1.GET(RouteStreamTrain, a.compute.StreamTrain)

	return r
}
