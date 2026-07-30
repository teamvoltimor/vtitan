package handlers

import (
	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/middleware"
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
	a.gallery.RegisterRoutes(v1)
	a.annotation.RegisterRoutes(v1)
	a.compute.RegisterRoutes(v1)
	a.system.RegisterRoutes(v1)

	return r
}
