// Package navigation provides the Navigation context's Gin HTTP handlers,
// wired to domain/navigation via the generated types in openapi.gen.go.
package navigation

import (
	"errors"
	"net/http"

	"github.com/gin-gonic/gin"

	domain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/navigation"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/problem"
)

// Handler holds the Navigation context's Gin HTTP handlers.
type Handler struct {
	svc domain.Service
}

// NewHandler returns a Handler backed by the given domain Service.
func NewHandler(svc domain.Service) *Handler {
	return &Handler{svc: svc}
}

// RegisterRoutes wires the Navigation context's routes onto rg (the /v1 group).
func (h *Handler) RegisterRoutes(rg *gin.RouterGroup) {
	nav := rg.Group("/navigation")
	nav.GET("/waypoints", h.listWaypoints)
	nav.POST("/waypoints", h.createWaypoint)
	nav.DELETE("/waypoints/:waypointId", h.deleteWaypoint)
	nav.GET("/route", h.getRoute)
	nav.POST("/route", h.planRoute)
	nav.GET("/status", h.getStatus)
	nav.GET("/clearance", h.getClearance)
	nav.GET("/tuning", h.getTuning)
	nav.PUT("/tuning", h.updateTuning)
}

func (h *Handler) listWaypoints(c *gin.Context) {
	wps, err := h.svc.ListWaypoints(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireWaypoints(wps))
}

func (h *Handler) createWaypoint(c *gin.Context) {
	var req CreateWaypointRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	wp, err := h.svc.CreateWaypoint(c.Request.Context(), fromCreateWaypointRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusCreated, toWireWaypoint(wp))
}

func (h *Handler) deleteWaypoint(c *gin.Context) {
	err := h.svc.DeleteWaypoint(c.Request.Context(), c.Param("waypointId"))
	if writeIfNotFound(c, err) {
		return
	}
	c.Status(http.StatusNoContent)
}

func (h *Handler) getRoute(c *gin.Context) {
	r, err := h.svc.Route(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireRoute(r))
}

func (h *Handler) planRoute(c *gin.Context) {
	var req PlanRouteRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	r, err := h.svc.PlanRoute(c.Request.Context(), fromPlanRouteRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireRoute(r))
}

func (h *Handler) getStatus(c *gin.Context) {
	st, err := h.svc.Status(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireStatus(st))
}

func (h *Handler) getClearance(c *gin.Context) {
	cl, err := h.svc.Clearance(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireClearance(cl))
}

func (h *Handler) getTuning(c *gin.Context) {
	t, err := h.svc.Tuning(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireTuning(t))
}

func (h *Handler) updateTuning(c *gin.Context) {
	var req NavigationTuning
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	t, err := h.svc.UpdateTuning(c.Request.Context(), fromWireTuning(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireTuning(t))
}

func writeIfNotFound(c *gin.Context, err error) bool {
	switch {
	case err == nil:
		return false
	case errors.Is(err, domain.ErrNotFound):
		problem.Write(c, http.StatusNotFound, "Not Found", err.Error())
	default:
		problem.InternalError(c)
	}
	return true
}
