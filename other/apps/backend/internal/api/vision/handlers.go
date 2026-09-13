// Package vision provides the Vision context's Gin HTTP handlers, wired to
// domain/vision via the generated types in openapi.gen.go.
package vision

import (
	"errors"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	domain "github.com/teamvoltimor/vtitan/apps/backend/domain/vision"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api"
	httpconstants "github.com/teamvoltimor/vtitan/apps/backend/internal/http"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/problem"
)

// Handler holds the Vision context's Gin HTTP handlers.
type Handler struct {
	svc domain.Service
}

// NewHandler returns a Handler backed by the given domain Service.
func NewHandler(svc domain.Service) *Handler {
	return &Handler{svc: svc}
}

// RegisterRoutes wires the Vision context's routes onto rg (the /v1 group).
func (h *Handler) RegisterRoutes(rg *gin.RouterGroup) {
	vis := rg.Group(api.RouteVision)
	vis.GET(api.RouteDetections, h.listDetections)
	vis.GET(api.RouteDetectionsCurrent, h.getCurrentDetections)
	vis.GET(api.RouteAnnotations, h.listAnnotations)
	vis.POST(api.RouteAnnotations, h.createAnnotation)
	vis.PUT(api.RouteAnnotation, h.updateAnnotation)
	vis.DELETE(api.RouteAnnotation, h.deleteAnnotation)
	vis.GET(api.RouteModel, h.getActiveModel)
	vis.PUT(api.RouteModel, h.setActiveModel)
	vis.GET(api.RoutePipelineStatus, h.getPipelineStatus)
}

func (h *Handler) listDetections(c *gin.Context) {
	limit := api.DefaultDetectionLimit
	if raw := c.Query(httpconstants.QueryLimit); raw != "" {
		if n, err := strconv.Atoi(raw); err == nil {
			limit = n
		}
	}
	var confidenceMin *float32
	if raw := c.Query(httpconstants.QueryConfidenceMin); raw != "" {
		if f, err := strconv.ParseFloat(raw, 32); err == nil {
			v := float32(f)
			confidenceMin = &v
		}
	}
	dets, err := h.svc.ListDetections(c.Request.Context(), limit, confidenceMin)
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireDetections(dets))
}

func (h *Handler) getCurrentDetections(c *gin.Context) {
	frame, err := h.svc.CurrentDetections(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	if frame == nil {
		problem.Write(c, http.StatusServiceUnavailable, "Service Unavailable", "no telemetry snapshot received yet")
		return
	}
	c.JSON(http.StatusOK, toWireDetectionFrame(frame))
}

func (h *Handler) listAnnotations(c *gin.Context) {
	anns, err := h.svc.ListAnnotations(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireAnnotations(anns))
}

func (h *Handler) createAnnotation(c *gin.Context) {
	var req CreateAnnotationRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	a, err := h.svc.CreateAnnotation(c.Request.Context(), fromCreateAnnotationRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusCreated, toWireAnnotation(a))
}

func (h *Handler) updateAnnotation(c *gin.Context) {
	var req UpdateAnnotationRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	a, err := h.svc.UpdateAnnotation(c.Request.Context(), c.Param(httpconstants.ParamAnnotationID), fromUpdateAnnotationRequest(req))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireAnnotation(a))
}

func (h *Handler) deleteAnnotation(c *gin.Context) {
	err := h.svc.DeleteAnnotation(c.Request.Context(), c.Param(httpconstants.ParamAnnotationID))
	if writeIfNotFound(c, err) {
		return
	}
	c.Status(http.StatusNoContent)
}

func (h *Handler) getActiveModel(c *gin.Context) {
	m, err := h.svc.ActiveModel(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireModelInfo(m))
}

func (h *Handler) setActiveModel(c *gin.Context) {
	var req SetModelRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	m, err := h.svc.SetActiveModel(c.Request.Context(), fromSetModelRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireModelInfo(m))
}

func (h *Handler) getPipelineStatus(c *gin.Context) {
	st, err := h.svc.PipelineStatus(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWirePipelineStatus(st))
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
