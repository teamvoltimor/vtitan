package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// ListClasses returns all annotation classes ordered by id. GET /classes
func (h *AnnotationHandler) ListClasses(c *gin.Context) {
	classes, err := h.annotation.ListClasses(c.Request.Context())
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, classesToOAPI(classes))
}

// UpsertClass inserts or updates a class by name and returns the full list.
// POST /classes
func (h *AnnotationHandler) UpsertClass(c *gin.Context) {
	var req UpsertClassRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	ctx := c.Request.Context()
	if err := h.annotation.UpsertClass(ctx, annotationdomain.UpsertClassReq{Name: req.Name, Color: req.Color}); err != nil {
		problem.FromDomain(c, err)
		return
	}
	classes, err := h.annotation.ListClasses(ctx)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, classesToOAPI(classes))
}
