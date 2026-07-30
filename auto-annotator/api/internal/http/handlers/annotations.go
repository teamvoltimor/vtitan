package handlers

import (
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// RegisterRoutes wires the annotation and class routes onto rg.
func (h *AnnotationHandler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(RouteAnnotations, h.GetAnnotations)
	rg.POST(RouteSaveAnnotations, h.SaveAnnotations)
	rg.POST(RouteSkipAnnotation, h.SkipImage)

	rg.GET(RouteListClasses, h.ListClasses)
	rg.POST(RouteUpsertClass, h.UpsertClass)
}

// GetAnnotations returns the saved annotation shapes for an image. GET /annotations/:id
func (h *AnnotationHandler) GetAnnotations(c *gin.Context) {
	id, err := strconv.ParseInt(c.Param(AnnotationIDParamName), 10, 64)
	if err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, ErrInvalidImageID, ErrTitleValidation)
		return
	}
	shapes, err := h.annotation.GetAnnotations(c.Request.Context(), id)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, annotationShapesToOAPI(shapes))
}
