package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/handlers/mapping"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// SaveAnnotations writes the label file + image copy, marks the image done,
// and returns the updated gallery. POST /annotations/save
func (h *AnnotationHandler) SaveAnnotations(c *gin.Context) {
	var req SaveAnnotationsRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	ctx := c.Request.Context()
	if err := h.annotation.Save(ctx, toAnnotationSaveReq(req)); err != nil {
		problem.FromDomain(c, err)
		return
	}
	g, err := h.gallery.GetGallery(ctx)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusCreated, mapping.ToGalleryResponse(g))
}

// SkipImage marks an image as skipped and returns the updated gallery.
// POST /annotations/skip
func (h *AnnotationHandler) SkipImage(c *gin.Context) {
	var req SkipRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	ctx := c.Request.Context()
	if err := h.annotation.Skip(ctx, req.ImageId); err != nil {
		problem.FromDomain(c, err)
		return
	}
	g, err := h.gallery.GetGallery(ctx)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, mapping.ToGalleryResponse(g))
}
