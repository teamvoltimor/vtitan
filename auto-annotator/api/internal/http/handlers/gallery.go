package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
)

// GetGallery returns all images with status counts. GET /gallery
func (h *GalleryHandler) GetGallery(c *gin.Context) {
	g, err := h.svc.GetGallery(c.Request.Context())
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toGalleryResponse(g))
}

// GetGroupedGallery returns parent images with augmentation counts. GET /gallery/grouped
func (h *GalleryHandler) GetGroupedGallery(c *gin.Context) {
	rows, err := h.svc.GetGroupedGallery(c.Request.Context())
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toParentImageItems(rows))
}

// ImportGallery accepts uploaded image files and returns the updated gallery.
// POST /gallery/import
func (h *GalleryHandler) ImportGallery(c *gin.Context) {
	form, err := c.MultipartForm()
	if err != nil {
		problem.Write(c, http.StatusBadRequest, err.Error(), "")
		return
	}
	files := form.File[GalleryImportFormFieldKey]
	if len(files) == 0 {
		problem.Write(c, http.StatusBadRequest, ErrNoFilesProvided, "")
		return
	}
	g, err := h.svc.Import(c.Request.Context(), files)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toGalleryResponse(g))
}
