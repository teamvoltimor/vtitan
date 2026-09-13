package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/handlers/mapping"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/handlers/routes"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// RegisterRoutes wires the gallery and image routes onto rg.
func (h *GalleryHandler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(routes.Gallery, h.GetGallery)
	rg.GET(routes.GalleryGrouped, h.GetGroupedGallery)
	rg.POST(routes.GalleryImport, h.ImportGallery)

	rg.GET(routes.Images, h.ServeImage)
	rg.GET(routes.ImageThumb, h.ServeThumbnail)
	rg.POST(routes.ImagesDelete, h.DeleteImages)
}

// GetGallery returns all images with status counts. GET /gallery
func (h *GalleryHandler) GetGallery(c *gin.Context) {
	g, err := h.svc.GetGallery(c.Request.Context())
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, mapping.ToGalleryResponse(g))
}

// GetGroupedGallery returns parent images with augmentation counts. GET /gallery/grouped
func (h *GalleryHandler) GetGroupedGallery(c *gin.Context) {
	rows, err := h.svc.GetGroupedGallery(c.Request.Context())
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, mapping.ToParentImageItems(rows))
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
	c.JSON(http.StatusOK, mapping.ToGalleryResponse(g))
}
