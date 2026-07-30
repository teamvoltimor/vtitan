package handlers

import (
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// ServeImage streams the raw image file for an id. GET /images/:id
func (h *GalleryHandler) ServeImage(c *gin.Context) {
	id, err := strconv.ParseInt(c.Param(ImageIDParamName), 10, 64)
	if err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, ErrInvalidImageID, ErrTitleValidation)
		return
	}
	path, err := h.svc.ImagePath(c.Request.Context(), id)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.Header(HeaderCacheControl, ImageCacheControl)
	c.File(path)
}

// ServeThumbnail streams a downscaled thumbnail, generating and caching it on
// first request. Falls back to the original if decoding fails. GET /images/:id/thumb
func (h *GalleryHandler) ServeThumbnail(c *gin.Context) {
	id, err := strconv.ParseInt(c.Param(ImageIDParamName), 10, 64)
	if err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, ErrInvalidImageID, ErrTitleValidation)
		return
	}
	thumbPath, err := h.svc.ThumbPath(c.Request.Context(), id)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.Header(HeaderCacheControl, ThumbCacheControl)
	c.File(thumbPath)
}

// DeleteImages soft-deletes images by id and returns the updated gallery.
// POST /images/delete
func (h *GalleryHandler) DeleteImages(c *gin.Context) {
	var req DeleteImagesRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	if len(req.ImageIds) == 0 {
		problem.Write(c, http.StatusBadRequest, ErrNoImagesToDelete, "")
		return
	}
	g, err := h.svc.Delete(c.Request.Context(), req.ImageIds)
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toGalleryResponse(g))
}
