package handlers

import (
	"database/sql"
	"errors"
	"net/http"
	"os"
	"strconv"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store/db"
)

// ServeImage streams the raw image file for an id. GET /images/:id
func (a *App) ServeImage(c *gin.Context) {
	id, err := strconv.ParseInt(c.Param(ImageIDParamName), 10, 64)
	if err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "invalid image id", "Validation Error")
		return
	}
	rec, err := a.Store.Q.GetImageByID(c.Request.Context(), id)
	if errors.Is(err, sql.ErrNoRows) {
		problem.Write(c, http.StatusNotFound, "Image not found", "")
		return
	}
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.File(rec.Path)
}

// DeleteImages soft-deletes images by id, removes their files, and returns the
// updated gallery. POST /images/delete
func (a *App) DeleteImages(c *gin.Context) {
	var req dto.DeleteImagesRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}
	if len(req.ImageIDs) == 0 {
		problem.Write(c, http.StatusBadRequest, "No images to delete", "")
		return
	}

	now := time.Now().UTC().Format(time.RFC3339)
	ctx := c.Request.Context()
	for _, id := range req.ImageIDs {
		rec, err := a.Store.Q.GetImageByID(ctx, id)
		if errors.Is(err, sql.ErrNoRows) {
			problem.Write(c, http.StatusNotFound, "Image not found", "")
			return
		}
		if err != nil {
			problem.Write(c, http.StatusInternalServerError, err.Error(), "")
			return
		}
		if err := a.Store.Q.SoftDeleteImage(ctx, db.SoftDeleteImageParams{
			DeletedAt: sql.NullString{String: now, Valid: true},
			ID:        id,
		}); err != nil {
			problem.Write(c, http.StatusInternalServerError, err.Error(), "")
			return
		}
		_ = os.Remove(rec.Path)
	}

	resp, err := a.buildGalleryResponse(ctx)
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, resp)
}
