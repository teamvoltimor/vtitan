package handlers

import (
	"database/sql"
	"errors"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/domain"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
)

// GetAnnotations returns the saved annotation shapes for a done image by reading
// its YOLO label file. GET /annotations/:id
func (a *App) GetAnnotations(c *gin.Context) {
	id, err := strconv.ParseInt(c.Param(AnnotationIDParamName), 10, 64)
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

	classes, err := a.Store.Q.ListClasses(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	classNames := make([]string, len(classes))
	for i, cls := range classes {
		classNames[i] = cls.Name
	}

	shapes := a.loadAnnotations(rec.ID, rec.Path, domain.StatusName(rec.Status), nullStr(rec.FormatUsed), classNames)
	if shapes == nil {
		shapes = []dto.Shape{}
	}
	c.JSON(http.StatusOK, shapes)
}
