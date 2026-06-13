package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store/db"
)

// ListClasses returns all annotation classes ordered by id. GET /classes
func (a *App) ListClasses(c *gin.Context) {
	rows, err := a.Store.Q.ListClasses(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, classRowsToItems(rows))
}

// UpsertClass inserts or updates a class by name and returns the full list.
// POST /classes
func (a *App) UpsertClass(c *gin.Context) {
	var req dto.UpsertClassRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}
	if err := a.Store.Q.UpsertClass(c.Request.Context(), db.UpsertClassParams{
		Name:  req.Name,
		Color: req.Color,
	}); err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	rows, err := a.Store.Q.ListClasses(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, classRowsToItems(rows))
}

func classRowsToItems(rows []db.ListClassesRow) []dto.ClassItem {
	items := make([]dto.ClassItem, 0, len(rows))
	for _, r := range rows {
		items = append(items, dto.ClassItem{ID: r.ID, Name: r.Name, Color: r.Color})
	}
	return items
}
