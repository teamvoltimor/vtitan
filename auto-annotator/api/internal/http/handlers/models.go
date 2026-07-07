package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
)

// ListModels returns the configured models from models.toml. GET /models
func (h *ComputeHandler) ListModels(c *gin.Context) {
	models, err := h.svc.ListModels()
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toModelsOAPI(models))
}
