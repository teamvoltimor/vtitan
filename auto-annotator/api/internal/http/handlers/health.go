package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
)

// Liveness reports that the process is up. GET /healthz
func (a *App) Liveness(c *gin.Context) {
	c.JSON(http.StatusOK, dto.LivenessResponse{Status: StatusOK})
}

// Readiness checks database connectivity. GET /readyz and /health
//
// Inference availability is wired to the SAM compute worker in a later phase;
// until then it is reported false and readiness reflects DB connectivity only.
func (a *App) Readiness(c *gin.Context) {
	dbOK := a.Store.DB.PingContext(c.Request.Context()) == nil
	message := "System ready"
	if !dbOK {
		message = "Database error"
	}
	c.JSON(http.StatusOK, dto.HealthStatus{
		Ready:              dbOK,
		InferenceAvailable: false,
		DatabaseAccessible: dbOK,
		Message:            message,
	})
}
