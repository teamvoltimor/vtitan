package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"
)

// Liveness reports that the process is up. GET /healthz
func (h *SystemHandler) Liveness(c *gin.Context) {
	c.JSON(http.StatusOK, LivenessResponse{Status: Ok})
}

// Readiness checks database connectivity. GET /readyz and /health
func (h *SystemHandler) Readiness(c *gin.Context) {
	dbOK := h.svc.Ping(c.Request.Context()) == nil
	message := MsgSystemReady
	if !dbOK {
		message = MsgDatabaseError
	}
	c.JSON(http.StatusOK, HealthStatus{
		Ready:              dbOK,
		InferenceAvailable: false,
		DatabaseAccessible: dbOK,
		Message:            message,
	})
}
