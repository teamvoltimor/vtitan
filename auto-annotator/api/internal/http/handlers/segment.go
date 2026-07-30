package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// Segment runs SAM inference for the given click points. POST /segment
func (h *ComputeHandler) Segment(c *gin.Context) {
	var req SegmentationRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	result, err := h.svc.Segment(c.Request.Context(), toSegmentReq(req))
	if err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusOK, toSegmentationResponse(result))
}
