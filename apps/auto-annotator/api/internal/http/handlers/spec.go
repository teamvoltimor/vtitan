package handlers

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/handlers/routes"
)

const DefaultOpenAPIPath = "api/openapi.yaml"

// RegisterRoutes wires the OpenAPI spec route onto rg.
func (h *SystemHandler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(routes.OpenAPISpec, h.ServeOpenAPISpec)
}

// ServeOpenAPISpec serves the OpenAPI YAML spec. GET /api/v1/openapi.yaml
func (h *SystemHandler) ServeOpenAPISpec(c *gin.Context) {
	data, err := h.svc.OpenAPISpec()
	if err != nil {
		c.AbortWithStatus(http.StatusNotFound)
		return
	}
	c.Header(HeaderContentType, ContentTypeYAML)
	c.String(http.StatusOK, string(data))
}
