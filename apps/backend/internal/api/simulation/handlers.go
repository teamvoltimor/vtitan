// Package simulation provides the Simulation context's Gin HTTP handlers,
// wired to domain/simulation via the generated types in openapi.gen.go.
package simulation

import (
	"errors"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	domain "github.com/teamvoltimor/vtitan/platform/backend/domain/simulation"
	"github.com/teamvoltimor/vtitan/platform/backend/internal/api"
	httpconstants "github.com/teamvoltimor/vtitan/platform/backend/internal/http"
	"github.com/teamvoltimor/vtitan/platform/backend/internal/problem"
)

// Handler holds the Simulation context's Gin HTTP handlers.
type Handler struct {
	svc domain.Service
}

// NewHandler returns a Handler backed by the given domain Service.
func NewHandler(svc domain.Service) *Handler {
	return &Handler{svc: svc}
}

// RegisterRoutes wires the Simulation context's routes onto rg (the /v1 group).
func (h *Handler) RegisterRoutes(rg *gin.RouterGroup) {
	sim := rg.Group(api.RouteSimulation)
	sim.GET(api.RouteScenarios, h.listScenarios)
	sim.POST(api.RouteScenarios, h.generateScenario)
	sim.GET(api.RouteScenario, h.getScenario)
	sim.DELETE(api.RouteScenario, h.deleteScenario)
	sim.GET(api.RouteRuns, h.listRuns)
	sim.POST(api.RouteRuns, h.startRun)
	sim.GET(api.RouteRun, h.getRun)
	sim.POST(api.RouteRun, h.controlRun)
	sim.GET(api.RouteEnvironment, h.listEnvironments)
}

func (h *Handler) listScenarios(c *gin.Context) {
	challenge := domain.Challenge(c.Query(httpconstants.QueryChallenge))
	limit := 0
	if raw := c.Query(httpconstants.QueryLimit); raw != "" {
		if n, err := strconv.Atoi(raw); err == nil {
			limit = n
		}
	}
	scs, err := h.svc.ListScenarios(c.Request.Context(), challenge, limit)
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireScenarioSummaries(scs))
}

func (h *Handler) generateScenario(c *gin.Context) {
	var req GenerateScenarioRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	sc, err := h.svc.GenerateScenario(c.Request.Context(), fromGenerateScenarioRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusCreated, toWireScenario(sc))
}

func (h *Handler) getScenario(c *gin.Context) {
	sc, err := h.svc.GetScenario(c.Request.Context(), c.Param(httpconstants.ParamScenarioID))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireScenario(sc))
}

func (h *Handler) deleteScenario(c *gin.Context) {
	err := h.svc.DeleteScenario(c.Request.Context(), c.Param(httpconstants.ParamScenarioID))
	if writeIfNotFound(c, err) {
		return
	}
	c.Status(http.StatusNoContent)
}

func (h *Handler) listRuns(c *gin.Context) {
	runs, err := h.svc.ListRuns(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireRuns(runs))
}

func (h *Handler) startRun(c *gin.Context) {
	var req StartRunRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	run, err := h.svc.StartRun(c.Request.Context(), fromStartRunRequest(req))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusCreated, toWireRun(run))
}

func (h *Handler) getRun(c *gin.Context) {
	run, err := h.svc.GetRun(c.Request.Context(), c.Param(httpconstants.ParamRunID))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireRun(run))
}

func (h *Handler) controlRun(c *gin.Context) {
	var req RunControlRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	run, err := h.svc.ControlRun(c.Request.Context(), c.Param(httpconstants.ParamRunID), domain.RunAction(req.Action))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireRun(run))
}

func (h *Handler) listEnvironments(c *gin.Context) {
	envs, err := h.svc.ListEnvironments(c.Request.Context())
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireEnvironments(envs))
}

func writeIfNotFound(c *gin.Context, err error) bool {
	switch {
	case err == nil:
		return false
	case errors.Is(err, domain.ErrNotFound):
		problem.Write(c, http.StatusNotFound, "Not Found", err.Error())
	default:
		problem.InternalError(c)
	}
	return true
}
