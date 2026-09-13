// Package robot provides the Robot context's Gin HTTP handlers, wired to
// domain/robot via the generated request/response types in openapi.gen.go.
package robot

import (
	"errors"
	"net/http"

	"github.com/gin-gonic/gin"

	domain "github.com/teamvoltimor/vtitan/apps/backend/domain/robot"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api"
	httpconstants "github.com/teamvoltimor/vtitan/apps/backend/internal/http"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/problem"
)

// Handler holds the Robot context's Gin HTTP handlers.
type Handler struct {
	svc domain.Service
}

// NewHandler returns a Handler backed by the given domain Service.
func NewHandler(svc domain.Service) *Handler {
	return &Handler{svc: svc}
}

// RegisterRoutes wires the Robot context's routes onto rg (expected to be the
// /v1 group, since the OpenAPI paths are /robots, not /robot/robots).
func (h *Handler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(api.RouteRobots, h.listRobots)
	rg.POST(api.RouteRobots, h.createRobot)
	rg.GET(api.RouteRobot, h.getRobot)
	rg.PATCH(api.RouteRobot, h.updateRobot)
	rg.DELETE(api.RouteRobot, h.deleteRobot)
	rg.GET(api.RouteRobotStatus, h.getRobotStatus)
	rg.PUT(api.RouteRobotConfig, h.updateRobotConfig)
	rg.POST(api.RouteRobotCommand, h.sendRobotCommand)
}

func (h *Handler) listRobots(c *gin.Context) {
	fleetID := c.Query(httpconstants.QueryFleetID)
	state := domain.State(c.Query(httpconstants.QueryState))

	robots, err := h.svc.List(c.Request.Context(), fleetID, state)
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusOK, toWireRobots(robots))
}

func (h *Handler) createRobot(c *gin.Context) {
	var req CreateRobotRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	r, err := h.svc.Create(c.Request.Context(), fromCreateRequest(req))
	if err != nil {
		problem.InternalError(c)
		return
	}
	c.JSON(http.StatusCreated, toWireRobot(r))
}

func (h *Handler) getRobot(c *gin.Context) {
	r, err := h.svc.Get(c.Request.Context(), c.Param(httpconstants.ParamRobotID))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireRobot(r))
}

func (h *Handler) updateRobot(c *gin.Context) {
	var req UpdateRobotRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	r, err := h.svc.Update(c.Request.Context(), c.Param(httpconstants.ParamRobotID), fromUpdateRequest(req))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireRobot(r))
}

func (h *Handler) deleteRobot(c *gin.Context) {
	err := h.svc.Delete(c.Request.Context(), c.Param(httpconstants.ParamRobotID))
	if writeIfNotFound(c, err) {
		return
	}
	c.Status(http.StatusNoContent)
}

func (h *Handler) getRobotStatus(c *gin.Context) {
	st, err := h.svc.Status(c.Request.Context(), c.Param(httpconstants.ParamRobotID))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireStatus(st))
}

func (h *Handler) updateRobotConfig(c *gin.Context) {
	var req UpdateRobotConfigRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	cfg, err := h.svc.UpdateConfig(c.Request.Context(), c.Param(httpconstants.ParamRobotID), fromUpdateConfigRequest(req))
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusOK, toWireConfig(cfg))
}

func (h *Handler) sendRobotCommand(c *gin.Context) {
	var req RobotCommand
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	cmd, err := fromCommand(req)
	if err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, "Validation Failed", err.Error())
		return
	}
	res, err := h.svc.Command(c.Request.Context(), c.Param(httpconstants.ParamRobotID), cmd)
	if writeIfNotFound(c, err) {
		return
	}
	c.JSON(http.StatusAccepted, toWireCommandResult(res))
}

// writeIfNotFound writes the appropriate Problem response for err and reports
// whether the caller should stop handling the request.
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
