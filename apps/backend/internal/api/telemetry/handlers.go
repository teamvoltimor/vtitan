// Package telemetry provides the Telemetry context's Gin HTTP handlers,
// wired to domain/telemetry (live snapshots) and domain/session (recorded
// session playback) via the generated request/response types in
// openapi.gen.go.
package telemetry

import (
	"errors"
	"log/slog"
	"net/http"
	"regexp"
	"strconv"

	"github.com/gin-gonic/gin"
	"google.golang.org/protobuf/types/known/timestamppb"

	robotdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/robot"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/session"
	domain "github.com/teamvoltimor/vtitan/apps/backend/domain/telemetry"
	telemetryv1 "github.com/teamvoltimor/vtitan/apps/backend/gen/telemetry/v1"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/config"
	httpconstants "github.com/teamvoltimor/vtitan/apps/backend/internal/http"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/problem"
)

// API version and status constants.
const (
	apiVersion    = "0.1.0"
	statusOK      = "ok"
	statusSuccess = "success"
)

// History API boundary constants.
const (
	historyDefaultLimit = 60
	historyMinLimit     = 10
	historyMaxLimit     = 360
)

var sessionIDPattern = regexp.MustCompile(`^session_\d+$`)

// Handler holds the Telemetry context's Gin HTTP handlers.
type Handler struct {
	telSvc  domain.TelemetryService
	sessSvc session.SessionService
	cfg     *config.Config

	// robotSvc/defaultRobotID back the legacy /v1/telemetry/robot/config/speed
	// endpoint, which delegates to the real Robot context config store (this
	// project has exactly one physical robot) instead of no-op'ing.
	robotSvc       robotdomain.Service
	defaultRobotID string
}

// NewHandler returns a Handler backed by the given domain services.
func NewHandler(
	telSvc domain.TelemetryService,
	sessSvc session.SessionService,
	robotSvc robotdomain.Service,
	defaultRobotID string,
	cfg *config.Config,
) *Handler {
	return &Handler{
		telSvc: telSvc, sessSvc: sessSvc,
		robotSvc: robotSvc, defaultRobotID: defaultRobotID,
		cfg: cfg,
	}
}

// RegisterRoutes wires the Telemetry context's REST routes onto rg (expected
// to be the /v1/telemetry group). The WebSocket route is registered
// separately by the root router, since it isn't part of the OpenAPI REST
// surface this Handler owns.
func (h *Handler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(api.RouteHealth, h.health)
	rg.GET(api.RouteLatest, h.latest)
	rg.GET(api.RouteHistory, h.history)
	rg.GET(api.RouteTopics, h.topics)
	rg.POST(api.RouteSpeed, h.updateSpeed)
	rg.GET(api.RouteConfig, h.getConfig)
	rg.GET(api.RouteSessions, h.listSessions)
	rg.GET(api.RouteSession, h.loadSession)
}

func (h *Handler) health(c *gin.Context) {
	c.JSON(http.StatusOK, HealthResponse{Status: statusOK, Version: apiVersion})
}

func (h *Handler) latest(c *gin.Context) {
	snap := h.telSvc.Latest()
	if snap == nil {
		problem.Write(c, http.StatusServiceUnavailable, "Service Unavailable", "no snapshot received yet")
		return
	}
	writeProto(c, http.StatusOK, snap)
}

func (h *Handler) history(c *gin.Context) {
	limit := historyDefaultLimit
	if raw := c.Query(httpconstants.QueryLimit); raw != "" {
		if n, err := strconv.Atoi(raw); err == nil && n >= historyMinLimit && n <= historyMaxLimit {
			limit = n
		}
	}
	snaps := h.telSvc.History(limit)
	writeProtoSlice(c, snaps)
}

func (h *Handler) topics(c *gin.Context) {
	topics := h.telSvc.LatestTopics()
	if topics == nil {
		topics = &telemetryv1.TopicsSnapshot{
			Timestamp: timestamppb.Now(),
			Topics:    []*telemetryv1.TopicUpdate{},
		}
	}
	writeProto(c, http.StatusOK, topics)
}

func (h *Handler) updateSpeed(c *gin.Context) {
	var body SpeedRequest
	if err := c.ShouldBindJSON(&body); err != nil {
		problem.Write(c, http.StatusBadRequest, "Bad Request", err.Error())
		return
	}
	// Persists to the Robot context's config store for the single default
	// robot (still not forwarded to ROS/hardware -- that path isn't wired up
	// yet -- but no longer a no-op: GET /v1/robots/{id}/config now reflects it).
	speed := body.MaxLinearSpeed
	if _, err := h.robotSvc.UpdateConfig(c.Request.Context(), h.defaultRobotID,
		robotdomain.UpdateConfigRequest{MaxLinearSpeed: &speed},
	); err != nil {
		slog.Error("persist speed config", "error", err)
		problem.InternalError(c)
		return
	}
	slog.Info("speed config update", "max_linear_speed", body.MaxLinearSpeed)
	c.JSON(http.StatusOK, SpeedUpdateResponse{Status: statusSuccess, MaxLinearSpeed: body.MaxLinearSpeed})
}

func (h *Handler) getConfig(c *gin.Context) {
	c.JSON(http.StatusOK, ConfigResponse{
		HttpAddr:    h.cfg.HTTPAddr,
		GrpcAddr:    h.cfg.GRPCAddr,
		HistorySize: h.cfg.HistorySize,
		Dev:         h.cfg.Dev,
		MaxSessions: h.cfg.MaxSessions,
		SessionsDir: h.cfg.SessionsDir,
	})
}

func (h *Handler) listSessions(c *gin.Context) {
	infos, err := h.sessSvc.ListSessions(c.Request.Context())
	if err != nil {
		slog.Error("list sessions", "error", err)
		problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "failed to list sessions")
		return
	}
	out := make([]SessionResponse, len(infos))
	for i, s := range infos {
		out[i] = SessionResponse{
			SessionId:  s.SessionID,
			CreatedAt:  s.CreatedAt.UTC(),
			EntryCount: s.EntryCount,
		}
	}
	c.JSON(http.StatusOK, out)
}

func (h *Handler) loadSession(c *gin.Context) {
	id := c.Param("id")
	if !sessionIDPattern.MatchString(id) {
		problem.Write(c, http.StatusBadRequest, "Bad Request", "invalid session id format")
		return
	}
	snaps, err := h.sessSvc.LoadSession(c.Request.Context(), id)
	if errors.Is(err, session.ErrSessionNotFound) {
		problem.Write(c, http.StatusNotFound, "Not Found", "session not found")
		return
	}
	if err != nil {
		slog.Error("load session", "session_id", id, "error", err)
		problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "failed to load session")
		return
	}
	writeProtoSlice(c, snaps)
}
