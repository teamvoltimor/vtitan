package edge

import (
	"encoding/json"
	"errors"
	"net/http"
	"regexp"
	"strconv"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	robotdomain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/robot"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/session"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/telemetry"
	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/config"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/problem"
)

var sessionIDPattern = regexp.MustCompile(`^session_\d+$`)

type handlers struct {
	telSvc  telemetry.TelemetryService
	sessSvc session.SessionService
	cfg     *config.Config
	log     *zap.Logger

	// robotSvc/defaultRobotID back the legacy /v1/telemetry/robot/config/speed
	// endpoint, which now delegates to the real Robot context config store
	// (this project has exactly one physical robot) instead of no-op'ing.
	robotSvc       robotdomain.Service
	defaultRobotID string
}

func (h *handlers) health(c *gin.Context) {
	c.JSON(http.StatusOK, HealthResponse{Status: statusOK, Version: apiVersion})
}

func (h *handlers) latest(c *gin.Context) {
	snap := h.telSvc.Latest()
	if snap == nil {
		problem.Write(c, http.StatusServiceUnavailable, "Service Unavailable", "no snapshot received yet")
		return
	}
	writeProto(c, http.StatusOK, snap, h.log)
}

func (h *handlers) history(c *gin.Context) {
	limit := historyDefaultLimit
	if raw := c.Query("limit"); raw != "" {
		if n, err := strconv.Atoi(raw); err == nil && n >= historyMinLimit && n <= historyMaxLimit {
			limit = n
		}
	}
	snaps := h.telSvc.History(limit)
	writeProtoSlice(c, snaps, h.log)
}

func (h *handlers) topics(c *gin.Context) {
	topics := h.telSvc.LatestTopics()
	if topics == nil {
		topics = &telemetryv1.TopicsSnapshot{
			Timestamp: timestamppb.Now(),
			Topics:    []*telemetryv1.TopicUpdate{},
		}
	}
	writeProto(c, http.StatusOK, topics, h.log)
}

func (h *handlers) updateSpeed(c *gin.Context) {
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
		h.log.Error("persist speed config", zap.Error(err))
		problem.InternalError(c)
		return
	}
	h.log.Info("speed config update", zap.Float64("max_linear_speed", body.MaxLinearSpeed))
	c.JSON(http.StatusOK, SpeedUpdateResponse{Status: statusSuccess, MaxLinearSpeed: body.MaxLinearSpeed})
}

// writeProto marshals a single proto message to JSON and writes it to the response.
func writeProto(c *gin.Context, code int, msg proto.Message, log *zap.Logger) {
	b, err := marshaler.Marshal(msg)
	if err != nil {
		log.Error("proto marshal", zap.Error(err))
		problem.InternalError(c)
		return
	}
	c.Data(code, contentTypeJSON, b)
}

// writeProtoSlice marshals a slice of proto messages to a JSON array.
func writeProtoSlice[T proto.Message](c *gin.Context, msgs []T, log *zap.Logger) {
	parts := make([]json.RawMessage, len(msgs))
	for i, msg := range msgs {
		b, err := marshaler.Marshal(msg)
		if err != nil {
			log.Error("proto marshal slice item", zap.Error(err), zap.Int("index", i))
			problem.InternalError(c)
			return
		}
		parts[i] = b
	}
	out, err := json.Marshal(parts)
	if err != nil {
		log.Error("json marshal slice", zap.Error(err))
		problem.InternalError(c)
		return
	}
	c.Data(http.StatusOK, contentTypeJSON, out)
}

func (h *handlers) getConfig(c *gin.Context) {
	c.JSON(http.StatusOK, ConfigResponse{
		HttpAddr:    h.cfg.HTTPAddr,
		GrpcAddr:    h.cfg.GRPCAddr,
		HistorySize: h.cfg.HistorySize,
		Dev:         h.cfg.Dev,
		MaxSessions: h.cfg.MaxSessions,
		SessionsDir: h.cfg.SessionsDir,
	})
}

func (h *handlers) listSessions(c *gin.Context) {
	infos, err := h.sessSvc.ListSessions(c.Request.Context())
	if err != nil {
		h.log.Error("list sessions", zap.Error(err))
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

func (h *handlers) loadSession(c *gin.Context) {
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
		h.log.Error("load session", zap.String("session_id", id), zap.Error(err))
		problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "failed to load session")
		return
	}
	writeProtoSlice(c, snaps, h.log)
}

// newRequestID generates a short random request ID using the current nanosecond timestamp.
func newRequestID() string {
	return strconv.FormatInt(now().UnixNano(), 36)
}
