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

	telemetryv1 "github.com/klevor/telemetry-backend/gen/telemetry/v1"
	"github.com/klevor/telemetry-backend/internal/config"
	"github.com/klevor/telemetry-backend/internal/recorder"
)

var sessionIDPattern = regexp.MustCompile(`^session_\d+$`)

type handlers struct {
	store    Store
	sessions SessionStore
	cfg      *config.Config
	log      *zap.Logger
}

func (h *handlers) health(c *gin.Context) {
	c.JSON(http.StatusOK, HealthResponse{Status: statusOK, Version: apiVersion})
}

func (h *handlers) latest(c *gin.Context) {
	snap := h.store.Latest()
	if snap == nil {
		c.JSON(http.StatusServiceUnavailable, ErrorResponse{Error: "no snapshot received yet"})
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
	snaps := h.store.History(limit)
	writeProtoSlice(c, snaps, h.log)
}

func (h *handlers) topics(c *gin.Context) {
	topics := h.store.LatestTopics()
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
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
		return
	}
	h.log.Info("speed config update", zap.Float64("max_linear_speed", body.MaxLinearSpeed))
	// Forward to robot via ROS when that path is wired up.
	c.JSON(http.StatusOK, SpeedUpdateResponse{Status: statusSuccess, MaxLinearSpeed: body.MaxLinearSpeed})
}

// writeProto marshals a single proto message to JSON and writes it to the response.
func writeProto(c *gin.Context, code int, msg proto.Message, log *zap.Logger) {
	b, err := marshaler.Marshal(msg)
	if err != nil {
		log.Error("proto marshal", zap.Error(err))
		c.Status(http.StatusInternalServerError)
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
			c.Status(http.StatusInternalServerError)
			return
		}
		parts[i] = b
	}
	out, err := json.Marshal(parts)
	if err != nil {
		log.Error("json marshal slice", zap.Error(err))
		c.Status(http.StatusInternalServerError)
		return
	}
	c.Data(http.StatusOK, contentTypeJSON, out)
}

func (h *handlers) getConfig(c *gin.Context) {
	c.JSON(http.StatusOK, ConfigResponse{
		HTTPAddr:    h.cfg.HTTPAddr,
		GRPCAddr:    h.cfg.GRPCAddr,
		HistorySize: h.cfg.HistorySize,
		Dev:         h.cfg.Dev,
		MaxSessions: h.cfg.MaxSessions,
		SessionsDir: h.cfg.SessionsDir,
	})
}

func (h *handlers) listSessions(c *gin.Context) {
	infos, err := h.sessions.ListSessions(c.Request.Context())
	if err != nil {
		h.log.Error("list sessions", zap.Error(err))
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "failed to list sessions"})
		return
	}
	out := make([]SessionResponse, len(infos))
	for i, s := range infos {
		out[i] = SessionResponse{
			SessionID:  s.SessionID,
			CreatedAt:  s.CreatedAt.UTC().Format(timeFormatISO),
			EntryCount: s.EntryCount,
		}
	}
	c.JSON(http.StatusOK, out)
}

func (h *handlers) loadSession(c *gin.Context) {
	id := c.Param("id")
	if !sessionIDPattern.MatchString(id) {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid session id format"})
		return
	}
	snaps, err := h.sessions.LoadSession(c.Request.Context(), id)
	if errors.Is(err, recorder.ErrSessionNotFound) {
		c.JSON(http.StatusNotFound, ErrorResponse{Error: "session not found"})
		return
	}
	if err != nil {
		h.log.Error("load session", zap.String("session_id", id), zap.Error(err))
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "failed to load session"})
		return
	}
	writeProtoSlice(c, snaps, h.log)
}

// newRequestID generates a short random request ID using the current nanosecond timestamp.
func newRequestID() string {
	return strconv.FormatInt(now().UnixNano(), 36)
}
