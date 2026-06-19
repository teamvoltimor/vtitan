package edge

import (
	"context"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"google.golang.org/protobuf/encoding/protojson"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/config"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/problem"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/recorder"
)

type (
	// Store is the read side of the memory store consumed by the edge layer.
	Store interface {
		Latest() *telemetryv1.RobotSnapshot
		History(limit int) []*telemetryv1.RobotSnapshot
		Subscribe() (snapshots <-chan *telemetryv1.RobotSnapshot, cancel func())
		LatestTopics() *telemetryv1.TopicsSnapshot
	}

	// SessionStore is the read side of the recorder consumed by the edge layer.
	SessionStore interface {
		ListSessions(ctx context.Context) ([]recorder.SessionInfo, error)
		LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error)
	}
)

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = false omits zero-value optional fields.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: false, UseProtoNames: true}

// NewRouter wires up the gin router for the REST + WebSocket edge.
func NewRouter(store Store, sessions SessionStore, cfg *config.Config, log *zap.Logger) *gin.Engine {
	if !cfg.Dev {
		gin.SetMode(gin.ReleaseMode)
	}

	r := gin.New()
	r.Use(recoverMiddleware(log))
	r.Use(corsMiddleware())
	r.Use(requestIDMiddleware(log))

	h := &handlers{store: store, sessions: sessions, cfg: cfg, log: log}
	ws := newWSManager(store, log)

	r.GET("/openapi.yaml", func(c *gin.Context) {
		c.File("../openapi/openapi.yaml")
	})

	v1 := r.Group("/v1/telemetry")
	v1.GET("/health", h.health)
	v1.GET("/latest", h.latest)
	v1.GET("/history", h.history)
	v1.GET("/topics", h.topics)
	v1.POST("/robot/config/speed", h.updateSpeed)
	v1.GET("/config", h.getConfig)
	v1.GET("/sessions", h.listSessions)
	v1.GET("/sessions/:id", h.loadSession)
	v1.GET("/ws", ws.handle)

	return r
}

func recoverMiddleware(log *zap.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		defer func() {
			if r := recover(); r != nil {
				log.Error("http panic", zap.Any("panic", r), zap.String("path", c.Request.URL.Path))
				problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "")
			}
		}()
		c.Next()
	}
}

func corsMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("Access-Control-Allow-Origin", corsAllowOrigin)
		c.Header("Access-Control-Allow-Methods", corsAllowMethods)
		c.Header("Access-Control-Allow-Headers", corsAllowHeaders)
		if c.Request.Method == http.MethodOptions {
			c.AbortWithStatus(http.StatusNoContent)
			return
		}
		c.Next()
	}
}

func requestIDMiddleware(log *zap.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		rid := c.GetHeader(headerRequestID)
		if rid == "" {
			rid = newRequestID()
		}
		c.Set(ctxKeyRequestID, rid)
		c.Header(headerRequestID, rid)
		start := time.Now()
		c.Next()
		log.Info("request",
			zap.String("method", c.Request.Method),
			zap.String("path", c.Request.URL.Path),
			zap.Int("status", c.Writer.Status()),
			zap.Int64("duration_ms", time.Since(start).Milliseconds()),
			zap.String("request_id", rid),
		)
	}
}
