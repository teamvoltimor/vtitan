package edge

import (
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"google.golang.org/protobuf/encoding/protojson"

	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/session"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/telemetry"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/config"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/problem"
)

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = false omits zero-value optional fields.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: false, UseProtoNames: true}

// NewRouter wires up the gin router for the REST + WebSocket edge.
func NewRouter(telSvc telemetry.TelemetryService, sessSvc session.SessionService, cfg *config.Config, log *zap.Logger) *gin.Engine {
	if !cfg.Dev {
		gin.SetMode(gin.ReleaseMode)
	}

	r := gin.New()
	r.Use(recoverMiddleware(log))
	r.Use(corsMiddleware())
	r.Use(requestIDMiddleware(log))

	h := &handlers{telSvc: telSvc, sessSvc: sessSvc, cfg: cfg, log: log}
	ws := newWSManager(telSvc, log)

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
