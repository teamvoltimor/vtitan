package edge

import (
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"google.golang.org/protobuf/encoding/protojson"

	navigationdomain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/navigation"
	robotdomain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/robot"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/session"
	simulationdomain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/simulation"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/telemetry"
	visiondomain "github.com/teamvoldemor/voldemorbot/platform/backend/domain/vision"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/api/navigation"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/api/robot"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/api/simulation"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/api/vision"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/config"
	"github.com/teamvoldemor/voldemorbot/platform/backend/internal/problem"
)

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = false omits zero-value optional fields.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: false, UseProtoNames: true}

// Services bundles the domain services NewRouter wires onto the edge.
type Services struct {
	Telemetry telemetry.TelemetryService
	Session   session.SessionService
	Robot     robotdomain.Service
	// DefaultRobotID is the singleton robot's ID, seeded at startup, that the
	// legacy /v1/telemetry/robot/config/speed endpoint delegates to (this
	// project has exactly one physical robot; the Robot context's fleet-shaped
	// CRUD is a superset of that reality).
	DefaultRobotID string
	Navigation     navigationdomain.Service
	Simulation     simulationdomain.Service
	Vision         visiondomain.Service
}

// NewRouter wires up the gin router for the REST + WebSocket edge.
func NewRouter(svcs Services, cfg *config.Config, log *zap.Logger) *gin.Engine {
	if !cfg.Dev {
		gin.SetMode(gin.ReleaseMode)
	}

	r := gin.New()
	// Order matters: RequestID must run first so the correlation ID it sets is
	// already in context by the time Recovery's deferred recover() (or any
	// later middleware/handler) needs it.
	r.Use(requestIDMiddleware(log))
	r.Use(recoverMiddleware(log))
	r.Use(corsMiddleware())

	h := &handlers{
		telSvc: svcs.Telemetry, sessSvc: svcs.Session,
		robotSvc: svcs.Robot, defaultRobotID: svcs.DefaultRobotID,
		cfg: cfg, log: log,
	}
	ws := newWSManager(svcs.Telemetry, log)

	r.GET("/openapi.yaml", func(c *gin.Context) {
		c.File(cfg.OpenAPISpecPath)
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

	apiV1 := r.Group("/v1")
	robot.NewHandler(svcs.Robot).RegisterRoutes(apiV1)
	navigation.NewHandler(svcs.Navigation).RegisterRoutes(apiV1)
	simulation.NewHandler(svcs.Simulation).RegisterRoutes(apiV1)
	vision.NewHandler(svcs.Vision).RegisterRoutes(apiV1)

	return r
}

func recoverMiddleware(log *zap.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		defer func() {
			if r := recover(); r != nil {
				log.Error("http panic",
					zap.Any("panic", r),
					zap.String("path", c.Request.URL.Path),
					zap.String("request_id", c.GetString(problem.CtxKeyRequestID)),
				)
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
		c.Set(problem.CtxKeyRequestID, rid)
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
