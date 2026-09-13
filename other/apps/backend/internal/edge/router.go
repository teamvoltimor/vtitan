package edge

import (
	"log/slog"
	"net/http"
	"strconv"
	"time"

	"github.com/gin-gonic/gin"
	"google.golang.org/protobuf/encoding/protojson"

	navigationdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/navigation"
	robotdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/robot"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/session"
	simulationdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/simulation"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/telemetry"
	visiondomain "github.com/teamvoltimor/vtitan/apps/backend/domain/vision"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api/navigation"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api/robot"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api/simulation"
	apitelemetry "github.com/teamvoltimor/vtitan/apps/backend/internal/api/telemetry"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/api/vision"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/config"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/problem"
)

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = true forces zero-valued *implicit*-presence fields
// (e.g. angular_velocity.x/y/z when the IMU reads exactly 0) onto the wire
// instead of omitting them -- the frontend's schema requires these numeric
// fields always be present. This does NOT affect proto3 `optional` fields
// (range_min, speed, etc.): those are backed by a synthetic oneof, which
// EmitUnpopulated explicitly excludes, so a genuinely-unset optional still
// doesn't get force-emitted -- the "not measured" vs "measured as zero"
// distinction those fields rely on is preserved.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: true, UseProtoNames: true}

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
func NewRouter(svcs Services, cfg *config.Config) *gin.Engine {
	if !cfg.Dev {
		gin.SetMode(gin.ReleaseMode)
	}

	r := gin.New()
	// Order matters: RequestID must run first so the correlation ID it sets is
	// already in context by the time Recovery's deferred recover() (or any
	// later middleware/handler) needs it.
	r.Use(requestIDMiddleware())
	r.Use(recoverMiddleware())
	r.Use(corsMiddleware())

	ws := newWSManager(svcs.Telemetry)

	r.GET(RouteOpenAPISpec, func(c *gin.Context) {
		c.File(cfg.OpenAPISpecPath)
	})

	v1 := r.Group(RouteV1Telemetry)
	apitelemetry.NewHandler(svcs.Telemetry, svcs.Session, svcs.Robot, svcs.DefaultRobotID, cfg).RegisterRoutes(v1)
	v1.GET(RouteWS, ws.handle)

	apiV1 := r.Group(RouteV1)
	robot.NewHandler(svcs.Robot).RegisterRoutes(apiV1)
	navigation.NewHandler(svcs.Navigation).RegisterRoutes(apiV1)
	simulation.NewHandler(svcs.Simulation).RegisterRoutes(apiV1)
	vision.NewHandler(svcs.Vision).RegisterRoutes(apiV1)

	return r
}

func recoverMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		defer func() {
			if r := recover(); r != nil {
				slog.Error("http panic",
					"panic", r,
					"path", c.Request.URL.Path,
					"request_id", c.GetString(problem.CtxKeyRequestID),
				)
				problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "")
			}
		}()
		c.Next()
	}
}

func corsMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header(headerAccessControlAllowOrigin, corsAllowOrigin)
		c.Header(headerAccessControlAllowMethods, corsAllowMethods)
		c.Header(headerAccessControlAllowHeaders, corsAllowHeaders)
		if c.Request.Method == http.MethodOptions {
			c.AbortWithStatus(http.StatusNoContent)
			return
		}
		c.Next()
	}
}

func requestIDMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		rid := c.GetHeader(headerRequestID)
		if rid == "" {
			rid = newRequestID()
		}
		c.Set(problem.CtxKeyRequestID, rid)
		c.Header(headerRequestID, rid)
		start := time.Now()
		c.Next()
		slog.Info("request",
			"method", c.Request.Method,
			"path", c.Request.URL.Path,
			"status", c.Writer.Status(),
			"duration_ms", time.Since(start).Milliseconds(),
			"request_id", rid,
		)
	}
}

// newRequestID generates a short random request ID using the current nanosecond timestamp.
func newRequestID() string {
	return strconv.FormatInt(now().UnixNano(), 36)
}
