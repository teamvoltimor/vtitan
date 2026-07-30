package edge

import (
	httpconstants "github.com/teamvoltimor/vtitan/platform/backend/internal/http"
)

// API version and status constants.
const (
	apiVersion    = "0.1.0"
	statusOK      = "ok"
	statusSuccess = "success"
)

// Content type constants.
const (
	contentTypeJSON = "application/json; charset=utf-8"
)

// History API boundary constants.
const (
	historyDefaultLimit = 60
	historyMinLimit     = 10
	historyMaxLimit     = 360
)

// Route prefixes.
const (
	RouteOpenAPISpec = "/openapi.yaml"
	RouteV1Telemetry = "/v1/telemetry"
	RouteV1          = "/v1"
)

// Telemetry sub-routes (registered under RouteV1Telemetry).
const (
	RouteHealth   = "/health"
	RouteLatest   = "/latest"
	RouteHistory  = "/history"
	RouteTopics   = "/topics"
	RouteSpeed    = "/robot/config/speed"
	RouteConfig   = "/config"
	RouteSessions = "/sessions"
	RouteSession  = "/sessions/:id"
	RouteWS       = "/ws"
)

// Exported shortcuts to http constants for compatibility.
var (
	headerRequestID                = httpconstants.HeaderRequestID
	headerAccessControlAllowOrigin  = httpconstants.HeaderAccessControlAllowOrigin
	headerAccessControlAllowMethods = httpconstants.HeaderAccessControlAllowMethods
	headerAccessControlAllowHeaders = httpconstants.HeaderAccessControlAllowHeaders
	corsAllowOrigin                 = httpconstants.CORSAllowOrigin
	corsAllowMethods                = httpconstants.CORSAllowMethods
	corsAllowHeaders                = httpconstants.CORSAllowHeaders
)

// Exported shortcuts to http parameter constants for compatibility.
const (
	ParamRobotID      = httpconstants.ParamRobotID
	ParamWaypointID   = httpconstants.ParamWaypointID
	ParamScenarioID   = httpconstants.ParamScenarioID
	ParamRunID        = httpconstants.ParamRunID
	ParamAnnotationID = httpconstants.ParamAnnotationID

	QueryLimit         = httpconstants.QueryLimit
	QueryFleetID       = httpconstants.QueryFleetID
	QueryState         = httpconstants.QueryState
	QueryChallenge     = httpconstants.QueryChallenge
	QueryConfidenceMin = httpconstants.QueryConfidenceMin
)

// HealthResponse, ProblemDetails, SessionResponse, ConfigResponse,
// SpeedRequest, SpeedUpdateResponse are generated from openapi/contexts/telemetry.yaml.
// See openapi.telemetry.gen.go for their definitions.
