package http

// Path parameter names.
const (
	ParamRobotID      = "robotId"
	ParamWaypointID   = "waypointId"
	ParamScenarioID   = "scenarioId"
	ParamRunID        = "runId"
	ParamAnnotationID = "annotationId"
)

// Query parameter names.
const (
	QueryLimit         = "limit"
	QueryFleetID       = "fleet_id"
	QueryState         = "state"
	QueryChallenge     = "challenge"
	QueryConfidenceMin = "confidence_min"
)

// HTTP header names.
const (
	HeaderRequestID                = "X-Request-Id"
	HeaderAccessControlAllowOrigin = "Access-Control-Allow-Origin"
	HeaderAccessControlAllowMethods = "Access-Control-Allow-Methods"
	HeaderAccessControlAllowHeaders = "Access-Control-Allow-Headers"
)

// CORS header values.
const (
	CORSAllowOrigin  = "*"
	CORSAllowMethods = "GET, POST, OPTIONS"
	CORSAllowHeaders = "Content-Type"
)
