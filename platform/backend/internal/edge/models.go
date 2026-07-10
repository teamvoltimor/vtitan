package edge

const (
	apiVersion      = "0.1.0"
	statusOK        = "ok"
	statusSuccess   = "success"
	contentTypeJSON = "application/json; charset=utf-8"

	historyDefaultLimit = 60
	historyMinLimit     = 10
	historyMaxLimit     = 360

	corsAllowOrigin  = "*"
	corsAllowMethods = "GET, POST, OPTIONS"
	corsAllowHeaders = "Content-Type"

	headerRequestID = "X-Request-Id"
)

// HealthResponse, ProblemDetails, SessionResponse, ConfigResponse,
// SpeedRequest, SpeedUpdateResponse are generated from openapi/contexts/telemetry.yaml.
// See openapi.telemetry.gen.go for their definitions.
