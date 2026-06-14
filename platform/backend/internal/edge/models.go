package edge

const (
	apiVersion      = "0.1.0"
	statusOK        = "ok"
	statusSuccess   = "success"
	contentTypeJSON = "application/json; charset=utf-8"
	timeFormatISO   = "2006-01-02T15:04:05.999Z07:00"

	historyDefaultLimit = 60
	historyMinLimit     = 10
	historyMaxLimit     = 360

	corsAllowOrigin  = "*"
	corsAllowMethods = "GET, POST, OPTIONS"
	corsAllowHeaders = "Content-Type"

	headerRequestID = "X-Request-Id"
	ctxKeyRequestID = "request_id"
)

type (
	// HealthResponse is the payload for GET /v1/telemetry/health.
	HealthResponse struct {
		Status  string `json:"status"`
		Version string `json:"version"`
	}

	// ErrorResponse is the standard error envelope for all edge endpoints.
	ErrorResponse struct {
		Error string `json:"error"`
	}

	// SessionResponse is the JSON shape returned by GET /v1/telemetry/sessions.
	SessionResponse struct {
		SessionID  string `json:"session_id"`
		CreatedAt  string `json:"created_at"`
		EntryCount int    `json:"entry_count"`
	}

	// ConfigResponse is the payload for GET /v1/telemetry/config.
	// String fields come first to minimize the GC-scanned pointer span.
	ConfigResponse struct {
		HTTPAddr    string `json:"http_addr"`
		GRPCAddr    string `json:"grpc_addr"`
		SessionsDir string `json:"sessions_dir"`
		HistorySize int    `json:"history_size"`
		MaxSessions int    `json:"max_sessions"`
		Dev         bool   `json:"dev"`
	}

	// SpeedRequest is the body for POST /v1/telemetry/robot/config/speed.
	SpeedRequest struct {
		MaxLinearSpeed float64 `json:"maxLinearSpeed" validate:"required,gt=0"`
	}

	// SpeedUpdateResponse is the payload returned after a speed update.
	SpeedUpdateResponse struct {
		Status         string  `json:"status"`
		MaxLinearSpeed float64 `json:"maxLinearSpeed"`
	}
)
