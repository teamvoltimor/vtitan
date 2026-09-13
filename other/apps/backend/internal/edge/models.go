package edge

import (
	httpconstants "github.com/teamvoltimor/vtitan/apps/backend/internal/http"
)

// Route prefixes, and the WebSocket route -- both root-router concerns, not
// owned by any one context. Per-context REST route paths (including
// Telemetry's) live in internal/api/routes.go.
const (
	RouteOpenAPISpec = "/openapi.yaml"
	RouteV1Telemetry = "/v1/telemetry"
	RouteV1          = "/v1"
	RouteWS          = "/ws"
)

// Exported shortcuts to http constants, used by the root middleware below.
var (
	headerRequestID                 = httpconstants.HeaderRequestID
	headerAccessControlAllowOrigin  = httpconstants.HeaderAccessControlAllowOrigin
	headerAccessControlAllowMethods = httpconstants.HeaderAccessControlAllowMethods
	headerAccessControlAllowHeaders = httpconstants.HeaderAccessControlAllowHeaders
	corsAllowOrigin                 = httpconstants.CORSAllowOrigin
	corsAllowMethods                = httpconstants.CORSAllowMethods
	corsAllowHeaders                = httpconstants.CORSAllowHeaders
)
