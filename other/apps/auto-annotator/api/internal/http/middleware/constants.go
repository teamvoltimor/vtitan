package middleware

const (
	// HeaderRequestID is the X-Request-ID header name.
	HeaderRequestID = "X-Request-ID"
	// HeaderAccessControlAllowOrig is the Access-Control-Allow-Origin header name.
	HeaderAccessControlAllowOrig = "Access-Control-Allow-Origin"
	// HeaderAccessControlMethods is the Access-Control-Allow-Methods header name.
	HeaderAccessControlMethods = "Access-Control-Allow-Methods"
	// HeaderAccessControlHeaders is the Access-Control-Allow-Headers header name.
	HeaderAccessControlHeaders = "Access-Control-Allow-Headers"
	// HeaderVary is the Vary header name.
	HeaderVary = "Vary"
	// HeaderOrigin is the Origin header name.
	HeaderOrigin = "Origin"

	// ContextKeyRequestID is the context key for storing request IDs.
	ContextKeyRequestID = "request_id"

	// CORSAllowedMethods are the HTTP methods allowed for CORS.
	CORSAllowedMethods = "GET, POST, PUT, DELETE, OPTIONS"
	// CORSAllowAllHeaders indicates that all headers are allowed for CORS.
	CORSAllowAllHeaders = "*"
	// CORSHeaderVaryValue is the value for the Vary header in CORS responses.
	CORSHeaderVaryValue = "Origin"
	// UnknownRequestIDFallback is the fallback value for unknown request IDs.
	UnknownRequestIDFallback = "unknown"
)
