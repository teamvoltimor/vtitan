// Package middleware provides cross-cutting Gin middleware: request IDs, structured
// logging, panic recovery (as RFC 7807), and CORS.
package middleware

import (
	"crypto/rand"
	"encoding/hex"
	"log/slog"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/problem"
)

// RequestID attaches a request id (from the inbound header or freshly generated)
// to the response header and the Gin context.
func RequestID() gin.HandlerFunc {
	return func(c *gin.Context) {
		id := c.GetHeader(HeaderRequestID)
		if id == "" {
			id = newID()
		}
		c.Set(ContextKeyRequestID, id)
		c.Header(HeaderRequestID, id)
		c.Next()
	}
}

// Logger emits one structured JSON log line per request.
func Logger() gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()
		c.Next()
		slog.Info("request",
			"method", c.Request.Method,
			"path", c.Request.URL.Path,
			"status", c.Writer.Status(),
			"duration_ms", time.Since(start).Milliseconds(),
			"request_id", c.GetString(ContextKeyRequestID),
		)
	}
}

// Recover converts a panic into an RFC 7807 500 response.
func Recover() gin.HandlerFunc {
	return func(c *gin.Context) {
		defer func() {
			if r := recover(); r != nil {
				slog.Error("panic", "error", r, "path", c.Request.URL.Path)
				problem.Write(c, http.StatusInternalServerError, "Internal Server Error", "")
			}
		}()
		c.Next()
	}
}

// CORS allows the configured origins, mirroring the permissive FastAPI setup
// (all methods/headers) while restricting Origin to the allow-list.
func CORS(allowed []string) gin.HandlerFunc {
	set := make(map[string]bool, len(allowed))
	for _, o := range allowed {
		set[o] = true
	}
	return func(c *gin.Context) {
		origin := c.GetHeader(HeaderOrigin)
		isAllowed := origin != "" && set[origin]
		if isAllowed {
			c.Header(HeaderAccessControlAllowOrig, origin)
			c.Header(HeaderAccessControlMethods, CORSAllowedMethods)
			c.Header(HeaderAccessControlHeaders, CORSAllowAllHeaders)
			c.Header(HeaderVary, CORSHeaderVaryValue)
		}
		if c.Request.Method == http.MethodOptions {
			// A cross-origin preflight from a non-allow-listed origin must not be
			// answered with a blanket success: only same-origin/no-Origin OPTIONS
			// calls (e.g. internal tooling) and allow-listed preflights get 204.
			if origin != "" && !isAllowed {
				c.AbortWithStatus(http.StatusForbidden)
				return
			}
			c.AbortWithStatus(http.StatusNoContent)
			return
		}
		c.Next()
	}
}

func newID() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return UnknownRequestIDFallback
	}
	return hex.EncodeToString(b[:])
}
