package problem

import (
	"encoding/json"
	"net/http"

	"github.com/gin-gonic/gin"
)

const contentType = "application/problem+json"

// CtxKeyRequestID is the gin context key requestIDMiddleware stores the
// per-request correlation ID under. Write reads it back so every Problem
// Details response can be matched to the corresponding server log line.
const CtxKeyRequestID = "request_id"

// Detail is an RFC 7807 Problem Details response body.
type Detail struct {
	Type          string `json:"type"`
	Title         string `json:"title"`
	Status        int    `json:"status"`
	Detail        string `json:"detail,omitempty"`
	Instance      string `json:"instance"`
	CorrelationID string `json:"correlation_id,omitempty"`
}

// Write emits an RFC 7807 Problem Details response and aborts the handler chain.
func Write(c *gin.Context, status int, title, detail string) {
	b, _ := json.Marshal(Detail{
		Type:          "about:blank",
		Title:         title,
		Status:        status,
		Detail:        detail,
		Instance:      c.Request.URL.Path,
		CorrelationID: c.GetString(CtxKeyRequestID),
	})
	c.Data(status, contentType, b)
	c.Abort()
}

// InternalError writes a 500 without leaking internal error messages to the caller.
func InternalError(c *gin.Context) {
	Write(c, http.StatusInternalServerError, "Internal Server Error", "")
}
