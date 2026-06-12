// Package problem emits RFC 7807 Problem Details responses, mirroring the
// Python exception_handlers so error bodies stay identical across backends.
package problem

import (
	"crypto/rand"
	"encoding/json"
	"fmt"
	"net/http"

	"github.com/gin-gonic/gin"
)

// ContentType is the media type for Problem Details responses.
const ContentType = "application/problem+json"

// Problem is the RFC 7807 body. Field names match the Python payload, including
// the snake_case correlation_id.
type Problem struct {
	Type          string `json:"type"`
	Title         string `json:"title"`
	Status        int    `json:"status"`
	Detail        any    `json:"detail"`
	Instance      string `json:"instance"`
	CorrelationID string `json:"correlation_id"`
}

// Write renders a Problem Details response and aborts the request. A blank
// title falls back to the HTTP status phrase.
func Write(c *gin.Context, status int, detail any, title string) {
	if title == "" {
		if title = http.StatusText(status); title == "" {
			title = "Error"
		}
	}
	body, _ := json.Marshal(Problem{
		Type:          "about:blank",
		Title:         title,
		Status:        status,
		Detail:        detail,
		Instance:      c.Request.URL.Path,
		CorrelationID: newCorrelationID(),
	})
	c.Abort()
	c.Data(status, ContentType, body)
}

func newCorrelationID() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "00000000-0000-0000-0000-000000000000"
	}
	b[6] = (b[6] & 0x0f) | 0x40 // version 4
	b[8] = (b[8] & 0x3f) | 0x80 // variant 10
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:16])
}
