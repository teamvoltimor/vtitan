package problem

import (
	"encoding/json"
	"net/http"

	"github.com/gin-gonic/gin"
)

const contentType = "application/problem+json"

type Detail struct {
	Type     string `json:"type"`
	Title    string `json:"title"`
	Status   int    `json:"status"`
	Detail   string `json:"detail,omitempty"`
	Instance string `json:"instance"`
}

// Write emits an RFC 7807 Problem Details response and aborts the handler chain.
func Write(c *gin.Context, status int, title, detail string) {
	b, _ := json.Marshal(Detail{
		Type:     "about:blank",
		Title:    title,
		Status:   status,
		Detail:   detail,
		Instance: c.Request.URL.Path,
	})
	c.Data(status, contentType, b)
	c.Abort()
}

// InternalError writes a 500 without leaking internal error messages to the caller.
func InternalError(c *gin.Context) {
	Write(c, http.StatusInternalServerError, "Internal Server Error", "")
}
