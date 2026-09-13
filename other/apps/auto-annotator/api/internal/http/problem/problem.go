// Package problem emits RFC 7807 Problem Details responses.
package problem

import (
	"crypto/rand"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"strings"

	"github.com/gin-gonic/gin"
	"github.com/go-playground/validator/v10"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

// ContentType is the media type for Problem Details responses.
const ContentType = "application/problem+json"

const (
	baseURI = "urn:auto-annotator:error"

	TypeNotFound      = baseURI + ":not-found"
	TypeConflict      = baseURI + ":conflict"
	TypeValidation    = baseURI + ":validation"
	TypeBadRequest    = baseURI + ":bad-request"
	TypeUnprocessable = baseURI + ":unprocessable-entity"
	TypeInternal      = baseURI + ":internal"
)

const (
	zeroUUID           = "00000000-0000-0000-0000-000000000000"
	uuidVersionBitMask = 0x0f
	uuidVersion4       = 0x40
	uuidVariantBitMask = 0x3f
	uuidVariant10      = 0x80
)

// FieldError is a per-field validation failure included in 400/422 responses.
type FieldError struct {
	Field   string `json:"field"`
	Message string `json:"message"`
}

// Problem is the RFC 7807 body.
type Problem struct {
	Detail        any          `json:"detail"`
	Type          string       `json:"type"`
	Title         string       `json:"title"`
	Instance      string       `json:"instance"`
	CorrelationID string       `json:"correlation_id"`
	Errors        []FieldError `json:"errors,omitempty"`
	Status        int          `json:"status"`
}

// Write renders a Problem Details response. The type URI is derived from the
// status code; a blank title falls back to the HTTP status phrase.
func Write(c *gin.Context, status int, detail any, title string) {
	if title == "" {
		if title = http.StatusText(status); title == "" {
			title = "Error"
		}
	}
	write(c, status, typeForStatus(status), title, detail, nil)
}

// FromDomain maps a domain or store sentinel error to the appropriate HTTP
// status and writes the Problem Details response. Unmapped errors become 500.
func FromDomain(c *gin.Context, err error) {
	switch {
	case errors.Is(err, sql.ErrNoRows), errors.Is(err, domain.ErrNotFound):
		write(c, http.StatusNotFound, TypeNotFound, "Not Found", err.Error(), nil)
	case errors.Is(err, domain.ErrConflict):
		write(c, http.StatusConflict, TypeConflict, "Conflict", err.Error(), nil)
	case errors.Is(err, domain.ErrInvalidInput):
		write(c, http.StatusBadRequest, TypeBadRequest, "Bad Request", err.Error(), nil)
	default:
		write(c, http.StatusInternalServerError, TypeInternal, "Internal Server Error", err.Error(), nil)
	}
}

// ValidationError extracts field-level errors from a go-playground/validator
// error and writes a 422 response. Falls back to a 400 for JSON decode errors.
func ValidationError(c *gin.Context, err error) {
	var ve validator.ValidationErrors
	if errors.As(err, &ve) {
		fields := make([]FieldError, 0, len(ve))
		for _, fe := range ve {
			fields = append(fields, FieldError{
				Field:   strings.ToLower(fe.Field()),
				Message: validationMessage(fe),
			})
		}
		write(
			c,
			http.StatusUnprocessableEntity,
			TypeValidation,
			"Validation Error",
			"request validation failed",
			fields,
		)
		return
	}
	write(c, http.StatusBadRequest, TypeBadRequest, "Bad Request", err.Error(), nil)
}

func validationMessage(fe validator.FieldError) string {
	switch fe.Tag() {
	case "required":
		return "field is required"
	case "oneof":
		return fmt.Sprintf("must be one of: %s", fe.Param())
	default:
		return fe.Tag()
	}
}

func typeForStatus(status int) string {
	switch status {
	case http.StatusNotFound:
		return TypeNotFound
	case http.StatusConflict:
		return TypeConflict
	case http.StatusBadRequest:
		return TypeBadRequest
	case http.StatusUnprocessableEntity:
		return TypeUnprocessable
	default:
		return TypeInternal
	}
}

func write(c *gin.Context, status int, typeURI, title string, detail any, fields []FieldError) {
	body, _ := json.Marshal(Problem{
		Type:          typeURI,
		Title:         title,
		Status:        status,
		Detail:        detail,
		Instance:      c.Request.URL.Path,
		CorrelationID: newCorrelationID(),
		Errors:        fields,
	})
	c.Abort()
	c.Data(status, ContentType, body)
}

func newCorrelationID() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return zeroUUID
	}
	b[6] = (b[6] & uuidVersionBitMask) | uuidVersion4
	b[8] = (b[8] & uuidVariantBitMask) | uuidVariant10
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:16])
}
