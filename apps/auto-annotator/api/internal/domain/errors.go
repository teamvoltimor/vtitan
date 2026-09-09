package domain

import "errors"

var (
	// ErrNotFound is a sentinel wrapped by domain/store errors when a lookup
	// finds no matching row; handlers map it to an HTTP 404.
	ErrNotFound = errors.New("not found")
	// ErrConflict is a sentinel wrapped when an operation would violate a
	// uniqueness or state invariant; handlers map it to an HTTP 409.
	ErrConflict = errors.New("conflict")
	// ErrInvalidInput is a sentinel wrapped when request data fails a domain
	// validation rule; handlers map it to an HTTP 400.
	ErrInvalidInput = errors.New("invalid input")
)
